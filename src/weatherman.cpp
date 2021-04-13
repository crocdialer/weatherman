#include "ArduinoLowPower.h"
#include "utils.h"
#include "Timer.hpp"

// lora / radiohead
#include "rfm95_config.h"
#include "NodeTypes.h"

// BME temperature/pressure/humidity sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// median-filtering of measurements
#include "RunningMedian.h"

const uint8_t g_lora_address = 114;

// analog-digital-converter (ADC)
#define ADC_BITS 10
constexpr uint32_t ADC_MAX = (1 << ADC_BITS) - 1U;

// update interval in ms
const int g_update_interval = 33;
char g_serial_buf[512];

//! time management
const int g_update_interval_params = 2000;
int g_time_accum = 0, g_time_accum_params = 0;
long g_last_time_stamp;

enum TimerEnum
{
  TIMER_LORA_SEND = 0,
  TIMER_BATTERY_MEASURE = 1,
  TIMER_SENSOR_MEASURE = 2,
  NUM_TIMERS
};
kinski::Timer g_timer[NUM_TIMERS];

// battery
#if defined(ARDUINO_SAMD_ZERO)
constexpr uint8_t g_battery_pin = A7;
#elif defined(ARDUINO_FEATHER_M4)
constexpr uint8_t g_battery_pin = A6;
#endif

uint8_t g_battery_val = 0;

// BME280 temperature/pressure/humidity sensor
uint8_t g_sensor_i2c_adress = 0x76;// SDO low: 0x76, SDO high 0x77
Adafruit_BME280 g_sensor;

// number of samples to combine for one measurement
const uint16_t g_num_samples = 5;

// interval between measurements in seconds
float g_measure_interval = 0.1f;

// last measurements
RunningMedian g_temperature{g_num_samples};

// in hPa
RunningMedian g_pressure{g_num_samples};

// relative in range [0..1]
RunningMedian g_humidity{g_num_samples};

////////////////////////////////////////////////////////////////////////////////
// forward declared functions

void blink_status_led();

////////////////////////////////////////////////////////////////////////////////

// lora assets
lora::config_t g_lora_config = {};

// bundle radio-driver, datagram-manager and crypto assets
lora::driver_struct_t m_rfm95 = {};

//! lora message buffer
uint8_t g_lora_buffer[RH_RF95_MAX_MESSAGE_LEN];

float g_lora_send_interval = 5.f;

////////////////////////////////////////////////////////////////////////////////

void set_address(uint8_t address)
{
    g_lora_config.address = address;

    // init RFM95 module
    if(lora::setup(g_lora_config, m_rfm95))
    {
        Serial.print("LoRa radio init complete -> now listening on adress: 0x");
        Serial.println(g_lora_config.address, HEX);
    }
    else
    {
       Serial.println("LoRa radio init failed");
       while(true){ blink_status_led(); }
    }
}

////////////////////////////////////////////////////////////////////////////////
void lora_receive()
{
    uint8_t len = sizeof(g_lora_buffer);
    uint8_t from, to, msg_id, flags;

    // check for messages addressed to this node
    if(m_rfm95.manager->recvfromAck(g_lora_buffer, &len, &from, &to, &msg_id, &flags))
    {
        // received something
    }
}

////////////////////////////////////////////////////////////////////////////////

template<typename T> bool lora_send_status(const T &data)
{
    // data + checksum
    constexpr size_t num_bytes = sizeof(T) + 1;

    uint8_t crc_data[3 + sizeof(T)];
    crc_data[0] = g_lora_config.address;
    crc_data[1] = RH_BROADCAST_ADDRESS;

    memcpy(crc_data + 2, &data, sizeof(T));
    crc_data[sizeof(crc_data) - 1] = crc8(crc_data, 2 + sizeof(T));

    // send a broadcast-message
    return m_rfm95.manager->sendto(crc_data + 2, num_bytes, RH_BROADCAST_ADDRESS);
}

////////////////////////////////////////////////////////////////////////////////

void blink_status_led()
{
    digitalWrite(13, LOW);
    delay(500);
    digitalWrite(13, HIGH);
    delay(500);
}

void setup()
{
    // drives our status LED
    pinMode(13, OUTPUT);

    // indicate "not ready"
    digitalWrite(13, HIGH);

    // while(!Serial){ blink_status_led(); }
    Serial.begin(115200);

    // battery measuring
    g_timer[TIMER_BATTERY_MEASURE].set_callback([]()
    {
        // voltage is divided by 2, so multiply back
        constexpr float voltage_divider = 2.f;
        auto raw_bat_measure = analogRead(g_battery_pin);

        float voltage = 3.3f * (float)raw_bat_measure * voltage_divider / (float)ADC_MAX;
        g_battery_val = static_cast<uint8_t>(map_value<float>(voltage, 3.6f, 4.2f, 0.f, 255.f));
        Serial.printf("battery-: %d%%\n", 100 * g_battery_val / 255);
        // Serial.printf("raw_bat_measure: %d\n", raw_bat_measure);
    });
    g_timer[TIMER_BATTERY_MEASURE].set_periodic();
    g_timer[TIMER_BATTERY_MEASURE].expires_from_now(10.f);

    // lora config
    set_address(g_lora_address);

    g_timer[TIMER_LORA_SEND].set_callback([]()
    {
        weather_t weather = {};
        weather.battery = g_battery_val;
        weather.temperature = map_value<float>(g_temperature.getMedian(), -50.f, 100.f, 0, 65535);
        weather.pressure = map_value<float>(g_pressure.getMedian(), 500.f, 1500.f, 0, 65535);
        weather.humidity = g_humidity.getMedian() * 255;

        lora_send_status(weather);
    });
    g_timer[TIMER_LORA_SEND].set_periodic();
    g_timer[TIMER_LORA_SEND].expires_from_now(g_lora_send_interval);

    // BME setup
    if(!g_sensor.begin(g_sensor_i2c_adress))
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while(true){ blink_status_led(); }
    }

    // sensor measuring
    g_timer[TIMER_SENSOR_MEASURE].set_callback([]()
    {
        g_sensor.takeForcedMeasurement();
        g_temperature.add(g_sensor.readTemperature());
        g_pressure.add(g_sensor.readPressure() / 100.f);
        g_humidity.add(g_sensor.readHumidity() / 100.f);
    });
    g_timer[TIMER_SENSOR_MEASURE].set_periodic();
    g_timer[TIMER_SENSOR_MEASURE].expires_from_now(g_measure_interval);

    digitalWrite(13, LOW);
}

void loop()
{
    // time measurement
    uint32_t delta_time = millis() - g_last_time_stamp;
    g_last_time_stamp = millis();
    g_time_accum += delta_time;

    float next_timer_event = 10.f;

    // poll Timer objects
    for(uint32_t i = 0; i < NUM_TIMERS; ++i)
    {
         g_timer[i].poll();
         next_timer_event = min(next_timer_event, g_timer[i].expires_from_now());
    }

    // wake up 2ms before next event
    int sleep_duration = max(next_timer_event * 1000 - 2, 0);
    LowPower.idle(sleep_duration);
}
