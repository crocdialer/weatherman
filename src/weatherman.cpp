#include "device_id.h"
#include "utils.h"
#include "Timer.hpp"

// lora / radiohead
#include "rfm95_config.h"
#include "NodeTypes.h"

// BME temperature/pressure/humidity sensor
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

// analog-digital-converter (ADC)
#define ADC_BITS 10
constexpr uint32_t ADC_MAX = (1 << ADC_BITS) - 1U;

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
constexpr uint8_t g_battery_pin = A7;
uint8_t g_battery_val = 0;

// BME280 temperature/pressure/humidity sensor
Adafruit_BME280 g_sensor;

// last measurements
float g_temperature;

// in hPa
float g_pressure;

// relative in range [0..1]
float g_humidity;

////////////////////////////////////////////////////////////////////////////////
// forward declared functions

void parse_line(char *the_line);

template <typename T> void process_input(T& the_device);

////////////////////////////////////////////////////////////////////////////////

// lora assets
lora::config_t g_lora_config = {};

// bundle radio-driver, datagram-manager and crypto assets
lora::driver_struct_t m_rfm95 = {};

//! lora message buffer
uint8_t g_lora_buffer[RH_RF95_MAX_MESSAGE_LEN];

float g_lora_send_interval = 2.f;

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
    else{ Serial.println("LoRa radio init failed"); }
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

bool lora_send_status()
{
    bool ret = false;
    constexpr size_t num_bytes = sizeof(weather_t) + 1;

    uint8_t data[num_bytes];

    weather_t &weather = *(weather_t*)data;
    weather = {};
    weather.battery = g_battery_val;
    weather.temperature = map_value<float>(g_temperature, -100.f, 100.f, 0, 65536);
    weather.pressure = map_value<float>(g_pressure, 0.f, 2000.f, 0, 65536);
    weather.humidity = g_humidity * 255;

    // checksum
    data[num_bytes - 1] = crc8(data, sizeof(weather_t));

    // send a message to the lora mesh-server
    if(m_rfm95.manager->sendtoWait(data, num_bytes, RH_BROADCAST_ADDRESS))
    {
        // the message has been reliably delivered to the next node.
        ret = true;
    }
    return ret;
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
        g_battery_val = static_cast<uint8_t>(map_value<float>(voltage, 3.3f, 4.2f, 0.f, 255.f));
        // Serial.printf("battery: %d%%\n", 100 * g_battery_val / 255);
        Serial.printf("raw_bat_measure: %d\n", raw_bat_measure);
    });
    g_timer[TIMER_BATTERY_MEASURE].set_periodic();
    g_timer[TIMER_BATTERY_MEASURE].expires_from_now(10.f);

    // lora config
    set_address(123);

    g_timer[TIMER_LORA_SEND].set_callback([](){ lora_send_status(); });
    g_timer[TIMER_LORA_SEND].set_periodic();
    g_timer[TIMER_LORA_SEND].expires_from_now(g_lora_send_interval);

    // BME setup
    if(!g_sensor.begin())
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while(true){ blink_status_led(); }
    }

    // sensor measuring
    g_timer[TIMER_SENSOR_MEASURE].set_callback([]()
    {
        g_sensor.takeForcedMeasurement();

        g_temperature = g_sensor.readTemperature();
        g_pressure = g_sensor.readPressure() / 100.f;
        g_humidity = g_sensor.readHumidity() / 100.f;

        Serial.print("Temperature = ");
        Serial.print(g_sensor.readTemperature());
        Serial.println(" *C");

        Serial.print("Pressure = ");

        Serial.print(g_sensor.readPressure() / 100.0F);
        Serial.println(" hPa");

        // Serial.print("Approx. Altitude = ");
        // Serial.print(g_sensor.readAltitude(SEALEVELPRESSURE_HPA));
        // Serial.println(" m");

        Serial.print("Humidity = ");
        Serial.print(g_sensor.readHumidity());
        Serial.println(" %");

        Serial.println();

      });
      g_timer[TIMER_SENSOR_MEASURE].set_periodic();
      g_timer[TIMER_SENSOR_MEASURE].expires_from_now(.4f);

    digitalWrite(13, LOW);
}

void loop()
{
    // time measurement
    uint32_t delta_time = millis() - g_last_time_stamp;
    g_last_time_stamp = millis();
    g_time_accum += delta_time;
    g_time_accum_params += delta_time;

    // poll Timer objects
    for(uint32_t i = 0; i < NUM_TIMERS; ++i){ g_timer[i].poll(); }
}

template <typename T> void process_input(T& the_device)
{
    uint16_t buf_idx = 0;

    while(the_device.available())
    {
        // get the new byte:
        char c = the_device.read();

        switch(c)
        {
            case '\r':
            case '\0':
                continue;

            case '\n':
                g_serial_buf[buf_idx] = '\0';
                buf_idx = 0;
                parse_line(g_serial_buf);
                break;

            default:
                g_serial_buf[buf_idx++] = c;
                break;
        }
    }
}

bool check_for_cmd(const char* the_str)
{
    if(strcmp(the_str, CMD_QUERY_ID) == 0)
    {
        char buf[32];
        sprintf(buf, "%s %s\n", the_str, DEVICE_ID);
        Serial.print(buf);
        return true;
    }
    return false;
}

void parse_line(char *the_line)
{
    const char* delim = " ";
    const size_t elem_count = 3;
    char *token = strtok(the_line, delim);
    int num_buf[elem_count];
    uint16_t i = 0;

    for(; token && (i < elem_count); i++)
    {
        if(check_for_cmd(token)){ break; }
        else{ num_buf[i] = atoi(token); }
        token = strtok(nullptr, delim);
    }
}
