
/*
  Solar-thermal controller: ESP32-S3 port of solarthermal.ino, for the Waveshare ESP32-S3-POE-ETH-8DI-8RO
  (8 changeover relays behind a TCA9554, 8 isolated inputs, W5500 Ethernet with PoE, PCF85063 RTC, isolated
  RS485) plus a Waveshare Modbus RTU Analog Input 8CH.  The control logic is the Arduino version's; only the
  I/O, the clock and the telemetry path are new.  The LCD and keys are gone; the stctl command replaces them.

  Creative Commons Licence
  Robert Bedichek
*/

#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <Wire.h>
#include <SPI.h>
#include <ETH.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <NetworkClientSecure.h>
#include <Update.h>
#include <Preferences.h>
#include <esp_task_wdt.h>
#include <esp_ota_ops.h>
#include <esp_sntp.h>

#include "secrets.h"  // ST_LAN_TOKEN and ST_INGEST_TOKEN, written by stctl; not in git

#define FIRMWARE_VERSION __DATE__ " " __TIME__
char reset_cause = 'P';
bool time_valid = false;

bool disable_electric_heat = false;

const int takagi_on_threshold_F = 120;  // If the tank falls below this temperature, turn the Takagi on
const int takagi_off_threshold_F = 125; // If the tank is this or above, turn the Takagi off and let the solar mass do all the heating

// The next four constants set the temperature that the tank must be to trigger start using and to trigger stop using solar heat and electric
// heat.  There difference between the high and low values creates a hysterisis that prevents cycling the solar heat and the electric heat
// too frequently.  It is ok for both the solar and the electric heat to be enabled at the same time.


const int tank_temp_threshold_spa_hex_high = 115;   // Solar spa heat enabled above this tank temperature
const int tank_temp_threshold_spa_hex_low = 110;    // Solar spa heat disabled below this tank temperature

const int tank_temp_threshold_spa_electric_heater_high = 118; // Electric spa heat disabled above this tank temperature
const int tank_temp_threshold_spa_electric_heater_low = 113;  // Electric spa heat disabled about this tank temperature


// The panels must be at least this much hotter than the tank to turn on the solar pump
const float tank_panel_difference_threshold_on_F = 20;

// When the panels drop to being just this much hotter than the tank, turn off the solar pump
// const float tank_panel_difference_threshold_off_F = -10; // Keep running pump until panels this much colder than tank

const unsigned long solar_pump_delay = 5 * 60 * 1000UL;         // Minimum of five minutes between solar-pump-on events

const unsigned long MAX_ULONG = 0xFFFFFFFF;

#define I2C_SDA_PIN      (42)
#define I2C_SCL_PIN      (41)
#define TCA9554_ADDR     (0x20)   // Relay expander: EXIO1..8 = relays 1..8, bit set = relay energized
#define PCF85063_ADDR    (0x51)
#define ETH_CS_PIN       (16)
#define ETH_INT_PIN      (12)
#define ETH_SCLK_PIN     (15)
#define ETH_MOSI_PIN     (13)
#define ETH_MISO_PIN     (14)
#define RS485_TX_PIN     (17)
#define RS485_RX_PIN     (18)
#define RS485_DE_PIN     (21)
#define SPA_HEAT_DIGITAL_IN_PIN (4)   // DI1: spa calling for heat, low = active
#define ROOF_VALVES_STATUS_PIN  (5)   // DI2: both roof valves in tank position, low = active

// The board's eight relays.  Relay numbers are provisional until the wiring is settled.
#define RELAY_SOLAR_PUMP   (1)
#define RELAY_RECIRC_PUMP  (2)
#define RELAY_TAKAGI       (3)
#define LV_RELAY1_SPA_ELEC_HEAT_ENABLE    (4)
#define LV_RELAY2_POOL_HEAT_REQUEST       (5)
#define LV_RELAY2_ROOF_VALVES_POWER       (6)
#define RELAY_ROOF_DIRECTION              (7)  // Energized = pool mode, released = tank (thermal mass) mode
#define LV_RELAY1_SPA_HEAT_EX_VALVE_OPEN  (8)  // Energized holds the spa valve open; released, it closes itself

class Onboard_Relays {
public:
  bool begin(void);
  void turnRelayOn(uint8_t relay)  { set(relay, true); }
  void turnRelayOff(uint8_t relay) { set(relay, false); }
  bool getState(uint8_t relay)     { return bits & (1 << (relay - 1)); }
  bool verify(void);
  uint8_t state(void)              { return bits; }
private:
  uint8_t bits = 0;
  void set(uint8_t relay, bool on);
};

Onboard_Relays *quad_lv_relay1;   // Both names are the same eight relays; the Arduino had two boards
Onboard_Relays *quad_lv_relay2;

unsigned long roof_valves_motion_start_time;

char cbuf[100];

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>
Scheduler ts;
#define SERIAL_BAUD (115200)

typedef enum {m_oper, m_poolheat, m_roof_valves, m_rpump, m_spump, m_takagi, m_spa_hex_valve, m_spa_elec, m_last} operating_mode_t;

void check_relays_callback(void);
void sync_rtc_callback(void);
void mark_firmware_good_callback(void);
void read_time_and_sensor_inputs_callback(void);
void print_status_to_serial_callback(void);
void monitor_spa_valve_callback(void);
void monitor_recirc_pump_callback(void);
void monitor_takagi_callback(void);
void monitor_diag_mode_callback(void);
void monitor_serial_console_callback(void);
void monitor_solar_pump_callback(void);
void monitor_spa_electric_heat_callback(void);
void monitor_roof_valves_callback(void);
enum temps_e {tank_e, left_panel_e, right_panel_e, spa_e, last_temp_e};

struct temperature_s {
  float temperature_F;
  bool temperature_valid;
  char channel;              // Analog module channel, 0-3
  int calibration_offset_F;
  int lower_bound_F;
  int upper_bound_F;
} temps [last_temp_e];

bool left_panel_sensor_failed = false;
bool right_panel_sensor_failed = false;
unsigned long first_high_temp_time;  // value of millis() when the average panel temperature was hot enough
float peak_tank_temperature_F;
unsigned long peak_tank_temperature_time;
                                     // to warrant turning on the solar pump

float ema_alpha = 0.2; // Smoothing factor (0 = slow response, 1 = no filtering)

Task read_time_and_sensor_inputs(500, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task print_status_to_serial(TASK_SECOND, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);

unsigned long last_valve_open_time; // millis() the last time a valve-open operation started

bool valve_verbose = false;

unsigned daily_valve_cycles;        // Number of valve-open operations per day

Task monitor_spa_valve(TASK_SECOND * 10, TASK_FOREVER, &monitor_spa_valve_callback, &ts, true);
Task monitor_roof_valves(TASK_SECOND, TASK_FOREVER, &monitor_roof_valves_callback, &ts, true);

bool pool_heating_season(void); // Returns true when today is a day we might heat the pool
bool pool_heating_inop = false; // Set true if heating the pool doesn't work


float average_panel_temperature_F;   // Average of leftmost and rightmost panels

unsigned long solar_pump_on_off_time;   // Set to millis()  when pump is turned on and off
unsigned long daily_milliseconds_of_solar_pump_on_time; // Number of milliseconds the solar pump has run today

bool solar_pump_on(void);
void turn_solar_pump_on(void);
void turn_solar_pump_off(const __FlashStringHelper *message);
Task monitor_solar_pump(TASK_SECOND * 60, TASK_FOREVER, &monitor_solar_pump_callback, &ts, true);
/*****************************************************************************************************/
bool spa_heater_relay_on(void);             // True if we have enabled the spa heater relay, off by default

unsigned long daily_milliseconds_of_spa_heater_on_time;
unsigned long spa_heater_relay_on_time;            // Set to millis() when heater is switched on
Task monitor_spa_electric_heat(TASK_SECOND * 60, TASK_FOREVER, &monitor_spa_electric_heat_callback, &ts, true);
/*****************************************************************************************************/
// Seconds since we made a change to the pool heat request relay
unsigned long pool_heat_request_relay_on_off_time;   

bool recirc_pump_on();

void turn_recirc_pump_on(void);
void turn_recirc_pump_off(const __FlashStringHelper *message);

Task monitor_recirc_pump(TASK_SECOND * 10, TASK_FOREVER, &monitor_recirc_pump_callback, &ts, true);
/*****************************************************************************************************/
#define TAKAGI_DELAY           (300)    // Minimum number of seconds between Tagaki on events
bool takagi_on(void);

void turn_takagi_on(void);
void turn_takagi_off(const __FlashStringHelper *message);

unsigned long daily_milliseconds_of_takagi_on_time;
unsigned long takagi_on_time;                 // Set to millis() the last time the Takagi was turned on

Task monitor_takagi(TASK_SECOND * 60, TASK_FOREVER, &monitor_takagi_callback, &ts, true);
/*****************************************************************************************************/

/*****************************************************************************************************/
operating_mode_t operating_mode;
unsigned long time_entering_diag_mode = 0;
#define DIAG_MODE_TIMEOUT (600000) // 10 minutes
Task monitor_diag_mode(TASK_SECOND * 60, TASK_FOREVER, &monitor_diag_mode_callback, &ts, false);
Task monitor_serial_console(100, TASK_FOREVER, &monitor_serial_console_callback, &ts, true);
Task check_relays(TASK_SECOND * 10, TASK_FOREVER, &check_relays_callback, &ts, true);
Task sync_rtc(TASK_SECOND, TASK_FOREVER, &sync_rtc_callback, &ts, true);
Task mark_firmware_good(TASK_SECOND * 60, TASK_ONCE, &mark_firmware_good_callback, &ts, false);
class Line_Buffer {
public:
  void init(size_t capacity, bool use_psram);
  void append(const char *line, size_t n);      // n includes the newline
  size_t length(void) { return len; }
  // Copies up to max bytes of whole lines from the front; returns the byte count
  size_t peek(char *dst, size_t max);
  void consume(size_t n);
  void copy_tail(String &dst, size_t max);
private:
  char *buf = nullptr;
  size_t cap = 0;
  size_t len = 0;
};

void Line_Buffer::init(size_t capacity, bool use_psram)
{
  buf = (char *)(use_psram && psramFound() ? ps_malloc(capacity) : malloc(capacity));
  cap = buf ? capacity : 0;
}

void Line_Buffer::append(const char *line, size_t n)
{
  if (n > cap) {
    return;
  }
  if (len + n > cap) {    // Drop the oldest whole lines to make room
    size_t drop = len + n - cap;
    while (drop < len && buf[drop - 1] != '\n') {
      drop++;
    }
    consume(drop);
  }
  memcpy(buf + len, line, n);
  len += n;
}

size_t Line_Buffer::peek(char *dst, size_t max)
{
  size_t n = len < max ? len : max;
  while (n > 0 && buf[n - 1] != '\n') {
    n--;
  }
  memcpy(dst, buf, n);
  return n;
}

void Line_Buffer::consume(size_t n)
{
  if (n >= len) {
    len = 0;
  } else {
    memmove(buf, buf + n, len - n);
    len -= n;
  }
}

void Line_Buffer::copy_tail(String &dst, size_t max)
{
  size_t start = len > max ? len - max : 0;
  while (start > 0 && start < len && buf[start - 1] != '\n') {
    start++;
  }
  dst.concat(buf + start, len - start);
}

Line_Buffer recent_lines;     // The last 64 KB, for /log
Line_Buffer upload_queue;     // Not yet accepted by bedichek.org; 1 MB in PSRAM is weeks of telemetry
bool upload_enabled = false;  // Saved in flash; off on the bench so test lines stay out of the pool log ("upload on|off")
SemaphoreHandle_t lines_mutex;

class Tee_Output : public Print {
public:
  size_t write(uint8_t c) override;
  size_t write(const uint8_t *b, size_t n) override { for (size_t i = 0; i < n; i++) write(b[i]); return n; }
private:
  char line[200];
  size_t n = 0;
};

size_t Tee_Output::write(uint8_t c)
{
  Serial.write(c);
  if (c == '\r') {
    return 1;
  }
  if (n < sizeof(line) - 1) {
    line[n++] = c;
  }
  if (c == '\n') {
    if (line[n - 1] != '\n') {
      line[n++] = '\n';
    }
    xSemaphoreTake(lines_mutex, portMAX_DELAY);
    recent_lines.append(line, n);
    if (upload_enabled) {
      upload_queue.append(line, n);
    }
    xSemaphoreGive(lines_mutex);
    n = 0;
  }
  return 1;
}

Tee_Output out;

#define ANALOG_MODBUS_ADDR  (1)
#define ANALOG_BAUD         (9600)
#define ANALOG_CHANNELS_USED (4)
HardwareSerial RS485(1);


time_t now(void)          { return time(nullptr); }
static struct tm tm_of(time_t t) { struct tm tm; localtime_r(&t, &tm); return tm; }
int hour(time_t t)        { return tm_of(t).tm_hour; }
int minute(time_t t)      { return tm_of(t).tm_min; }
int second(time_t t)      { return tm_of(t).tm_sec; }
int day(time_t t)         { return tm_of(t).tm_mday; }
int month(time_t t)       { return tm_of(t).tm_mon + 1; }
int year(time_t t)        { return tm_of(t).tm_year + 1900; }

#define TIMEZONE "PST8PDT,M3.2.0,M11.1.0"

#define LOW_MEMORY_LIMIT (20000)
uint32_t lowest_memory = UINT32_MAX;
void check_free_memory(const __FlashStringHelper *caller)
{
  static bool warned = false;
  uint32_t fm = ESP.getFreeHeap();
  if (fm < LOW_MEMORY_LIMIT && !warned) {
    out.print(F("# alert low memory: "));
    out.println(fm);
    warned = true;
  }
  if (fm < lowest_memory) {
    lowest_memory = fm;
  }
}

void record_error(const __FlashStringHelper *warning_message)
{
  out.print(F("# alert: "));
  out.println(warning_message);
}

bool roof_valves_status_in_tank_mode() { return digitalRead(ROOF_VALVES_STATUS_PIN) == LOW; }
bool spa_calling_for_heat(void)        { return digitalRead(SPA_HEAT_DIGITAL_IN_PIN) == LOW; }
bool takagi_on()                       { return quad_lv_relay1 != nullptr && quad_lv_relay1->getState(RELAY_TAKAGI); }
bool solar_pump_on()                   { return quad_lv_relay1 != nullptr && quad_lv_relay1->getState(RELAY_SOLAR_PUMP); }
bool recirc_pump_on(void)              { return quad_lv_relay1 != nullptr && quad_lv_relay1->getState(RELAY_RECIRC_PUMP); }
static void relay_write(uint8_t relay, bool on)
{
  if (quad_lv_relay1 != nullptr) {
    if (on) quad_lv_relay1->turnRelayOn(relay); else quad_lv_relay1->turnRelayOff(relay);
  }
}

// The roof valves' direction is one relay here (the Arduino had one per pole of the reversal).
bool roof_valves_set_to_tank_mode()
{
  return quad_lv_relay2 != nullptr && !quad_lv_relay2->getState(RELAY_ROOF_DIRECTION);
}
bool roof_valves_set_to_pool_mode(void) { return !roof_valves_set_to_tank_mode(); }

void turn_roof_valves_power_on(void)
{
  roof_valves_motion_start_time = millis();
  relay_write(LV_RELAY2_ROOF_VALVES_POWER, true);
}
void turn_roof_valves_power_off(void) { relay_write(LV_RELAY2_ROOF_VALVES_POWER, false); }
bool roof_valves_power_is_on(void)    { return quad_lv_relay2 != nullptr && quad_lv_relay2->getState(LV_RELAY2_ROOF_VALVES_POWER); }

void turn_roof_valves_to_tank_mode(const __FlashStringHelper *message)
{
  out.print(message);
  turn_roof_valves_power_on();
  relay_write(RELAY_ROOF_DIRECTION, false);
}
void turn_roof_valves_to_pool_mode(const __FlashStringHelper *message)
{
  out.print(message);
  turn_roof_valves_power_on();
  relay_write(RELAY_ROOF_DIRECTION, true);
}

const unsigned long max_roof_valves_power_on_time = 60 * 1000UL;

// Returns true when today is a day we might heat the pool
bool pool_heating_season(void)
{
  int m = month(now());
  return (m >= 5 && m <= 9);
}

static uint16_t modbus_crc(const uint8_t *b, size_t n)
{
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < n; i++) {
    crc ^= b[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }
  return crc;
}

// Sends a request (the CRC is appended here) and reads a reply of reply_len bytes.  Returns true
// if the reply has the right length, address, function and CRC.
static bool modbus_transaction(uint8_t *req, size_t req_len, uint8_t *reply, size_t reply_len)
{
  uint16_t crc = modbus_crc(req, req_len);
  req[req_len] = crc & 0xFF;
  req[req_len + 1] = crc >> 8;
  while (RS485.available()) {
    RS485.read();
  }
  RS485.write(req, req_len + 2);
  RS485.flush();
  size_t got = 0;
  unsigned long start = millis();
  while (got < reply_len && millis() - start < 500) {   // A settings write can take the module longer than a read
    if (RS485.available()) {
      reply[got++] = RS485.read();
    }
  }
  if (got != reply_len || reply[0] != req[0] || reply[1] != req[1]) {
    static unsigned reports;
    if (reports++ < 10) {
      out.printf("# modbus fn %02X reg %02X%02X: got %u of %u bytes, first %02X %02X %02X\n", req[1], req[2], req[3],
                 (unsigned)got, (unsigned)reply_len, got > 0 ? reply[0] : 0, got > 1 ? reply[1] : 0, got > 2 ? reply[2] : 0);
    }
    return false;
  }
  return modbus_crc(reply, reply_len - 2) == (reply[reply_len - 2] | (reply[reply_len - 1] << 8));
}


uint16_t analog_last_mv[ANALOG_CHANNELS_USED];
bool analog_mode_set = false;
unsigned analog_failures;

bool analog_read_millivolts(uint16_t mv[ANALOG_CHANNELS_USED])
{
  uint8_t req[8] = {ANALOG_MODBUS_ADDR, 0x04, 0x00, 0x00, 0x00, ANALOG_CHANNELS_USED};
  uint8_t reply[5 + 2 * ANALOG_CHANNELS_USED];
  if (!modbus_transaction(req, 6, reply, sizeof(reply))) {
    return false;
  }
  for (int i = 0; i < ANALOG_CHANNELS_USED; i++) {
    mv[i] = (reply[3 + 2 * i] << 8) | reply[4 + 2 * i];
    if (mv[i] > 0) {
      // Measured against a DM3058, 0.1-4.5 V, 2026-11-02: reading = 0.9963 x true - 3.3 mV on all four channels
      mv[i] = (uint16_t)((mv[i] + 3.3) / 0.9963 + 0.5);
    }
  }
  return true;
}

bool analog_set_voltage_mode(void)
{
  uint8_t req[8] = {ANALOG_MODBUS_ADDR, 0x03, 0x10, 0x00, 0x00, ANALOG_CHANNELS_USED};
  uint8_t reply[5 + 2 * ANALOG_CHANNELS_USED];
  if (!modbus_transaction(req, 6, reply, sizeof(reply))) {
    return false;
  }
  for (int i = 0; i < ANALOG_CHANNELS_USED; i++) {
    uint16_t type = (reply[3 + 2 * i] << 8) | reply[4 + 2 * i];
    if (type != 0) {
      uint8_t wreq[8] = {ANALOG_MODBUS_ADDR, 0x06, 0x10, (uint8_t)i, 0x00, 0x00};
      uint8_t wreply[8];
      if (!modbus_transaction(wreq, 6, wreply, sizeof(wreply))) {
        out.print(F("# analog channel "));
        out.print(i + 1);
        out.println(F(" did not confirm the 0-5V setting"));
        return false;
      }
      out.print(F("# analog channel "));
      out.print(i + 1);
      out.println(F(" set to 0-5V mode"));
    }
  }
  return true;
}

static void insertionSort(int arr[], int n) {
    for (int i = 1; i < n; i++) {
        int key = arr[i];
        int j   = i - 1;
        while (j >= 0 && arr[j] > key) {
            arr[j + 1] = arr[j];
            j--;
        }
        arr[j + 1] = key;
    }
}

// Twice a second: one reading of each channel, the last 16 trimmed and averaged, then the Arduino
// version's EMA and range checks.
void read_time_and_sensor_inputs_callback(void)
{
#define NUM_SAMPLES (16)
#define TRIM_COUNT  (4)
  static int rings[last_temp_e][NUM_SAMPLES];
  static int samples;
  static bool first_time = true;

  static unsigned long last_mode_try;
  if (!analog_mode_set && (last_mode_try == 0 || millis() - last_mode_try > 60 * 1000UL)) {
    last_mode_try = millis();         // Not every half second: the module stores this setting in its flash
    analog_mode_set = analog_set_voltage_mode();
  }
  uint16_t mv[ANALOG_CHANNELS_USED];
  if (!analog_mode_set || !analog_read_millivolts(mv)) {
    if (++analog_failures == 20) {
      out.println(F("# alert analog module not answering"));
      for (int i = 0; i < last_temp_e; i++) {
        temps[i].temperature_valid = false;
      }
      samples = 0;
      first_time = true;
    }
    return;
  }
  if (analog_failures >= 20) {
    out.println(F("# alert analog module answering again"));
  }
  analog_failures = 0;
  memcpy(analog_last_mv, mv, sizeof(analog_last_mv));

  int slot = samples % NUM_SAMPLES;
  for (int i = 0; i < last_temp_e; i++) {
    rings[i][slot] = mv[(int)temps[i].channel];
  }
  samples++;
  if (samples < NUM_SAMPLES) {
    return;
  }
  samples = NUM_SAMPLES + slot + 1;

  for (int i = 0; i < last_temp_e; i++) {
    struct temperature_s *t = &temps[i];
    int sorted[NUM_SAMPLES];
    memcpy(sorted, rings[i], sizeof(sorted));
    insertionSort(sorted, NUM_SAMPLES);
    float millivolts = 0;
    for (int k = TRIM_COUNT; k < NUM_SAMPLES - TRIM_COUNT; k++) {
      millivolts += sorted[k];
    }
    millivolts /= NUM_SAMPLES - 2 * TRIM_COUNT;
    float temp_F = (millivolts - 500.0) / 10.0 * 9.0 / 5.0 + 32.0 + t->calibration_offset_F;  // TMP36
    if (temp_F >= t->lower_bound_F && temp_F <= t->upper_bound_F) {
      t->temperature_valid = true;
      t->temperature_F = first_time ? temp_F : ema_alpha * temp_F + (1 - ema_alpha) * t->temperature_F;
    } else {
      t->temperature_valid = false;
    }
  }
  first_time = false;

  if (temps[left_panel_e].temperature_valid && !left_panel_sensor_failed) {
    if (temps[right_panel_e].temperature_valid && !right_panel_sensor_failed) {
      average_panel_temperature_F = (temps[left_panel_e].temperature_F + temps[right_panel_e].temperature_F) / 2.0;
    } else {
      average_panel_temperature_F = temps[left_panel_e].temperature_F;
      if (!right_panel_sensor_failed) {
        record_error(F("right panel sensor failed"));
        right_panel_sensor_failed = true;
      }
    }
  } else {
    if (temps[right_panel_e].temperature_valid && !right_panel_sensor_failed) {
      average_panel_temperature_F = temps[right_panel_e].temperature_F;
      if (!left_panel_sensor_failed) {
        record_error(F("left panel sensor failed"));
        left_panel_sensor_failed = true;
      }
    } else {
      // Both temperature sensors have failed, print error and leave panel_temperature_F as it was´
      if (!right_panel_sensor_failed || !left_panel_sensor_failed) {
        record_error(F("both panel sensors failed"));
        right_panel_sensor_failed = true;
        left_panel_sensor_failed = true;
      }
    }
  }
}

const char *operating_mode_to_string(operating_mode_t operating_mode)
{
  const char *s[] = {"Oper", "Pool", "Roof", "D-RP", "D-SP", "D-TK", "D-HX", "D-EL"};
  if (operating_mode < m_last) {
    return s[operating_mode];
  }
  return "ERR";
}

void monitor_diag_mode_callback(void)
{
  check_free_memory(F("monitor_diag_mode"));
  if ((millis() - time_entering_diag_mode) > DIAG_MODE_TIMEOUT) {
    operating_mode = m_oper;
    out.println(F("# diag mode timed out, mode now Oper"));
    monitor_diag_mode.disable();
  }
}

void plus_key(void)
{
    switch (operating_mode) {
      case m_oper:
        out.println(F("# no effect in Oper mode"));
        break;

      case m_poolheat:
        turn_pool_heat_request_relay_on(F("# pool heat req="));
        out.println(pool_heat_request_relay_on());
        break;

      case m_roof_valves:
        turn_roof_valves_to_pool_mode(F("# roof valves status="));
        out.println(roof_valves_status_in_tank_mode() ? F("tank mode") : F("pool mode"));
        break;

      case m_rpump:
        turn_recirc_pump_on();
        break;

      case m_spump:
        turn_solar_pump_on();
        break;

      case m_takagi:
        turn_takagi_on();
        break;

      case m_spa_hex_valve:    
        open_spa_heat_exchanger_valve(F("process_pressed_keys_.."));
        if (!spa_calling_for_heat()) {
          out.println(F("# WARNING: spa not calling for heat"));
        }
        break;

      case m_spa_elec:
        turn_spa_heater_relay_on(F("# spa heater relay="));
        out.println(spa_heater_relay_on());
        break; 
    }
}

void minus_key(void)
{
    switch (operating_mode) {
      case m_oper:
        out.println(F("# no effect in Oper mode"));
        break;

      case m_poolheat:
        turn_pool_heat_request_relay_off(F("# pool heat req="));
        out.println(pool_heat_request_relay_on());
        break;

      case m_roof_valves:
        turn_roof_valves_to_tank_mode(F("# roof valve status="));
        out.println(roof_valves_status_in_tank_mode() ? F("tank mode") : F("pool mode"));
        break;

      case m_rpump:
        turn_recirc_pump_off(F("# recirc pump="));
        out.println(recirc_pump_on());
        break;

      case m_spump:
        turn_solar_pump_off(F("# solar pump="));
        out.println(solar_pump_on());
        break;

      case m_takagi:
        turn_takagi_off(F("# Takagi="));
        out.println(takagi_on());
        break;

      case m_spa_hex_valve:          
        close_spa_heat_exchanger_valve(F("process_pressed_keys_.."));
        break;

      case m_spa_elec:
        turn_spa_heater_relay_off(F("# spa heater relay="));
        out.println(spa_heater_relay_on());
        break; 
    }
}

void turn_solar_pump_on()
{
  solar_pump_on_off_time = millis();
  relay_write(RELAY_SOLAR_PUMP, true); // Turn on solar pump
}

void turn_solar_pump_off(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.print(F("# turn_solar_pump_off(): "));
    out.println(message);
  }
  if (solar_pump_on()) {
    unsigned long elapsed = millis() - solar_pump_on_off_time;
    if (elapsed < (MAX_ULONG / 2)) {
      daily_milliseconds_of_solar_pump_on_time += elapsed;
    }
  }
  solar_pump_on_off_time = millis();
  first_high_temp_time = 0UL;
  relay_write(RELAY_SOLAR_PUMP, false); // Turn off solar pump;
}

// If the following are true, turn on the solar pump:
// 1. The solar pump is off
// 2. the panel temperature and the tank temperature is valid
// 3. the panel temperature is enough higher than the tank for it to be worthwhile to turn on the solar pump
// 4. we haven't turned the solar pump on too recently (to avoid cycling the system too frequently)
//
// If the solar pump is on and the solar panel temperature has fallen enough relative to the tank, then turn off the solar pump
// Also, if the solar pump is on and it has gotten so late in the evening that the panels couldn't possibly be warm, turn off the solar pump
void monitor_solar_pump_callback(void)
{
  check_free_memory(F("monitor_solar"));
  if (!time_valid) {
    return;   // These decisions depend on the hour; wait for the RTC or NTP
  }
  if (operating_mode != m_oper) {
    return;
  }

  int h = hour(now());
  int m = month(now());
  bool winter_month = m <= 2 || m >= 11;
  bool spring_or_fall_month = m == 3 || m == 10;

  if (temps[left_panel_e].temperature_valid || temps[right_panel_e].temperature_valid) {
    static unsigned spew_counter = 0;
    // If it has been more than five minutes and the panels are super hot, give an alert
    if (average_panel_temperature_F > 180) {
      if ((millis() - solar_pump_on_off_time) > 300 * 1000UL) {
        if (pool_heat_request_relay_on()) {
          // Pool controller not sending water to panels, despite our request.
          // Make the pool controller as not working and switch back to heating 
          // the termal mass 
          pool_heating_inop = true;
          turn_pool_heat_request_relay_off(F("# heating pool water failed"));      
          turn_roof_valves_to_tank_mode(F("# go back to heating the tank\n"));
          if (spew_counter < 20) {
            out.println(F("# alert panel overtemp with pool request"));
            spew_counter++;
          }
        }
      }
    } else {
      spew_counter = 0;
    }

    if (temps[tank_e].temperature_valid) {
      float tank_F = temps[tank_e].temperature_F;
      // If the solar pump is off and the tank is not super hot and 
      // the average solar panel temperature is more than the tank temperature plus a threshold
      // and both of the measured panels is warmer than the tank temperature and it has been
      // five minutes since these conditions were met and it hasn't been to short a time since
      // the last time we turned the solar pump on or off, then turn it on.
      
      if (!solar_pump_on() && tank_F < 165 &&
          average_panel_temperature_F > (tank_F + tank_panel_difference_threshold_on_F) &&
          (!temps[left_panel_e].temperature_valid || temps[left_panel_e].temperature_F > tank_F) &&
          (!temps[right_panel_e].temperature_valid || temps[right_panel_e].temperature_F > tank_F)) {
        if (first_high_temp_time == 0) {
          first_high_temp_time = millis();
        } else if ((millis() - first_high_temp_time) > 5 * 60 * 1000UL) {
          // It has been five minutes since the average panel temperature reached the threshold of it being
          // worthwhile to turn on the solar pump and the roof valves are in tank mode, then turn on the solar pump
          
          if (roof_valves_status_in_tank_mode() && ((millis() - solar_pump_on_off_time) > solar_pump_delay)) {
            turn_solar_pump_on();
            peak_tank_temperature_F = temps[tank_e].temperature_F;
            peak_tank_temperature_time = millis();
          }
        }
      }

      if (solar_pump_on()) {
        // Record the peak tank temperature
        if (temps[tank_e].temperature_F > peak_tank_temperature_F) {
          peak_tank_temperature_F = temps[tank_e].temperature_F;
          peak_tank_temperature_time = millis();
        }
        // If the tank is too hot in absolute valve or the tank is too warm in comparison to the panels,
        // then turn the solar pump off.
        if (temps[tank_e].temperature_F > 170) {
          turn_solar_pump_off(F("alert: tank too hot"));
        } else 
          // Turn off the solar pump if the following conditions are met:
          // 1. The average panel temperature is less than the tank temperature
          // 2. The tank has cooled by a degree farenheight from its peak temperature
          // 3. It has been at least 15 minutes since the tank reached its peak temperature
             if (average_panel_temperature_F < temps[tank_e].temperature_F &&
            temps[tank_e].temperature_F < (peak_tank_temperature_F - 1.0) && (millis() - peak_tank_temperature_time) > 15 * 60 * 1000UL){
            turn_solar_pump_off(F("panels colder than tank and tank cooling"));
        } else if (h >= 20) {
          turn_solar_pump_off(F("alert: solar pump still on at 8PM"));
        } else if (spring_or_fall_month && h >= 18) {
          turn_solar_pump_off(F("alert: solar pump still on at 6PM"));
        } else if (winter_month && h >= 17) {
          turn_solar_pump_off(F("alert: solar pump still on at 5PM"));
        } else if (pool_heating_season() && !pool_heating_inop && temps[tank_e].temperature_F > 140) {
          if ((millis() - solar_pump_on_off_time) > 60 * 60 * 1000UL) {
            // During summer months, turn off the solar pump  if the thermal
            // mass is warm enough to last through the night and we have run the solar pump for at
            // least an hour.  This is to allow the pool to be heated.  But only do this if we 
            // haven't given up on the pool heat.
            turn_solar_pump_off(nullptr);
          }
        }
      }
    } else {
      // The panel temperature is valid, but the tank temperature is not.  Just turn on the pump if the panels are above
      // 170F and turn it off at 1700.
      if (!solar_pump_on() && average_panel_temperature_F > 170 && roof_valves_status_in_tank_mode() && 
              (millis() - solar_pump_on_off_time) > solar_pump_delay) {
        turn_solar_pump_on();
      } else if (solar_pump_on() && h >= 17 && (millis() - solar_pump_on_off_time) > 60 * 60 * 1000UL) {
        turn_solar_pump_off(nullptr);
      }
    }
  } else {
    // We can't use the panel temperature, so just turn on the solar pump in the late morning and turn it off in the late
    // afternoon
    if (!solar_pump_on() && h >= 11 && roof_valves_status_in_tank_mode() && (millis() - solar_pump_on_off_time) > solar_pump_delay) {
      turn_solar_pump_on();
    } else if (solar_pump_on() && h >= 17 && (millis() - solar_pump_on_off_time) > 60 * 60 * 1000UL) {
      // If we don't know the panel temperatures and we have run the solar pump for at least an hour and it is after 5PM,
      // turn off the solar pump.
      turn_solar_pump_off(nullptr);
    }
  }
}

bool spa_heater_relay_on(void)
{
  bool v = false;
  if (quad_lv_relay1 != nullptr) {
    v = quad_lv_relay1->getState(LV_RELAY1_SPA_ELEC_HEAT_ENABLE);
  }
  return v;
}

void turn_spa_heater_relay_on(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.print(message);
  }
  spa_heater_relay_on_time = millis();
  if (quad_lv_relay1 != nullptr) {    
    quad_lv_relay1->turnRelayOn(LV_RELAY1_SPA_ELEC_HEAT_ENABLE);
  }
}

void turn_spa_heater_relay_off(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.print(message);
  }
  unsigned long elapsed = millis() - spa_heater_relay_on_time;
  if (elapsed < (MAX_ULONG / 2)) {
    daily_milliseconds_of_spa_heater_on_time += elapsed;
  }
  if (quad_lv_relay1 != nullptr) {
    quad_lv_relay1->turnRelayOff(LV_RELAY1_SPA_ELEC_HEAT_ENABLE);
  }
}

bool pool_heat_request_relay_on(void)
{
  bool v = false;
  if (quad_lv_relay2 != nullptr) {
    v = quad_lv_relay2->getState(LV_RELAY2_POOL_HEAT_REQUEST);
  }
  return v;
}

void turn_pool_heat_request_relay_on(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.print(message);
  }
  pool_heat_request_relay_on_off_time = millis();
  if (quad_lv_relay2 != nullptr) {
    quad_lv_relay2->turnRelayOn(LV_RELAY2_POOL_HEAT_REQUEST);
  }
}

void turn_pool_heat_request_relay_off(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.print(message);
  }
  pool_heat_request_relay_on_off_time = millis();
  if (quad_lv_relay2 != nullptr) {
    // daily_seconds_of_pool_heat_request_on_time += (millis() - pool_heat_request_relay_on_time) / 1000;
    quad_lv_relay2->turnRelayOff(LV_RELAY2_POOL_HEAT_REQUEST);
  }
}


// This contains the normal basic operational logic of this controller, comparing temperatures
// and deciding which pumps to turn on, how to set the spa heat exchanger valve, etc.

void monitor_spa_electric_heat_callback(void)
{
  check_free_memory(F("monitor_spa"));
  if (temps[tank_e].temperature_valid) {
    if (operating_mode == m_oper) {

      // If the tank is not warm enough to heat the spa, enable the relay we inserted in the spa controller 
      // that allows the spa to heat itself with its own electric heater.  If we are currently heating with
      // solar (the valve is open), then the threshold for turning on the spa electric heater is 110F.  Below
      // that temperature, the valve will close.  If the spa valve is already closed, then turn off the electric
      // heater if the tank is less than  113F (tank_temp_threshold_spa_hex_high).
    
      if (spa_heater_relay_on()) {
        // If the tank is warm enough to heat the spa and we haven't turned the
        // relay on within the hour or if the spa is not calling for heat, 
        // disable electric spa heat
        unsigned long milliseconds_spa_heater_relay_on = millis() - spa_heater_relay_on_time;
        if (temps[tank_e].temperature_F >= tank_temp_threshold_spa_electric_heater_high && (milliseconds_spa_heater_relay_on > (3600 * 1000UL))) {
          turn_spa_heater_relay_off(nullptr);
        }
        if (disable_electric_heat) {
          turn_spa_heater_relay_off(nullptr);
        }
      } else {
        
        unsigned long milliseconds_valve_open = millis() - last_valve_open_time;
        // If the tank is too cool or if the spa valve has been open for more than 90 (daytime) to 120 minutes (wee hours), enable the spa's electric heater
        // for at least an hour
        unsigned h = hour(now());
        // Be more patient heating the spa with solar during hours when it is unlikely anyone will 
        // be using the spa
        unsigned long minutes_limit = (1 <= h && h <= 6) ? 120 : 90;  // 90 minutes daytime, 2u hours in the middle of the night.
        if (!disable_electric_heat && temps[tank_e].temperature_F <= tank_temp_threshold_spa_electric_heater_low ||
           (spa_heat_ex_valve_open() && (milliseconds_valve_open > (minutes_limit * 60 * 1000UL)))) {
            if (valve_verbose) {
              snprintf(cbuf, sizeof(cbuf), "# spaH=%d valve_open=%d time=%u",
                spa_heater_relay_on(), spa_heat_ex_valve_open(), (unsigned)(milliseconds_valve_open / 1000UL));
              out.println(cbuf);
           }
          turn_spa_heater_relay_on(nullptr);
        }
      }
    }
  } else {
    if (!disable_electric_heat && !spa_heater_relay_on()) {
      // If the tank temperature is not valid, play it safe and turn on the electric heater
      turn_spa_heater_relay_on(F("# alert tank temp invalid, spa heater on, t="));
      out.println((int)temps[tank_e].temperature_F);
    }
  }
  check_free_memory(F("monitor_spa_ exit"));
}

const bool spa_status_debug = false;

bool spa_heat_ex_valve_open(void)
{
  bool v = false;
  if (quad_lv_relay1 != nullptr) {
    v = quad_lv_relay1->getState(LV_RELAY1_SPA_HEAT_EX_VALVE_OPEN);
  }
  if (spa_status_debug) {
    out.print(F("# spa open="));
    out.println(v);
  }
  return v;
}

// This intiates the valve opening process by applying the correct polarity of power to the valve motor
// and by enabling the task that monitors the opening process.

unsigned long delayed_close_spa_heat_exchanger_valve_time;
const __FlashStringHelper *delayed_caller;

void open_spa_heat_exchanger_valve(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.println(message);
  }
  if (quad_lv_relay1 != nullptr) {
    quad_lv_relay1->turnRelayOn(LV_RELAY1_SPA_HEAT_EX_VALVE_OPEN);
  }
  delayed_close_spa_heat_exchanger_valve_time = 0UL;
  last_valve_open_time = millis();
}


// This intiates the valve closing ing process by applying the correct polarity of power to the valve motor
// and by enabling the task that monitors the opening process.

void close_spa_heat_exchanger_valve(const __FlashStringHelper *caller)
{
  if (quad_lv_relay1 != nullptr) {
    quad_lv_relay1->turnRelayOff(LV_RELAY1_SPA_HEAT_EX_VALVE_OPEN);
  }
  delayed_close_spa_heat_exchanger_valve_time = 0ULL;
}


// This will cause the valve to close at some point in the future (like in 5 minutes)

void delayed_close_spa_heat_exchanger_valve(const __FlashStringHelper *caller)
{
  if (delayed_close_spa_heat_exchanger_valve_time == 0UL) {
    delayed_caller = caller;
    delayed_close_spa_heat_exchanger_valve_time = millis();
  }
}


// This is called every ten seconds and decides whether to open the valve allowing the spa
// to be heated with solar, to close the valve, or to leave it alone.
// The first bit of code handles the unlikely case where the valve status does not
// show the valve to be fully open or fully closed and it attempts to close it.

void monitor_spa_valve_callback(void)
{ 
  check_free_memory(F("monitor_spa_valve.."));
  
  if (operating_mode != m_spa_hex_valve) {
    if (delayed_close_spa_heat_exchanger_valve_time != 0) {
      if ((millis() - delayed_close_spa_heat_exchanger_valve_time) > (1 * 60 * 1000UL)) {
        close_spa_heat_exchanger_valve(delayed_caller);
      }
    }
    if (temps[tank_e].temperature_valid) {
  
      // If the following are true, open the spa valve so that the spa water is warmed from solar
      // 1. the spa is calling for heat
      // 2. The tank temperature reading is valid and the tank is at 113 F (tank_temp_threshold_spa_hex_high) or above
      // 3. the spa valve is closed
      // 4. We are in normal operation mode
      // It is ok if the electric heater is on too

      
      if (spa_calling_for_heat()) {
        if (temps[tank_e].temperature_F >= tank_temp_threshold_spa_hex_high &&
            !spa_heat_ex_valve_open() &&
             operating_mode == m_oper) {
          open_spa_heat_exchanger_valve(nullptr);
        } else if (spa_heat_ex_valve_open() && (temps[tank_e].temperature_F <= tank_temp_threshold_spa_hex_low)) {
  
          // Close the valve if the tank is too cool to be effective at heating the spa even though
          // the spa is calling for heat.
          delayed_close_spa_heat_exchanger_valve(F("low tank temperature"));
        }
      } else {
  
        // If the following are true, close the heat exchanger valve
        // 1. spa is not calling for heat
        // 2. the spa valve is open (or, at least, not closed
        // 3. the valve is not in motion
  
        if (spa_heat_ex_valve_open()) {
          delayed_close_spa_heat_exchanger_valve(F("no call for heat"));
        }
      }
    } else {
      // If the tank temperature isn't valid, play it safe by making sure the spa valve is closed.
      // The code that controls the spa's electric heater will turn it on in this case.
      if (spa_heat_ex_valve_open()) {
        close_spa_heat_exchanger_valve(F("tank temperature not valid"));
      }
    }
  }
  check_free_memory(F("monitor_spa_valve.. exit"));
}

void print_periodic_header_and_summary_data(void)
{
  check_free_memory(F("print_periodic.."));
  static bool daily_stats_reset = false;
  snprintf(cbuf, sizeof(cbuf), "# %4u-%02u-%02u %02u:%02u:%02u ", 
        year(now()),
        month(now()), 
        day(now()), 
        hour(now()),
        minute(now()), 
        second(now()));
  
  out.print(cbuf);
  unsigned long sp_on = 0;
  if (solar_pump_on()) {
    sp_on = millis() - solar_pump_on_off_time;
  }
  unsigned long eh_on = 0;
  if (spa_heater_relay_on()) {
    eh_on = millis() - spa_heater_relay_on_time;
  }
  unsigned long t_on = 0;
  if (takagi_on()) {
    t_on = millis() - takagi_on_time;
  }

  snprintf(cbuf, sizeof(cbuf), " solarthermal %u %u %u %u ",
    daily_valve_cycles, 
    (unsigned)(daily_milliseconds_of_solar_pump_on_time + sp_on) / 1000UL,
    (unsigned)(daily_milliseconds_of_spa_heater_on_time + eh_on) / 1000UL, 
    (unsigned)(daily_milliseconds_of_takagi_on_time + t_on) / 1000UL);
  
  out.print(cbuf);
  out.println(F("built: " __DATE__ " " __TIME__));

  if (hour(now()) == 0) {
    if (!daily_stats_reset) {
      daily_valve_cycles = 0;
      daily_milliseconds_of_solar_pump_on_time = 0;
      daily_milliseconds_of_spa_heater_on_time = 0;
      daily_milliseconds_of_takagi_on_time = 0;
      daily_stats_reset = true;
    } else {
      daily_stats_reset = false;
    }
  }
  out.println(F("# Date     Time     Tank LPan RPan Aveg SpaT SpaH Spmp Rpmp Taka Call Open Roof Pool Mode"));
}
// Called periodically.  More frequently after starting, then slows.
//  Sends relevant telemetry back over the USB-serial link.
void print_status_to_serial_callback(void)
{
  check_free_memory(F("print_status_to.."));
  if (!time_valid) {
    return;   // Lines without a real date would land in the wrong place on the graphs
  }
  static char line_counter = 0;
  static int records_skipped;
  
  if (line_counter == 0) {
    print_periodic_header_and_summary_data();
    line_counter = 30;
  }

  static int last_tank_F;
  static int last_avg_panel_F;
  static bool last_spa_heat;
  static bool last_solar_pump;
  static bool last_recirc;
  static bool last_spa_call;
  static bool last_spa_open;
  static bool last_roof_valves_to_pool;
  static bool last_pool_heat;
  static operating_mode_t last_operating_mode;
  
  // Only print a line if the tank's temperature has shifted by more than on degree, the average panel temp by more than 2
  // or any of the booleans.

  if (abs(last_tank_F - (int)temps[tank_e].temperature_F) > 1 ||
      abs(last_avg_panel_F - (int)average_panel_temperature_F) > 2 ||
      last_spa_heat != spa_heater_relay_on() ||
      last_solar_pump != solar_pump_on() ||
      last_recirc != recirc_pump_on() ||
      last_spa_call != spa_calling_for_heat() ||
      last_spa_open != spa_heat_ex_valve_open() ||
      last_roof_valves_to_pool != !roof_valves_status_in_tank_mode() ||
      last_pool_heat != pool_heat_request_relay_on() ||
      last_operating_mode != operating_mode ||
      records_skipped++ > 800) {

    last_tank_F = (int)temps[tank_e].temperature_F;
    last_avg_panel_F = (int)average_panel_temperature_F;
    last_spa_heat = spa_heater_relay_on();
    last_solar_pump = solar_pump_on();
    last_recirc = recirc_pump_on();
    last_spa_call = spa_calling_for_heat();
    last_spa_open = spa_heat_ex_valve_open();
    last_roof_valves_to_pool = !roof_valves_status_in_tank_mode();
    last_pool_heat = pool_heat_request_relay_on();
    last_operating_mode = operating_mode;

    // We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
    // read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.

    // Sample output
    // # Date     Time Year Mode Tank Panel SpaT SpaH Spump Rpump Taka Call Open Clsd Time Erro
    // Mar 25 10:23:24 2025 Oper  138  157    99    0     0     0    0    0    0    1    0    0
    time_t t = now();
    snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ", 
        year(t),
        month(t), 
        day(t), 
        hour(t),
        minute(t), 
        second(t));

    out.print(cbuf);
    snprintf(cbuf, sizeof(cbuf), "%4d %4d %4d %4d %4d ",     
        (int)temps[tank_e].temperature_F,
        (int)temps[left_panel_e].temperature_F,
        (int)temps[right_panel_e].temperature_F,
        (int)average_panel_temperature_F,
        (int)temps[spa_e].temperature_F);
    out.print(cbuf);

    snprintf(cbuf, sizeof(cbuf), "%4d %4d %4d %4d %4d %4d ",
        spa_heater_relay_on(),
        solar_pump_on(),
        recirc_pump_on(),
        takagi_on(),
        spa_calling_for_heat(),
        spa_heat_ex_valve_open());
    out.print(cbuf);


    snprintf(cbuf, sizeof(cbuf), "%4d %4d %4d ",
        !roof_valves_status_in_tank_mode(),
        pool_heat_request_relay_on(),
        operating_mode);
    
    out.println(cbuf);
    line_counter--;
    records_skipped = 0;
  }
}

void turn_recirc_pump_on(void)
{
  relay_write(RELAY_RECIRC_PUMP, true); // Ground the low side of the SSR, turning on recirculation pump
}

void turn_recirc_pump_off(const __FlashStringHelper *message) 
{
  if (message != nullptr) {
    out.println(message);
  }
  relay_write(RELAY_RECIRC_PUMP, false); // Un-Ground the low side of the SSR, turning off recirculation pump
}

void monitor_recirc_pump_callback(void)
{
  check_free_memory(F("monitor_recirc_.."));
  if (!time_valid) {
    return;   // These decisions depend on the hour; wait for the RTC or NTP
  }
  unsigned h = hour(now());
  unsigned m = minute(now());
  
// Turn the recirc pump on at the top of the hour and at the half hour for two minutes.  However, if the
// the Takagi is on, restrict the operating time to 8AM to 11PM

  if (!recirc_pump_on()) {
    if ((h >= 7 && h <= 23) || !takagi_on()) {
      switch (m) {
        case 0: 
        case 30:
          turn_recirc_pump_on();
          break;
      }
    }
  } else {

  // We turn off the takagi regardless of the hour or whether the Takagi is on.  This is because
  // the Takagi state could have changed since we turned on the recirc pump.  We want to avoid 
  // the situation where the recirc pump is not turned off when it should.

    switch (m) {
      case 2:
      case 32:
          turn_recirc_pump_off(nullptr);
        break;
    }
  }
}

void monitor_roof_valves_callback()
{

  // Turn off the roof valve power if it has been awhile since the last time we caused them to change position.
  if (roof_valves_power_is_on() && (millis() - roof_valves_motion_start_time) > max_roof_valves_power_on_time) {
    turn_roof_valves_power_off();
  }

  // If the solar pump is off and the tank is hot, and this is a day in the pool heating season,
  //  and we are not yet requesting that the pool controller send pool water to the roof, and we
  // have not given up on heating the pool (i.e., we haven't marked it "INOP"), and the panels
  // are hot enough for it to be worthwhile, then
  // first turn the roof valves to pool mode once the solar pump has been off for two minutes
  // (to give the panels time to drain) and then turn the pool-heat-request-relay on at 
  // least 20 seconds after we have started the roof valves in motion.

  if (!solar_pump_on() && !pool_heat_request_relay_on() && pool_heating_season() &&
      temps[tank_e].temperature_valid && temps[tank_e].temperature_F >= 140 &&
      average_panel_temperature_F > 120) {
    // If we have given the tank water time to drain back to the tank, then turn the roof
    // valves to pool mode.

    if (roof_valves_set_to_tank_mode()) {
      // Turn the roof valves to pool mode if it has been at least two minutes since
      // we last turned the solar pump off and at least five minutes since the last
      // time we turned the roof valves.
      if ((millis() - solar_pump_on_off_time) > 2 * 60 * 1000UL) {
        // This is defensive programming, in case of logic bugs elsewhere, don't change
        // the roof valve position too often:
        if ((millis() - roof_valves_motion_start_time) > 5 * 60 * 1000UL) {
          turn_roof_valves_to_pool_mode(F("# turning roof valves to pool\n"));
        } 
      }
    }
  
   //then make that request to the pool controller if it has been more
   // than 30 seconds since we started the roof valves in motion
  
    if (roof_valves_set_to_pool_mode() && average_panel_temperature_F > 120) {
      if ((millis() - roof_valves_motion_start_time) > 30 * 1000UL) {
        if (roof_valves_status_in_tank_mode()) {
          static unsigned char spew_counter;
          if (spew_counter < 10) {
            out.println(F("# alert roof valve timeout"));
            spew_counter++;
          }
        } else {
          turn_pool_heat_request_relay_on(F("# pool-heat request\n"));
        }
      } 
    }
  }

   // Turn off pool-heat request if the the panels are too cold to be effective
   // for even heating the pool
  if (pool_heat_request_relay_on()) {
    // Defensive programming: ensure that we don't toggle the pool heat request relay too often
    if ((millis() - pool_heat_request_relay_on_off_time) > 20 * 60 * 1000UL) {
      if (average_panel_temperature_F < 90.0) {
        turn_pool_heat_request_relay_off(F("# pool heat request off: low panel temperature\n"));
      } else if (!pool_heating_season()) {
        turn_pool_heat_request_relay_off(F("# pool heat request off: not pool heating season"));
      }
    }
  } else {
  // If the roof valves are set to to pool-heat mode, but we are no longer asking for 
  // the pool pump controller to send pool water to the roof, and
  // it has been four minutes since we de-asserted the pool heat request, assume
  // the panels have drained of pool water and turn the roof valves back to tank mode
  // The poolpumpcontrol will not activate the main pump for five minutes after deasserting
  // of the pool heat request, to ensure that it does not send water to the panels during
  // the four minute drain-down period.  This extra minute is to cover valve-transition times,
  // delays in recognizing signals, etc.

    if (roof_valves_set_to_pool_mode() &&
        (millis() - pool_heat_request_relay_on_off_time) > (4 * 60 * 1000UL)) {
          // Give time for the pool heat request relay (after we have turned the
          // roof valves to pool mode) to turn on.  Once that happens, we won't 
          // infer from the roof valve position that we have stopped wanting to solar-heat
          // the pool and thus, incorrectly, turn the roof valves back to tank mode.
          // This is also defensive programming: ensure that whatever other bugs
          // might be in the controller code, we don't swing the valve back and forther
          // too frequently.
      if ((millis() - roof_valves_motion_start_time) > 5 * 60 * 1000UL) {
        turn_roof_valves_to_tank_mode(F("# roof valves to tank mode after drain-down\n"));
      }
    }
  }
  // If it has been 30 seconds since we last commanded the roof valves to change position, check to ensure
  // that their position matches what the status indicator says.
  if ((millis() - roof_valves_motion_start_time) > 30 * 1000UL) {
    static unsigned spew_count;
    bool r = roof_valves_status_in_tank_mode();
    if (r != roof_valves_set_to_tank_mode() && spew_count < 20) {
      out.print(F("# alert roof valve status mismatch, status="));
      out.println(r);
      spew_count++;
    }
  }
}

void turn_takagi_on(void)
{
  takagi_on_time = millis();
  relay_write(RELAY_TAKAGI, true); // Turn on Takagi flash heater
}

void turn_takagi_off(const __FlashStringHelper *message)
{
  if (message != nullptr) {
    out.println(message);
  }
  if (takagi_on_time > 0UL) {
    unsigned long elapsed = millis() - takagi_on_time;
    if (elapsed < (MAX_ULONG / 2)) {
      daily_milliseconds_of_takagi_on_time += elapsed;
    }
    takagi_on_time = 0UL;
  }
  relay_write(RELAY_TAKAGI, false); // Turn off Takagi flash heater
}

void monitor_takagi_callback(void)
{
  check_free_memory(F("mt_"));
  static unsigned long last_takagi_change = 0;

  if (operating_mode == m_oper) {
    if (temps[tank_e].temperature_valid) {
      if (takagi_on() && temps[tank_e].temperature_F >= takagi_off_threshold_F) {
        turn_takagi_off(nullptr);
      } else if (takagi_on() == false && temps[tank_e].temperature_F <  takagi_on_threshold_F) {
        if (last_takagi_change == 0 || (millis() - last_takagi_change) > TAKAGI_DELAY * 1000UL) {
          turn_takagi_on();
          last_takagi_change = millis();
        }
      }
    } else {
      // If the tank temperature isn't valid, then just turn on the Takagi and leave it on
      turn_takagi_on();
    }
  }
}


// TCA9554: register 1 is the output latch, register 3 the direction (1 = input).  On the 8RO a latch bit
// of 1 energizes its relay.  The expander keeps its state through an ESP32 reset.
static bool tca_write(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static int tca_read(uint8_t reg)
{
  Wire.beginTransmission(TCA9554_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(TCA9554_ADDR, 1) != 1) {
    return -1;
  }
  return Wire.read();
}

bool Onboard_Relays::begin(void)
{
  int direction = tca_read(3);
  int latch = tca_read(1);
  if (direction < 0 || latch < 0) {
    return false;
  }
  if (direction == 0x00) {          // Already set up: keep the relays as they are
    bits = latch;
    out.print(F("# relays kept through reset: 0x"));
    out.println(bits, HEX);
    return true;
  }
  bits = 0;
  return tca_write(1, 0x00) && tca_write(3, 0x00);
}

void Onboard_Relays::set(uint8_t relay, bool on)
{
  uint8_t b = on ? (bits | (1 << (relay - 1))) : (bits & ~(1 << (relay - 1)));
  if (tca_write(1, b)) {
    bits = b;
  } else {
    out.print(F("# alert relay board did not answer setting relay "));
    out.println(relay);
  }
}

bool Onboard_Relays::verify(void)
{
  int latch = tca_read(1);
  int direction = tca_read(3);
  if (latch < 0 || direction < 0) {
    return false;
  }
  if (latch != bits || direction != 0x00) {
    out.print(F("# alert relay latch was 0x"));
    out.print(latch, HEX);
    out.print(F(", rewriting 0x"));
    out.println(bits, HEX);
    tca_write(1, bits);
    tca_write(3, 0x00);
  }
  return true;
}

void check_relays_callback(void)
{
  static int failures;
  if (quad_lv_relay1 == nullptr) {
    Onboard_Relays *r = new Onboard_Relays();
    if (r->begin()) {
      quad_lv_relay1 = quad_lv_relay2 = r;
    } else {
      delete r;
      if (failures++ % 60 == 0) {
        out.println(F("# alert relay board not found"));
      }
    }
    return;
  }
  if (!quad_lv_relay1->verify()) {
    if (failures++ % 60 == 0) {
      out.println(F("# alert relay board did not answer"));
    }
  } else {
    failures = 0;
  }
}

void set_operating_mode(operating_mode_t m)
{
  operating_mode = m;
  out.print(F("# mode now "));
  out.println(operating_mode_to_string(operating_mode));
  if (operating_mode == m_oper) {
    monitor_diag_mode.disable();
  } else {
    monitor_diag_mode.enable();
    time_entering_diag_mode = millis();
  }
}

// Commands, from USB serial or stctl (POST /cmd):
//   mode oper|poolheat|roof|rpump|spump|takagi|hex|elec   diag modes revert to Oper after 10 minutes
//   + / -        what the plus and minus keys did in the diag modes
//   reboot       restart the ESP32 (the relays stay as they are)
//   upload on|off  send telemetry to bedichek.org or not (saved in flash)
void run_command(const char *command)
{
  out.print(F("# command: "));
  out.println(command);
  if (strncmp(command, "mode ", 5) == 0) {
    const char *names[] = {"oper", "poolheat", "roof", "rpump", "spump", "takagi", "hex", "elec"};
    for (int m = 0; m < m_last; m++) {
      if (strcmp(command + 5, names[m]) == 0) {
        set_operating_mode((operating_mode_t)m);
        return;
      }
    }
    out.println(F("# modes are oper, poolheat, roof, rpump, spump, takagi, hex, elec"));
  } else if (strcmp(command, "+") == 0) {
    plus_key();
    time_entering_diag_mode = millis();
  } else if (strcmp(command, "-") == 0) {
    minus_key();
    time_entering_diag_mode = millis();
  } else if (strcmp(command, "upload on") == 0 || strcmp(command, "upload off") == 0) {
    upload_enabled = strcmp(command, "upload on") == 0;
    Preferences prefs;
    prefs.begin("solar", false);
    prefs.putBool("upload", upload_enabled);
    prefs.end();
    out.println(upload_enabled ? F("# uploads to bedichek.org on") : F("# uploads to bedichek.org off"));
  } else if (strcmp(command, "reboot") == 0) {
    out.println(F("# rebooting"));
    delay(500);
    ESP.restart();
  } else {
    out.print(F("# Unknown command: "));
    out.println(command);
  }
}

void monitor_serial_console_callback(void)
{
  static char command_buf[40];
  while (Serial.available() > 0) {
    char c = Serial.read();
    size_t l = strlen(command_buf);
    if (c == '\r') {
      continue;
    }
    if (l >= sizeof(command_buf) - 1) {
      command_buf[0] = '\0';
    } else if (c == '\n') {
      run_command(command_buf);
      command_buf[0] = '\0';
    } else {
      command_buf[l] = c;
      command_buf[l + 1] = '\0';
    }
  }
}


static uint8_t bcd2bin(uint8_t v) { return (v & 0x0F) + 10 * (v >> 4); }
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

bool rtc_read(time_t *t)
{
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(PCF85063_ADDR, 7) != 7) {
    return false;
  }
  uint8_t r[7];
  for (int i = 0; i < 7; i++) {
    r[i] = Wire.read();
  }
  if (r[0] & 0x80) {          // Oscillator stopped: the time is not valid
    return false;
  }
  struct tm tm = {};
  tm.tm_sec = bcd2bin(r[0] & 0x7F);
  tm.tm_min = bcd2bin(r[1] & 0x7F);
  tm.tm_hour = bcd2bin(r[2] & 0x3F);
  tm.tm_mday = bcd2bin(r[3] & 0x3F);
  tm.tm_mon = bcd2bin(r[5] & 0x1F) - 1;
  tm.tm_year = bcd2bin(r[6]) + 100;
  if (tm.tm_year < 125 || tm.tm_year > 140) {   // Before 2025: never set; after 2040: garbage (the demo firmware left 2056)
    return false;
  }
  // mktime() works in local time; the RTC holds UTC, so convert with TZ set to UTC
  setenv("TZ", "UTC0", 1);
  tzset();
  *t = mktime(&tm);
  setenv("TZ", TIMEZONE, 1);
  tzset();
  return true;
}

bool rtc_write(time_t t)
{
  struct tm tm;
  gmtime_r(&t, &tm);
  Wire.beginTransmission(PCF85063_ADDR);
  Wire.write(0x04);
  Wire.write(bin2bcd(tm.tm_sec));       // Also clears the oscillator-stopped flag
  Wire.write(bin2bcd(tm.tm_min));
  Wire.write(bin2bcd(tm.tm_hour));
  Wire.write(bin2bcd(tm.tm_mday));
  Wire.write(tm.tm_wday);
  Wire.write(bin2bcd(tm.tm_mon + 1));
  Wire.write(bin2bcd(tm.tm_year - 100));
  return Wire.endTransmission() == 0;
}

volatile bool ntp_synced = false;

void time_sync_notification(struct timeval *tv)
{
  ntp_synced = true;
}

void sync_rtc_callback(void)
{
  static unsigned long last_rtc_write;
  if (ntp_synced) {
    ntp_synced = false;
    if (!time_valid) {
      time_valid = true;
      out.println(F("# clock set from NTP"));
    }
    if (last_rtc_write == 0 || millis() - last_rtc_write > 3600 * 1000UL) {
      if (rtc_write(time(nullptr))) {
        last_rtc_write = millis();
      } else {
        out.println(F("# alert could not write the RTC"));
      }
    }
  }
}

void setup_time(void)
{
  setenv("TZ", TIMEZONE, 1);
  tzset();
  time_t t;
  if (rtc_read(&t)) {
    struct timeval tv = {t, 0};
    settimeofday(&tv, nullptr);
    time_valid = true;
  } else {
    out.println(F("# alert RTC has no valid time; waiting for NTP"));
  }
  sntp_set_time_sync_notification_cb(time_sync_notification);
  configTzTime(TIMEZONE, "pool.ntp.org", "time.apple.com");
}

#define HOSTNAME "solarthermal"
#define STATIC_IP IPAddress(192, 168, 1, 9)
#define GATEWAY_IP IPAddress(192, 168, 1, 1)
#define SUBNET_MASK IPAddress(255, 255, 255, 0)
#define DNS2_IP IPAddress(1, 1, 1, 1)
#define INGEST_URL "https://bedichek.org/home/api/ingest/solarthermal"

volatile bool eth_connected = false;
bool status_served = false;              // Someone fetched /status since boot: the network works

void network_event(arduino_event_id_t event, arduino_event_info_t info)
{
  switch (event) {
    case ARDUINO_EVENT_ETH_START:
      ETH.setHostname(HOSTNAME);
      break;
    case ARDUINO_EVENT_ETH_CONNECTED:   // The address is static, so link up means ready
    case ARDUINO_EVENT_ETH_GOT_IP:
      eth_connected = true;
      break;
    case ARDUINO_EVENT_ETH_LOST_IP:
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    case ARDUINO_EVENT_ETH_STOP:
      eth_connected = false;
      break;
    default:
      break;
  }
}

WebServer server(80);

static bool authorized(void)
{
  if (server.header("X-Token") == ST_LAN_TOKEN) {
    return true;
  }
  server.send(403, "text/plain", "forbidden\n");
  return false;
}

void handle_status(void)
{
  status_served = true;
  time_t t = now();
  struct tm tm = tm_of(t);
  char when[32];
  strftime(when, sizeof(when), "%Y-%m-%d %H:%M:%S %Z", &tm);
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
  esp_ota_get_state_partition(running, &state);
  String s;
  s.reserve(1200);
  s += "{\n  \"time\": \"" + String(when) + "\", \"time_valid\": " + String(time_valid ? "true" : "false") + ",\n";
  s += "  \"mode\": \"" + String(operating_mode_to_string(operating_mode)) + "\",\n";
  const char *names[] = {"tank", "left_panel", "right_panel", "spa"};
  for (int i = 0; i < last_temp_e; i++) {
    s += "  \"" + String(names[i]) + "_F\": " + String(temps[i].temperature_F, 1) + ", \"" + String(names[i]) +
         "_valid\": " + String(temps[i].temperature_valid ? "true" : "false") + ",\n";
  }
  s += "  \"average_panel_F\": " + String(average_panel_temperature_F, 1) + ",\n";
  s += "  \"solar_pump\": " + String(solar_pump_on()) + ", \"recirc_pump\": " + String(recirc_pump_on()) +
       ", \"takagi\": " + String(takagi_on()) + ", \"spa_electric\": " + String(spa_heater_relay_on()) + ",\n";
  s += "  \"spa_valve_open\": " + String(spa_heat_ex_valve_open()) + ", \"spa_call\": " + String(spa_calling_for_heat()) + ",\n";
  s += "  \"roof_status_tank\": " + String(roof_valves_status_in_tank_mode()) + ", \"roof_set_tank\": " + String(roof_valves_set_to_tank_mode()) +
       ", \"roof_power\": " + String(roof_valves_power_is_on()) + ", \"pool_request\": " + String(pool_heat_request_relay_on()) + ",\n";
  s += "  \"relays\": " + String(quad_lv_relay1 != nullptr ? quad_lv_relay1->state() : 0) +
       ", \"analog_module\": " + String(analog_failures < 20 ? "true" : "false") + ",\n";
  {
    uint8_t req[8] = {ANALOG_MODBUS_ADDR, 0x03, 0x10, 0x00, 0x00, ANALOG_CHANNELS_USED};
    uint8_t reply[5 + 2 * ANALOG_CHANNELS_USED];
    s += "  \"analog_types\": [";
    if (modbus_transaction(req, 6, reply, sizeof(reply))) {
      for (int i = 0; i < ANALOG_CHANNELS_USED; i++) {
        s += String((reply[3 + 2 * i] << 8) | reply[4 + 2 * i]) + (i < ANALOG_CHANNELS_USED - 1 ? ", " : "");
      }
    }
    s += "], \"analog_mode_set\": " + String(analog_mode_set ? "true" : "false") + ",\n";
  }
  s += "  \"analog_mv\": [" + String(analog_last_mv[0]) + ", " + String(analog_last_mv[1]) + ", " +
       String(analog_last_mv[2]) + ", " + String(analog_last_mv[3]) + "],\n";
  s += "  \"upload\": " + String(upload_enabled ? "true" : "false") + ", \"upload_queue_bytes\": " + String(upload_queue.length()) + ",\n";
  s += "  \"uptime_s\": " + String(millis() / 1000) + ", \"reset_cause\": \"" + String(reset_cause) + "\",\n";
  s += "  \"firmware\": \"" FIRMWARE_VERSION "\", \"partition\": \"" + String(running->label) +
       "\", \"firmware_verified\": " + String(state == ESP_OTA_IMG_PENDING_VERIFY ? "false" : "true") + ",\n";
  s += "  \"free_heap\": " + String(ESP.getFreeHeap()) + ", \"lowest_heap\": " + String(lowest_memory) + "\n}\n";
  server.send(200, "application/json", s);
}

void handle_log(void)
{
  size_t max = server.hasArg("bytes") ? server.arg("bytes").toInt() : 8000;
  String s;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  recent_lines.copy_tail(s, max);
  xSemaphoreGive(lines_mutex);
  server.send(200, "text/plain", s);
}

void handle_cmd(void)
{
  if (!authorized()) {
    return;
  }
  String c = server.arg("plain");
  c.trim();
  size_t before = 0;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  before = recent_lines.length();
  xSemaphoreGive(lines_mutex);
  run_command(c.c_str());
  // Reply with what the command printed
  String s;
  xSemaphoreTake(lines_mutex, portMAX_DELAY);
  recent_lines.copy_tail(s, recent_lines.length() > before ? recent_lines.length() - before : 0);
  xSemaphoreGive(lines_mutex);
  server.send(200, "text/plain", s);
}

bool update_authorized;

void handle_update_done(void)
{
  if (!update_authorized) {
    server.send(403, "text/plain", "forbidden\n");
    return;
  }
  if (Update.hasError()) {
    server.send(500, "text/plain", String("update failed: ") + Update.errorString() + "\n");
    out.print(F("# alert firmware update failed: "));
    out.println(Update.errorString());
    return;
  }
  server.send(200, "text/plain", "update written, restarting\n");
  out.println(F("# firmware update written, restarting"));
  delay(500);
  ESP.restart();
}

void handle_update_upload(void)
{
  HTTPUpload &upload = server.upload();
  esp_task_wdt_reset();
  if (upload.status == UPLOAD_FILE_START) {
    update_authorized = server.header("X-Token") == ST_LAN_TOKEN;
    if (update_authorized) {
      out.print(F("# firmware update starting: "));
      out.println(upload.filename);
      Update.begin(UPDATE_SIZE_UNKNOWN);
    }
  } else if (!update_authorized) {
    return;
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    Update.write(upload.buf, upload.currentSize);
  } else if (upload.status == UPLOAD_FILE_END) {
    Update.end(true);
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
  }
}

void setup_network(void)
{
  Network.onEvent(network_event);
  SPI.begin(ETH_SCLK_PIN, ETH_MISO_PIN, ETH_MOSI_PIN);
  ETH.begin(ETH_PHY_W5500, 1, ETH_CS_PIN, ETH_INT_PIN, -1, SPI);
  ETH.config(STATIC_IP, GATEWAY_IP, SUBNET_MASK, GATEWAY_IP, DNS2_IP);

  const char *headers[] = {"X-Token"};
  server.collectHeaders(headers, 1);
  server.on("/status", HTTP_GET, handle_status);
  server.on("/", HTTP_GET, handle_status);
  server.on("/log", HTTP_GET, handle_log);
  server.on("/cmd", HTTP_POST, handle_cmd);
  server.on("/update", HTTP_POST, handle_update_done, handle_update_upload);
  server.begin();
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
}

void upload_task(void *arg)
{
  static char chunk[32768];
  unsigned long backoff_ms = 10000;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(backoff_ms));
    if (!eth_connected) {
      continue;
    }
    xSemaphoreTake(lines_mutex, portMAX_DELAY);
    size_t n = upload_queue.peek(chunk, sizeof(chunk));
    xSemaphoreGive(lines_mutex);
    if (n == 0) {
      continue;
    }
    NetworkClientSecure client;
    client.useBuiltinCACertBundle();
    HTTPClient http;
    http.setTimeout(15000);
    int code = -1;
    if (http.begin(client, INGEST_URL)) {
      http.addHeader("Authorization", "Bearer " ST_INGEST_TOKEN);
      http.addHeader("Content-Type", "text/plain");
      code = http.POST((uint8_t *)chunk, n);
      http.end();
    }
    if (code == 200) {
      xSemaphoreTake(lines_mutex, portMAX_DELAY);
      upload_queue.consume(n);
      xSemaphoreGive(lines_mutex);
      backoff_ms = 10000;
    } else {
      Serial.printf("# upload to bedichek.org failed: %d\n", code);
      backoff_ms = backoff_ms < 300000 ? backoff_ms * 2 : 300000;
    }
  }
}

extern "C" bool verifyRollbackLater()   // C linkage, or the core keeps its own version, which marks every image good
{
  return true;
}

void mark_firmware_good_callback(void)
{
  const esp_partition_t *running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
    if (quad_lv_relay1 != nullptr && status_served) {
      esp_ota_mark_app_valid_cancel_rollback();
      out.println(F("# new firmware marked good"));
    } else {
      out.println(quad_lv_relay1 == nullptr ? F("# alert new firmware cannot reach the relay board; going back to the previous firmware")
                                           : F("# alert new firmware was not reached over the network; going back to the previous firmware"));
      delay(500);
      esp_ota_mark_app_invalid_rollback_and_reboot();
    }
  }
}

void setup_reset_cause(void)
{
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  reset_cause = 'P'; break;
    case ESP_RST_BROWNOUT: reset_cause = 'B'; break;
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      reset_cause = 'W'; break;
    case ESP_RST_SW:       reset_cause = 'S'; break;
    case ESP_RST_PANIC:    reset_cause = 'C'; break;
    default:               reset_cause = 'X'; break;
  }
  if (reset_cause == 'W' || reset_cause == 'C' || reset_cause == 'B') {
    Preferences prefs;
    prefs.begin("solar", false);
    uint32_t count = prefs.getUInt("bad_resets", 0) + 1;
    prefs.putUInt("bad_resets", count);
    prefs.end();
    out.print(F("# alert Reset cause: "));
    out.print(reset_cause);
    out.print(F(", total count="));
    out.println(count);
  }
}

void setup_temperature_sensors()
{
  temps[tank_e].channel = 0;            // Measures the voltage from the LM35 glued to the tank
  temps[tank_e].calibration_offset_F = 9;  // Set 2026-11-02 so the module matches the Arduino's 108 F (which was calibrated against the 18B20 on the tank)
  temps[tank_e].lower_bound_F = 40;
  temps[tank_e].upper_bound_F = 190;

  temps[left_panel_e].channel = 1;  // Leftmost solar panel temperature
  temps[left_panel_e].calibration_offset_F = 3; 
  temps[left_panel_e].lower_bound_F = 10;
  temps[left_panel_e].upper_bound_F = 280;

  temps[spa_e].channel = 2;             // Temperature of metal tube in spa after recirc pump
  temps[spa_e].calibration_offset_F = 14;  // Correct when  panels are 34F, dynamic correction in sample routine
  temps[spa_e].lower_bound_F = 32;
  temps[spa_e].upper_bound_F = 120;

  temps[right_panel_e].channel = 3;    // Rightmost solar panel temperature
  temps[right_panel_e].calibration_offset_F = 2; 
  temps[right_panel_e].lower_bound_F = 10;
  temps[right_panel_e].upper_bound_F = 280;
}

void setup(void)
{
  Serial.begin(SERIAL_BAUD);
  lines_mutex = xSemaphoreCreateMutex();
  recent_lines.init(64 * 1024, true);
  upload_queue.init(1024 * 1024, true);

  Preferences prefs;
  prefs.begin("solar", true);
  upload_enabled = prefs.getBool("upload", false);
  prefs.end();

  setup_reset_cause();
  for (int pin = 4; pin <= 11; pin++) {
    pinMode(pin, INPUT_PULLUP);       // DI1..DI8
  }
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 100000);
  Wire.setTimeOut(25);

  RS485.begin(ANALOG_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  RS485.setPins(-1, -1, -1, RS485_DE_PIN);
  RS485.setMode(UART_MODE_RS485_HALF_DUPLEX);

  setup_time();
  out.print(F("# alert reboot solarthermal (ESP32) " FIRMWARE_VERSION " reset cause "));
  out.println(reset_cause);

  check_relays_callback();     // This sets quad_lv_relay1 and quad_lv_relay2
  if (pool_heat_request_relay_on() && roof_valves_set_to_tank_mode()) {
    turn_pool_heat_request_relay_off(F("# alert: request relay on, but roof valves in tank mode\n"));
  }
  setup_temperature_sensors();
  setup_network();
  xTaskCreatePinnedToCore(upload_task, "upload", 8192, nullptr, 1, nullptr, 0);
  mark_firmware_good.enableDelayed(TASK_SECOND * 60);

  esp_task_wdt_config_t wdt_config = {.timeout_ms = 8000, .idle_core_mask = 0, .trigger_panic = true};
  esp_task_wdt_reconfigure(&wdt_config);
  esp_task_wdt_add(nullptr);
}

void loop(void)
{
  esp_task_wdt_reset();
  ts.execute();
  server.handleClient();
}
