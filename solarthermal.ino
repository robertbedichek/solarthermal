
/*
   This controls:
      An SSR that controls a 120VAC water pump that moves water from the tank to the panels (and back)
      An SSR that controls the 120VAC water pump that recirculates hot water through our home
      An SSR that controls our taka natural-gas-fired domestic water heater
        The Takagi is fed by water that first goes through a heat exchanger in a 4000 pound thermal mass (water)
      A 12V signal that powers a small 12V relay added to the spa controller that enables the electrical spa heater (the "native" heater for the spa)

   This senses:
      Tank temperature via an LM35
      Panel temperature via an LM335
      Spa heat exchanger temperature output via an LM35
      A digital input that signals whether the spa is calling for heat
      
   Creative Commons Licence
   Robert Bedichek
*/

#include <string.h> //Use the string Library
#include <ctype.h>

#include <EEPROM.h>
#include <assert.h>
#include <RTClib.h>
RTC_DS3231 rtc;
#define RTC_I2C_ADDR (0x68)
const bool force_RTC_reload_from_build_time = false;
const bool verbose_rtc = false; // Adds 70 bytes to RAM demand if true
const bool verbose_I2C = false;
/*
 * We have two Sparkfun relay boards and possibly other 1-wire devices
 */
#include <Wire.h>

/*
 * Low voltage relay bank.  Relay 1 enables the electric spa heater (the built-in heater that is part of the spa) when set to '1',
 * this allows the spa's thermostatic system to control the spa heater normally, as it did when it came from the factory.
 * 
 * Relay 2 commands the motorized valve to open, allowing the spa's built-in recirculation pump to push water through the heat
 * exchanger in the solar mass.
 * 
 * Relay 3 commands the motorized valve to close, making the spa's built-in recirculation pump to do what it did when it came from the
 * factory, just push water through the electric heater (which might or might not be enabled) and onto the spa.
 * 
 * Relay 4 is unused.
 */
#include "SparkFun_Qwiic_Relay.h"
#define LV_RELAY_I2C_ADDR  (0x6D)                 // Default I2C address of the mechanical, low voltage, 4-relay board

Qwiic_Relay *quad_lv_relay;

#include <SerLCD.h>
SerLCD *lcd;
#define SERLCD_I2C_ADDR (0x72)
const bool fast_lcd_comm = false;

// This string variable is used by multiple functions below, but not at the same time
char cbuf[77];

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

#include <TimeLib.h>     // for update/display of time
/*
    Central (and only) task scheduler data structure.
*/
Scheduler ts;

/*
   Set this to set the time to the build time.  This is currently the only way to set time (let this be defined, compile, and
   run right away so that the compile-time time is pretty close to the actual time).
*/
// #define SETTIME

/*
   Speed at which we run the serial connection (both via USB and RS-485)
*/
#define SERIAL_BAUD (115200)

/*
 * We have a Sparkfun SSR 4-relay board.  However, it is (1) not present on the I2C bus, and (2) obsolete.
 * We use three of the four relays, but we drive them directly from Arudino digital outputs.
 * To enable an SSR, we drive its control output to ground.
 */
#define SSR_SOLAR_PUMP_PIN  (12)    // writing '0' turns on solar pump
#define SSR_RECIRC_PUMP_PIN (11)    // writing '0' turns on the recirculation pump
// We have reused this pin, so it is not availble #define SSR_SPARE       (10)
#define SSR_TAKAGI_PIN      (9)    // writing '0' turns on the Takagi natural-gas fired water heater

/*
 * There is a motorized valve that I added to the spa.  When it is open, some of the water coming from the recirculation pump in the spa
 * will go through the stainless steel heat exchanger that I added to the solar tank.  This valve is controlled by two 12VDC signals.  When
 * the "open" signal is +12V and the "close" signal is ground, the valve will open.  When the "close" signal is +12VDC and the "open" signal
 * is ground, then the valve will close.  These two signals are enabled by relays 3 and 2 in the Sparkfun relay board.  Relay 1 is unused.
 */
#define LV_RELAY_SPA_HEAT_EX_VALVE_OPEN  (2)   // Turning this relay on will cause the spa heat exchanger valve to open
#define LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE (3)   // Turning on this relay will cause the spa heat exchanger valve to close

#define LV_RELAY_SPA_ELEC_HEAT_ENABLE    (4) // Writing '1' enables the spa relay to power the electric water heater when the spa is calling for heat

/// Digital signal that indicates spa is calling for heat
#define SPA_HEAT_DIGITAL_IN_PIN (8)

#define KEY_1_PIN (7)  // Top most input key
#define KEY_2_PIN (6)
#define KEY_3_PIN (5)
#define KEY_4_PIN (4)  // Bottom most input key

#define SPA_HEAT_EX_VALVE_STATUS_OPEN_PIN   (10)  // Green wire from the Solid Valve, which goes to Green CAT5. Valve is open when this is a zero
#define SPA_HEAT_EX_VALVE_STATUS_CLOSED_PIN (3)  // Red wire on Solid valve, which goes to Orange CAT5.  Valve is closed when this is zero.

typedef enum {m_oper, m_safe, m_rpump, m_spump, m_takagi, m_spa_hex_valve, m_spa_elec, m_last} operating_mode_t;

void monitor_valve_closing_callback(void);
void monitor_valve_opening_callback(void);
void read_time_and_sensor_inputs_callback(void);
void print_status_to_serial_callback(void);
void poll_keys_callback(void);
void process_pressed_keys_callback(void);
void update_lcd_callback(void);
void monitor_spa_valve_callback(void);
void monitor_recirc_pump_callback(void);
void monitor_takagi_callback(void);
void monitor_clock_callback(void);
void monitor_diag_mode_callback(void);
void monitor_serial_console_callback(void);
void monitor_solar_pump_callback(void);
void monitor_spa_electric_heat_callback(void);

/*****************************************************************************************************/
enum temps_e {tank_e, left_panel_e, right_panel_e, spa_e, last_temp_e};

struct temperature_s {
  float temperature_F;
  bool temperature_valid;
  char input_pin;
  unsigned last_valid_time; // millis() / 1000 of last time temperature was valid
  int calibration_offset_F;;
  int lower_bound_F;
  int upper_bound_F;
} temps [last_temp_e];

float ema_alpha = 1.0; // Smoothing factor (0 = slow response, 1 = no filtering)

Task read_time_and_sensor_inputs(TASK_SECOND, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
/*****************************************************************************************************/
// We have two options for how to recognize pressing of one of the four keys.  One is to rapidly poll
// the state of the digital input pins to which the keys are connected.  The other is to enable any-input-change
// interrupts.  We have to use the polling method if we want to use certain libraries, e.g., SoftSerial.  Otherwise,
// using interrupts is preferable.  Since we do not need SoftSerial, we use interrupts.  But it is handy to have
// the option to switch to polling for some debugging purposes.

volatile bool select_key_pressed = false;
volatile bool enter_key_pressed = false;
volatile bool plus_key_pressed = false;
volatile bool minus_key_pressed = false;

// We have two ways of recognizing pressed keys, the interrupt method and the polling method.
// Our prefered and default method is via interrupts.  If we get a future peripheral with conflicting
// resource needs (e.g., SoftSerial) or we want to try polling to isolate certain bugs, we may want
// to enabling polling, so the option remains in the code.

// #define POLL_KEYS
#ifndef POLL_KEYS
ISR(PCINT2_vect) 
{
  poll_keys_callback();
}
const bool poll_keys_bool = false;
Task process_pressed_keys(100, TASK_FOREVER, &process_pressed_keys_callback, &ts, true);
#else
const bool poll_keys_bool = true;
Task poll_keys(25, TASK_FOREVER, &poll_keys_callback, &ts, true);
#endif

volatile unsigned long lastInterruptTime = 0;
const int debounce_delay = 150;  // 150ms debounce time
/*****************************************************************************************************/
// const unsigned long one_second_in_milliseconds = 1000;
// const unsigned long five_seconds_in_milliseconds = 5 * one_second_in_milliseconds;
// const unsigned long one_minute_in_milliseconds = 60 * one_second_in_milliseconds;
// const unsigned long five_minutes_in_milliseconds = 5 * one_minute_in_milliseconds;

// We vary the frequency of status lines sent to the serial output from once per second when we are opening
// or closing the spa heat exchanger valve, to five seconds during start up, to once per minute during normal
// operations.

Task print_status_to_serial(TASK_SECOND, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
/*****************************************************************************************************/
// Update slightly faster than once per second so that the LCD time advances regularly in a way that 
// looks normal to humans (i.e., make sure that the seconds display counts up once per second).

unsigned fail_message_time;

Task update_lcd(900, TASK_FOREVER, &update_lcd_callback, &ts, true);
/*****************************************************************************************************/
// These global variables describe the state of the heat exchanger valve.  It can be open,
// closed, in the process of opening or closing, or in an indeterminate state after we attempted to
// open or close it.

unsigned last_valve_open_second; // millis() / 1000 the last time a valve-open operation started

unsigned long valve_motion_start_time = 0; // millis() when the last valve open or closed operation started

bool valve_timeout = false;         // True if it takes too long to open or close the spa heat exchanger valve
                                    // Reset to false on next (attempted) valve operation 

bool valve_error = false;           // True if the spa valve control API is called in a way that it does not expect
                                    // Reset to false on next completed valve operation
bool valve_status_failed = false;   // Set true if we found valve neither open nor closed and were unable to close it
bool spa_heat_ex_status_closed;     // Use this if the valve status circuit has failed
bool spa_heat_ex_status_open;       // ditto
bool valve_verbose = false;

unsigned daily_valve_cycles;        // Number of valve-open operations per day

bool spa_heat_ex_valve_status_closed(void);
bool spa_heat_ex_valve_status_open(void);

Task monitor_valve_closing(200, TASK_FOREVER, &monitor_valve_closing_callback, &ts, false);
Task monitor_valve_opening(200, TASK_FOREVER, &monitor_valve_opening_callback, &ts, false);
Task monitor_spa_valve(TASK_SECOND * 10, TASK_FOREVER, &monitor_spa_valve_callback, &ts, true);
/*****************************************************************************************************/

#define SOLAR_PUMP_DELAY       (300)         // Minimum number of seconds between solar pump on events

float average_panel_temperature_F;   // Average of leftmost and rightmost panels

// The panels must be at least this much hotter than the tank to turn on the solar pump
const float tank_panel_difference_threshold_on_F = 25;

// When the panels drop to being just this much hotter than the tank, turn off the solar pump
const float tank_panel_difference_threshold_off_F = -10; // Keep running pump until panels this much colder than tank
unsigned long solar_pump_on_time;   // Set to millis() / 1000 when pump is turned on
unsigned daily_seconds_of_solar_pump_on_time; // Number of seconds the solar pump has run today

bool solar_pump_on(void);
void turn_solar_pump_on(void);
void turn_solar_pump_off(void);
Task monitor_solar_pump(TASK_SECOND * 60, TASK_FOREVER, &monitor_solar_pump_callback, &ts, true);
/*****************************************************************************************************/
bool spa_heater_relay_on(void);             // True if we have enabled the spa heater relay, off by default
#define HEATER_RELAY_DELAY     (300)          // Minimum time, in seconds, between heater-on events
unsigned daily_seconds_of_spa_heater_on_time;
unsigned spa_heater_relay_on_time;            // Set to millis() / 1000 when heater is switched on
Task monitor_spa_electric_heat(TASK_SECOND * 60, TASK_FOREVER, &monitor_spa_electric_heat_callback, &ts, true);
/*****************************************************************************************************/
const unsigned recirc_on_time_in_seconds = 90;
bool recirc_pump_on();

void turn_recirc_pump_on(void);
void turn_recirc_pump_off(void);

Task monitor_recirc_pump(TASK_SECOND * 10, TASK_FOREVER, &monitor_recirc_pump_callback, &ts, true);
/*****************************************************************************************************/
#define TAKAGI_DELAY           (300)    // Minimum number of seconds between Tagaki on eventsbool takagi_on_var;                         // True if we are powering the Takagi flash heater
const int takagi_on_threshold_F = 120;  // If the tank falls below this temperature, turn the Takagi on
const int takagi_off_threshold_F = 125; // If the tank is this or above, turn the Takagi off and let the solar mass do all the heating

bool takagi_on();

void turn_takagi_on(void);
void turn_takagi_off(void);

unsigned daily_seconds_of_takagi_on_time;
unsigned takagi_on_time;                 // Set to millis()/ 100 the last time the Takagi was turned on

Task monitor_takagi(TASK_SECOND * 60, TASK_FOREVER, &monitor_takagi_callback, &ts, true);
/*****************************************************************************************************/
//   All the operational code uses this time structure.  This is initialized at start time from the battery-backed up DS3231 RTC.
time_t arduino_time;

Task monitor_clock(TASK_SECOND * 3600, TASK_FOREVER, &monitor_clock_callback, &ts, true);
/*****************************************************************************************************/
operating_mode_t operating_mode;

unsigned long time_entering_diag_mode = 0;
#define DIAG_MODE_TIMEOUT (600000) // 10 minutes
Task monitor_diag_mode(TASK_SECOND * 60, TASK_FOREVER, &monitor_diag_mode_callback, &ts, false);
/*****************************************************************************************************/

Task monitor_serial_console(TASK_SECOND, TASK_FOREVER, &monitor_serial_console_callback, &ts, true);

// From ChatGPT
extern unsigned int __heap_start;
extern void *__brkval;

int free_memory() {
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

#define LOW_MEMORY_LIMIT (200)


int lowest_memory = 2000; // This will be overwritten on the first call to "check_free_memory(F("..."));"
void check_free_memory(const __FlashStringHelper *caller)
{
  static int trace_initial_calls = 0;
  int fm = free_memory();
  if (fm < LOW_MEMORY_LIMIT) {
    Serial.print(F("# alert low memory: "));
    Serial.println(fm);
  }
  if (fm < lowest_memory) {
    lowest_memory = fm;
  }
  if (trace_initial_calls > 0) {
    Serial.print(F("# alert "));
    Serial.print(caller);
    Serial.print(F(" free memory="));
    Serial.println(fm);
    trace_initial_calls--;
  }
}

/* Called by paths that sense an inconsistency, but we may be able to continue operating, so we don't
 * want to call fail().
 */
void record_error(const __FlashStringHelper *warning_message)
{
  Serial.print(F("# alert: "));
  Serial.println(warning_message);

  if (lcd != 0) {
    lcd->setCursor(0, 3);
    lcd->print(F("Fail: "));
    lcd->print(warning_message);         // Display this on the third row, left-adjusted
    lcd->print(F("  "));
  }
  fail_message_time = millis() / 1000;
}

// From ChatGPT3
long readVcc() 
{
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);  // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);  // Start conversion
  while (bit_is_set(ADCSRA, ADSC));
  uint16_t result = ADC;
  return 1125300L / result;  // 1.1V * 1023 * 1000
}

bool spa_calling_for_heat(void)
{
  return digitalRead(SPA_HEAT_DIGITAL_IN_PIN) == LOW;
}

/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback(void)
{
  arduino_time = now();

  unsigned second_now = millis() / 1000;
  // EMA -- exponential moving average filter, courtesy of ChatGPT 4o
  if (ema_alpha > 0.03) {
    ema_alpha -= 0.2;
    if (ema_alpha < 0.03) {
      ema_alpha = 0.03;
    }
  }

  const int samples = 20;
  
  for (struct temperature_s *t = &temps[0] ; t < &temps[last_temp_e] ; t++) {
    float temp_F = 0.0;
    int adc_value;
    float voltage;
    float temp_C;

    for (int sample = 0 ; sample < samples ; sample++ ) {
      adc_value = analogRead(t->input_pin);
      delay(2);          // Give ADC a chance to settle
      voltage = (float)adc_value * (5.0 / 1023.0); 
      temp_C = (voltage - 0.5) * 100.0;       /* LM36 gives voltage of 0.5 for 0C and 10mv per degree C*/ ;
      temp_F += (temp_C * (90.0 / 50.0)) + 32.0 + t->calibration_offset_F;
    }
    temp_F /= samples;

    if (t == &temps[spa_e]) {
      // Hack for spa temperature probe, which is not perfectly in the stream of spa water,
      // but is affected by outside temperature.  When the panels are 34F, the spa temp is correctly
      // set with the calibration_offset_F value we set in setup().  When the panels are 169, that's when
      // it is warm outside and the spa sensor reads 6F higher than actual.
      int correction_F = ((average_panel_temperature_F - 34.0) / 135.0) * 6.0;
      temp_F -= correction_F;
    }
 
    // If the temperature is valid, then let this sample's voltage be averaged with
    // previous samples.  If not, toss it out.
    if (temp_F >= t->lower_bound_F && temp_F <= t->upper_bound_F) {
      t->last_valid_time = second_now;
    } else {
      char v_str[10], c_str[10], f_str[10];

      dtostrf(voltage, 4, 3, v_str);
      dtostrf(temp_C, 4, 1, c_str);
      dtostrf(temp_F, 4, 1, f_str);

      snprintf(cbuf,sizeof(cbuf), "# OOR [%d] adc=%d v=%s C=%s F=%s", t - &temps[0], adc_value, v_str, c_str, f_str);
      Serial.println(cbuf);
    }
    t->temperature_F = ema_alpha * temp_F + (1 - ema_alpha) * t->temperature_F;

    if ((second_now - t->last_valid_time) > 600) {
      // If it has been 10 minutes since the last valid sample, then flag this sensor as not valid
      t->temperature_valid = false;
    } else {
      // Given the logic above to eliminate out-of-range samples, the temperature should always be valid
      // But it doesn't cost much to be sure by doing this check again, post-filter
      t->temperature_valid = t->temperature_F >= t->lower_bound_F && t->temperature_F <= t->upper_bound_F;
    }
  }

  if (temps[left_panel_e].temperature_valid) {
    if (temps[right_panel_e].temperature_valid) {
      average_panel_temperature_F = (temps[left_panel_e].temperature_F + temps[right_panel_e].temperature_F) / 2.0;
    } else {
      average_panel_temperature_F = temps[left_panel_e].temperature_F;
      static bool first_time = true;
      if (first_time) {
        record_error(F("right panel sensor failed"));
        first_time = false;
      }
    }
  } else {
    if (temps[right_panel_e].temperature_valid) {
      average_panel_temperature_F = temps[right_panel_e].temperature_F;
      static bool first_time = true;
      if (first_time) {
        record_error(F("left panel sensor failed"));
        first_time = false;
      }
    } else {
      // Both temperature sensors have failed, print error and leave panel_temperature_F as it was´
      static bool first_time = true;
      if (first_time) {
        record_error(F("both panel sensors failed"));
        first_time = false;
      }
    }
  }

  const bool read_vcc = false;
  if (read_vcc) {
    static long last_vcc = 0;
    long vcc = readVcc();
    if (last_vcc != vcc) {
      if (last_vcc != 0) {
        snprintf(cbuf, sizeof(cbuf), "# vcc=%ld last_vcc=%ld");
        Serial.println(cbuf);
      }
    }
  }
}

// Returns a string that desribes the current diag mode.  For some reason, it stopped working to pass
// in the diag mode, but since we always passed in the global variable "diag_mode", just commenting out
// the parameter here, and the passed arguments where this is called allowed this to compile again.

const char *operating_mode_to_string(operating_mode_t operating_mode) 
{
  char *s[] = {"Oper", "Safe", "D-RP", "D-SP", "D-TK", "D-HX", "D-EL"};
  if (operating_mode < m_last) {
    return s[operating_mode];
  }
  return "ERR";
}

// This task is enabled when we enter diag mode.  It checks to see if we've been in diag mode
// too long and reverts to operational mode if so.
void monitor_diag_mode_callback(void)
{
  check_free_memory(F("monitor_diag_mode"));
  unsigned long now = millis();
  
  if ((now - time_entering_diag_mode) > DIAG_MODE_TIMEOUT) {
    operating_mode = m_oper;
    monitor_diag_mode.disable();
  }
}

void poll_keys_callback(void)
{
  
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounce_delay) {  // Debounce check
  
    bool key_select = !(PIND & (1 << PD7));
    bool key_enter = !(PIND & (1 << PD6));
    bool key_plus = !(PIND & (1 << PD5));
    bool key_minus = !(PIND & (1 << PD4));

    if (key_select) {
      select_key_pressed = true;
    }
    if (key_enter) {
      enter_key_pressed = true;
    }
    if (key_plus) {
      plus_key_pressed = true;
    }
    if (key_minus) {
      minus_key_pressed = true;
    }
    
    lastInterruptTime = currentTime;  // Update debounce timer
    if (poll_keys_bool) {
      process_pressed_keys_callback();
    }        
  } 
}

// The interrupt routine above will set global variables indicating which keys have been pressed.
// In this function, we look at those global variables and take action required by the key presses
// and then reset those global variables.

void process_pressed_keys_callback(void)
{
  check_free_memory(F("process_keys.."));
  bool some_key_pressed = select_key_pressed | enter_key_pressed | plus_key_pressed | minus_key_pressed;

  if (select_key_pressed) {
    operating_mode = (operating_mode + 1) % m_last;
    Serial.print(F("# amode now "));
    Serial.println(operating_mode_to_string(operating_mode));
    select_key_pressed = false;
    if (lcd != 0) {
      lcd->setCursor(15 /* column */, 0 /* row */);
      lcd->print(operating_mode_to_string(operating_mode)); 
    }
  }
 
  if (plus_key_pressed) {
    switch (operating_mode) {
      case m_oper:
      case m_safe:
        adjustTime(600);
        break;

      case m_rpump:
        turn_recirc_pump_on();
        Serial.print(F("# recirc pump="));
        Serial.println(recirc_pump_on());
        break;

      case m_spump:
        turn_solar_pump_on();
        Serial.print(F("# solar pump="));
        Serial.println(solar_pump_on());
        break;

      case m_takagi:
        turn_takagi_on();
        Serial.print(F("# Takagi=%d"));
        Serial.println(takagi_on());
        break;

      case m_spa_hex_valve:          
        open_spa_heat_exchanger_valve();
        Serial.print(F("# spa valve open="));
        Serial.print(spa_heat_ex_valve_status_open());
        Serial.print(F(" closed="));
        Serial.println(spa_heat_ex_valve_status_closed());
        break;

      case m_spa_elec:
        turn_spa_heater_relay_on();
        Serial.print(F("# spa heater relay="));
        Serial.println(spa_heater_relay_on());
        break; 
    }
    plus_key_pressed = false;
  }

  if (minus_key_pressed) {
    switch (operating_mode) {
      case m_oper:
        adjustTime(-600);
        break;

      case m_rpump:
        turn_recirc_pump_off();
        Serial.print(F("# recirc pump="));
        Serial.println(recirc_pump_on());
        break;

      case m_spump:
        turn_solar_pump_off();
        Serial.print(F("# solar pump="));
        Serial.println(solar_pump_on());
        break;

      case m_takagi:
        turn_takagi_off();
        Serial.print(F("# Takagi=%d"));
        Serial.println(takagi_on());
        break;

      case m_spa_hex_valve:          
        close_spa_heat_exchanger_valve(F("process_pressed_keys_.."));
        Serial.print(F("# spa valve open="));
        Serial.print(spa_heat_ex_valve_status_open());
        Serial.print(F(" closed="));
        Serial.println(spa_heat_ex_valve_status_closed());
        break;

      case m_spa_elec:
        turn_spa_heater_relay_off();
        Serial.print(F("# spa heater rela="));
        Serial.println(spa_heater_relay_on());
        break; 
    }
    minus_key_pressed = false;
  }
  if (some_key_pressed) {
    if (operating_mode == m_oper || operating_mode == m_safe) {
      monitor_diag_mode.disable();
    } else {
      monitor_diag_mode.enable();
      time_entering_diag_mode = millis();
    }
  } 
}

bool solar_pump_on()
{
  return digitalRead(SSR_SOLAR_PUMP_PIN) == LOW;
}

void turn_solar_pump_on(void)
{
  solar_pump_on_time = millis() / 1000;
  digitalWrite(SSR_SOLAR_PUMP_PIN, LOW); // Turn on solar pump
}

void turn_solar_pump_off(void)
{
  if (solar_pump_on_time > 0) {
    daily_seconds_of_solar_pump_on_time += millis() / 1000 - solar_pump_on_time;
    solar_pump_on_time = 0;
  }
  digitalWrite(SSR_SOLAR_PUMP_PIN, HIGH); // Turn off solar pump;
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
  if (operating_mode != m_oper) {
    return;
  }

  int h = hour(arduino_time);

  if (temps[left_panel_e].temperature_valid || temps[right_panel_e].temperature_valid) {
    if (temps[tank_e].temperature_valid) {
      if (solar_pump_on() == false && temps[tank_e].temperature_F < 165 &&
          average_panel_temperature_F > (temps[tank_e].temperature_F + tank_panel_difference_threshold_on_F)) {
        static unsigned long last_solar_pump_on_time = 0;
        unsigned long current_time = millis() / 1000;
        if (last_solar_pump_on_time == 0 || ((current_time - last_solar_pump_on_time) > SOLAR_PUMP_DELAY)) {
          if (h <= 9 || h >= 20) {
            record_error(F("wrong time of day for solar pump on"));
          } else {
            turn_solar_pump_on();
            last_solar_pump_on_time = current_time;
          }
        }
      }

      if (solar_pump_on()) {
        // If the tank is too hot in absolute valve or the tank is too warm in comparison to the panels,
        // then turn the solar pump off.
        if (temps[tank_e].temperature_F > 170 ||
            (average_panel_temperature_F < (temps[tank_e].temperature_F + tank_panel_difference_threshold_off_F))) {
          turn_solar_pump_off();
        } 
        
        if (h >= 20) {
          turn_solar_pump_off();
          record_error(F("solar pump still on at 8PM"));
        }
      }
    } else {
      // The panel temperature is valid, but the tank temperature is not.  Just turn on the pump if the panels are above
      // 170F and turn it off at 1700.
      if (solar_pump_on() == false && average_panel_temperature_F > 170) {
        turn_solar_pump_on();
      }
      if (solar_pump_on() && h >= 17) {
        turn_solar_pump_off();
      }
    }
  } else {
    // We can't use the panel temperature, so just turn on the solar pump in the late morning and turn it off in the late
    // afternoon
    if (solar_pump_on() == false && h >= 11) {
      turn_solar_pump_on();
    }
    if (solar_pump_on() && h >= 17) {
      turn_solar_pump_off();
    }
  }
}

bool spa_heater_relay_on(void)
{
  return quad_lv_relay->getState(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
}

void turn_spa_heater_relay_on(void)
{
  if (quad_lv_relay != 0) {
    spa_heater_relay_on_time = millis() / 1000;
    quad_lv_relay->turnRelayOn(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
  }
}

void turn_spa_heater_relay_off(void)
{
  if (quad_lv_relay != 0) {
    daily_seconds_of_spa_heater_on_time += millis()/1000 - spa_heater_relay_on_time;
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
  }
}

// This contains the normal basic operational logic of this controller, comparing temperatures
// and deciding which pumps to turn on, how to set the spa heat exchanger valve, etc.

void monitor_spa_electric_heat_callback(void)
{
  check_free_memory(F("monitor_spa"));
  if (temps[tank_e].temperature_valid) {
    if (operating_mode == m_oper) {
      unsigned second_now = millis() / 1000;
      // If the tank is not warm enough to heat the spa, enable the relay we inserted in the spa controller 
      // that allows the spa to heat itself with its own electric heater.  If we are currently heating with
      // solar (the valve is open), then the threshold for turning on the spa electric heater is 110F.  Below
      // that temperature, the valve will close.  If the spa valve is already closed, then turn off the electric
      // heater if the tank is less than 113F.
    
      if (spa_heater_relay_on()) {
        // If the tank is warm enough to heat the spa and we haven't turned the
        // relay on within the hour or if the spa is not calling for heat, 
        // disable electric spa heat
        unsigned seconds_spa_heater_relay_on = second_now - spa_heater_relay_on_time;
        if (temps[tank_e].temperature_F >= 115 && 
            (seconds_spa_heater_relay_on > 3600) || !spa_calling_for_heat()) {
          turn_spa_heater_relay_off();
        }
      } else {
        
        unsigned seconds_valve_open = second_now - last_valve_open_second;
        // If the tank is too cool or if the spa valve has been open for more than 30 minutes, enable the spa's electric heater
        // for at least an hour
        unsigned h = hour(arduino_time);
        // Be more patient heating the spa with solar during hours when it is unlikely anyone will 
        // be using the spa
        unsigned seconds_limit = (1 <= h && h <= 6) ? 5400 : 2700;  // 45 minutes daytime, 1.5 hours in the middle of the night.
        if (temps[tank_e].temperature_F <= (spa_heat_ex_valve_status_closed() ? 113 : 110) ||
           (spa_heat_ex_valve_status_open() && (seconds_valve_open > seconds_limit))) {
            if (valve_verbose) {
              snprintf(cbuf, sizeof(cbuf), "# spaH=%d valve_open=%d valve_closed=%d time=%u",
                spa_heater_relay_on(), spa_heat_ex_valve_status_open(), spa_heat_ex_valve_status_closed(), seconds_valve_open);
              Serial.println(cbuf);
           }
          turn_spa_heater_relay_on();
        }
      }
    }
  } else {
    // If the tank temperature is not valid, play it safe and turn on the electric heater
    Serial.println(F("# tank temp not valid, turning on spa heater"));
    turn_spa_heater_relay_on();
  }
  check_free_memory(F("monitor_spa_ exit"));
}

const bool spa_status_debug = false;

bool spa_heat_ex_valve_status_open(void)
{
  if (valve_status_failed) {
    return spa_heat_ex_status_open;
  }
  bool v = digitalRead(SPA_HEAT_EX_VALVE_STATUS_OPEN_PIN) == LOW;
  if (spa_status_debug) {
    Serial.print(F("# spa open="));
    Serial.println(v);
  }
  return v;
}

bool spa_heat_ex_valve_status_closed(void)
{
  if (valve_status_failed) {
    return spa_heat_ex_status_closed;
  }
  bool v = digitalRead(SPA_HEAT_EX_VALVE_STATUS_CLOSED_PIN) == LOW;
  if (spa_status_debug) {
    Serial.print(F("# spa closed="));
    Serial.println(v);
  }
  return v;
}

// This intiates the valve opening process by applying the correct polarity of power to the valve motor
// and by enabling the task that monitors the opening process.

void open_spa_heat_exchanger_valve(void)
{
  bool opening =  monitor_valve_opening.isEnabled();
  bool closing = monitor_valve_closing.isEnabled();
  if (!opening && !closing && quad_lv_relay != 0) {
    if (valve_verbose) {
      Serial.println(F("# alert opening valve"));
    }
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    valve_motion_start_time = millis();
    valve_timeout = false;
    monitor_valve_opening.enable();
    daily_valve_cycles++;
    last_valve_open_second = millis() / 1000;
    spa_heat_ex_status_closed = false;
  } else {
    Serial.print(F("# alert error in open_spa_heat_exchanger_valve() "));
    if (opening) {
      Serial.print(F("opening "));
    }
    if (closing) {
      Serial.print(F("closing "));
    }
    valve_error = true;
  }
}

// This task is just enabled while the valve is opening, normally a 12 second process.
// It checks to see if the vavle signals that it has opened, at which time it stops the valve
// power and disables itself. If 15 seconds elpases without this signal, it also stops
// the valve power and disables itself (and signals an valve motion error).

void monitor_valve_opening_callback(void)
{
  check_free_memory(F("monitor_valve"));
  unsigned long current_time = millis();
  if ((current_time - valve_motion_start_time) > 15000) {
    // We turn off the valve-opening process once the valve signal says it is open or
    // after 15 seconds, as it should just take 13 seconds
    Serial.println(F("# alert timeout on valve opening"));
    valve_timeout = true;
  }
  if (spa_heat_ex_valve_status_open() || valve_timeout) {
    if (quad_lv_relay != 0) {
      quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    } else {
      // We should never get here, if the relay is gone, we should not have
      // enabled this task.
      record_error(F("# monitor_valve_opening_callback()"));
    }
    monitor_valve_opening.disable();
    valve_motion_start_time = 0;    
    if (valve_timeout == false) {
      valve_error = false;         // Reset error flag on successful opening of valve
    } else {
      valve_status_failed = true;   // Assume the problem is with the status circuit and not the valve itself
    }
    spa_heat_ex_status_open = true; // Backup flag in case status circuit fails
  }
}

// This intiates the valve closing ing process by applying the correct polarity of power to the valve motor
// and by enabling the task that monitors the opening process.

void close_spa_heat_exchanger_valve(const __FlashStringHelper *caller)
{
  bool opening = monitor_valve_opening.isEnabled();
  bool closing = monitor_valve_closing.isEnabled();
  bool valve_not_in_motion = !opening && !closing;
  if (valve_not_in_motion && quad_lv_relay != 0) {
    if (valve_verbose) {
      Serial.print(F("# alert closing valve: "));
      Serial.println(caller);
    }
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    valve_motion_start_time = millis();
    valve_timeout = false;
    monitor_valve_closing.enable();
    valve_not_in_motion = !monitor_valve_opening.isEnabled() && !monitor_valve_closing.isEnabled();
    if (valve_not_in_motion) {
      Serial.println(F("# alert valve_not_in_motion is true unexpectly"));
    }
    spa_heat_ex_status_open = false;
  } else {
    Serial.print(F("# alert error in close_spa_heat_exchanger_valve() "));
    if (opening) {
      Serial.print(F("opening "));
    }
    if (closing) {
      Serial.print(F("closing "));
    }
    valve_error = true;
  }
}

// This task is just enabled while the valve is closing, normally a 12 second process.
// It checks to see if the vavle signals that it has closeed, at which time it stops the valve
// power and disables itself.  If 15 seconds elpases without this signal, it also stops
// the valve power and disables itself (and signals an valve motion error).

void monitor_valve_closing_callback(void)
{
  check_free_memory(F("monitor_valve_closing.."));
  unsigned long current_time = millis();
  
  if ((current_time - valve_motion_start_time) > 15000) {
    // We turn off the close-valve process after 15 seconds, as it should take just 13 seconds
   Serial.println(F("# alert timeout on valve closing"));
   valve_timeout = true;
  }

  if (spa_heat_ex_valve_status_closed() || valve_timeout) {
    if (quad_lv_relay != 0) {
      quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    } else {
      // We should never get here, if the relay is gone, we should not have
      // enabled this task.
      record_error(F("# monitor_valve_closing_callback()"));
    }
    monitor_valve_closing.disable();
    valve_motion_start_time = 0;
    if (valve_timeout == false) {
      valve_error = false;           // Reset error flag on successful closing of valve
    } else {
      valve_status_failed = true;   // Assume the problem is with the status circuit and not the valve itself
    }
    spa_heat_ex_status_closed = true;  // Backup flag in case status circuit fails
  }
}

// This is called every ten seconds and decides whether to open the valve allowing the spa
// to be heated with solar, to close the valve, or to leave it alone.
// The first bit of code handles the unlikely case where the valve status does not
// show the valve to be fully open or fully closed and it attempts to close it.

void monitor_spa_valve_callback(void)
{ 
  check_free_memory(F("monitor_spa_valve.."));
  // The status from the valve is inconsistent, maybe it got stuck between opening and closing
  // when a reboot happened.  This should be extremely rare.
  bool valve_not_in_motion = (monitor_valve_opening.isEnabled() == false) && (monitor_valve_closing.isEnabled() == false);
  if (operating_mode != m_spa_hex_valve) {
    if (spa_heat_ex_valve_status_open() == false && spa_heat_ex_valve_status_closed() == false &&
        valve_not_in_motion &&
        valve_status_failed == false) {
          // Try to rectify the situation by asking for the valve to close.
      Serial.println(F("# alert valve status neither open nor closed, attempting to close"));
      close_spa_heat_exchanger_valve(F("recover from inconsistent valve state"));
      monitor_spa_valve.delay(15000);
      if (spa_heat_ex_valve_status_closed() && spa_heat_ex_valve_status_open() == false) {
        Serial.println(F("# alert successfully closed valve"));
      } else {
        Serial.println(F("# alert failed to close valve"));
        valve_status_failed = true;
      }
    }
  }

  if (temps[tank_e].temperature_valid) {

    // If the following are true, open the spa valve so that the spa water is warmed from solar
    // 1. the spa is calling for heat
    // 2. The tank temperature reading is valid and the tank is at 113F or above
    // 3. the spa valve is closed
    // 4. We are in normal operation mode
    // It is ok if the electric heater is on too

    if (spa_calling_for_heat()) {
      if (temps[tank_e].temperature_F >= 113 &&
          spa_heat_ex_valve_status_open() == false &&
          valve_not_in_motion && operating_mode == m_oper) {
        open_spa_heat_exchanger_valve();
      } else if (spa_heat_ex_valve_status_closed() == false && (temps[tank_e].temperature_F <= 110) &&
          valve_not_in_motion) {

        // Close the valve if the tank is too cool to be effective at heating the spa even though
        // the spa is calling for heat.
        close_spa_heat_exchanger_valve(F("low tank temperature"));
      }
    } else {

      // If the following are true, close the heat exchanger valve
      // 1. spa is not calling for heat
      // 2. the spa valve is open (or, at least, not closed
      // 3. the valve is not in motion

      if (spa_heat_ex_valve_status_closed() == false && valve_not_in_motion) {
        close_spa_heat_exchanger_valve(F("no call for heat"));
      }
    }
  } else {
    // If the tank temperature isn't valid, play it safe by making sure the spa valve is closed.
    // The code that controls the spa's electric heater will turn it on in this case.
    if (spa_heat_ex_valve_status_closed() == false &&
        valve_not_in_motion) {
      close_spa_heat_exchanger_valve(F("tank temperature not valid"));
    }
  }
  check_free_memory(F("monitor_spa_valve.. exit"));
}

// Called every second to emit a line of telemetry if values of sensors change enough of if
// any key booleans change.

void print_periodic_header_and_summary_data(void)
{
  check_free_memory(F("print_periodic.."));
  static bool daily_stats_reset = false;
  snprintf(cbuf, sizeof(cbuf), "# %4u-%02u-%02u %02u:%02u:%02u ", 
        year(arduino_time),
        month(arduino_time), 
        day(arduino_time), 
        hour(arduino_time),
        minute(arduino_time), 
        second(arduino_time));
  
  Serial.print(cbuf);
  unsigned sp_on = 0;
  if (solar_pump_on()) {
    sp_on = millis() / 1000 - solar_pump_on_time;
  }
  unsigned eh_on = 0;
  if (spa_heater_relay_on()) {
    eh_on = millis() / 1000 - spa_heater_relay_on_time;
  }
  unsigned t_on = 0;
  if (takagi_on()) {
    t_on = millis() / 1000 - takagi_on_time;
  }

  snprintf(cbuf, sizeof(cbuf), " solarthermal %u %u %u %u ",
    daily_valve_cycles, 
    daily_seconds_of_solar_pump_on_time + sp_on,
    daily_seconds_of_spa_heater_on_time + eh_on, 
    daily_seconds_of_takagi_on_time + t_on);
  
  Serial.print(cbuf);
  Serial.println(F("built: " __DATE__ " " __TIME__));

  if (hour(arduino_time) == 0) {
    if (!daily_stats_reset) {
      daily_valve_cycles = 0;
      daily_seconds_of_solar_pump_on_time = 0;
      daily_seconds_of_spa_heater_on_time = 0;
      daily_seconds_of_takagi_on_time = 0;
      daily_stats_reset = true;
    } else {
      daily_stats_reset = false;
    }
  }
  Serial.println(F("# Date     Time     Tank LPan RPan Aveg SpaT SpaH Spmp Rpmp Taka Call Open Clsd Time Erro"));
}
// Called periodically.  More frequently after starting, then slows.
//  Sends relevant telemetry back over the USB-serial link.
void print_status_to_serial_callback(void)
{
  check_free_memory(F("print_status_to.."));
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
  static bool last_spa_closed;
  
  // Only print a line if the tank's temperature has shifted by more than on degree, the average panel temp by more than 2
  // or any of the booleans.

  if (abs(last_tank_F - (int)temps[tank_e].temperature_F) > 1 ||
      abs(last_avg_panel_F - (int)average_panel_temperature_F) > 2 ||
      last_spa_heat != spa_heater_relay_on() ||
      last_solar_pump != solar_pump_on() ||
      last_recirc != recirc_pump_on() ||
      last_spa_call != spa_calling_for_heat() ||
      last_spa_open != spa_heat_ex_valve_status_open() ||
      last_spa_closed != spa_heat_ex_valve_status_closed() ||
      records_skipped++ > 1200) {

    last_tank_F = (int)temps[tank_e].temperature_F;
    last_avg_panel_F = (int)average_panel_temperature_F;
    last_spa_heat = spa_heater_relay_on();
    last_solar_pump = solar_pump_on();
    last_recirc = recirc_pump_on();
    last_spa_call = spa_calling_for_heat();
    last_spa_open = spa_heat_ex_valve_status_open();
    last_spa_closed = spa_heat_ex_valve_status_closed();

    // We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
    // read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.

    // Sample output
    // # Date     Time Year Mode Tank Panel SpaT SpaH Spump Rpump Taka Call Open Clsd Time Erro
    // Mar 25 10:23:24 2025 Oper  138  157    99    0     0     0    0    0    0    1    0    0
    snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ", 
        year(arduino_time),
        month(arduino_time), 
        day(arduino_time), 
        hour(arduino_time),
        minute(arduino_time), 
        second(arduino_time));

    Serial.print(cbuf);
    snprintf(cbuf, sizeof(cbuf), "%4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d %4d",     
        (int)temps[tank_e].temperature_F,
        (int)temps[left_panel_e].temperature_F,
        (int)temps[right_panel_e].temperature_F,
        (int)average_panel_temperature_F,
        (int)temps[spa_e].temperature_F,
        spa_heater_relay_on(),
        solar_pump_on(),
        recirc_pump_on(),
        takagi_on(),
        spa_calling_for_heat(),
        spa_heat_ex_valve_status_open(),
        spa_heat_ex_valve_status_closed(),
        valve_timeout,
        valve_error);
    
    Serial.println(cbuf);
    line_counter--;
    records_skipped = 0;
  }
}

// Called frequently to poll for and act on received console input.  This recognizes characters
// that correspond to keys on the controller, 's' for select, '+' for the plus key, and '-' for the minus key.

void monitor_serial_console_callback(void)
{
  check_free_memory(F("monitor_serial_console_.."));
  cbuf[0] = '\0';

  while (Serial.available() > 0) {  // Check if data is available to read
    char received_char = Serial.read();  // Read one character
    int l = strlen(cbuf);
    if (l >= sizeof(cbuf) - 1) {
      Serial.println(F("# command buffer overflow"));
      break;
    }
    if (received_char == '\n') {
      Serial.print(F("# Recieved: "));
      Serial.println(cbuf);

      switch (cbuf[0]) {
        case 's':
          select_key_pressed = true;
          break;

        case '+':
          plus_key_pressed = true;
          break;

        case '-':
          minus_key_pressed = true;
          break;

        default:
          Serial.print(F("# Unknown command: "));
          Serial.println(cbuf);
          Serial.println(F("# choices are: 's', +, -"));
          break;
      }
      break;
    } else {
      cbuf[l] = received_char;
      cbuf[l+1] = '\0';
    }
  }
}

void print_2_digits_to_lcd(int number)
{
  if (lcd != 0) {
    if (number >= 0 && number < 10) {
      lcd->write('0');
    }
    lcd->print(number);
  }
}

// This is called several times a second and updates the LCD display with the latest values and status.

void update_lcd_callback(void)
{
  check_free_memory(F("update_lcd.."));
  if (lcd != 0) {
    char valve_cbuf[5];
    unsigned seconds_now = millis() / 1000;
    
    if (fail_message_time != 0 && (seconds_now - fail_message_time) > 64000) {
      fail_message_time = 0;
    } else {
      lcd->setCursor(0 /* column */, 0 /* row */);
      if (valve_motion_start_time > 0) {
        snprintf(valve_cbuf, sizeof(valve_cbuf), "%4ul", (millis() - valve_motion_start_time) / 1000UL);
      } else {
        if (spa_heat_ex_valve_status_open()) {
          strncpy(valve_cbuf, "Open", sizeof(valve_cbuf));
        } else if (spa_heat_ex_valve_status_closed()) {
          strncpy(valve_cbuf, "Clsd", sizeof(valve_cbuf));
        } else {
          strncpy(valve_cbuf, "????", sizeof(valve_cbuf));
        }
      }
      snprintf(cbuf, sizeof(cbuf), "%02d:%02d:%02d %4s%c%c%4s ", 
        hour(arduino_time), minute(arduino_time), second(arduino_time), 
        valve_cbuf,
        valve_timeout ? 'T' : '-',
        valve_error ? 'E' : '-',
        operating_mode_to_string(operating_mode));
      lcd->print(cbuf);
    }

    lcd->setCursor(0, 1);
    snprintf(cbuf, sizeof(cbuf), "T %3d LP %3d RP %3d", 
             (int)temps[tank_e].temperature_F, (int)temps[left_panel_e].temperature_F, (int)temps[right_panel_e].temperature_F);
    lcd->print(cbuf);
    
    lcd->setCursor(0, 2);
    snprintf(cbuf, sizeof(cbuf), "RPump%c SPump%c Tak%c ", 
            recirc_pump_on() ? '+' : '-', solar_pump_on() ? '+' : '-', takagi_on() ? '+' : '-');
    lcd->print(cbuf);
    
    snprintf(cbuf, sizeof(cbuf), "S %3d Sele%c Scal%c ", 
            (int)temps[spa_e].temperature_F, spa_heater_relay_on() ? '+' : '-', spa_calling_for_heat() ? '+' : '-');
    lcd->setCursor(0,3);
    lcd->print(cbuf);
  }
}

// Returns true if the reciculation pump is on, false otherwise
bool recirc_pump_on(void)
{
  return digitalRead(SSR_RECIRC_PUMP_PIN) == LOW;
}

void turn_recirc_pump_on(void) 
{
  digitalWrite(SSR_RECIRC_PUMP_PIN, LOW); // Ground the low side of the SSR, turning on recirculation pump
}

void turn_recirc_pump_off(void) 
{
  digitalWrite(SSR_RECIRC_PUMP_PIN, HIGH); // Un-Ground the low side of the SSR, turning off recirculation pump
}

// This controls the recirculation pump SSR and is currently simple, it just runs for one minute every 30 minutes
// between 8AM and 11PM.

void monitor_recirc_pump_callback(void)
{
  check_free_memory(F("monitor_recirc_.."));
  unsigned h = hour(arduino_time);
  unsigned m = minute(arduino_time);
  
// Turn the recirc pump on at the top of the hour and at the half hour for two minutes.  However, if the
// the Takagi is on, restrict the operating time to 8AM to 11PM

  if ((h >= 8 && h <= 23) || takagi_on() == false) {
    switch (m) {
      case 0: 
      case 30:
        if (recirc_pump_on() == false) {
          turn_recirc_pump_on();
        }
        break;

      case 2:
      case 32:
        if (recirc_pump_on() == true) {
          turn_recirc_pump_off();
        }
        break;

      default:
        break;
    }
  }
}

bool takagi_on()
{
  return digitalRead(SSR_TAKAGI_PIN) == LOW;
}

void turn_takagi_on(void)
{
  takagi_on_time = millis() / 1000;
  digitalWrite(SSR_TAKAGI_PIN, LOW); // Turn on Takagi flash heater
}

void turn_takagi_off(void)
{
  if (takagi_on_time > 0) {
    daily_seconds_of_takagi_on_time += millis() / 1000 - takagi_on_time;
    takagi_on_time = 0;
  }
  digitalWrite(SSR_TAKAGI_PIN, HIGH); // Turn off Takagi flash heater
}

void monitor_takagi_callback(void)
{
  check_free_memory(F("monitor_takagi_"));
  static unsigned long last_takagi_change = 0;

  if (operating_mode == m_oper || operating_mode == m_safe) {
    if (temps[tank_e].temperature_valid) {
      if (takagi_on() && temps[tank_e].temperature_F >= takagi_off_threshold_F) {
        turn_takagi_off();
      } else if (takagi_on() == false && temps[tank_e].temperature_F <  takagi_on_threshold_F) {
        unsigned long current_time = millis() / 1000;
        if (last_takagi_change == 0 || (current_time - last_takagi_change) > TAKAGI_DELAY) {
          turn_takagi_on();
          last_takagi_change = current_time;
        }
      }
    } else {
      // If the tank temperature isn't valid, then just turn on the Takagi and leave it on
      turn_takagi_on();
    }
  }
}

// Once an hour, read the RTC and set Arduino time based on it.
void monitor_clock_callback(void)
{
  check_free_memory(F("monitor_clock.."));
  arduino_time = now();
  
  DateTime rtc_now = rtc.now();
  int rtc_h = rtc_now.hour();
  int rtc_m = rtc_now.minute();
  int rtc_s = rtc_now.second();
  int rtc_d = rtc_now.day();
  int rtc_mo = rtc_now.month();
  int rtc_y = rtc_now.year();

  int arduino_h = hour(arduino_time);
  int arduino_m = minute(arduino_time);
  int arduino_s = second(arduino_time);
  int arduino_d = day(arduino_time);
  int arduino_mo = month(arduino_time);
  int arduino_y = year(arduino_time);
  
  if (rtc_y != arduino_y || rtc_mo != arduino_mo || rtc_d != arduino_d ||
   rtc_h != arduino_h || rtc_m != arduino_m || rtc_s != arduino_s) {
    
    if (verbose_rtc) {
      
      snprintf(cbuf, sizeof(cbuf), "# RTC: %4u-%02u-%02u %02u:%02u:%02u", 
            rtc_y, rtc_mo, rtc_d, rtc_h, rtc_m, rtc_s);
      Serial.println(cbuf);

      snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u ", 
        year(arduino_time),
        month(arduino_time), 
        day(arduino_time), 
        hour(arduino_time),
        minute(arduino_time), 
        second(arduino_time));
      Serial.println(cbuf);
   }

    // Set Arduino time to RTC time
    setTime(rtc_h, rtc_m, rtc_s, rtc_d, rtc_mo, rtc_y);
  }
}

void setup_arduino_pins(void)
{
  pinMode(KEY_1_PIN, INPUT_PULLUP);
  pinMode(KEY_2_PIN, INPUT_PULLUP);
  pinMode(KEY_3_PIN, INPUT_PULLUP);
  pinMode(KEY_4_PIN, INPUT_PULLUP);

  turn_solar_pump_off();
  pinMode(SSR_SOLAR_PUMP_PIN, OUTPUT); 

  turn_recirc_pump_off();
  pinMode(SSR_RECIRC_PUMP_PIN, OUTPUT);

  turn_takagi_off();
  pinMode(SSR_TAKAGI_PIN, OUTPUT);
  
  pinMode(SPA_HEAT_EX_VALVE_STATUS_OPEN_PIN, INPUT_PULLUP);
  pinMode(SPA_HEAT_EX_VALVE_STATUS_CLOSED_PIN, INPUT_PULLUP);

  pinMode(SPA_HEAT_DIGITAL_IN_PIN, INPUT_PULLUP);
  
  pinMode(LED_BUILTIN, OUTPUT); 
}

void setup_temperature_sensors()
{
  temps[tank_e].input_pin = A0;            // Measures the voltage from the LM35 glued to the tank
  temps[tank_e].calibration_offset_F = 15; // Calibrated by comparing with the 18B20 that has been on the tank for years
  temps[tank_e].lower_bound_F = 40;
  temps[tank_e].upper_bound_F = 190;

  temps[left_panel_e].input_pin = A1;  // Leftmost solar panel temperature
  temps[left_panel_e].calibration_offset_F = 3; 
  temps[left_panel_e].lower_bound_F = 10;
  temps[left_panel_e].upper_bound_F = 280;

  temps[spa_e].input_pin = A2;             // Temperature of metal tube in spa after recirc pump
  temps[spa_e].calibration_offset_F = 14;  // Correct when  panels are 34F, dynamic correction in sample routine
  temps[spa_e].lower_bound_F = 32;
  temps[spa_e].upper_bound_F = 120;

  temps[right_panel_e].input_pin = A3;    // Rightmost solar panel temperature
  temps[right_panel_e].calibration_offset_F = 2; 
  temps[right_panel_e].lower_bound_F = 10;
  temps[right_panel_e].upper_bound_F = 280;

  // Give approximate initial values to the temperature readings so that at start up, we don't
  // have values that are way off (we don't want to wait until the EMA catches up).  Run this
  // many times and take the last reading.
  const bool need_for_seed = true;
  if (need_for_seed) {
    const int seed_samples = 50;
    for (int i = 0 ; i < seed_samples ; i++ ) {
      for (struct temperature_s *t = &temps[0] ; t < &temps[last_temp_e] ; t++) {
        int adc_value = analogRead(t->input_pin);
        delay(2);   // Let ADC settle
        float voltage = (float)adc_value * (5.0 / 1023.0);
        float temp_C = (voltage - 0.5) * 100.0; /* LM36 gives voltage of 0.5 for 0C and 10mv per degree C */
        t->temperature_F += temp_C * (90.0 / 50.0) + 32.0 + t->calibration_offset_F;
      }
    }
   
    for (struct temperature_s *t = &temps[0] ; t < &temps[last_temp_e] ; t++) {
      t->temperature_F /= seed_samples;
      
      t->temperature_valid = t->temperature_F >= t->lower_bound_F && t->temperature_F <= t->upper_bound_F;
    }
    average_panel_temperature_F = (temps[left_panel_e].temperature_F + temps[right_panel_e].temperature_F) / 2.0;
  
    // Hack for spa temperature probe, which is not perfectly in the stream of spa water,
    // but is affected by outside temperature.  When the panels are 34F, the spa temp is correctly
    // set with the calibration_offset_F value we set in setup().  When the panels are 169, that's when
    // it is warm outside and the spa sensor reads 6F higher than actual.
    int correction_F = ((average_panel_temperature_F - 34.0) / 135.0) * 6.0;
    temps[spa_e].temperature_F -= correction_F;
  }
}
void setup_lcd(void)
{
  if (lcd != 0) {
    if (fast_lcd_comm) {
      Wire.beginTransmission(SERLCD_I2C_ADDR);
      Wire.write(0x7C);      // Special command indicator
      Wire.write(0x2B);      // Change baud rate command
      Wire.write(4);         // 4 = 115200 baud (see table below)
      Wire.endTransmission();
    }

    lcd->begin(Wire, SERLCD_I2C_ADDR);         // Default I2C address of Sparkfun 4x20 SerLCD
    lcd->setBacklight(255, 255, 255);            // Green and blue backlight while booting
    lcd->setContrast(5);
    lcd->clear();
    
    lcd->setCursor(0 /* column */, 0 /* row */);
    lcd->print(F("SolarThermal "));  // Display this on the first row
    
    lcd->setCursor(0,1);
    lcd->print(F(__DATE__));         // Display this on the second row, left-adjusted
    
    lcd->setCursor(0, 2);
    lcd->print(F(__TIME__));         // Display this on the third row, left-adjusted
  }
}

void setup_i2c_bus(void)
{
  {
    byte error, address;
    int nDevices;
  
    nDevices = 0;
    for (address = 1; address < 127; address++ )
    {
      // The i2c_scanner uses the return value of
      // the Write.endTransmisstion to see if
      // a device did acknowledge to the address.
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
  
      if (error == 0)
      {
        if (verbose_I2C) {
          Serial.print(F("# I2C device found at address 0x"));
          if (address < 16) {
            Serial.print("0");
          } 
          Serial.print(address, HEX);
        }
        switch (address) {
         case SERLCD_I2C_ADDR:
           lcd = new SerLCD();
           if (verbose_I2C) {
             Serial.print(F(" (SerLCD 4x20)"));
           }
           break;

         case LV_RELAY_I2C_ADDR:
           if (verbose_I2C) {
             Serial.print(F(" (Quad Qwiic Relay)"));
           }
           quad_lv_relay = new Qwiic_Relay(address);
           if (quad_lv_relay->begin() == 0) {
             Serial.print(F("# Failure to start quad qwiic relay object"));
           }
           break;

        case RTC_I2C_ADDR:
          if (verbose_I2C) {            
            Serial.print(F(" (DS3231 RTC)"));
          }
          break;
          
         default:
          if (quad_lv_relay == (void *)0) {
            Serial.print(F(" (unexpected, will guess that it is the Quad Qwiic Relay at the wrong address)\n"));
            quad_lv_relay = new Qwiic_Relay(address);
            if (quad_lv_relay->begin()) {
              Serial.print(F("# Wayward Qwiic relay found, remapping it to where it is suppose to be\n"));
              quad_lv_relay->changeAddress(LV_RELAY_I2C_ADDR);
            } else {
              Serial.print(F("# unexpected device, unable to treat it as a quad qwiic relay\n"));
            }
          } else {
            Serial.print(F("# unexpected device after finding Qwiic quad relay\n"));
          }
        }
        if (verbose_I2C) {
          Serial.print("\n");
        }
  
        nDevices++;
      } else if (error == 4) {
        Serial.print(F("# Unknown error at address 0x"));
        if (address < 16) {
          Serial.print("0");
        }
        Serial.println(address, HEX);
      }
    }
    if (nDevices == 0) {
      Serial.println(F("# No I2C devices found\n"));
    }
  }
  if (quad_lv_relay == (void *)0) {
    record_error(F("REL LV"));
  }
}

void setup_rtc(void)
{
  // If we fail to communicate with the RTC (a DS3231), set the time and date to the build time of this software.
  if(!rtc.begin()) {
    record_error(F("No RTC"));
    int hh, mmin, ss, dd, mm, yy;
    char monthStr[4];
    sscanf(__TIME__, "%d:%d:%d", &hh, &mmin, &ss);
    sscanf(__DATE__, "%s %d %d", monthStr, &dd, &yy);
    int month = (strcmp(monthStr, "Jan") == 0) ? 1 :
                  (strcmp(monthStr, "Feb") == 0) ? 2 :
                  (strcmp(monthStr, "Mar") == 0) ? 3 :
                  (strcmp(monthStr, "Apr") == 0) ? 4 :
                  (strcmp(monthStr, "May") == 0) ? 5 :
                  (strcmp(monthStr, "Jun") == 0) ? 6 :
                  (strcmp(monthStr, "Jul") == 0) ? 7 :
                  (strcmp(monthStr, "Aug") == 0) ? 8 :
                  (strcmp(monthStr, "Sep") == 0) ? 9 :
                  (strcmp(monthStr, "Oct") == 0) ? 10 :
                  (strcmp(monthStr, "Nov") == 0) ? 11 : 12;

    setTime(hh, mmin, ss, dd, month, yy);
    monitor_clock.disable();
  } else {
    if(rtc.lostPower() || force_RTC_reload_from_build_time ) {
      // this will adjust to the date and time at compilation
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

      // Add 15 seconds to compensate for the time to upload the sketch
      DateTime newTime = rtc.now() + TimeSpan(0, 0, 0, 15);  

    // Update the RTC
      rtc.adjust(newTime);

      Serial.println(F("# Setting RTC from build time"));
    }
    //we don't need the 32K Pin, so disable it
    rtc.disable32K();
    // set alarm 1, 2 flag to false (so alarm 1, 2 didn't happen so far)
    // if not done, this easily leads to problems, as both register aren't reset on reboot/recompile
    rtc.clearAlarm(1);
    rtc.clearAlarm(2);

    // stop oscillating signals at SQW Pin
    // otherwise setAlarm1 will fail
    rtc.writeSqwPinMode(DS3231_OFF);

    // turn off alarm 2 (in case it isn't off already)
    // again, this isn't done at reboot, so a previously set alarm could easily go overlooked
    rtc.disableAlarm(2);

    DateTime now = rtc.now();
    int h = now.hour();
    int m = now.minute();
    int s = now.second();
    int d = now.day();
    int mo = now.month();
    int y = now.year();
    setTime(h, m, s, d, mo, y);

    if (verbose_rtc) {
      Serial.print(F("# Set time from RTC: "));
      snprintf(cbuf, sizeof(cbuf), "%4u-%02u-%02u %02u:%02u:%02u");
      Serial.println(cbuf);
    }
  }
}
/*
   This is the function that the Arudino run time system calls once, just after start up.  We have to set the
   pin modes of the ATMEGA correctly as inputs or outputs.  We also fetch values from EEPROM for use during
   our operation and emit a startup message.
*/
void setup(void)
{
  setup_arduino_pins();
  analogReference(DEFAULT);

  Wire.begin();
  Wire.setClock(400000); 

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag
  Serial.println(F("# alert reboot"));              // Put the first line we print on a fresh line (i.e., left column of output)
  
  setup_i2c_bus(); // This sets "quad_lv_relay"
    
  // Ensure that the motorized valve is unpowered
  if (quad_lv_relay != 0) {
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);

    // By default, the spa electric heater is unpowered.  If the tank is too
    // cold, it will quickly be turned back on.
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
  }
  setup_lcd();
  setup_rtc();

  if (poll_keys_bool == false) {
    PCICR |= (1 << PCIE2);                                        // Enable Pin Change Interrupt for PORTD  
    PCICR &= ~((1 << PCIE0) | (1 << PCIE1));                      // Disable PCINT0 (PORTB) and PCINT1 (PORTC)
    PCMSK2 |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);  // Enable for D4–D7

    sei();  // Enable global interrupts
  }

  if (false /* debug option */) {
    Serial.print(F("# PCICR="));
    Serial.println(PCICR);
    Serial.print(F("# PCMSK1="));
    Serial.print(PCMSK1);
    Serial.print(F("PCMSK2="));
    Serial.println(PCMSK2);
  }
  arduino_time = now();  // should not be necessary
  setup_temperature_sensors();
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop(void)
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}
