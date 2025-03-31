
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
SerLCD lcd;
#define SERLCD_I2C_ADDR (0x72)
const bool fast_lcd_comm = false;

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
  int temperature_F;
  bool temperature_valid;
  char input_pin;
  unsigned last_valid_time; // millis() / 1000 of last time temperature was valid
  int calibration_offset_F;;
  int lower_bound_F;
  int upper_bound_F;
} temps [last_temp_e];

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
Task update_lcd(900, TASK_FOREVER, &update_lcd_callback, &ts, true);
/*****************************************************************************************************/
// These global variables describe the state of the heat exchanger valve.  It can be open,
// closed, in the process of opening or closing, or in an indeterminate state after we attempted to
// open or close it.

unsigned long valve_motion_start_time = 0;

bool valve_timeout = false;         // True if it takes too long to open or close the spa heat exchanger valve
                                    // Reset to false on next (attempted) valve operation 

bool valve_error = false;           // True if the spa valve control API is called in a way that it does not expect
                                    // Reset to false on next completed valve operation
unsigned daily_valve_cycles;        // Number of valve-open operations per day

// These two booleans represent the latest values sensed from the valve itself.  

bool spa_heat_ex_valve_status_open = false;
bool spa_heat_ex_valve_status_closed = false;
bool spa_calling_for_heat = false;  // True if the spa is signalling that it wants heat

Task monitor_valve_closing(200, TASK_FOREVER, &monitor_valve_closing_callback, &ts, false);
Task monitor_valve_opening(200, TASK_FOREVER, &monitor_valve_opening_callback, &ts, false);
Task monitor_spa_valve(TASK_SECOND * 10, TASK_FOREVER, &monitor_spa_valve_callback, &ts, true);
/*****************************************************************************************************/

#define SOLAR_PUMP_DELAY       (300)         // Minimum number of seconds between solar pump on events

int panel_temperature_F;   // Average of leftmost and rightmost panels

// The panels must be at least this much hotter than the tank to turn on the solar pump
const unsigned tank_panel_difference_threshold_on_F = 10;

// When the panels drop to being just this much hotter than the tank, turn off the solar pump
const unsigned tank_panel_difference_threshold_off_F = -10; // Keep running pump until panels 5F colder than tank
bool solar_pump_on;                 // True if we are powering the solar tank hot water pump
unsigned long solar_pump_on_time;   // Set to millis() / 1000 when pump is turned on
unsigned daily_seconds_of_solar_pump_on_time; // Number of seconds the solar pump has run today

Task monitor_solar_pump(TASK_SECOND * 60, TASK_FOREVER, &monitor_solar_pump_callback, &ts, true);
/*****************************************************************************************************/
bool spa_heater_relay_on = false;             // True if we have enabled the spa heater relay, off by default
#define HEATER_RELAY_DELAY     (300)          // Minimum time, in seconds, between heater-on events
unsigned daily_seconds_of_spa_heater_on_time;
unsigned spa_heater_relay_on_time;            // Set to millis() / 1000 when heater is switched on
Task monitor_spa_electric_heat(TASK_SECOND * 60, TASK_FOREVER, &monitor_spa_electric_heat_callback, &ts, true);
/*****************************************************************************************************/
bool recirc_pump_on;                // True if we are powering the recirculation pump

void turn_recirc_pump_on(void);
void turn_recirc_pump_off(void);

Task monitor_recirc_pump(TASK_SECOND * 10, TASK_FOREVER, &monitor_recirc_pump_callback, &ts, true);
/*****************************************************************************************************/
#define TAKAGI_DELAY           (300)    // Minimum number of seconds between Tagaki on events

bool takagi_on;                         // True if we are powering the Takagi flash heater
const int takagi_on_threshold_F = 120;  // If the tank falls below this temperature, turn the Takagi on
const int takagi_off_threshold_F = 125; // If the tank is this or above, turn the Takagi off and let the solar mass do all the heating

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

/* Called by paths that sense an inconsistency, but we may be able to continue operating, so we don't
 * want to call fail().
 */
void record_error(const __FlashStringHelper *warning_message)
{
  Serial.print(F("# WARNING: "));
  Serial.println(warning_message);
  valve_error = true;
}

/*
   Called by failure paths that should never happen.  When we get the RS-485 input working, we'll allow the user
   to do things in this case and perhaps resume operation.
*/
void fail(const __FlashStringHelper *fail_message)
{
  Serial.print(F("FAIL: "));
  Serial.println(fail_message);
 
  lcd.setCursor(0, 3);
  lcd.print(F("Fail: "));
  lcd.print(fail_message);         // Display this on the third row, left-adjusted
  lcd.print(F("  "));
  delay(500); // Give the serial link time to propogate the error message before execution ends
  abort();
}

/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback(void)
{
  arduino_time = now();
  unsigned second_now = millis() / 1000;
  // EMA -- exponential moving average filter, courtesy of ChatGPT 4o
  const float alpha = 0.1; // Smoothing factor (0 = slow response, 1 = no filtering)

  for (struct temperature_s *t = &temps[0] ; t < &temps[last_temp_e] ; t++) {
    int adc_value = analogRead(t->input_pin);
    float voltage = (float)adc_value * (5.0 / 1023.0); 
    float temp_C = (voltage - 0.5) * 100.0;       /* LM36 gives voltage of 0.5 for 0C and 10mv per degree C*/ ;
    float temp_F = (temp_C * (90.0 / 50.0)) + 32.0 + t->calibration_offset_F;
    
    // If the temperature is valid, then let this sample's voltage be averaged with
    // previous samples.  If not, toss it out.
    if (temp_F >= t->lower_bound_F && temp_F <= t->upper_bound_F) {
      t->last_valid_time = second_now;
    } else {
      char buf[50];
      char v_str[10], c_str[10], f_str[10];

      dtostrf(voltage, 4, 3, v_str);
      dtostrf(temp_C, 4, 1, c_str);
      dtostrf(temp_F, 4, 1, f_str);

      snprintf(buf,sizeof(buf), "# OOR [%d] adc=%d v=%s C=%s F=%s", t - &temps[0], adc_value, v_str, c_str, f_str);
      Serial.println(buf);
    }
    t->temperature_F = alpha * temp_F + (1 - alpha) * t->temperature_F;

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
      panel_temperature_F = (temps[left_panel_e].temperature_F + temps[right_panel_e].temperature_F) / 2;
    } else {
      panel_temperature_F = temps[left_panel_e].temperature_F;
      static bool first_time = true;
      if (first_time) {
        char buf[50];
        snprintf(buf, sizeof(buf), "# right temp sensor failed t=%d", temps[right_panel_e].temperature_F);
        Serial.println(buf);
        first_time = false;
      }
    }
  } else {
    if (temps[right_panel_e].temperature_valid) {
      panel_temperature_F = temps[right_panel_e].temperature_F;
      static bool first_time = true;
      if (first_time) {
        Serial.println(F("# left temp sensors failed"));
        first_time = false;
      }
    } else {
      // Both temperature sensors have failed, print error and leave panel_temperature_F as it was´
      static bool first_time = true;
      if (first_time) {
        Serial.println(F("# both temp sensors failed"));
        first_time = false;
      }
    }
  }
  spa_calling_for_heat = digitalRead(SPA_HEAT_DIGITAL_IN_PIN) == LOW;

  spa_heat_ex_valve_status_open = digitalRead(SPA_HEAT_EX_VALVE_STATUS_OPEN_PIN) == LOW;
  spa_heat_ex_valve_status_closed = digitalRead(SPA_HEAT_EX_VALVE_STATUS_CLOSED_PIN) == LOW;
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
  unsigned long now = millis();
  
  if ((now - time_entering_diag_mode) > DIAG_MODE_TIMEOUT) {
    operating_mode = m_oper;
    monitor_diag_mode.disable();
  }
}

// This intiates the valve opening process by applying the correct polarity of power to the valve motor
// and by enabling the task that monitors the opening process.

void open_spa_heat_exchanger_valve(void)
{
  if (monitor_valve_opening.isEnabled() == false && monitor_valve_closing.isEnabled() == false) {
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    valve_motion_start_time = millis();
    valve_timeout = false;
    monitor_valve_opening.enable();
    daily_valve_cycles++;
  } else {
    valve_error = true;
  }
}

// This task is just enabled while the valve is opening, normally a 12 second process.
// It checks to see if the vavle signals that it has opened, at which time it stops the valve
// power and disables itself. If 15 seconds elpases without this signal, it also stops
// the valve power and disables itself (and signals an valve motion error).

void monitor_valve_opening_callback(void)
{
  unsigned long current_time = millis();
  if ((current_time - valve_motion_start_time) > 15000) {
    // We turn off the valve-opening process once the valve signal says it is open or
    // after 15 seconds, as it should just take 13 seconds
    valve_timeout = true;
  }
  if (spa_heat_ex_valve_status_open || valve_timeout) {
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    monitor_valve_opening.disable();
    valve_motion_start_time = 0;    
    if (valve_timeout == false) {
      valve_error = false;         // Reset error flag on successful opening of valve
    }
  }
}

// This intiates the valve closing ing process by applying the correct polarity of power to the valve motor
// and by enabling the task that monitors the opening process.

void close_spa_heat_exchanger_valve(void)
{
  if (monitor_valve_opening.isEnabled() == false && monitor_valve_closing.isEnabled() == false) {
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    valve_motion_start_time = millis();
    valve_timeout = false;
    monitor_valve_closing.enable();
  } else {
    valve_error = true;
  }
}

// This task is just enabled while the valve is closing, normally a 12 second process.
// It checks to see if the vavle signals that it has closeed, at which time it stops the valve
// power and disables itself.  If 15 seconds elpases without this signal, it also stops
// the valve power and disables itself (and signals an valve motion error).

void monitor_valve_closing_callback(void)
{
  unsigned long current_time = millis();
  if ((current_time - valve_motion_start_time) > 15000) {
    // We turn off the close-valve process after 15 seconds, as it should take just 13 seconds
   valve_timeout = true;
  }
  if (spa_heat_ex_valve_status_closed || valve_timeout) {
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    monitor_valve_closing.disable();
    valve_motion_start_time = 0;
    if (valve_timeout == false) {
      valve_error = false;           // Reset error flag on successful closing of valve
    }
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
  bool some_key_pressed = select_key_pressed | enter_key_pressed | plus_key_pressed | minus_key_pressed;

  if (select_key_pressed) {
    operating_mode = (operating_mode + 1) % m_last;
    select_key_pressed = false;
    lcd.setCursor(15 /* column */, 0 /* row */);
    lcd.print(operating_mode_to_string(operating_mode)); 
  }
 
  if (plus_key_pressed) {
    switch (operating_mode) {
      case m_oper:
      case m_safe:
        adjustTime(600);
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
        open_spa_heat_exchanger_valve();
        break;

      case m_spa_elec:
        turn_spa_heater_relay_on();
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
        break;

      case m_spump:
        turn_solar_pump_off();
        break;

      case m_takagi:
        turn_takagi_off();
        break;

      case m_spa_hex_valve:          
        close_spa_heat_exchanger_valve();
        break;

      case m_spa_elec:
        turn_spa_heater_relay_off();
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

void turn_solar_pump_on(void)
{
  solar_pump_on_time = millis() / 1000;
  digitalWrite(SSR_SOLAR_PUMP_PIN, LOW); // Turn on solar pump
  solar_pump_on = true;
}

void turn_solar_pump_off(void)
{
  daily_seconds_of_solar_pump_on_time += millis() / 1000 - solar_pump_on_time;
  digitalWrite(SSR_SOLAR_PUMP_PIN, HIGH); // Turn off solar pump
  solar_pump_on = false;
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
  if (operating_mode != m_oper) {
    return;
  }

  int h = hour(arduino_time);

  if (temps[left_panel_e].temperature_valid || temps[right_panel_e].temperature_valid) {
    if (temps[tank_e].temperature_valid) {
      if (solar_pump_on == false && temps[tank_e].temperature_F < 165 &&
          panel_temperature_F > (temps[tank_e].temperature_F + tank_panel_difference_threshold_on_F)) {
        static unsigned long last_solar_pump_change = 0;
        unsigned long current_time = millis() / 1000;
        if (last_solar_pump_change == 0 || ((current_time - last_solar_pump_change) > SOLAR_PUMP_DELAY)) {
          if (h <= 9 || h >= 20) {
            record_error(F("wrong time of day for solar pump on"));
          } else {
            turn_solar_pump_on();
            last_solar_pump_change = current_time;
          }
        }
      }

      if (solar_pump_on) {
        // If the tank is too hot in absolute valve or the tank is too warm in comparison to the panels,
        // then turn the solar pump off.
        if (temps[tank_e].temperature_F > 170 ||
            panel_temperature_F < (temps[tank_e].temperature_F + tank_panel_difference_threshold_off_F)) {
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
      if (solar_pump_on == false && panel_temperature_F > 170) {
        turn_solar_pump_on();
      }
      if (solar_pump_on && h >= 17) {
        turn_solar_pump_off();
      }
    }
  } else {
    // We can't use the panel temperature, so just turn on the solar pump in the late morning and turn it off in the late
    // afternoon
    if (solar_pump_on == false && h >= 11) {
      turn_solar_pump_on();
    }
    if (solar_pump_on && h >= 17) {
      turn_solar_pump_off();
    }
  }
}

void turn_spa_heater_relay_on(void)
{
  spa_heater_relay_on_time = millis() / 1000;
  quad_lv_relay->turnRelayOn(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
  spa_heater_relay_on = true;
}

void turn_spa_heater_relay_off(void)
{
  daily_seconds_of_spa_heater_on_time += (millis()/1000) - spa_heater_relay_on_time;
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
  spa_heater_relay_on = false;
}

// This contains the normal basic operational logic of this controller, comparing temperatures
// and deciding which pumps to turn on, how to set the spa heat exchanger valve, etc.

static unsigned last_valve_open_second;

void monitor_spa_electric_heat_callback(void)
{
  if (temps[tank_e].temperature_valid) {
    if (operating_mode == m_oper) {
    
      // If the tank is not warm enough to heat the spa, enable the relay we inserted in the spa controller 
      // that allows the spa to heat itself with its own electric heater.  If we are currently heating with
      // solar (the valve is open), then the threshold for turning on the spa electric heater is 110F.  Below
      // that temperature, the valve will close.  If the spa valve is already closed, then turn off the electric
      // heater if the tank is less than 113F.
    
      if (spa_heater_relay_on) {
        // If the tank is warm enough to heat the spa, ensure that its electric heater is off
      
        if (temps[tank_e].temperature_F >= 115) {
          turn_spa_heater_relay_off();
        }
      } else {
        unsigned second_now = millis() / 1000;
        // If the tank is too cool or if the spa valve has been open for more than 30 minutes, turn on the spa's electric heater
        if (temps[tank_e].temperature_F <= (spa_heat_ex_valve_status_closed ? 113 : 110) ||
           (spa_heat_ex_valve_status_open && (last_valve_open_second - second_now) > 1800)) {
            char buf[50];
            snprintf(buf, sizeof(buf), "# spaH=%d valve_open=%d valve_closed=%d time=%u",
              spa_heater_relay_on, spa_heat_ex_valve_status_open, spa_heat_ex_valve_status_closed, last_valve_open_second - second_now);
            Serial.println(buf);
          turn_spa_heater_relay_on();
        }
      }
    }
  } else {
    // If the tank temperature is not valid, play it safe and turn on the electric heater
    Serial.println(F("# tank temp not valid, turning on spa heater"));
    turn_spa_heater_relay_on();
  }
}

void monitor_spa_valve_callback(void)
{
  
  if (temps[tank_e].temperature_valid) {
    if (operating_mode == m_oper) {

      // If the following are true, open the spa valve so that the spa water is warmed from solar
      // 1. the electric heater is off (which means we intend to heat with solar
      // 2. the spa is calling for heat
      // 3. The tank temperature reading is valid and the tank is at 115F or above
      // 4. the apa valve is closed

      if (spa_calling_for_heat) {
        if (spa_heater_relay_on == false && 
            temps[tank_e].temperature_F >= 113 &&
            spa_heat_ex_valve_status_closed) {
          open_spa_heat_exchanger_valve();
          last_valve_open_second = millis() / 1000;
        }
      } else {

        // If the following are true, close the heat exchanger valve
        // 1. spa is not calling for heat, ensure that the spa's heat exchanger valve is closed
        // 2. the spa valve is open
        // or
        // 3. The tank temperature is valid and at or below 110F
        if (spa_heat_ex_valve_status_open || (temps[tank_e].temperature_F <= 110)) {
          close_spa_heat_exchanger_valve();
        }
      }
    }
  } else {
    // If the tank temperature isn't valid, play it safe by making sure the spa valve is closed.
    // The code that controls the spa's electric heater will turn it on in this case.
    if (spa_heat_ex_valve_status_open) {
      close_spa_heat_exchanger_valve();
    }
  }
}

// Called periodically.  More frequently after starting, then slows.
//  Sends relevant telemetry back over the USB-serial link.
void print_status_to_serial_callback(void)
{
  char data_buf[70];
  static char last_data_buf[70];
  static char line_counter = 0;
  static int duplicate_line_counter = 0;
  static bool daily_stats_reset = false;
  int h = hour(arduino_time);
  int m = minute(arduino_time);
  char date_buf[25];

  if (line_counter == 0) {
    snprintf(date_buf, sizeof(date_buf), "# %s %d %02d:%02d:%02d %4d ", 
        monthShortStr(month(arduino_time)), 
        day(arduino_time), 
        h,
        m, 
        second(arduino_time), 
        year(arduino_time));

    unsigned sp_on = 0;
    if (solar_pump_on) {
      sp_on = millis() / 1000 - solar_pump_on_time;
    }
    unsigned eh_on = 0;
    if (spa_heater_relay_on) {
      eh_on = millis() / 1000 - spa_heater_relay_on_time;
    }
    unsigned t_on = 0;
    if (takagi_on) {
      t_on = millis() / 1000 - takagi_on_time;
    }

    snprintf(data_buf, sizeof(data_buf), "solarthermal %u %u %u %u ",
      daily_valve_cycles, 
      daily_seconds_of_solar_pump_on_time + sp_on,
      daily_seconds_of_spa_heater_on_time + eh_on, 
      daily_seconds_of_takagi_on_time + t_on);
    Serial.print(date_buf);
    Serial.print(data_buf);
    Serial.println(F("built: " __DATE__ " " __TIME__));

    if (h == 0) {
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
    Serial.println(F("# Date     Time Year Mode Tank LPanl RPanl SpaT  SpaH Spump Rpump Taka Call Open Clsd Time Erro"));
  
    line_counter = 30;
  }

  // We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
  // read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.

  // Sample output
  // # Date     Time Year Mode Tank Panel SpaT SpaH Spump Rpump Taka Call Open Clsd Time Erro
  // Mar 25 10:23:24 2025 Oper  138  157    99    0     0     0    0    0    0    1    0    0

  snprintf(data_buf, sizeof(data_buf), "%s %4d %4d  %4d %4d  %4d  %4d %4d %4d %4d %4d %4d %4d",     
        operating_mode_to_string(operating_mode),
        temps[tank_e].temperature_F,
        temps[left_panel_e].temperature_F,
        temps[right_panel_e].temperature_F,
        temps[spa_e].temperature_F,
        spa_heater_relay_on,
        solar_pump_on,
        recirc_pump_on,
        takagi_on,
        spa_calling_for_heat,
        spa_heat_ex_valve_status_open,
        spa_heat_ex_valve_status_closed,
        valve_timeout,
        valve_error);
  if (strncmp(data_buf, last_data_buf, sizeof(data_buf)) || duplicate_line_counter > 600) {
    strncpy(last_data_buf, data_buf, sizeof(last_data_buf));
    char date_buf[23];
    snprintf(date_buf, sizeof(date_buf), "%s %d %02d:%02d:%02d %4d ", 
        monthShortStr(month(arduino_time)), 
        day(arduino_time), 
        h,
        m, 
        second(arduino_time), 
        year(arduino_time));
    
    Serial.print(date_buf);
    Serial.println(data_buf);
    line_counter--;
    duplicate_line_counter = 0;
  } else {
    duplicate_line_counter++;
  }
}

// Called frequently to poll for and act on received console input.  This recognizes characters
// that correspond to keys on the controller, 's' for select, '+' for the plus key, and '-' for the minus key.

void monitor_serial_console_callback(void)
{
  char command_buf[3];
  command_buf[0] = '\0';

  while (Serial.available() > 0) {  // Check if data is available to read
    char received_char = Serial.read();  // Read one character
    int l = strlen(command_buf);
    if (l >= sizeof(command_buf) - 1) {
      Serial.println(F("# command buffer overflow"));
      break;
    }
    if (received_char == '\n') {
      Serial.print(F("# Recieved: "));
      Serial.println(command_buf);

      switch (command_buf[0]) {
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
          Serial.println(command_buf);
          Serial.println(F("# choices are: 's', +, -"));
          break;
      }
      break;
    } else {
      command_buf[l] = received_char;
      command_buf[l+1] = '\0';
    }
  }
}

void print_2_digits_to_lcd(int number)
{
  if (number >= 0 && number < 10) {
    lcd.write('0');
  }
  lcd.print(number);
}

// This is called several times a second and updates the LCD display with the latest values and status.

void update_lcd_callback(void)
{
  if (second(arduino_time) > 3) {
    char cbuf[21];
    char valve_cbuf[5];
    
    lcd.setCursor(0 /* column */, 0 /* row */);
    if (valve_motion_start_time > 0) {
      snprintf(valve_cbuf, sizeof(valve_cbuf), "%4ul", (millis() - valve_motion_start_time) / 1000UL);
    } else {
      if (spa_heat_ex_valve_status_open) {
        strcpy(valve_cbuf, "Open");
      } else if (spa_heat_ex_valve_status_closed) {
        strcpy(valve_cbuf, "Clsd");
      } else {
        strcpy(valve_cbuf, "????");
      }
    }
    snprintf(cbuf, sizeof(cbuf), "%02d:%02d:%02d %4s%c%c%4s ", 
      hour(arduino_time), minute(arduino_time), second(arduino_time), 
      valve_cbuf,
      valve_timeout ? 'T' : '-',
      valve_error ? 'E' : '-',
      operating_mode_to_string(operating_mode));
    lcd.print(cbuf);
   
    lcd.setCursor(0, 1);
    snprintf(cbuf, sizeof(cbuf), "T %3d LP %3d RP %3d", temps[tank_e].temperature_F, temps[left_panel_e].temperature_F, temps[right_panel_e].temperature_F);
    lcd.print(cbuf);
    
    lcd.setCursor(0, 2);
    snprintf(cbuf, sizeof(cbuf), "RPump%c SPump%c Tak%c ", recirc_pump_on ? '+' : '-', solar_pump_on ? '+' : '-', takagi_on ? '+' : '-');
    lcd.print(cbuf);
    
    snprintf(cbuf, sizeof(cbuf), "S %3d Sele%c Scal%c ", temps[spa_e].temperature_F, spa_heater_relay_on ? '+' : '-', spa_calling_for_heat ? '+' : '-');
    lcd.setCursor(0,3);
    lcd.print(cbuf);
  }
}

void turn_recirc_pump_on(void) 
{
  recirc_pump_on = true;
  digitalWrite(SSR_RECIRC_PUMP_PIN, LOW); // Ground the low side of the SSR, turning on recirculation pump
}

void turn_recirc_pump_off(void) 
{
  recirc_pump_on = false;
  digitalWrite(SSR_RECIRC_PUMP_PIN, HIGH); // Un-Ground the low side of the SSR, turning off recirculation pump
}

// This controls the recirculation pump SSR and is currently simple, it just runs for one minute every 30 minutes
// between 8AM and 11PM.

void monitor_recirc_pump_callback(void)
{
  unsigned h = hour(arduino_time);
  unsigned m = minute(arduino_time);
  
// Between 8AM and 23:30, turn the recirc pump on at the top of the hour and at the half hour.
// Turn it off after a minute

  if (h >= 8 && h <= 23) {
    if (m == 0 || m == 30) {
      if (recirc_pump_on == false) {
        turn_recirc_pump_on();
      }
    } else if (m == 1 || m == 31) {
        // Leave the recirc pump on
      } else {
      if (recirc_pump_on) {
        turn_recirc_pump_off();
      }
    }
  }
}

void turn_takagi_on(void)
{
  takagi_on_time = millis() / 1000;
  digitalWrite(SSR_TAKAGI_PIN, LOW); // Turn on Takagi flash heater
  takagi_on = true;
}

void turn_takagi_off(void)
{
  daily_seconds_of_takagi_on_time += millis() / 1000 - takagi_on_time;
  digitalWrite(SSR_TAKAGI_PIN, HIGH); // Turn off Takagi flash heater
  takagi_on = false;
}

void monitor_takagi_callback(void)
{
  static unsigned long last_takagi_change = 0;

  if (operating_mode == m_oper || operating_mode == m_safe) {
    if (temps[tank_e].temperature_valid) {
      if (takagi_on && temps[tank_e].temperature_F >= takagi_off_threshold_F) {
        turn_takagi_off();
      } else if (takagi_on == false && temps[tank_e].temperature_F <  takagi_on_threshold_F) {
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
      char buf[40];
      snprintf(buf, sizeof(buf), "# RTC: %s %d %02d:%02d:%02d %4d", 
            monthShortStr(rtc_mo), rtc_d, rtc_h, rtc_m, rtc_s, rtc_y);  
      Serial.println(buf);

      snprintf(buf, sizeof(buf), "# Arduino Clock: %s %d %02d:%02d:%02d %4d", 
            monthShortStr(arduino_mo), arduino_d, arduino_h, arduino_m, arduino_s, arduino_y);  
      Serial.println(buf);
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
  temps[tank_e].calibration_offset_F = 14; // Calibrated by comparing with the 18B20 that has been on the tank for years
  temps[tank_e].lower_bound_F = 40;
  temps[tank_e].upper_bound_F = 190;

  temps[left_panel_e].input_pin = A1;  // Leftmost solar panel temperature
  temps[left_panel_e].calibration_offset_F = 3; 
  temps[left_panel_e].lower_bound_F = 10;
  temps[left_panel_e].upper_bound_F = 280;

  temps[spa_e].input_pin = A2;         // Temperature of metal tube in spa after recirc pump
  temps[spa_e].calibration_offset_F = 13;
  temps[spa_e].lower_bound_F = 32;
  temps[spa_e].upper_bound_F = 120;

  temps[right_panel_e].input_pin = A3;    // Rightmost solar panel temperature
  temps[right_panel_e].calibration_offset_F = 6; 
  temps[right_panel_e].lower_bound_F = 10;
  temps[right_panel_e].upper_bound_F = 280;

  // Give approximate initial values to the temperature readings so that at start up, we don't
  // have values that are way off (we don't want to wait until the EMA catches up)
  for (struct temperature_s *t = &temps[0] ; t < &temps[last_temp_e] ; t++) {
    int adc_value = analogRead(t->input_pin);
    float voltage = (float)adc_value * (5.0 / 1023.0);
    float temp_C = (voltage - 0.5) * 100.0; /* LM36 gives voltage of 0.5 for 0C and 10mv per degree C */
    t->temperature_F = temp_C * (90.0 / 50.0) + 32.0 + t->calibration_offset_F;
    t->temperature_valid = t->temperature_F >= t->lower_bound_F && t->temperature_F <= t->upper_bound_F;
  }
}
void setup_lcd(void)
{
  if (fast_lcd_comm) {
    Wire.beginTransmission(SERLCD_I2C_ADDR);
    Wire.write(0x7C);      // Special command indicator
    Wire.write(0x2B);      // Change baud rate command
    Wire.write(4);         // 4 = 115200 baud (see table below)
    Wire.endTransmission();
  }

  lcd.begin(Wire, SERLCD_I2C_ADDR);         // Default I2C address of Sparkfun 4x20 SerLCD
  lcd.setBacklight(255, 255, 255);            // Green and blue backlight while booting
  lcd.setContrast(5);
  lcd.clear();
  
  lcd.setCursor(0 /* column */, 0 /* row */);
  lcd.print(F("SolarThermal "));  // Display this on the first row
  
  lcd.setCursor(0,1);
  lcd.print(F(__DATE__));         // Display this on the second row, left-adjusted
  
  lcd.setCursor(0, 2);
  lcd.print(F(__TIME__));         // Display this on the third row, left-adjusted
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
    fail(F("REL LV"));
  }
}

void setup_rtc(void)
{
  // If we fail to communicate with the RTC (a DS3231), set the time and date to the build time of this software.
  if(!rtc.begin()) {
    Serial.println(F("# No RTC!"));
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
      char buf[41];
      Serial.print(F("# Set time from RTC: "));
      snprintf(buf, sizeof(buf), "%s %d %02d:%02d:%02d %4d", 
        monthShortStr(mo), d, h, m, s, y);
      Serial.println(buf);
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
  setup_temperature_sensors();

  setup_arduino_pins();
  analogReference(DEFAULT);
  Wire.begin();
  Wire.setClock(400000); 

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag
  Serial.println("# reboot");              // Put the first line we print on a fresh line (i.e., left column of output)
  
  setup_i2c_bus(); // This sets "quad_lv_relay"
    
  // Ensure that the motorized valve is unpowered
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);

  // By default, the spa electric heater is unpowered.  If the tank is too
  // cold, it will quickly be turned back on.
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_ELEC_HEAT_ENABLE);

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
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop(void)
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}
