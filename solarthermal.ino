
/*
   This controls:
      An SSR that controls a 120VAC water pump that moves water from the tank to the panels (and back)
      An SSR that controls the 120VAC water pump that recirculates hot water through our home
      An SSR that controls our Takagi natural-gas-fired domestic water heater
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
bool force_RTC_reload_from_build_time = false;

// We have two options for how to recognize pressing of one of the four keys.  One is to rapidly poll
// the state of the digital input pins to which the keys are connected.  The other is to enable any-input-change
// interrupts.  We have to use the polling method if we want to use certain libraries, e.g., SoftSerial.  Otherwise,
// using interrupts is preferable.

// We only use the #ifdef when we have to to avoid a linker error, otherwise we use an if statement to guard
// execution of the two alterative methods.

#undef POLL_KEYS
#ifdef POLL_KEYS
const bool poll_keys_bool = true;
#else
const bool poll_keys_bool = false;
#endif

const unsigned one_minute_in_milliseconds = 60000;
const unsigned five_seconds_in_milliseconds = 5000;
  
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

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

#include <TimeLib.h>     // for update/display of time

// #include <SoftwareSerial.h>

enum diag_mode_e {d_oper, d_rpump, d_spump, d_takagi, d_spa_hex_valve, d_spa_elec, d_last} diag_mode;

const unsigned long rs232_baud = 9600 ;

const byte rxPin = 2; // Wire this to Tx Pin of RS-232 level shifter
const byte txPin = 3; // Wire this to Rx Pin of RS-232 level shifter
 
// SoftwareSerial rs232 (rxPin, txPin);
bool send_to_rs232 = false;

//    This is for the 1307 RTC we installed on the Ocean Controls main board.
// #include <DS1307RTC.h>

//   All the operational code uses this time structure.  This is initialized at start time from the battery-backed up DS1307 RTC.
time_t arduino_time;

//   If we are able to read the DS1307 RTC over I2C, then we set this true and we can depend on
//  the time of day being valid.
bool time_of_day_valid = false;

bool solar_pump_on = false;         // True if we are powering the solar tank hot water pump

bool recirc_pump_on = false;        // True if we are powering the recirculation pump

bool takagi_on = true;              // True if we are powering the Takagi flash heater
const int takagi_on_threshold_F = 120;  // If the tank falls below this temperature, turn the Takagi on
const int takagi_off_threshold_F = 125; // If the tank is this or above, turn the Takagi off and let the solar mass do all the heating

bool spa_heater_relay_on = false;   // True if we have enabled the spa heater relay

bool spa_calling_for_heat = false;  // True if the spa is signalling that it wants heat

// THe next five global variables describe the state of the heat exchanger valve.  It can be open,
// closed, in the process of opening or closing, or in an indeterminate state after we attempted to
// open or close it.

unsigned long valve_motion_start_time = 0;

bool valve_timeout = false;         // True if it takes too long to open or close the spa heat exchanger valve
                                    // Reset to false on next (attempted) valve operation 

bool valve_error = false;           // True if the spa valve control API is called in a way that it does not expect
                                    // Reset to false on next completed valve operation

// These two booleans represent the latest values sensed from the valve itself.  

bool spa_heat_ex_valve_status_open = false;
bool spa_heat_ex_valve_status_closed = false;


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

void setup_time(void);

enum temps_e {tank_e, panel_e, spa_e, last_temp_e};
enum sensor_e {lm36_e, lm335_e};

struct temperature_s {
  int temperature_F;
  bool temperature_valid;
  char input_pin;
  int calibration_offset_F;
  enum sensor_e sensor_type;
  int lower_bound_F;
  int upper_bound_F;
} temps [last_temp_e];

void monitor_valve_closing_callback();
void monitor_valve_opening_callback();
void read_time_and_sensor_inputs_callback();
void print_status_to_serial_callback();
void poll_keys_callback();
void process_pressed_keys_callback();
void update_lcd_callback();
void frob_relays_callback();
void control_recirc_pump_callback();
void monitor_clock_callback();
void monitor_diag_mode_callback();
void monitor_serial_console_callback();

unsigned long time_entering_diag_mode = 0;
#define DIAG_MODE_TIMEOUT (600000) // 10 minutes


Task monitor_valve_closing(500, TASK_FOREVER, &monitor_valve_closing_callback, &ts, false);
Task monitor_valve_opening(500, TASK_FOREVER, &monitor_valve_opening_callback, &ts, false);
Task read_time_and_sensor_inputs(1000, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task poll_keys(25, TASK_FOREVER, &poll_keys_callback, &ts, poll_keys_bool);
Task process_pressed_keys(100, TASK_FOREVER, &process_pressed_keys_callback, &ts, !poll_keys_bool);

Task print_status_to_serial(TASK_SECOND * 5, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
Task update_lcd(TASK_SECOND, TASK_FOREVER, &update_lcd_callback, &ts, true);
Task frob_relays(TASK_SECOND * 10, TASK_FOREVER, &frob_relays_callback, &ts, true);
Task control_recirc_pump(TASK_SECOND * 30, TASK_FOREVER, &control_recirc_pump_callback, &ts, true);
Task monitor_clock(TASK_SECOND * 3600, TASK_FOREVER, &monitor_clock_callback, &ts, true);
Task monitor_diag_mode(TASK_SECOND * 60, TASK_FOREVER, &monitor_diag_mode_callback, &ts, false);
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
void read_time_and_sensor_inputs_callback()
{
  arduino_time = now();
  
  for (struct temperature_s *t = &temps[0] ; t < &temps[last_temp_e] ; t++) {
    unsigned temp_volts_raw = analogRead(t->input_pin);
    unsigned temp_millivolts = ((unsigned long)temp_volts_raw * 5000UL) / 1023UL;
    int temp_C_x10 = temp_millivolts - 500;
    if (t->sensor_type == lm335_e) {
      temp_C_x10 -= 2731;
    }
    
    t->temperature_F = (((((long)temp_C_x10 * 90L) / 50L) + 320L) / 10L) + t->calibration_offset_F;
    t->temperature_valid = t->temperature_F >= t->lower_bound_F && t->temperature_F <= t->upper_bound_F;
  }
  
  spa_calling_for_heat = digitalRead(SPA_HEAT_DIGITAL_IN_PIN) == HIGH;

  spa_heat_ex_valve_status_open = digitalRead(SPA_HEAT_EX_VALVE_STATUS_OPEN_PIN) == LOW;
  spa_heat_ex_valve_status_closed = digitalRead(SPA_HEAT_EX_VALVE_STATUS_CLOSED_PIN) == LOW;
}

char *diag_mode_to_string(enum diag_mode_e diag_mode) 
{
  switch (diag_mode) {
    case d_oper:          return "Oper";
    case d_rpump:         return "D-RP";
    case d_spump:         return "D-SP";
    case d_takagi:        return "D-TK";
    case d_spa_hex_valve: return "D-HX";    
    case d_spa_elec:      return "D-EL";
    default:              return "ERR";
  }
  return "E";
}

// This task is enabled when we enter diag mode.  It checks to see if we've been in diag mode
// too long and reverts to operational mode if so.
void monitor_diag_mode_callback()
{
  unsigned long now = millis();
  
  if ((now - time_entering_diag_mode) > DIAG_MODE_TIMEOUT) {
    diag_mode = d_oper;
    monitor_diag_mode.disable();
  }
}

void turn_recirc_pump_on() 
{
  recirc_pump_on = true;
  digitalWrite(SSR_RECIRC_PUMP_PIN, LOW); // Ground the low side of the SSR, turning on recirculation pump
}

void turn_recirc_pump_off() 
{
  recirc_pump_on = false;
  digitalWrite(SSR_RECIRC_PUMP_PIN, HIGH); // Un-Ground the low side of the SSR, turning off recirculation pump
}

void turn_solar_pump_on()
{
  digitalWrite(SSR_SOLAR_PUMP_PIN, LOW); // Turn on solar pump
  solar_pump_on = true;
}

void turn_solar_pump_off()
{
  digitalWrite(SSR_SOLAR_PUMP_PIN, HIGH); // Turn off solar pump
  solar_pump_on = false;
}

void turn_takagi_on()
{
  digitalWrite(SSR_TAKAGI_PIN, LOW); // Turn on Takagi flash heater
  takagi_on = true;
}

void turn_takagi_off()
{
  digitalWrite(SSR_TAKAGI_PIN, HIGH); // Turn off Takagi flash heater
  takagi_on = false;
}

void monitor_valve_closing_callback()
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
    if (print_status_to_serial.getInterval() < five_seconds_in_milliseconds) {
      print_status_to_serial.setInterval(five_seconds_in_milliseconds);
    }
    if (valve_timeout == false) {
      valve_error = false;           // Reset error flag on successful closing of valve
    }
  }
}

void monitor_valve_opening_callback()
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
    if (print_status_to_serial.getInterval() < five_seconds_in_milliseconds) {
      print_status_to_serial.setInterval(five_seconds_in_milliseconds);
    }
    if (valve_timeout == false) {
      valve_error = false;         // Reset error flag on successful opening of valve
    }
  }
}

const unsigned long one_second_in_milliseconds = 1000;

void open_spa_heat_exchanger_valve()
{
  Serial.println(F("# Open"));
  if (monitor_valve_opening.isEnabled() == false && monitor_valve_closing.isEnabled() == false) {
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    valve_motion_start_time = millis();
    valve_timeout = false;
    monitor_valve_opening.enable();
    if (print_status_to_serial.getInterval() > one_second_in_milliseconds) {
      print_status_to_serial.setInterval(one_second_in_milliseconds);
    }
  } else {
    valve_error = true;
  }
}

// We start the valve closing and then enable a task that will turn off the closing signal after 11 seconds.

void close_spa_heat_exchanger_valve()
{
  Serial.println(F("# Close"));
  if (monitor_valve_opening.isEnabled() == false && monitor_valve_closing.isEnabled() == false) {
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    valve_motion_start_time = millis();
    valve_timeout = false;
    monitor_valve_closing.enable();
    if (print_status_to_serial.getInterval() > one_second_in_milliseconds) {
      print_status_to_serial.setInterval(one_second_in_milliseconds);
    }
  } else {
    valve_error = true;
  }
}

void turn_spa_heater_relay_on()
{
   quad_lv_relay->turnRelayOn(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
   spa_heater_relay_on = true;
}

void turn_spa_heater_relay_off()
{
   quad_lv_relay->turnRelayOff(LV_RELAY_SPA_ELEC_HEAT_ENABLE);
   spa_heater_relay_on = false;
   
}

bool select_key_pressed = false;
bool enter_key_pressed = false;
bool plus_key_pressed = false;
bool minus_key_pressed = false;

#define PIN_CHANGE_INTERRUPT_VECTOR PCINT2_vect  // PCINT2 covers D4–D7
volatile unsigned long lastInterruptTime = 0;
const int debounce_delay = 150;  // 150ms debounce time

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

#ifndef POLL_KEYS
ISR(PIN_CHANGE_INTERRUPT_VECTOR) 
{
  poll_keys_callback();
}
#endif

// The interrupt routine above will set global variables indicating which keys have been pressed.
// In this function, we look at those global variables and take action required by the key presses
// and then reset those global variables.

void process_pressed_keys_callback()
{
  bool some_key_pressed = select_key_pressed | enter_key_pressed | plus_key_pressed | minus_key_pressed;

  if (select_key_pressed) {
    diag_mode = (diag_mode + 1) % d_last;
    select_key_pressed = false;
    lcd.setCursor(15 /* column */, 0 /* row */);
    lcd.print(diag_mode_to_string(diag_mode)); 
  }
 
  if (plus_key_pressed) {
    switch (diag_mode) {
      case d_oper:
        adjustTime(600);
        break;

      case d_rpump:
        turn_recirc_pump_on();
        break;

      case d_spump:
        turn_solar_pump_on();
        break;

      case d_takagi:
        turn_takagi_on();
        break;

      case d_spa_hex_valve:          
        open_spa_heat_exchanger_valve();
        break;

      case d_spa_elec:
        turn_spa_heater_relay_on();
        break; 
    }
    plus_key_pressed = false;
  }

  if (minus_key_pressed) {
    switch (diag_mode) {
      case d_oper:
        adjustTime(-600);
        break;

      case d_rpump:
        turn_recirc_pump_off();
        break;

      case d_spump:
        turn_solar_pump_off();
        break;

      case d_takagi:
        turn_takagi_off();
        break;

      case d_spa_hex_valve:          
        close_spa_heat_exchanger_valve();
        break;

      case d_spa_elec:
        turn_spa_heater_relay_off();
        break; 
    }
    minus_key_pressed = false;
  }
  if (some_key_pressed) {
//    Serial.println(F("# Some key pressed"));
    if (diag_mode == d_oper) {
      monitor_diag_mode.disable();
    } else {
      // When going into a diag mode, emit telemetry more often
      if (print_status_to_serial.getInterval() > five_seconds_in_milliseconds) {
        print_status_to_serial.setInterval(five_seconds_in_milliseconds);
      }
  //    Serial.println(F("# Enabling monitor_diag_mode\n"));
      monitor_diag_mode.enable();
      time_entering_diag_mode = millis();
    }
  } 
}

#define MINUTES_TO_MILLISECONDS(x) ((x) * 60UL * 1000UL)

#define SOLAR_PUMP_DELAY    MINUTES_TO_MILLISECONDS(5)
#define HEATER_RELAY_DELAY  MINUTES_TO_MILLISECONDS(5)
#define SPA_VALVE_DELAY     MINUTES_TO_MILLISECONDS(5)
#define TAKAGI_DELAY        MINUTES_TO_MILLISECONDS(5)

// This contains the normal basic operational logic of this controller, comparing temperatures
// and deciding which pumps to turn on, how to set the spa heat exchanger valve, etc.

void frob_relays_callback()
{
  static unsigned long last_solar_pump_change = 0;
  static unsigned long last_spa_heater_relay_change = 0;
  static unsigned long last_spa_valve_change = 0;
  const unsigned tank_panel_difference_threshold_F = 20;
  
  unsigned long current_time = millis();
  
  if (diag_mode == d_oper) {
    if (solar_pump_on == false && temps[panel_e].temperature_valid && temps[tank_e].temperature_valid && 
    temps[panel_e].temperature_F > (temps[tank_e].temperature_F + tank_panel_difference_threshold_F)) {
      if (last_solar_pump_change == 0 || ((current_time - last_solar_pump_change) > SOLAR_PUMP_DELAY)) {
        turn_solar_pump_on();
        last_solar_pump_change = current_time;
      }
    }
    if (solar_pump_on && temps[panel_e].temperature_valid && temps[tank_e].temperature_valid && 
          temps[panel_e].temperature_F < temps[tank_e].temperature_F) {
      if (last_solar_pump_change == 0 || (current_time - last_solar_pump_change) > SOLAR_PUMP_DELAY) { 
        turn_solar_pump_off();
        last_solar_pump_change = current_time;
      }  
    }
  
    // If the tank is not warm enough to heat the spa, enable the relay we inserted in the spa controller 
    // that passes the spa's call-for-heat signal to big power relay that controls the spa's electric heater
  
    if (spa_heater_relay_on == false && temps[tank_e].temperature_valid && temps[tank_e].temperature_F < 110) {
      if (last_spa_heater_relay_change == 0 || (current_time - last_spa_heater_relay_change) > HEATER_RELAY_DELAY) {
        turn_spa_heater_relay_on();
        last_spa_heater_relay_change = current_time;
      }
    }
  
    // If the tank is warm enough to heat the spa, ensure that its electric heater is off
    
    if (spa_heater_relay_on && temps[tank_e].temperature_valid && temps[tank_e].temperature_F >= 110) {
      if (last_spa_heater_relay_change == 0 || ((current_time - last_spa_heater_relay_change) > HEATER_RELAY_DELAY)) {
        turn_spa_heater_relay_off();
        last_spa_heater_relay_change = current_time;
      }
    }
  
    // If the electric heater is off and the spa is calling for heat, ensure that the spa's heat
    // exchanger pump is on.  
    if (spa_heater_relay_on == false /* we are heating with solar */) {
      if (spa_calling_for_heat && spa_heat_ex_valve_status_open == false) {
        if (last_spa_valve_change == 0 || ((current_time - last_spa_valve_change) > SPA_VALVE_DELAY)) {
          open_spa_heat_exchanger_valve();
          last_spa_valve_change = current_time;
        }
      } 
    }

    // If the spa is not calling for heat, ensure that the spa's heat exchanger valve is closed
    if (spa_heat_ex_valve_status_open && spa_calling_for_heat == false) {
      if (spa_heater_relay_on) {
        record_error(F("valve open and heater on"));
      }
      if (last_spa_valve_change == 0 || (current_time - last_spa_valve_change) > SPA_VALVE_DELAY) {
        close_spa_heat_exchanger_valve();
        last_spa_valve_change = current_time;
      }
    }
  }
}

// Called periodically.  More frequently after starting, then slows.
//  Sends relevant telemetry back over the USB-serial link.
void print_status_to_serial_callback(void)
{
  static char line_counter = 0;
  
  if (line_counter == 0) {
    if (print_status_to_serial.getInterval() < one_minute_in_milliseconds && millis() > 1000L * 60L * 10L) { 
      // After ten minutes, reduce the frequency of the output to once per minute
      print_status_to_serial.setInterval(one_minute_in_milliseconds);
    }

    Serial.println(F("# Date     Time Year Mode Tank Panel SpaT SpaH Spump Rpump Taka Call Open Clsd Time Erro"));
  
    line_counter = 20;
  } else {
    line_counter--;
  }

// We generate the output line in chunks, to conversve memory.  But it also makes the code easier to
// read because we don't have one humongous snprintf().  The size of buf is carefully chosen to be just large enough.

// Sample output
// # Date    Time Year Diag Tank Panel SpaT SpaH Spump Rpump Takagi
// Feb 21 13:04:33 2025  649   47   84    46    1     1     0    1
// Feb 21 13:04:38 2025  649   40   78    40    1     1     0    1

  char buf[101];  // Current output string is 89 characters, plus \0, one extra for good luck.
  snprintf(buf, sizeof(buf), "%s %d %02d:%02d:%02d %4d %s %4d %4d  %4d %4d  %4d  %4d %4d %4d %4d %4d %4d %4d", 
      monthShortStr(month(arduino_time)), 
      day(arduino_time), 
      hour(arduino_time),
      minute(arduino_time), 
      second(arduino_time), 
      year(arduino_time),
      diag_mode_to_string(diag_mode),
      temps[tank_e].temperature_F,
      temps[panel_e].temperature_F,
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
  
  Serial.println(buf);
}

void monitor_serial_console_callback()
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

void update_lcd_callback()
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
      diag_mode_to_string(diag_mode));
    lcd.print(cbuf);
   
    lcd.setCursor(0, 1);
    snprintf(cbuf, sizeof(cbuf), "T: %3d P: %3d S: %3d", temps[tank_e].temperature_F, temps[panel_e].temperature_F, temps[spa_e].temperature_F);
    lcd.print(cbuf);
    
    lcd.setCursor(0, 2);
    snprintf(cbuf, sizeof(cbuf), "RPump%c SPump%c Tak%c ", recirc_pump_on ? '+' : '-', solar_pump_on ? '+' : '-', takagi_on ? "+" : "-");
    lcd.print(cbuf);
    
    snprintf(cbuf, sizeof(cbuf), "Spa-elec%c Spa-call%c ", spa_heater_relay_on ? '+' : '-', spa_calling_for_heat ? '+' : '-');
    lcd.setCursor(0,3);
    lcd.print(cbuf);
  }
}


// This controls the recirculation pump SSR and is currently simple, it just runs for one minute every 30 minutes
// between 8AM and 11PM.

void control_recirc_pump_callback()
{
  static unsigned long last_takagi_change = 0;

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
  if (takagi_on && temps[tank_e].temperature_valid && temps[tank_e].temperature_F >= takagi_off_threshold_F) {
    unsigned long current_time = millis();
    if (last_takagi_change == 0 || (current_time - last_takagi_change) > TAKAGI_DELAY) {
        turn_takagi_off();
        last_takagi_change = current_time;
      }
  } else if (takagi_on == false && temps[tank_e].temperature_valid && temps[tank_e].temperature_F <  takagi_on_threshold_F) {
    unsigned long current_time = millis();
    if (last_takagi_change == 0 || (current_time - last_takagi_change) > TAKAGI_DELAY) {
      turn_takagi_on();
      last_takagi_change = current_time;
    }
  }
}


// Once an hour, read the RTC and set Arduino time based on it.
void monitor_clock_callback()
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
    
    char buf[80];
    snprintf(buf, sizeof(buf), "# RTC: %s %d %02d:%02d:%02d %4d", 
          monthShortStr(rtc_mo), rtc_d, rtc_h, rtc_m, rtc_s, rtc_y);  
    Serial.println(buf);

    snprintf(buf, sizeof(buf), "# Arduino Clock: %s %d %02d:%02d:%02d %4d", 
          monthShortStr(arduino_mo), arduino_d, arduino_h, arduino_m, arduino_s, arduino_y);  
    Serial.println(buf);

    // Set Arduino time to RTC time
    setTime(rtc_h, rtc_m, rtc_s, rtc_d, rtc_mo, rtc_y);
  }
}

void setup_arduino_pins()
{
  pinMode(KEY_1_PIN, INPUT_PULLUP);
  pinMode(KEY_2_PIN, INPUT_PULLUP);
  pinMode(KEY_3_PIN, INPUT_PULLUP);
  pinMode(KEY_4_PIN, INPUT_PULLUP);

  pinMode(SSR_SOLAR_PUMP_PIN, OUTPUT); 
  pinMode(SSR_RECIRC_PUMP_PIN, OUTPUT);
  pinMode(SPA_HEAT_EX_VALVE_STATUS_OPEN_PIN, INPUT_PULLUP);
  pinMode(SPA_HEAT_EX_VALVE_STATUS_CLOSED_PIN, INPUT_PULLUP);
  pinMode(SSR_TAKAGI_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT); 
}

void setup_lcd()
{
  Wire.beginTransmission(SERLCD_I2C_ADDR);
  Wire.write(0x7C);      // Special command indicator
  Wire.write(0x2B);      // Change baud rate command
  Wire.write(4);         // 4 = 115200 baud (see table below)
  Wire.endTransmission();

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

void setup_i2c_bus()
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
        Serial.print(F("# I2C device found at address 0x"));
        if (address < 16)
          Serial.print("0");
        Serial.print(address, HEX);
        switch (address) {
         case SERLCD_I2C_ADDR:
           Serial.print(F(" (SerLCD 4x20)"));
           break;

         case LV_RELAY_I2C_ADDR:
           Serial.print(F(" (Quad Qwiic Relay)"));
           quad_lv_relay = new Qwiic_Relay(address);
           if (quad_lv_relay->begin() == 0) {
             Serial.print(F("# Failure to start quad qwiic relay object"));
           }
           break;

        case RTC_I2C_ADDR:
          Serial.print(F(" (DS3231 RTC)"));
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
        Serial.print("\n");
  
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

void setup_rtc()
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

    char buf[80];
    snprintf(buf, sizeof(buf), "# Set time from RTC: %s %d %02d:%02d:%02d %4d", 
      monthShortStr(mo), d, h, m, s, y);
  
    Serial.println(buf);
  }
}
/*
   This is the function that the Arudino run time system calls once, just after start up.  We have to set the
   pin modes of the ATMEGA correctly as inputs or outputs.  We also fetch values from EEPROM for use during
   our operation and emit a startup message.
*/
void setup()
{
  delay(1000); // In case something further along crashes and we restart quickly, this will give a one-second pause


  temps[tank_e].input_pin = A0;  // Measures the voltage from the LM35 glued to the tank
  temps[tank_e].sensor_type = lm36_e;
  temps[tank_e].calibration_offset_F = 7; // Calibrated by comparing with 18B20 on tank for years
  temps[tank_e].lower_bound_F = 32;
  temps[tank_e].upper_bound_F = 200;

  temps[panel_e].input_pin = A1;  // Solar panel temperature
  temps[panel_e].sensor_type = lm335_e;
  temps[panel_e].calibration_offset_F = 88; // ?
  temps[panel_e].lower_bound_F = 20;
  temps[panel_e].upper_bound_F = 300;

  temps[spa_e].input_pin = A2;    // Temperature of vinyl tube in spa after recirc pump, before electric heater
  temps[spa_e].sensor_type = lm36_e;
  temps[spa_e].calibration_offset_F = 7; // Outside temperature around 80F, spa at 102F
  temps[spa_e].lower_bound_F = 32;
  temps[spa_e].upper_bound_F = 210;

  setup_arduino_pins();
  analogReference(DEFAULT);
  Wire.begin();
  Wire.setClock(400000); 

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag

  Serial.print(F("\n\r# Solarthermal "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));
 
  // Turn off the solar pump and the recirc pump
 
  digitalWrite(SSR_SOLAR_PUMP_PIN, HIGH);
  digitalWrite(SSR_RECIRC_PUMP_PIN, HIGH);

  takagi_on = true;
  digitalWrite(SSR_TAKAGI_PIN, LOW); // Turn on Takagi

  setup_lcd();
  delay(500);

  setup_i2c_bus(); // This sets "quad_lv_relay"
    
  // Ensure that the motorized valve is unpowered
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);

  // By default, the spa electric heater should be able to heat the spa
  quad_lv_relay->turnRelayOn(LV_RELAY_SPA_ELEC_HEAT_ENABLE);

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
void loop()
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}
