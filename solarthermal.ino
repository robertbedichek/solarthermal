
/*
   This controls:
      An SSR that controls a 120VAC water pump that moves water from the tank to the panels (and back)
      An SSR that controls the 120VAC water pump that recirculates hot water through our home
      An SSR that controls our Takagi natural-gas-fired domestic water heater
        The Takagi is fed by water that first goes through a heat exchanger in a 4000 pound thermal mass (water)
      A 12V signal that powers a small 12V relay added to the spa controller that enables the electrical spa heater (the "native" heater for the spa)

   This senses:
      Tank temperature via an LM35
      Panel temperature via an LM35
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

enum diag_mode_e {d_oper, d_rpump, d_spump, d_spa_hex_valve, d_spa_elec, d_last} diag_mode;

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

bool solar_pump_on = false;

bool recirc_pump_on = false;

bool spa_heater_relay_on = false;

bool spa_calling_for_heat = false;

// This boolean keeps track of whether the valve has most recently been commanded open or closed.
bool spa_heat_ex_valve_open = false;

// These two booleans represent the latest values sensed from the valve itself.  Because the valve over-closes
// we do not let it close for more than 11 seconds and that's not enough for the heat_ex_status_closed signal
// to signal.  So in normal operation, this signal is never asserted.

bool spa_heat_ex_valve_status_open = false;
bool spa_heat_ex_valve_status_closed = false;

bool takagi_on = true;

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
#define SERIAL_BAUD (38400)

/*
 * We have a Sparkfun SSR 4-relay board.  However, it is (1) not present on the I2C bus, and (2) obsolete.
 * We use three of the four relays, but we drive them directly from Arudino digital outputs.
 * To enable an SSR, we drive its control output to ground.
 */
#define SSR_SOLAR_PUMP  (12)    // writing '0' turns on solar pump
#define SSR_RECIRC_PUMP (11)    // writing '0' turns on the recirculation pump
// We have reused this pin, so it is not availble #define SSR_SPARE       (10)
#define SSR_TAKAGI      (9)    // writing '0' turns on the Takagi natural-gas fired water heater

/*
 * There is a motorized valve that I added to the spa.  When it is open, some of the water coming from the recirculation pump in the spa
 * will go through the stainless steel heat exchanger that I added to the solar tank.  This valve is controlled by two 12VDC signals.  When
 * the "open" signal is +12V and the "close" signal is ground, the valve will open.  When the "close" signal is +12VDC and the "open" signal
 * is ground, then the valve will close.  These two signals are enabled by relays 3 and 2 in the Sparkfun relay board.  Relay 1 is unused.
 */
#define LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE (3)   // Turning on this relay will cause the spa heat exchanger valve to close
#define LV_RELAY_SPA_HEAT_EX_VALVE_OPEN  (2)   // Turning this relay on will cause the spa heat exchanger valve to open

#define LV_RELAY_SPA_ELEC_HEAT_ENABLE (4) // Writing '1' enables the spa relay to power the electric water heater when the spa is calling for heat

//    Arudino Analog In 0, measures the voltage from the LM35 glued to the tank
// See TMP36 temperature sensor (https://learn.adafruit.com/tmp36-temperature-sensor)
#define TANK_TEMP_ANALOG_IN (A0)

// Solar panel temperature
#define PANEL_TEMP_ANALOG_IN (A1)

// Temperature of output side of spa heat exchanger
#define SPA_TEMP_ANALOG_IN (A2)

// Digital signal that indicates spa is calling for heat
#define SPA_HEAT_DIGITAL_IN (8)

#define KEY_1 (7)  // Top most input key
#define KEY_2 (6)
#define KEY_3 (5)
#define KEY_4 (4)  // Bottom most input key

#define SPA_HEAT_EX_VALVE_STATUS_OPEN   (10)  // Pin green wire from the Solid Valve and indicates that the valve is open when this is a zero

void setup_time(void);

/*
   These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is glued to the solar hot water tank.
*/
int tank_temperature_F;                // Degrees C converter to Farenheight
int panel_temperature_F;
int spa_temperature_F;

bool tank_temperature_valid;
bool panel_temperature_valid;
bool spa_temperature_valid;

void monitor_valve_closing_callback();
void monitor_valve_opening_callback();
void read_time_and_sensor_inputs_callback();
void print_status_to_serial_callback();
void process_pressed_keys_callback();
void update_lcd_callback();
void frob_relays_callback();
void control_recirc_pump_callback();

Task monitor_valve_closing(200, TASK_FOREVER, &monitor_valve_closing_callback, &ts, false);
Task monitor_valve_opening(200, TASK_FOREVER, &monitor_valve_opening_callback, &ts, false);
Task read_time_and_sensor_inputs(1000, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task process_pressed_keys(100, TASK_FOREVER, &process_pressed_keys_callback, &ts, true);
Task print_status_to_serial(TASK_SECOND * 5, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
Task update_lcd(250, TASK_FOREVER, &update_lcd_callback, &ts, true);
Task frob_relays(TASK_SECOND, TASK_FOREVER, &frob_relays_callback, &ts, true);
Task control_recirc_pump(TASK_SECOND * 30, TASK_FOREVER, &control_recirc_pump_callback, &ts, true);
/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback()
{
  arduino_time = now();
  
  unsigned tank_temp_volts_raw = analogRead(TANK_TEMP_ANALOG_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned tank_temp_millivolts = (((unsigned long)tank_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int tank_temp_C_x10 = tank_temp_millivolts - 500;
  tank_temperature_F = ((((long)tank_temp_C_x10 * 90L) / 50L) + 320L) / 10L;
  tank_temperature_valid = (tank_temperature_F >= 32 && tank_temperature_F < 200);


  unsigned panel_temp_volts_raw = analogRead(PANEL_TEMP_ANALOG_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned panel_temp_millivolts = (((unsigned long)panel_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int panel_temp_C_x10 = panel_temp_millivolts - 500;
  panel_temperature_F = ((((long)panel_temp_C_x10 * 90L) / 50L) + 320L) / 10L;
  panel_temperature_valid = (panel_temperature_F > 20 && panel_temperature_F < 300);

  unsigned spa_temp_volts_raw = analogRead(SPA_TEMP_ANALOG_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned spa_temp_millivolts = (((unsigned long)spa_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int spa_temp_C_x10 = spa_temp_millivolts - 500;
  spa_temperature_F = ((((long)spa_temp_C_x10 * 90L) / 50L) + 320L) / 10L;
  spa_temperature_valid = (spa_temperature_F > 40 && spa_temperature_F < 120);

  spa_calling_for_heat = digitalRead(SPA_HEAT_DIGITAL_IN) == LOW;

  spa_heat_ex_valve_status_open = digitalRead(SPA_HEAT_EX_VALVE_STATUS_OPEN) == 0;
}

char *diag_mode_to_string(enum diag_mode_e diag_mode) 
{
  switch (diag_mode) {
    case d_oper:          return "Oper";
    case d_rpump:         return "D-RP";
    case d_spump:         return "D-SP";
    case d_spa_hex_valve: return "D-HX";    
    case d_spa_elec:      return "D-EL";
    default:              return "ERR";
  }
  return "E";
}
    
void turn_recirc_pump_on() 
{
  recirc_pump_on = true;
  digitalWrite(SSR_RECIRC_PUMP, LOW); // Ground the low side of the SSR, turning on recirculation pump
}

void turn_recirc_pump_off() 
{
  recirc_pump_on = false;
  digitalWrite(SSR_RECIRC_PUMP, HIGH); // Un-Ground the low side of the SSR, turning off recirculation pump
}

void turn_solar_pump_on()
{
  digitalWrite(SSR_SOLAR_PUMP, LOW); // Turn on Solar pump
  solar_pump_on = true;
}

void turn_solar_pump_off()
{
  digitalWrite(SSR_SOLAR_PUMP, HIGH); // Turn off Solar pump
  solar_pump_on = false;
}


volatile unsigned long valve_motion_time_start = 0;



void monitor_valve_closing_callback()
{
  unsigned long current_time = millis();
  if ((current_time - valve_motion_time_start) > 11000) {
    // We turn off the close-valve process after 11 seconds, otherwise the valve turns
    // too far and is left slightly open.
  
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
    spa_heat_ex_valve_open = false;
    monitor_valve_closing.disable();
  }
}

void monitor_valve_opening_callback()
{
  unsigned long current_time = millis();
  if (spa_heat_ex_valve_open || (current_time - valve_motion_time_start) > 15000) {
    // We turn off the valve-opening process once the valve signal says it is open or
    // after 15 seconds, as it should just take 13 seconds
  
    quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
    spa_heat_ex_valve_open = true;
    monitor_valve_opening.disable();
  }
}

void open_spa_heat_exchanger_valve()
{
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
  quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
  valve_motion_time_start = millis();
  monitor_valve_opening.enable();
}

// We start the valve closing and then enable a task that will turn off the closing signal after 11 seconds.

void close_spa_heat_exchanger_valve()
{
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);
  quad_lv_relay->turnRelayOn(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
  valve_motion_time_start = millis();
  monitor_valve_closing.enable();
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

bool key_select_pressed = false;
bool key_enter_pressed = false;
bool key_plus_pressed = false;
bool key_minus_pressed = false;

#define PIN_CHANGE_INTERRUPT_VECTOR PCINT2_vect  // PCINT2 covers D4–D7
volatile unsigned long lastInterruptTime = 0;
#define DEBOUNCE_DELAY 500  // 50ms debounce time


ISR(PIN_CHANGE_INTERRUPT_VECTOR) 
{
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > DEBOUNCE_DELAY) {  // Debounce check
  
    bool key_select = !(PIND & (1 << PD7));
    bool key_enter = !(PIND & (1 << PD6));
    bool key_plus = !(PIND & (1 << PD5));
    bool key_minus = !(PIND & (1 << PD4));

    if (key_select) {
      key_select_pressed = true;
    }
    if (key_enter) {
      key_enter_pressed = true;
    }
    if (key_plus) {
      key_plus_pressed = true;
    }
    if (key_minus) {
      key_minus_pressed = true;
    }
    
    lastInterruptTime = currentTime;  // Update debounce timer
  } 
}

// The interrupt routine above will set global variables indicating which keys have been pressed.
// In this function, we look at those global variables and take action required by the key presses
// and then reset those global variables.

void process_pressed_keys_callback()
{
  if (key_select_pressed) {
    diag_mode = (diag_mode + 1) % d_last;
    key_select_pressed = false;
    lcd.setCursor(15 /* column */, 0 /* row */);
    lcd.print(diag_mode_to_string(diag_mode)); 
  }
 
  if (key_plus_pressed) {
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

      case d_spa_hex_valve:          
        open_spa_heat_exchanger_valve();
        break;

      case d_spa_elec:
        turn_spa_heater_relay_on();
        break; 
    }
    key_plus_pressed = false;
  }

  if (key_minus_pressed) {
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

      case d_spa_hex_valve:          
        close_spa_heat_exchanger_valve();
        break;

      case d_spa_elec:
        turn_spa_heater_relay_off();
        break; 
    }
    key_minus_pressed = false;
  }
}


#define SOLAR_PUMP_DELAY (15 * 60 * 1000)
#define HEATER_RELAY_DELAY (15 * 60 * 1000)
#define SPA_VALVE_DELAY (30 * 60 * 1000)

// This contains the normal basic operational logic of this controller, comparing temperatures
// and deciding which pumps to turn on, how to set the spa heat exchanger valve, etc.

void frob_relays_callback()
{
  static unsigned long last_solar_pump_change = 0;
  static unsigned long last_spa_heater_relay_change = 0;
  static unsigned long last_spa_valve_change = 0;
  
  unsigned long current_time = millis();
  
  if (diag_mode == d_oper) {
    if (solar_pump_on == false && panel_temperature_valid && tank_temperature_valid && panel_temperature_F > (tank_temperature_F + 40)) {
      if ((current_time - last_solar_pump_change) > SOLAR_PUMP_DELAY) {
        turn_solar_pump_on();
        last_solar_pump_change = current_time;
      }
    }
    if (solar_pump_on && panel_temperature_valid && tank_temperature_valid && panel_temperature_F < tank_temperature_F) {
      if ((current_time - last_solar_pump_change) > SOLAR_PUMP_DELAY) { 
        turn_solar_pump_off();
        last_solar_pump_change = current_time;
      }  
    }
  
    // If the tank is not warm enough to heat the spa, enable the relay we inserted in the spa controller 
    // that passes the spa's call-for-heat signal to big power relay that controls the spa's electric heater
  
    if (spa_heater_relay_on == false && tank_temperature_valid && tank_temperature_F < 110) {
      if ((current_time - last_spa_heater_relay_change) > HEATER_RELAY_DELAY) {
        turn_spa_heater_relay_on();
        last_spa_heater_relay_change = current_time;
      }
    }
  
    // If the tank is warm enough to heat the spa, ensure that its electric heater is off
    
    if (spa_heater_relay_on && tank_temperature_valid && tank_temperature_F > 115) {
      if ((current_time - last_spa_heater_relay_change) > HEATER_RELAY_DELAY) {
        turn_spa_heater_relay_off();
        last_spa_heater_relay_change = current_time;
      }
    }
  
    // If the electric heater is off and the spa is calling for heat, ensure that the spa's heat
    // exchanger pump is on.  
    if (spa_heater_relay_on == false /* we are heating with solar */) {
      if (spa_calling_for_heat && spa_heat_ex_valve_open == false) {
        if ((current_time - last_spa_valve_change) > SPA_VALVE_DELAY) {
          open_spa_heat_exchanger_valve();
          last_spa_valve_change = current_time;
        }
      } 
    }

    // If the spa is not calling for heat, ensure that the spa's heat exchanger valve is closed
    if (spa_heat_ex_valve_open && spa_calling_for_heat == false) {
      assert(spa_heater_relay_on == false);
      if ((current_time - last_spa_valve_change) > SPA_VALVE_DELAY) {
        close_spa_heat_exchanger_valve();
        last_spa_valve_change = current_time;
      }
    }
  }
}

/*
   This is the main system tracing function.  It emits a line of ASCII to the RS-485 line with lots of information.
   We display this information in a form that gnuplot(1) can readily absorb.
*/

// Send the passed string to one or both of our serial channels
void print_buf(char *b)
{
  Serial.print(b);  
  if (send_to_rs232) {
    // rs232.print(b);
  }
}

// Called periodically.  Sends relevant telemetry back over one or both of the serial channels
void print_status_to_serial_callback(void)
{
  static char line_counter = 0;

  if (line_counter == 0) {
    const char *m = "# Date    Time Year  Mode Tank Panel SpaT SpaH Spump Rpump Takagi Valve\n\r";
    Serial.print(m);
    if (send_to_rs232) {
    // rs232.print(m);
    }
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

  char buf[80];
  snprintf(buf, sizeof(buf), "%s %d %02d:%02d:%02d %4d %s %4d %4d  %4d %4d  %4d  %4d %4d %4d\n", 
      monthShortStr(month(arduino_time)), 
      day(arduino_time), 
      hour(arduino_time),
      minute(arduino_time), 
      second(arduino_time), 
      year(arduino_time),
      diag_mode_to_string(diag_mode),
      tank_temperature_F,
      panel_temperature_F,
      spa_temperature_F,
      spa_heater_relay_on,
      solar_pump_on,
      recirc_pump_on,
      takagi_on,
      spa_heat_ex_valve_status_open);
  
  print_buf(buf);
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
    char buf[21];

    
    lcd.setCursor(0 /* column */, 0 /* row */);
    snprintf(buf, sizeof(buf), "%02d:%02d:%02d %4s %4s ", 
      hour(arduino_time), minute(arduino_time), second(arduino_time), spa_heat_ex_valve_status_open ? " Open" : " ----", diag_mode_to_string(diag_mode));
    lcd.print(buf);
   
    lcd.setCursor(0, 1);
    snprintf(buf, sizeof(buf), "T: %3d P: %3d S: %3d", tank_temperature_F, panel_temperature_F, spa_temperature_F);
    lcd.print(buf);
    
    lcd.setCursor(0, 2);
    snprintf(buf, sizeof(buf), "RPump%c SPump%c HEX%c ", recirc_pump_on ? '+' : '-', solar_pump_on ? '+' : '-', spa_heat_ex_valve_open ? '+' : '-');
    lcd.print(buf);
    
    snprintf(buf, sizeof(buf), "Spa-elec%c Spa-call%c ", spa_heater_relay_on ? '+' : '-', spa_calling_for_heat ? '+' : '-');
    lcd.setCursor(0,3);
    lcd.print(buf);
  }
}

// This controls the recirculation pump SSR and is currently simple, it just runs for one minute every 30 minutes
// between 8AM and 11PM.

void control_recirc_pump_callback()
{
  if (diag_mode == d_oper) {
    unsigned h = hour(arduino_time);
    unsigned m = minute(arduino_time);
  
  // Between 8AM and 23:30, turn the recirc pump on at the top of the hour and at the half hour.
  // Turn it off after a minute
  
    if (h >= 8 && h <= 23) {
      if (m == 0 || m == 30) {
        if (recirc_pump_on == false) {
          turn_recirc_pump_on();
        }
      } else {
        if (recirc_pump_on) {
          turn_recirc_pump_off();
        }
      }
    }
  }
}


/*
   Called by failure paths that should never happen.  When we get the RS-485 input working, we'll allow the user
   to do things in this case and perhaps resume operation.
*/
void fail(const char *fail_message)
{
  Serial.println(fail_message);
 
  lcd.setCursor(0, 3);
  lcd.print(F("Fail: "));
  lcd.print(fail_message);         // Display this on the third row, left-adjusted
  lcd.print(F("  "));
  delay(500); // Give the serial link time to propogate the error message before execution ends
  abort();
}


/*
   This is the function that the Arudino run time system calls once, just after start up.  We have to set the
   pin modes of the ATMEGA correctly as inputs or outputs.  We also fetch values from EEPROM for use during
   our operation and emit a startup message.
*/
void setup()
{
  delay(1000); // In case something further along crashes and we restart quickly, this will give a one-second pause
 
  analogReference(DEFAULT);
  Wire.begin();
  Wire.setClock(400000); 

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag

  Serial.print(F("\n\r#Solarthermal "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));
 
  pinMode(LED_BUILTIN, OUTPUT); 

  // Turn off the solar pump and the recirc pump
  pinMode(SSR_SOLAR_PUMP, OUTPUT); 
  digitalWrite(SSR_SOLAR_PUMP, HIGH);
  
  pinMode(SSR_RECIRC_PUMP, OUTPUT);
  digitalWrite(SSR_RECIRC_PUMP, HIGH);

  takagi_on = true;
  pinMode(SSR_TAKAGI, OUTPUT);
  digitalWrite(SSR_TAKAGI, LOW); // Turn on Takagi

  pinMode(KEY_1, INPUT_PULLUP);
  pinMode(KEY_2, INPUT_PULLUP);
  pinMode(KEY_3, INPUT_PULLUP);
  pinMode(KEY_4, INPUT_PULLUP);

  pinMode(SPA_HEAT_EX_VALVE_STATUS_OPEN, INPUT_PULLUP);

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

  delay(500);

  {
    byte error, address;
    int nDevices;
  
    Serial.println(F("Scanning..."));
  
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
        Serial.print(F("I2C device found at address 0x"));
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
             Serial.print(F("Failure to start quad qwiic relay object"));
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
              Serial.print(F(" Wayward Qwiic relay found, remapping it to where it is suppose to be\n"));
              quad_lv_relay->changeAddress(LV_RELAY_I2C_ADDR);
            } else {
              Serial.print(F(" unexpected device, unable to treat it as a quad qwiic relay\n"));
            }
          } else {
            Serial.print(F(" unexpected device after finding Qwiic quad relay\n"));
          }
        }
        Serial.print("\n");
  
        nDevices++;
      } else if (error == 4) {
        Serial.print(F("Unknown error at address 0x"));
        if (address < 16) {
          Serial.print("0");
        }
        Serial.println(address, HEX);
      }
    }
    if (nDevices == 0) {
      Serial.println(F("No I2C devices found\n"));
    }
  }
  if (quad_lv_relay == (void *)0) {
    fail("REL LV");
  }
  
  // Ensure that the motorized valve is unpowered
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_CLOSE);
  quad_lv_relay->turnRelayOff(LV_RELAY_SPA_HEAT_EX_VALVE_OPEN);

  // By default, the spa electric heater should be able to heat the spa
  quad_lv_relay->turnRelayOn(LV_RELAY_SPA_ELEC_HEAT_ENABLE);

  
  // Set the global variable that keeps track of whether the spa heat exchanger valve is open based
  // on the "open" status from the valve itself.
  spa_heat_ex_valve_status_open = digitalRead(SPA_HEAT_EX_VALVE_STATUS_OPEN) == 0;
  if (spa_heat_ex_valve_status_open) {
    spa_heat_ex_valve_open = true;
    Serial.println(F("Spa heat exchanger valve is open"));
  } else {
    Serial.println(F("Spa heat exchanger valve is closed"));
    spa_heat_ex_valve_open = false;
  }
  if(!rtc.begin()) {
    Serial.println(F("No RTC!"));
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

  } else {
    if(rtc.lostPower() || force_RTC_reload_from_build_time ) {
      // this will adjust to the date and time at compilation
      rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

      // Add 15 seconds to compensate for the time to upload the sketch
      DateTime newTime = rtc.now() + TimeSpan(0, 0, 0, 15);  

    // Update the RTC
      rtc.adjust(newTime);

      Serial.println(F("Setting RTC from build time"));
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
    snprintf(buf, sizeof(buf), "Set time from RTC: %s %d %02d:%02d:%02d %4d\n", 
      monthShortStr(mo), d, h, m, s, y);
  
    print_buf(buf);
  }

  PCICR |= (1 << PCIE2);                                        // Enable Pin Change Interrupt for PORTD  
  PCICR &= ~((1 << PCIE0) | (1 << PCIE1));                      // Disable PCINT0 (PORTB) and PCINT1 (PORTC)
  PCMSK2 |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);  // Enable for D4–D7

  sei();  // Enable global interrupts

#ifdef NOTDEF  
  Serial.print(F("PCICR="));
  Serial.println(PCICR);
  Serial.print(F("PCMSK1="));
  Serial.print(PCMSK1);
  Serial.print(F("PCMSK2="));
  Serial.println(PCMSK2);
#endif

}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop()
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}
