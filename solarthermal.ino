
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
#define LV_RELAY_ADDR  (0x6D)                 // Default I2C address of the mechanical, low voltage, 4-relay board
Qwiic_Relay *quad_lv_relay;

#include <SerLCD.h>
SerLCD lcd;

#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

#include <TimeLib.h>     // for update/display of time

#include <SoftwareSerial.h>

enum diag_mode_e {d_oper, d_rpump, d_spump, d_spa_hex_valve, d_spa_elec, d_last} diag_mode;

const unsigned long rs232_baud = 9600 ;

const byte rxPin = 2; // Wire this to Tx Pin of RS-232 level shifter
const byte txPin = 3; // Wire this to Rx Pin of RS-232 level shifter
 
SoftwareSerial rs232 (rxPin, txPin);
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

bool spa_heat_ex_valve_open = false;

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
#define SSR_SPARE       (10)
#define SSR_TAKAGI      (9)    // writing '0' turns on the Takagi natural-gas fired water heater

/*
 * There is a motorized valve that I added to the spa.  When it is open, some of the water coming from the recirculation pump in the spa
 * will go through the stainless steel heat exchanger that I added to the solar tank.  This valve is controlled by two 12VDC signals.  When
 * the "open" signal is +12V and the "close" signal is ground, the valve will open.  When the "close" signal is +12VDC and the "open" signal
 * is ground, then the valve will close.  These two signals are enabled by relays 1 and 2 in the Sparkfun relay board.
 */
#define LV_SPA_HEAT_EX_VALVE_CLOSE (2)   // Turning on this relay will cause the spa heat exchanger valve to close
#define LV_SPA_HEAT_EX_VALVE_OPEN  (1)   // Turning this relay on will cause the spa heat exchanger valve to open

#define LV_SPA_ELEC_HEAT_ENABLE (4) // Writing '1' enables the spa relay to power the electric water heater when the spa is calling for heat

//    Arudino Analog In 0, measures the voltage from the LM35 glued to the tank
// See TMP36 temperature sensor (https://learn.adafruit.com/tmp36-temperature-sensor)
#define TANK_TEMP_ANALOG_IN (0)

// Solar panel temperature
#define PANEL_TEMP_ANALOG_IN (1)

// Temperature of output side of spa heat exchanger
#define SPA_HEAT_EX_TEMP_ANALOG_IN (2)

// Digital signal that indicates spa is calling for heat
#define SPA_HEAT_DIGITAL_IN (8)

#define KEY_1 (7)  // Top most input key
#define KEY_2 (6)
#define KEY_3 (5)
#define KEY_4 (4)  // Bottom most input key

#define VALVE_STATUS_OPEN   (A4)  // Pin AA/D18 connects to the green wire from the Solid Valve and indicates that the valve is open
#define VALVE_STATUS_CLOSED (A5)  // Pin A5/D19 connects to the red wire from the Solid Valve and indicates that the valve is closed

void setup_time(void);

/*
   These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is glued to the solar hot water tank.
*/
int tank_temperature_F;                // Degrees C converter to Farenheight
int panel_temperature_F;
int spa_heat_exchanger_temperature_F;

void read_time_and_sensor_inputs_callback();
void print_status_to_serial_callback();
void read_keys_callback();
void update_lcd_callback();
void frob_relays_callback();
void control_recirc_pump_callback();

Task read_time_and_sensor_inputs(1000, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task read_keys(100, TASK_FOREVER, &read_keys_callback, &ts, true);
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

  unsigned panel_temp_volts_raw = analogRead(PANEL_TEMP_ANALOG_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned panel_temp_millivolts = (((unsigned long)panel_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int panel_temp_C_x10 = panel_temp_millivolts - 500;
  panel_temperature_F = ((((long)panel_temp_C_x10 * 90L) / 50L) + 320L) / 10L;

  unsigned spa_hex_temp_volts_raw = analogRead(SPA_HEAT_EX_TEMP_ANALOG_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned spa_hex_temp_millivolts = (((unsigned long)spa_hex_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int spa_hex_temp_C_x10 = spa_hex_temp_millivolts - 500;
  spa_heat_exchanger_temperature_F = ((((long)spa_hex_temp_C_x10 * 90L) / 50L) + 320L) / 10L;

  spa_calling_for_heat = digitalRead(SPA_HEAT_DIGITAL_IN) == LOW;
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

void open_spa_heat_exchanger_valve()
{
  quad_lv_relay->turnRelayOff(LV_SPA_HEAT_EX_VALVE_CLOSE);
  quad_lv_relay->turnRelayOn(LV_SPA_HEAT_EX_VALVE_OPEN);

  // 13 seconds should be enough time for the valve to fully open
  delay(13000);
  quad_lv_relay->turnRelayOff(LV_SPA_HEAT_EX_VALVE_OPEN);
  spa_heat_ex_valve_open = true;
}

void close_spa_heat_exchanger_valve()
{
  quad_lv_relay->turnRelayOff(LV_SPA_HEAT_EX_VALVE_OPEN);
  quad_lv_relay->turnRelayOn(LV_SPA_HEAT_EX_VALVE_CLOSE);

  // We turn off the close-valve process after 11 seconds, otherwise the valve turns
  // too far and is left slightly open.
  delay(11000);
  quad_lv_relay->turnRelayOff(LV_SPA_HEAT_EX_VALVE_CLOSE);
  spa_heat_ex_valve_open = false;
}

void turn_spa_heater_relay_on()
{
   quad_lv_relay->turnRelayOn(LV_SPA_ELEC_HEAT_ENABLE);
   spa_heater_relay_on = true;
}

void turn_spa_heater_relay_off()
{
   quad_lv_relay->turnRelayOff(LV_SPA_ELEC_HEAT_ENABLE);
   spa_heater_relay_on = false;
   
}
void read_keys_callback()
{
  static int debounce_timer = 0;
  bool key_select = digitalRead(KEY_1) == LOW;
  bool key_enter = digitalRead(KEY_2) == LOW;
  bool key_plus = digitalRead(KEY_3) == LOW;
  bool key_minus = digitalRead(KEY_4) == LOW;
    
  if (debounce_timer > 0) {
    if ((key_select|key_enter|key_plus|key_minus) == false) {
      debounce_timer--;
    }
  } else {  
    if (key_select) {
      diag_mode = (diag_mode + 1) % d_last;
      debounce_timer = 2;
    }
      // Set time: setTime(Hour, Minute, Second, Day, Month, Year)
   
    if (key_plus) {
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
      debounce_timer = 2;
    }

    if (key_minus) {
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
      debounce_timer = 2;
    }
  }
}

void frob_relays_callback()
{
  if (diag_mode == d_oper) {
    if (solar_pump_on == false && panel_temperature_F > (tank_temperature_F + 40)) {
      turn_solar_pump_on();
    }
    if (solar_pump_on && panel_temperature_F < tank_temperature_F) {
      turn_solar_pump_off();      
    }
  
    // If the tank is not warm enough to heat the spa, enable the relay we inserted in the spa controller 
    // that passes the spa's call-for-heat signal to big power relay that controls the spa's electric heater
  
    if (spa_heater_relay_on == false && tank_temperature_F < 110) {
      turn_spa_heater_relay_on();
    }
  
    // If the tank is warm enough to heat the spa, ensure that its electric heater is off
    
    if (spa_heater_relay_on && tank_temperature_F > 115) {
      turn_spa_heater_relay_off();
    }
  
  
    // If the electric heater is off and the spa is calling for heat, ensure that the spa's heat
    // exchanger pump is on.  
    if (spa_heater_relay_on == false /* we are heating with solar */) {
      if (spa_calling_for_heat && spa_heat_ex_valve_open == false) {
        open_spa_heat_exchanger_valve();
      } 
    }

    // If the spa is not calling for heat, ensure that the spa's heat exchanger valve is closed
    if (spa_heat_ex_valve_open && spa_calling_for_heat == false) {
      assert(spa_heater_relay_on == false);
      close_spa_heat_exchanger_valve();
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
    rs232.print(b);
  }
}

// Called periodically.  Sends relevant telemetry back over one or both of the serial channels
void print_status_to_serial_callback(void)
{
  static char line_counter = 0;

  if (line_counter == 0) {
    const char *m = "# Date    Time Year  Mode Tank Panel SpaT SpaH Spump Rpump Takagi\n\r";
    Serial.print(m);
    if (send_to_rs232) {
      rs232.print(m);
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
  snprintf(buf, sizeof(buf), "%s %d %02d:%02d:%02d %4d %s %4d %4d  %4d %4d  %4d  %4d %4d\n", 
      monthShortStr(month(arduino_time)), 
      day(arduino_time), 
      hour(arduino_time),
      minute(arduino_time), 
      second(arduino_time), 
      year(arduino_time),
      diag_mode_to_string(diag_mode),
      tank_temperature_F,
      panel_temperature_F,
      spa_heat_exchanger_temperature_F,
      spa_heater_relay_on,
      solar_pump_on,
      recirc_pump_on,
      takagi_on);
  
  print_buf(buf);
}

void print_2_digits_to_lcd(int number)
{
  if (number >= 0 && number < 10) {
    lcd.write('0');
  }
  lcd.print(number);
}

void update_lcd_callback()
{
  if (second(arduino_time) > 3) {
    lcd.setCursor(0 /* column */, 0 /* row */);
    print_2_digits_to_lcd(hour(arduino_time));  // Display this on the first row
    lcd.print(":");
    print_2_digits_to_lcd(minute(arduino_time));
    lcd.print(":");
    print_2_digits_to_lcd(second(arduino_time));

    lcd.print(" ");
    //lcd.print(digitalRead(VALVE_STATUS_OPEN) ? "O" : "-");
    //lcd.print(digitalRead(VALVE_STATUS_CLOSED) ? "C" : "-");
    
    lcd.print("  ");
    lcd.print(diag_mode_to_string(diag_mode));      
   
    lcd.setCursor(0, 1);
    lcd.print(F("T: "));
    lcd.print(tank_temperature_F);
    lcd.print(F(" P: "));
    lcd.print(panel_temperature_F);
    lcd.print(F(" H: "));
    lcd.print(spa_heat_exchanger_temperature_F);
    lcd.print(F("  "));

    lcd.setCursor(0, 2);
    char buf[21];
    snprintf(buf, sizeof(buf), "RPump%c SPump%c HEX%c ", recirc_pump_on ? '+' : '-', solar_pump_on ? '+' : '-', spa_heat_ex_valve_open ? '+' : '-');
    lcd.print(buf);
    
    snprintf(buf, sizeof(buf), "Spa-elec%c Spa-call%c ", spa_heater_relay_on ? '+' : '-', spa_calling_for_heat ? '+' : '-');
    lcd.setCursor(0,3);
    lcd.print(buf);
  }
}

void control_recirc_pump_callback()
{
  if (diag_mode == false) {
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
  
  rs232.begin(rs232_baud);
#ifdef SETTIME
  setup_rtc();
#endif

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag

  Serial.print(F("\n\r#Solarthermal "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));

  rs232.println(F("\n\r#Solarthermal "));
  
  
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

  // pinMode(VALVE_STATUS_OPEN, INPUT_PULLUP);
  // pinMode(VALVE_STATUS_CLOSED, INPUT_PULLUP);

  lcd.begin(Wire, 0x72);         // Default I2C address of Sparkfun 4x20 SerLCD
  lcd.setBacklight(255, 255, 255);
  // lcd.setContrast(5);
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
         case 0x72:
           Serial.print(F(" (SerLCD 4x20)"));
           break;

         case LV_RELAY_ADDR:
           Serial.print(F(" (Quad Qwiic Relay)"));
           quad_lv_relay = new Qwiic_Relay(address);
           if (quad_lv_relay->begin() == 0) {
             Serial.print(F("Failure to start quad qwiic relay object"));
           }
           break;

         default:
          if (quad_lv_relay == (void *)0) {
            Serial.print(F(" (unexpected, will guess that it is the Quad Qwiic Relay at the wrong address)\n"));
            quad_lv_relay = new Qwiic_Relay(address);
            if (quad_lv_relay->begin()) {
              Serial.print(F(" Wayward Qwiic relay found, remapping it to where it is suppose to be\n"));
              quad_lv_relay->changeAddress(LV_RELAY_ADDR);
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
  quad_lv_relay->turnRelayOff(LV_SPA_HEAT_EX_VALVE_CLOSE);
  quad_lv_relay->turnRelayOff(LV_SPA_HEAT_EX_VALVE_OPEN);

  // By default, the spa electric heater should be able to heat the spa
  quad_lv_relay->turnRelayOn(LV_SPA_ELEC_HEAT_ENABLE);

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
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop()
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}
