
/*
   This controls:
      An SSR that controls a 120VAC water pump that moves water from the tank to the panels (and back)
      An SSR that controls the 120VAC water pump that recirculates hot water through our home
      An SSR that controls a pump that moves water through a heat exchanger (in the hot water tank) that heats spa water
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


#define _TASK_SLEEP_ON_IDLE_RUN
#include <TaskScheduler.h>

#include <TimeLib.h>     // for update/display of time

#include <SoftwareSerial.h>

const unsigned long rs232_baud = 9600 ;

const byte rxPin = 2; // Wire this to Tx Pin of RS-232 level shifter
const byte txPin = 3; // Wire this to Rx Pin of RS-232 level shifter
 
SoftwareSerial rs232 (rxPin, txPin);
bool send_to_rs232 = true;

//    This is for the 1307 RTC we installed on the Ocean Controls main board.
#include <DS1307RTC.h>

//   All the operational code uses this time structure.  This is initialized at start time from the battery-backed up DS1307 RTC.
time_t arduino_time;

//   If we are able to read the DS1307 RTC over I2C, then we set this true and we can depend on
//  the time of day being valid.
bool time_of_day_valid = false;

bool solar_pump_on = false;

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

#define SSR_SOLAR_PUMP_OUT (7) // Arduino output pin 7 on J4, writing '1' turns on solar pump SSR

#define SSR_RECIRC_PUMP_OUT (-1)  // Assign a pin

#define SSR_SPA_HEAT_EX_PUMP_OUT (-1) // Assign a pin

#define SPA_ELEC_HEAT_ENABLE_OUT (-1) // Assign a pin

//    Arudino Analog In 0, measures the voltage from the LM35 glued to the tank
// See TMP36 temperature sensor (https://learn.adafruit.com/tmp36-temperature-sensor)
#define TANK_TEMP_IN (0)

// Solar panel temperature
#define PANEL_TEMP_IN (1)

// Temperature of output side of spa heat exchanger
#define SPA_HEAT_EX_TEMP_IN (2)

// Digital signal that indicates spa is calling for heat
#define SPA_HEAT_IN (3)

void setup_time(void);

/*
   These come from Arduino Analog input 2, which is driven by a TMP36 temperature sensor that is glued to the solar hot water tank.
*/
int tank_temperature_F;                // Degrees C converter to Farenheight
int panel_temperature_F;
int spa_heat_exchanger_temperature_F;

void read_time_and_sensor_inputs_callback();
void print_status_to_serial_callback();
void frob_relays_callback();
void control_recirc_pump_callback();

Task read_time_and_sensor_inputs(1000, TASK_FOREVER, &read_time_and_sensor_inputs_callback, &ts, true);
Task print_status_to_serial(TASK_SECOND * 5, TASK_FOREVER, &print_status_to_serial_callback, &ts, true);
Task frob_relays(TASK_SECOND, TASK_FOREVER, &frob_relays_callback, &ts, true);
Task control_recirc_pump(TASK_SECOND, TASK_FOREVER, &control_recirc_pump_callback, &ts, true);
/*
   This reads all the sensors frequently, does a little filtering of some of them, and deposits the results in global variables above.
*/
void read_time_and_sensor_inputs_callback()
{
  arduino_time = now();
  
  unsigned tank_temp_volts_raw = analogRead(TANK_TEMP_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned tank_temp_millivolts = (((unsigned long)tank_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int tank_temp_C_x10 = tank_temp_millivolts - 500;
  tank_temperature_F = ((((long)tank_temp_C_x10 * 90L) / 50L) + 320L) / 10L;

  unsigned panel_temp_volts_raw = analogRead(PANEL_TEMP_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned panel_temp_millivolts = (((unsigned long)panel_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int panel_temp_C_x10 = panel_temp_millivolts - 500;
  panel_temperature_F = ((((long)panel_temp_C_x10 * 90L) / 50L) + 320L) / 10L;

  unsigned spa_hex_temp_volts_raw = analogRead(SPA_HEAT_EX_TEMP_IN);  // Raw ADC value: 0..1023 for 0..5V
  unsigned spa_hex_temp_millivolts = (((unsigned long)spa_hex_temp_volts_raw * 5000UL) / 1023UL) + 20;  /* Calibration value */
  int spa_hex_temp_C_x10 = spa_hex_temp_millivolts - 500;
  spa_heat_exchanger_temperature_F = ((((long)spa_hex_temp_C_x10 * 90L) / 50L) + 320L) / 10L;

}

void frob_relays_callback()
{
  if (solar_pump_on == false && panel_temperature_F > (tank_temperature_F + 40)) {
    digitalWrite(SSR_SOLAR_PUMP_OUT, HIGH);
    solar_pump_on = true;
  }
  if (solar_pump_on && panel_temperature_F < tank_temperature_F) {
    digitalWrite(SSR_SOLAR_PUMP_OUT, LOW);
    solar_pump_on = false;
  }
}

/*
   This is the main system tracing function.  It emits a line of ASCII to the RS-485 line with lots of information.
   We display this information in a form that gnuplot(1) can readily absorb.


# Date    Time Year Md  Pos  Dif  Sol Delt Rain  Volts Tmp Amp Mot Drk UpL LwL GUp GDn CoL OvC  Knots
Apr 15 09:47:46 2023 1   56  -56   31    0  330 11.600  59   0   0   0   0   1   0   0   0   0  0.0
Apr 15 09:47:56 2023 1   56    0   31    0  330 11.568  59   0   0   0   0   1   0   0   0   0  0.1
Apr 15 09:48:06 2023 1   56    0   31    0  330 11.584  59   0   0   0   0   1   0   0   0   0  0.2

  
   Here is a gnuplot file that works to parse some of these lines where the text has been put in a file 'tracker.dat'.
   Still to do is to have gnuplot understand fields 13 through 22 (booleans from "Dark" to "Overcurrent").

set term png size 1500, 300
set output 'tracker.png'
set xdata time
set xrange [time(0) - 3*24*60*60:]
set timefmt "%b %d %H:%M:%S %Y"
set ylabel "Position/Volts/Temperature/Knots"
set yrange [-10:1200]
set xlabel " "
set grid
# set size 1.2 ,0.5
set key top left
set datafile separator whitespace

# Typical data

# Date Time      Year Mode Pos Diff  Sol Rain  Volts  Temp Amps MotT Drk UpL LwL GUp GDn CoL OvC KTS
# Dec 25 19:08:18 2022 2   51  -51   19  328  11.728   63    3   0   0   0   1   0   0   0   0   0

plot 'tracker.data' using 1:(($5 * 250.0))    t "Mode" with points lc rgb "black", \
     'tracker.data' using 1:(($6 * 3.0))      t "Position" with lines lc rgb "blue", \
     'tracker.data' using 1:(($8 * 10.0))     t "light sensor" with lines lc rgb "red", \
     'tracker.data' using 1:(($9 * 10.0))     t "light sensor" with lines lc rgb "red", \
     'tracker.data' using 1:((1000 - ($10 * 3))) t "Rain sensor" with lines lc rgb "green", \
     'tracker.data' using 1:(($11 * 100.0))    t "System Voltage (x100)" with lines lc rgb "black", \
     'tracker.data' using 1:(($12 * 10.0))    t "Contactor Temperature (x10 F)" with lines lc rgb "gray", \
     'tracker.data' using 1:(($13 * 10.0))    t "Motor Amps (x10)" with lines lc rgb "purple", \
     'tracker.data' using 1:(($14 * 10.0))   t "Motor On Time" with lines lc rgb "cyan", \
     'tracker.data' using 1:(($22 * 185.0))  t "Windspeed" with lines lc rgb "orange"

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
    const char *m PROGMEM = "# Date    Time Year Md  Pos  Dif  Sol Delt Rain  Volts Tmp Amp Mot Drk UpL LwL GUp GDn CoL OvC  Knots\n\r";
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
    
  char buf[45];
  snprintf(buf, sizeof(buf), "%s %d %02d:%02d:%02d %4d %4d %4d", 
      monthShortStr(month(arduino_time)), 
      day(arduino_time), 
      hour(arduino_time),
      minute(arduino_time), 
      second(arduino_time), 
      year(arduino_time) - 30,
      tank_temperature_F,
      panel_temperature_F,
      spa_heat_exchanger_temperature_F);
  
  print_buf(buf);
}


int dst_correction(tmElements_t *tm)
{
  if (tm->Month > 3 && tm->Month < 11) {
     return 1;
  }
  if (tm->Month == 3 && tm->Day >= 11) {
    return 1;
  }
  if (tm->Month == 11 && tm->Day < 6) {
    return 1;
  }
  return 0;
}
/*
   Read the RTC chip and set the 'Arduino' time based on it.  We do this on system start and once per day.
   The Arduino clock is not as accurate as the RTC.
*/
void setup_time(void)
{
  tmElements_t tm;

  if (RTC.read(tm)) {
    setTime(tm.Hour + dst_correction(&tm), tm.Minute, tm.Second, tm.Day, tm.Month, tm.Year);
    time_of_day_valid = true;
  } else {
    if (RTC.chipPresent()) {
      Serial.println(F("#The DS1307 is stopped.  Please run the SetTime"));
      Serial.println(F("#example to initialize the time and begin running."));
      Serial.println();
    } else {
      Serial.println(F("#DS1307 read error!  Please check the circuitry."));
      Serial.println();
    }
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
 
  digitalWrite(SSR_SOLAR_PUMP_OUT, LOW);
  digitalWrite(SSR_RECIRC_PUMP_OUT, LOW);
  digitalWrite(SSR_SPA_HEAT_EX_PUMP_OUT, LOW);
  digitalWrite(SPA_ELEC_HEAT_ENABLE_OUT, LOW);

  analogReference(DEFAULT);
  
  rs232.begin(rs232_baud);
#ifdef SETTIME
  setup_rtc();
#endif

  Serial.begin(SERIAL_BAUD);
  UCSR0A = UCSR0A | (1 << TXC0); //Clear Transmit Complete Flag

  Serial.print(F("\n\r#Suntracker "));
  Serial.print(F(__DATE__));
  Serial.write(' ');
  Serial.println(F(__TIME__));

  rs232.println(F("\n\r#Suntracker "));
  
}

/*
   This is the function that the Arduino run time system calls repeatedly after it has called setup().
*/
void loop()
{
  // All code after setup() executes inside of tasks, so the only thing to do here is to call the task scheduler's execute() method.
  ts.execute();
}

/*
   This serial code _was_ working, then it seemed to throw away most of the input characters and
   when it did receive some, they were often values like 0xff or other bytes with
   most bits set.  Converting to using serialEvent() does not seem to have changed that.
*/
void monitor_rs485_input_callback()
{
#ifdef RS485
  if (txenabled == true) {
    fail("monitor_rs485...");
  }
#endif
}

//   If SETTIME is defined, we compile-in the code below to set the device's time to the build time.
#ifdef SETTIME
bool getTime(const char *str, tmElements_t *tm)
{
  int Hour, Min, Sec;

  if (sscanf(str, "%d:%d:%d", &Hour, &Min, &Sec) != 3) return false;
  tm->Hour = Hour;
  tm->Minute = Min;
  tm->Second = Sec;
  return true;
}

bool getDate(const char *str, tmElements_t *tm)
{
  char Month[12];
  int Day, Year;
  uint8_t monthIndex;
  const char *monthName[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
  };

  if (sscanf(str, "%s %d %d", Month, &Day, &Year) != 3) return false;
  for (monthIndex = 0; monthIndex < 12; monthIndex++) {
    if (strcmp(Month, monthName[monthIndex]) == 0) break;
  }
  if (monthIndex >= 12) return false;
  tm->Day = Day;
  tm->Month = monthIndex + 1;
  tm->Year = CalendarYrToTm(Year);
  return true;
}
void setup_rtc()
{

  tmElements_t tm;

  bool parse = false;
  bool config = false;

  // get the date and time the compiler was run
  if (getDate(build_date, &tm) && getTime(build_time, &tm)) {
    parse = true;

    // The value we store into the RTC is the non-daylight-savings-time value
    if (dst_correction(&tm)) {
      if (tm.Hour == 0) {
        fail("T");
      }
      tm.Hour--;   
      tm.Second += 10; // Empirical: difference between build time and time to run this code
      if (tm.Second > 59) {
        tm.Minute++;
        if (tm.Minute > 59) {
          tm.Hour++;
          if (tm.Hour > 23) {
            tm.Day++;
          }
        }
      }
    }
    // and configure the RTC with this info
    if (RTC.write(tm)) {
      config = true;
    }
  }

  if (parse && config) {
    Serial.print(F("DS1307 configured Time"));
  } else if (parse) {
    fail("DS1307");

  } else {
    fail("time string");
  }

}
#endif
