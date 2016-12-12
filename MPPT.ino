/*
   Created by Zvi Schneider 18-09-2016
   Last Modified 12-12-2016
*/

#include <Wire.h>

//Set PWM frequency to either 200 or 100 KHz
//#define PWM_PERIOD 79       // Set the PWM period to 80 CPU clock ticks which gives 200 KHz of PWM frequency
#define PWM_PERIOD 159      // Set the PWM period to 160 CPU clock ticks which gives 100 KHz of PWM frequency

// Define names for all arduino I/O pins
#define V_PV_IN A0
#define I_PV_IN A1
#define V_PUMP_IN A2
#define I_PUMP_IN A3
#define PWM_ON_IO_PIN 2
#define PWM_OUT 3
#define PV_EN 4
#define OVER_VOLTAGE_IO_PIN 5 // Notice: when over voltage occured the Over_voltage pin is LOW
#define OVER_VOLTAGE_LED 6
#define MEDIUM_VOLTAGE_LED 7
#define LOW_VOLTAGE_LED 8
#define OVER_TEMPERATURE_LED 9
#define TANK_FULL_LED 10
#define PUMP_CURRENT_LED 11
#define ON_OFF_IO_PIN 12       // Notice: when the front panel On/Off switch is "ON" the ON_OFF_IO_PIN is "LOW"
#define TANK_FULL_IO_PIN 13    // Notice: when the Tank is full the TANK_FULL_IO_PIN is "LOW"

//Define parameters
//#define OVER_VOLTAGE_PARAMETER 55     // 55 volts. At this stage this parameter is not used by the code 16-11-2016
#define OVER_TEMPERATURE_PARAMETER 80   // 80 °C
#define OVER_TEMPERATURE_HYSTERESIS 10  // 10 °C hysteresis on over temperature
#define TANK_FULL_DELAY 10000           // 10 seconds delay after tank full indication is low again
#define I_PUMP_THRESHOLD 1              // if I pump > 1A turn on pump current LED
#define LOW_VOLTAGE_PARAMETER 35        // If V_pump < 35V turn Low voltage LED on
#define UNDER_VOLTAGE_PROTECTION 22     // (UVP) The MPPT will not start until Voc is above UNDER_VOLTAGE_PROTECTION
#define UVP_HYSTERESIS  7               // The MPPT will not stop until Vmp is below (UNDER_VOLTAGE_PROTECTIO - UVP_HYSTERESIS)

//Define coeficients for the translation of analog read values into currents and voltages
//Coeficients for I_pv and I_panel current measurements using ACS711EEXLT-15AB-T
//ACS711EEXLT-15AB-T: sensitivity 90mv/A @ 3.3V Vcc, 136.36mv/A @ 5V Vcc; output for 0A = 2.5V (or 512 @ A/D reading)
//I = (AtoD_reading-512)*5/1023/0.13636 = (AtoD_reading-512)*k_i
float k_i = 5.0 / 1023.0 / 0.13636;
//Coeficient for V_pv voltage measurement using R20 (39.2Kohm) and R26 (4.42 Kohm) voltage divider
//V_pv=AtoD_reading*5/1023*(R20+R26)/R26=AtoD_reading*5/1023*(39200+4420)/4420=k_v_pv
float k_v_pv = 5.0 / 1023.0 * (39200.0 + 4420.0) / 4420.0;
//Coeficient for V_pump voltage measurement using R23 (39.2Kohm) and R39 (3.48 Kohm) voltage divider
//V_pump=AtoD_reading*5/1023*(R20+R26)/R26=AtoD_reading*5/1023*(39200+3480)/3480=k_v_pump
float k_v_pump = 5.0 / 1023.0 * (39200.0 + 3480.0) / 3480.0;

//Global variables declaration
float i_pv_n;
float i_pv_n_1; // n - 1
float v_pv_n;
float v_pv_n_1; // n - 1
float delta_i_in;
float delta_v_in;
float i_pump;
float v_pump;
int duty_cycle = 50; //Initial duty cycle of = 50/160
double last_millis;
double last_temperature_measure;
double last_tank_full_time;
int loop_time = 1000; //delay at the end of the loop;
int temperature = 25; //25 will be used until the temperature sensor will be available
boolean over_temperatue_flag = LOW;
boolean over_voltage_flag = LOW;
boolean tank_full_flag = LOW;
boolean medium_voltage_flag = LOW;
boolean low_voltage_flag = LOW;
boolean pump_current_flag = LOW;
boolean manual = false;         // by default start in "run" mode
boolean last_manual = false;
boolean uvp_flag = true;        // (UVP) Under Voltage Protection. UVP flag is "true" on startup and turn to "true" whenever the input voltage goes below (UNDER_VOLTAGE_PROTECTION - UVP_HYSTERESIS). UVP flag turns to "false" whenever the input voltage goes above UNDER_VOLTAGE_PROTECTION.
boolean last_on_off = true;     //
long counter = 0;
String in_string;
String man = "man";
String mppt_run = "run";
String q1_on = "on";
String q1_off = "off";
String pwm_active = "pwm_on";
String pwm_off = "pwm_off";

void setup() {
  Serial.begin(115200);
  Wire.begin(); //Join the I2C as master

  //Set all output ports to LOW state
  //This will ensure starting with Q1 and Q2 in cutoff state and all LEDs are off
  //Except for the POWER LED which is always on when power is applied.
  digitalWrite(PWM_ON_IO_PIN, LOW);
  digitalWrite(PWM_OUT, LOW);
  digitalWrite(PV_EN, LOW);
  digitalWrite(OVER_TEMPERATURE_LED, LOW);
  digitalWrite(TANK_FULL_LED, LOW);
  digitalWrite(OVER_VOLTAGE_LED, LOW);
  digitalWrite(MEDIUM_VOLTAGE_LED, LOW);
  digitalWrite(LOW_VOLTAGE_LED, LOW);
  digitalWrite(PUMP_CURRENT_LED, LOW);

  //Set the following I/O ports to output (All other ports are input by default)
  pinMode(PWM_ON_IO_PIN, OUTPUT);
  pinMode(PV_EN, OUTPUT);
  pinMode(OVER_TEMPERATURE_LED, OUTPUT);
  pinMode(TANK_FULL_LED, OUTPUT);
  pinMode(OVER_VOLTAGE_LED, OUTPUT);
  pinMode(MEDIUM_VOLTAGE_LED, OUTPUT);
  pinMode(LOW_VOLTAGE_LED, OUTPUT);
  pinMode(PUMP_CURRENT_LED, OUTPUT);

  //Set the following I/O ports to input with pullup
  pinMode (ON_OFF_IO_PIN, INPUT_PULLUP);

  /* Set Timer 2 to PWM with max count of 80. The selected clock is CPU clock whith prescaler = 1 */

  OCR2A = PWM_PERIOD;
  OCR2B = duty_cycle; //Set initial Duty Cycle

  //Set timer2 to Fast PWM mode
  TCCR2A |= ((1 << WGM21) | (1 << WGM20));
  TCCR2B |= (1 << WGM22);

  //Clear OC2B on Compare Match, Set OC2B at BOTTOM
  TCCR2A |= (1 << COM2B1);

  //Set the prescaler to CPU clock (i.e. no prescaler)
  TCCR2B |= (1 << CS20);
  TCCR2B &= ~((1 << CS22) | (1 << CS21));

  pinMode(PWM_OUT, OUTPUT); //Starts the PWM at the Arduino D3 pin.

  /* end of Timer 2 set up */

  //  digitalWrite(PV_EN, HIGH); //Turn Q1 on
  //  delay(1000); //Let input capacitor bank to get some charge
  //  digitalWrite(PWM_ON_IO_PIN, HIGH); //enable the N-MOSFET gate driver
  //  delay(1000); //let the system sabilize

  //Read initial IPv and Vpv values
  i_pv_n_1 = float(analogRead(I_PV_IN) - 512) * k_i;
  v_pv_n_1 = float(analogRead(V_PV_IN) - 512) * k_i;
  last_millis = millis();
  last_temperature_measure = millis();
  last_tank_full_time = millis();
  delay(loop_time);
  Serial.println();
  Serial.println("Starting");
}

void loop() {
  digitalWrite(OVER_TEMPERATURE_LED, HIGH);//debug
  delay(1);
  digitalWrite(OVER_TEMPERATURE_LED, LOW);//debug
  last_millis = millis();
  measure_I_V();
  under_voltage_protection();
  check_monitor();
  if (!manual)
    MPPT();
  OCR2B = duty_cycle;//set timer2 (PWM) to the new duty cycle
  temperature = over_temperature_check();
  over_voltage_check();
  tank_full_check();
  On_Off_decision();//Remark: in manual mode the system does not return automatically to "ON"
  Update_LEDs();
  //watch dog timer
  //brown out detector
  Debug_print();
  //Prepare variables for next iteration through the loop // moved here on 03-11-2016
  i_pv_n_1 = i_pv_n; //i_pv_n-1
  v_pv_n_1 = v_pv_n; //v_pv_n-1
  while ((millis() - last_millis) < loop_time);
}

void check_monitor() {
  Serial.println("\f");
  if (!manual) {
    Serial.println("\t****************");
    Serial.println("\t* MPPT running *");
    Serial.println("\t****************");
    Serial.println("\ttype \"man\" to go to Manual mode\n");
  }
  else {
    Serial.println("\t***************");
    Serial.println("\t* Manual Mode *");
    Serial.println("\t***************");
    Serial.println("\ttype \"run\" to go to MPPT running mode\n");
    Serial.println("\ttype \"on\" to to turn Q1 on\n");
    Serial.println("\ttype \"off\" to to turn Q1 off\n");
    Serial.println("\ttype \"pwm_on\" to to turn the PWM on\n");
    Serial.println("\ttype \"pwm_off\" to to turn the PWM off\n");
#if PWM_PERIOD == 79
    Serial.println("\ttype a number between \"0\" and \"53\" to set the PWM duty cycle\n"); // for 200KHz PWM uncomment this line
#elif PWM_PERIOD ==  159
    Serial.println("\ttype a number between \"0\" and \"106\" to set the PWM duty cycle\n"); // for 100KHz PWM uncomment this line
#endif
  }
  if (Serial.available()) {
    in_string = Serial.readString();
    Serial.println(in_string);
    if (mppt_run.substring(0, 2) == in_string.substring(0, 2)) {
      manual = false;
      if (manual != last_manual) {
        last_manual = manual;
        digitalWrite(PV_EN, HIGH); //Turn  Q1 on
        delay (1000);
        digitalWrite(PWM_ON_IO_PIN, HIGH); //enable the N-MOSFET gate driver
        delay (1000);
      }
    }
    if (man.substring(0, 2) == in_string.substring(0, 2)) {
      manual = true;
      if (manual != last_manual) {
        last_manual = manual;
        digitalWrite(PV_EN, LOW); //Turn  Q1 off
        digitalWrite(PWM_ON_IO_PIN, LOW); //disable the N-MOSFET gate driver
      }
    }
    if (manual) {
      if (q1_on.substring(0, 1) == in_string.substring(0, 1)) {
        digitalWrite(PV_EN, HIGH); //Turn  Q1 on
        delay (1000);
      }
      if (q1_off.substring(0, 2) == in_string.substring(0, 2)) {
        digitalWrite(PV_EN, LOW); //Turn  Q1 off
      }
      if (pwm_active.substring(0, 5) == in_string.substring(0, 5)) {
        digitalWrite(PWM_ON_IO_PIN, HIGH); //enable the N-MOSFET gate driver
        delay (1000);
      }
      if (pwm_off.substring(0, 6) == in_string.substring(0, 6)) {
        digitalWrite(PWM_ON_IO_PIN, LOW); //disable the N-MOSFET gate driver
        delay (1000);
      }
      in_string = in_string.substring(0, 2);
      in_string[2] = "\0";
      if (isDigit(in_string[0]) && isDigit(in_string[1])) {
        Serial.println(in_string);
        int i = in_string.toFloat();
        Serial.println(i);
#if PWM_PERIOD == 79
        if (i >= 0 && i <= 53)// 53/80 is the maximum aloud dut cycle for 200KHz PWM. above it we might get too high output voltage.
#elif PWM_PERIOD ==  159
        if (i >= 0 && i <= 106)// 106/160 is the maximum aloud dut cycle for 100KHz PWM. above it we might get too high output voltage.
#endif
          duty_cycle = i;
      }
    }
  }
}

void measure_I_V() {
  int  Ipv = analogRead(I_PV_IN) - 512;
  int  Vpv = analogRead(V_PV_IN);
  int  Ipump = analogRead(I_PUMP_IN) - 512;
  int  Vpump = analogRead(V_PUMP_IN);
  i_pv_n = float(Ipv) * k_i;
  v_pv_n = k_v_pv * float(Vpv);
  i_pump = float(Ipump) * k_i;
  v_pump = k_v_pump * float(Vpump);
}

void under_voltage_protection() {
  if (uvp_flag) {
    if (v_pv_n > UNDER_VOLTAGE_PROTECTION) uvp_flag = false;
  }
  else {
    if (v_pv_n < (UNDER_VOLTAGE_PROTECTION - UVP_HYSTERESIS)) uvp_flag = true;
  }
}

void MPPT() {
  delta_i_in = i_pv_n - i_pv_n_1;
  delta_v_in = v_pv_n - v_pv_n_1;
  if ((i_pv_n / v_pv_n) > -(delta_i_in / delta_v_in))
    if (duty_cycle < PWM_PERIOD - 1)
      duty_cycle++;
    else //just in case
      duty_cycle = PWM_PERIOD - 1;

  //  if ((i_pv_n / v_pv_n) == -(delta_i_in / delta_v_in)) {do noting}

  if ((i_pv_n / v_pv_n) < -(delta_i_in / delta_v_in))
    if (duty_cycle > 0)
      duty_cycle --;
    else //just in case
      duty_cycle = 0;
#if PWM_PERIOD == 79
  if (duty_cycle > 53) duty_cycle = 53;
#elif PWM_PERIOD == 159
  if (duty_cycle > 106) duty_cycle = 106;
#endif
}

int over_temperature_check() {
  int c[2];                    // array for two temp bytes
  int x = 1;                   // counter for array (msb is send first)
  if ((millis() - last_temperature_measure) > 300) {//over temperature should be checked in greater than 300ms intervals
    last_temperature_measure = millis();
    Wire.requestFrom(B1001000, 2);    // request 2 bytes from slave device #72
    delay(10);
    while (Wire.available()) {   // get the two bytes
      c[x] = Wire.read();
      x--;
    }
    //  Action to take in case of over temperature
    if (c[1] >= OVER_TEMPERATURE_PARAMETER) { // We only check the MSB of the temperature reading (ignoring the fraction part of the degree)
      over_temperatue_flag = HIGH;
    }
    //  Action to take in case of normal temperature (Over_temperature - 10°C)
    if (over_temperatue_flag == HIGH)
      if (c[1] <= (OVER_TEMPERATURE_PARAMETER - OVER_TEMPERATURE_HYSTERESIS)) {
        over_temperatue_flag = LOW;
      }
  }
  return (c[1]);
}

void over_voltage_check() {
  if (!digitalRead(OVER_VOLTAGE_IO_PIN)) { // Notice: when over voltage occured the Over_voltage pin is LOW
    over_voltage_flag = HIGH;
  }
  else {
    over_voltage_flag = LOW;
  }
}

void tank_full_check() {
  if (!digitalRead(TANK_FULL_IO_PIN)) { // Notice: when Tank full occured the Tank_full pin is LOW
    tank_full_flag = HIGH;
    last_tank_full_time = millis();
  }
  else {
    if ((millis() - last_tank_full_time) > TANK_FULL_DELAY) {
      tank_full_flag = LOW;
    }
  }
}

void On_Off_decision() {
  if (over_temperatue_flag/* || over_voltage_flag */ || tank_full_flag || uvp_flag || digitalRead(ON_OFF_IO_PIN)) {
    digitalWrite(PV_EN, LOW); //Cutoff P-MOSFET Q1
    digitalWrite(PWM_ON_IO_PIN, LOW); //Stop the PWM
  }
  else {
    if (!manual) { //in run mode only, the system  return automatically to "ON" when  all protection flags goes "false"
      digitalWrite(PV_EN, HIGH); //Turn on P-MOSFET Q1
      digitalWrite(PWM_ON_IO_PIN, HIGH); //Run the PWM
    }
  }
}

void Update_LEDs() {

  if (over_temperatue_flag)
    digitalWrite(OVER_TEMPERATURE_LED, HIGH); //Turn on over temperature LED
  else
    digitalWrite(OVER_TEMPERATURE_LED, LOW); //Turn off over temperature LED

  if (tank_full_flag)
    digitalWrite(TANK_FULL_LED, HIGH); //Turn on tank full LED
  else
    digitalWrite(TANK_FULL_LED, LOW); //Turn off tank full LED

  if (over_voltage_flag)
    digitalWrite(OVER_VOLTAGE_LED, HIGH); //Turn on over voltage LED
  else
    digitalWrite(OVER_VOLTAGE_LED, LOW); //Turn off over voltage LED

  if ((v_pump >= LOW_VOLTAGE_PARAMETER) && !over_voltage_flag) {
    digitalWrite(MEDIUM_VOLTAGE_LED, HIGH); //Turn on medium voltage LED
    medium_voltage_flag = HIGH;
  }
  else {
    digitalWrite(MEDIUM_VOLTAGE_LED, LOW); //Turn off medium voltage LED
    medium_voltage_flag = LOW;
  }

  if (v_pump < LOW_VOLTAGE_PARAMETER) {
    digitalWrite(LOW_VOLTAGE_LED, HIGH); //Turn on Low voltage LED
    low_voltage_flag = HIGH;
  }
  else {
    digitalWrite(LOW_VOLTAGE_LED, LOW); //Turn off Low voltage LED
    low_voltage_flag = LOW;
  }

  if (i_pump > I_PUMP_THRESHOLD) {
    digitalWrite(PUMP_CURRENT_LED, HIGH); //Turn on pump current LED
    pump_current_flag = HIGH;
  }
  else {
    digitalWrite(PUMP_CURRENT_LED, LOW); //Turn off pump current LED
    pump_current_flag = LOW;
  }

}

void Debug_print() {
  Serial.print ("\tAlgorithm time: "); Serial.print (millis() - last_millis); Serial.println ("[milisec]");
  Serial.println("\n\t-----------------READINGS-----------------");
  Serial.print("\tIpv = "); Serial.print(-i_pv_n); Serial.print("[A]\tIpv(n-1) = "); Serial.print(-i_pv_n_1); Serial.print("[A]\tdelta Iin = "); Serial.print(-delta_i_in); Serial.println("[A]");
  Serial.print("\tVpv = "); Serial.print(v_pv_n); Serial.print("[V]\tVpv(n-1) = "); Serial.print(v_pv_n_1); Serial.print("[V]\tdelta Vin = "); Serial.print(delta_v_in); Serial.println("[V]");
  Serial.println();
  Serial.print("\tIpump = "); Serial.print(-i_pump); Serial.println("[A]");
  Serial.print("\tVpump = "); Serial.print(v_pump); Serial.println("[V]");
  Serial.println();
  Serial.println("\tLEDs status:");
  Serial.println("\tOver\tTank\tOver\tMedium\tLOW\tPump");
  Serial.println("\ttemp.\tFull\tvoltage\tvoltage\tvoltage\tcurrent");
  Serial.print ("\t"); Serial.print(over_temperatue_flag); Serial.print("\t"); Serial.print(tank_full_flag); Serial.print("\t"); Serial.print(over_voltage_flag); Serial.print("\t"); Serial.print(medium_voltage_flag); Serial.print("\t"); Serial.print(low_voltage_flag); Serial.print("\t"); Serial.println(pump_current_flag);
  Serial.println();
  Serial.print("\tDuty Cycle = "); Serial.print(100 * float(duty_cycle) / float(PWM_PERIOD)); Serial.print("%; OCR2B = "); Serial.println(OCR2B);
  Serial.print("\tTemperature: "); Serial.print(temperature); Serial.println("°C");
  Serial.print ("\tcounter: "); Serial.println(counter++);
  Serial.println("\t******************************************\n");

}


