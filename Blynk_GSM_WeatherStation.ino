#define BLYNK_TEMPLATE_ID "XXXXXXXXXXXXXXXXXX" //Enter your Blynk Template ID
#define BLYNK_TEMPLATE_NAME "XXXXXXXXXXXXXXXXXXXXXXXXX" //Enter your Blynk Template Name
#define BLYNK_AUTH_TOKEN            "XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX" //Enter your Blynk auth token
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space

// Default heartbeat interval for GSM is 60
// If you want override this value, uncomment and set this option:
// #define BLYNK_HEARTBEAT 30
 #define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <BlynkSimpleTinyGSM.h>
#include <Wire.h>                                        //I2C needed for sensors
#include "SparkFunMPL3115A2.h"                           //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFun_Si7021_Breakout_Library.h"            //Humidity sensor - Search "SparkFun Si7021" and install from Library Manager
#include "SparkFun_Weather_Meter_Kit_Arduino_Library.h"  //Weather meter kit - Search "SparkFun Weather Meter" and install from Library Manager

//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;
const byte STAT2 = 8;
// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//long lastSecond;  //The millis counter to see when a second rolls by
float humidity = 0;  // [%]
float tempC = 0;     // [temperature C]
float pressure = 0;
float wind_dir = 0;    // [degrees (Cardinal)]
float wind_speed = 0;  // [kph]
float rain = 0;        // [mm]
float batt_lvl = 11.8;  //[analog value from 0 to 1023]
float light_lvl = 455;  //[analog value from 0 to 1023]
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Set serial for debug console (to the Serial Monitor, default speed 115200)
#define SerialMon Serial
#include <SoftwareSerial.h>
SoftwareSerial SerialAT(7, 8);  // RX, TX

// Your GPRS credentials, if any
const char apn[]  = "XXXXXXXXXXXXXXXXXXXXXXXXXXXX"; //Enter your APN
const char user[] = "";
const char pass[] = "";

TinyGsm modem(SerialAT);
BlynkTimer timer;

MPL3115A2 myPressure;                                      //Create an instance of the pressure sensor
SI7021 myHumidity;                                        //Create an instance of the humidity sensor
SFEWeatherMeterKit myweatherMeterKit(WDIR, WSPEED, RAIN);  // Create an instance of the weather meter kit


void setup()
{
  // Set console baud rate
  SerialMon.begin(9600);
  delay(10);
  
  // Set GSM module baud rate
  SerialAT.begin(9600);
  delay(6000);
  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  //SerialMon.println("Initializing modem...");
  modem.restart();
  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);
  Blynk.begin(BLYNK_AUTH_TOKEN, modem, apn, user, pass);
  // Setup a function to be called every second
  timer.setInterval(1000L, myTimerEvent);
  
  myPressure.begin();               // Get sensor online
  myPressure.setModeBarometer();    // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7);  // Set Oversample to the recommended 128
  myPressure.enableEventFlags();    // Enable all three pressure and temp event flags
  myHumidity.begin();               //Configure the humidity sensor
  myweatherMeterKit.setADCResolutionBits(10); // Configuring a 10-bit ADC resolution for the ATmega328 (RedBoard/Uno)
  myweatherMeterKit.begin(); // Begin weather meter kit
  Serial.println("Weather Shield online!");

}

void loop()
{
  Blynk.run();
  digitalWrite(STAT1, HIGH);  //Blink stat LED
  timer.run(); // Initiates BlynkTimer
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level() {
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float lightSensor = analogRead(LIGHT);
  operatingVoltage = 3.3 / operatingVoltage;  //The reference voltage is 3.3V
  lightSensor = operatingVoltage * lightSensor;
  return (lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level() {
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.30 / operatingVoltage;  //The reference voltage is 3.3V
  rawVoltage = operatingVoltage * rawVoltage;  //Convert the 0 to 1023 int to actual voltage on BATT pin
  rawVoltage *= 4.90;  //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage
  return (rawVoltage);
}
void myTimerEvent()
{
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
   humidity = myHumidity.getRH(); // Calc humidity from Si7021 sensor
  Blynk.virtualWrite(V2,humidity);
   tempC = myHumidity.getTemperature(); //Calc temp from Si7021 sensor
  Blynk.virtualWrite(V3,tempC);
   pressure = myPressure.readPressure()*0.01; //Calc pressure from MPL3115A2
  Blynk.virtualWrite(V4,pressure);
   wind_dir = myweatherMeterKit.getWindDirection();  //Calc Wind Direction
  Blynk.virtualWrite(V5,wind_dir);
    wind_speed = myweatherMeterKit.getWindSpeed();  //Calc Wind Speed
  Blynk.virtualWrite(V6,wind_speed);
 rain = myweatherMeterKit.getTotalRainfall(); //Calc Rain
  Blynk.virtualWrite(V7,rain); 
    light_lvl = get_light_level(); //Calc light level
  Blynk.virtualWrite(V8,light_lvl);
   batt_lvl = get_battery_level(); //Calc battery level
  Blynk.virtualWrite(V9,batt_lvl);
  digitalWrite(STAT1, LOW);  //Turn off stat LED
}
