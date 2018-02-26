/*
 * This is a multi probe temperature control system for sous vide or kilns or whatever you want to use it with. 
 * It uses a generic PCF8574 I2C LCD screen, a MAX6675 thermocouple board, and a DS18B20 temperature sensor.
 * Pick the type of temperature probe you're going to use before turning it on.
 */


//set the default temperature on startup, different temps for different probes
const int DefaultHigh = 200;
const int DefaultLow = 130;





#include <DallasTemperature.h>
#include <PID_v1.h>
#include "max6675.h"
#include <PID_AutoTune_v0.h>
#include <Automaton.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>


Atm_button autotunebtn, upbtn, downbtn; // An Automaton button machine

// ************************************************
// Pin definitions
// ************************************************

#define ONE_WIRE_BUS 2  //onewire temperature sensor is on this port
#define RELAY_PIN 13     //relay output pin
#define hightemppin 10 //temperature switch changes between thermocouple and sensor, closed is sensor, open is thermocouple
#define autotunebtnpin 7
#define upbtnpin 8
#define downbtnpin 9
//#define tunepin 13

//thermocouple pins
const int thermoDO = 4;
const int thermoCS = 5;
const int thermoCLK = 6;


// Pass our oneWire reference to Dallas Temperature. 
// ************************************************
// Sensor Variables and constants
// Data wire is plugged into port 2 on the Arduino

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device address
DeviceAddress tempSensor;




MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);



// ************************************************
// PID Variables and constants
// ************************************************

//Define Variables we'll be connecting to
double Setpoint;
double Input;
double Output;


volatile long onTime = 0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

double temperature;
boolean hightemp;
long tempdisplay;
long setdisplay;
unsigned long previousMillis = 0;        // will store last time LED was updated
const long interval = 300;           // interval at which to blink (milliseconds)
boolean relay;

// 10 second Time Proportional Output window
int WindowSize = 10000; 
unsigned long windowStartTime;

// EEPROM addresses for persisted data
const int SpAddresslow = 0;
const int KpAddresslow = 8;
const int KiAddresslow = 16;
const int KdAddresslow = 24;
const int SpAddresshigh = 32;
const int KpAddresshigh = 40;
const int KiAddresshigh = 48;
const int KdAddresshigh = 56;

// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;

boolean tuning = false;

PID_ATune aTune(&Input, &Output);

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;

LiquidCrystal_PCF8574 lcd(0x27);  // set the LCD address to 0x27 for a 16 chars and 2 line display



void setup() {

  int error;

  // See http://playground.arduino.cc/Main/I2cScanner
  Wire.begin();
  Wire.beginTransmission(0x27);
  error = Wire.endTransmission();
  lcd.begin(16, 2); // initialize the lcd
  lcd.setBacklight(255);
    lcd.home(); lcd.clear();
    lcd.print("Temperature");
    lcd.setCursor(0,1);
    lcd.print("Control");


 windowStartTime = millis();
  pinMode(hightemppin, INPUT);
  digitalWrite(hightemppin, HIGH);
  delay(2000);
  hightemp = digitalRead(hightemppin);

 // Initialize the PID and related variables
   LoadParameters();
   myPID.SetTunings(Kp,Ki,Kd);

   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);
   myPID.SetMode(AUTOMATIC);
  
 // Start up the library
  sensors.begin();
  sensors.setResolution(tempSensor, 12);
  sensors.setWaitForConversion(false);


  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH);
  //pinMode(tunepin, OUTPUT);
  //digitalWrite(tunepin, LOW);

//buttons
    upbtn.begin(upbtnpin) 
    .onPress(pressup)
    .repeat();
    downbtn.begin(downbtnpin)
    .onPress(pressdown)
    .repeat();
    autotunebtn.begin(autotunebtnpin)
    .onPress(StartAutoTune);

        lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();
   if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    
  
  // put your main code here, to run repeatedly:
  if (hightemp == 1) {
    temperature = thermocouple.readFahrenheit();
  }
  else {

    temperature = sensors.getTempFByIndex(0);  

  }

  Input = temperature;
  sensors.requestTemperatures(); // Send the command to get temperatures
  
  //LCD control
//lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set:");
    setdisplay = Setpoint;
    lcd.print(setdisplay);
    lcd.print(" F");
        if (tuning) {
      lcd.print(" TUNING");
    }
    else {
      lcd.print("       ");
    }
    lcd.setCursor(0, 1);
    lcd.print("Current:");
    tempdisplay = temperature;
    lcd.print(tempdisplay);
    lcd.print(" F");
    if(relay) {
      lcd.setCursor(15,1);
      lcd.print("0");
    }
    else {
      lcd.setCursor(15,1);
      lcd.print(" ");
    }
   }
  
  if (tuning) // run the auto-tuner
  {
    //digitalWrite(tunepin, HIGH);
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
    //digitalWrite(tunepin, LOW);
     myPID.Compute();
  }
    onTime = Output;
   /************************************************
   * turn the output pin on/off based on pid output
   ************************************************/
   long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 100) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(RELAY_PIN,LOW);
     relay = 1;
  }
  else
  {
     digitalWrite(RELAY_PIN,HIGH);
     relay = 0;
  }


  automaton.run();  
  

}

void pressup() {
  if (hightemp == 1) {
    Setpoint = Setpoint + 10;
  }
  else {
    Setpoint = Setpoint +1;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
    lcd.print("Set:");
    setdisplay = Setpoint;
    lcd.print(setdisplay);
    lcd.print(" F");
}
 
void pressdown() {
  if (hightemp == 1) {
    Setpoint = Setpoint - 10;
  }
  else {
    Setpoint = Setpoint - 1;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
    lcd.print("Set:");
    setdisplay = Setpoint;
    lcd.print(setdisplay);
    lcd.print(" F");
}

// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}
// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune()
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters(){
if (hightemp)
{
//   if (Setpoint != EEPROM_readDouble(SpAddresshigh))
//   {
//      EEPROM_writeDouble(SpAddresshigh, Setpoint);
//   }
   if (Kp != EEPROM_readDouble(KpAddresshigh))
   {
      EEPROM_writeDouble(KpAddresshigh, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddresshigh))
   {
      EEPROM_writeDouble(KiAddresshigh, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddresshigh))
   {
      EEPROM_writeDouble(KdAddresshigh, Kd);
   }
}
if (!hightemp)
{
//   if (Setpoint != EEPROM_readDouble(SpAddresslow))
//   {
//      EEPROM_writeDouble(SpAddresslow, Setpoint);
//   }
   if (Kp != EEPROM_readDouble(KpAddresslow))
   {
      EEPROM_writeDouble(KpAddresslow, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddresslow))
   {
      EEPROM_writeDouble(KiAddresslow, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddresslow))
   {
      EEPROM_writeDouble(KdAddresslow, Kd);
   }
}
}
// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()

{
  if (hightemp){
  // Load from EEPROM
//   Setpoint = EEPROM_readDouble(SpAddresshigh);
   Setpoint = DefaultHigh;
   Kp = EEPROM_readDouble(KpAddresshigh);
   Ki = EEPROM_readDouble(KiAddresshigh);
   Kd = EEPROM_readDouble(KdAddresshigh);
   
   // Use defaults if EEPROM values are invalid
//   if (isnan(Setpoint))
//   {
//     Setpoint = 60;
//   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   } 
  }
  if (!hightemp)
  {
  // Load from EEPROM
//   Setpoint = EEPROM_readDouble(SpAddresslow);
   Setpoint = DefaultLow;
   Kp = EEPROM_readDouble(KpAddresslow);
   Ki = EEPROM_readDouble(KiAddresslow);
   Kd = EEPROM_readDouble(KdAddresslow);
   
   // Use defaults if EEPROM values are invalid
//   if (isnan(Setpoint))
//   {
//     Setpoint = 60;
//   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 0.5;
   }
   if (isnan(Kd))
   {
     Kd = 0.1;
   }  
} 
}
// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}
