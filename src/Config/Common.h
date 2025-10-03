#ifndef Common_h
#define Common_h

#include <SdFat.h> //SdFat library writes way faster than the standard SD library, but it's more complicated
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h> //wtf does this one even do idk.
#include <Adafruit_AHRS.h> 
#include <Adafruit_ADXL375.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20948.h>
#include <BMP388_DEV.h>
#include <TinyGPS++.h> //for built-in UC6580 GNSS on the Heltec Wireless Tracker
#include <RadioLib.h>
#include "Buzzer.h" //buzzer tone functions - this was first built for aruduino Nano boards, could be updated for ESP32.
#include "Control.h" //deployment logic etc.
#include "TFT/TFTscreen.h" //for built-in TFT screen on the Heltec Wireless Tracker
#include "Config/Serial_Debug.h" //Serial monitor debugging messages
#include "Config/Hardware_Config.h"
#include "Config/Flight_Config.h"
#include "Config/UC6580_Config.h" //Library for configuring the GPS sentences and output rates etc. Not finished yet, few things to work through yet,
#include "Kalman.h"
#endif