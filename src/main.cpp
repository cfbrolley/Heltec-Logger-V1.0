//Libraries
#include <SdFat.h> //SdFat library writes way faster than the standard SD library, but it's more complicated
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h> //wtf does this one even do idk.
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

//SD pins
const uint8_t SD_CS_PIN = 46;
const uint8_t SOFT_MISO_PIN = 17;
const uint8_t SOFT_MOSI_PIN = 15;
const uint8_t SOFT_SCK_PIN = 16;
SoftSpiDriver<SOFT_MISO_PIN, SOFT_MOSI_PIN, SOFT_SCK_PIN> softSpi;

//Global variables
//float maxaccel, TTA; //max acceleration and time to apogee (TTA) not currently implemented
float pressure, altitude, temperature, altioffset, correctedalt; //barometer variables
float ax, ay, az, gx, gy, gz, mx, my, mz, hrax, hray, hraz; //accelerometer, gyro, magnetometer variables
float maxalt = 0;
float lastLatitude = 0.0;
float lastLongitude = 0.0;
float GPSalt = 0.0;
float satsfound = 0.0;
unsigned long lastGPSUpdate = 0;
unsigned long buttonPressTime = 0; // Track button press duration
byte droguecount = 0;
byte maincount = 0;
byte logtimeout = 0;
unsigned long timer, LEDtimer, radiotimer, flushclock, batteryclock, apogeetime, liftofftime, timetoapogee; //various timers. some of these yet not implemented
bool bmpready = false; //flag for barometer data collected
bool GPSready = false;
bool LEDstate = false; //flag for onboard LED on or off
bool GPSlock = true;
bool screenstate = true;
int logNumber = 0; //file numbering
int msgID = 1000;
int logtime = 4; //set a minimum logging interval in ms.
const uint8_t rowMarker[4] = {0xFF, 0xFE, 0xFD, 0xFC};  // Unique row marker
String dataToSend = "";
String msgstring = "";
String latstring = "";
String longstring = "";
String altstring = "";

//Function forward declarations
void commsTask(void *parameter);
void loggingTask(void *parameter);
void events(int state);

//The rest
TaskHandle_t commsTaskHandle = NULL;
TaskHandle_t loggingTaskHandle = NULL;
TwoWire SensorWire = TwoWire(0); //I2c bus for the sensors
BMP388_DEV bmp(7, 6, SensorWire); //BMP library is passed alternate I2C pins and bus object
Adafruit_ADXL375 adxl = Adafruit_ADXL375(0x53, &SensorWire);
Adafruit_ICM20948 icm;
TFTscreen screen;
TinyGPSPlus GPS;
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RESET, LORA_DIO2);
SdFat SD;
SdFile logfile;
Buzzer Buzz(45);
Control ControlInstance(droguepin, mainpin, lockout, events); //set pins to trigger ejection chargers, safety lockout altitude (so ejection charges don't accidentally go off at ground level) and what function to callback to on state change
Serial_Debug Debug(115200);
UCCONFIG UC6580;

void setup() {
/****************************************/
/* Start buzzer library
 * Display splash screen
/****************************************/
  pinMode(LEDpin, OUTPUT);
  SensorWire.begin(I2C_SDA, I2C_SCL, 400000); //start alternate I2C bus
  screen.TFT_init(); //start screen
  screen.fill_screen(BLACK);
  digitalWrite(LEDpin, HIGH); // onboard LED on
  Buzz.startup();
  for(int x = 0; x < TFTWIDTH; x++) {
     screen.draw_pixel(x, 0, RED);
     screen.draw_pixel(x, TFTHEIGHT-1, RED);
     }
  for(int y = 0; y < TFTHEIGHT; y++) {
     screen.draw_pixel(0, y, RED);
     screen.draw_pixel(TFTWIDTH-1, y, RED);
     }
  screen.write_str(50, 22, "ROLLEY", Font_Medium, RED, BLACK);
  screen.write_str(35, 47, "ROCKETRY", Font_Medium, RED, BLACK);
  delay(2000);
  screen.fill_screen(BLACK);
  screen.write_str(0, 0, "Running setup", Font_Small, GREEN, BLACK);
  delay(200);

/****************************************/
/* Battery monitoring setup
/****************************************/
  pinMode(ADC_CTRL, OUTPUT);
  digitalWrite(ADC_CTRL, HIGH); // Enable voltage divider
  analogReadResolution(12);
  analogSetPinAttenuation(1,ADC_2_5db); // Sets the input attenuation, default is ADC_11db, range is ADC_0db, ADC_2_5db, ADC_6db, ADC_11db

/****************************************/
/* Setup control functions
/****************************************/
  ControlInstance.begin();
  ControlInstance.SetSafetyLock(20);
  ControlInstance.SetMainAltitude(mainalti);
  digitalWrite(LEDpin, LOW); // onboard LED Off
  delay(100);

/****************************************/
/* Start GPS
/****************************************/
  digitalWrite(LEDpin, HIGH); // onboard LED on
  screen.write_str(0, 15, "Starting GPS...", Font_Small, GREEN, BLACK);
  Serial1.begin(GNSS_BAUD, SERIAL_8N1, GNSS_RX, GNSS_TX);
  UC6580.AutoConfig();
  pinMode(VGNSS_CTRL, OUTPUT);
  digitalWrite(VGNSS_CTRL, HIGH);
  delay(150);
  digitalWrite(LEDpin, LOW); // onboard LED Off
  screen.write_str(105, 15, " OK!", Font_Small, GREEN, BLACK);
  delay(75);

/****************************************/
/* Start LoRa module
/****************************************/
  screen.write_str(0, 27, "Starting LoRa...", Font_Small, GREEN, BLACK);
  int state = radio.begin(LORA_BAND, LORA_BANDWIDTH, LORA_SPREAD_FACTOR, LORA_CODING_RATE, LORA_SYNC, TX_OUTPUT_POWER, LORA_PREAMBLE, TX_VOLTAGE, false);
  if (state == RADIOLIB_ERR_NONE) {
      screen.write_str(105, 27, " OK!", Font_Small, GREEN, BLACK);
      } else {
             Buzz.error();
             digitalWrite(LEDpin, LOW); // onboard LED off
             screen.write_str(105, 27, " ERROR!", Font_Small, RED, BLACK);
             while (1);
             }

/****************************************/
/* Start BMP388
/****************************************/
  digitalWrite(LEDpin, HIGH); // onboard LED on
  screen.write_str(0, 39, "Starting BMP...", Font_Small, GREEN, BLACK);
  int rslt;
  if (!bmp.begin(BMP388_I2C_ALT_ADDR)) { //the BMP388 on the GY-912 breakout board uses alternate IIC address
     Debug.debugBMP(1, 0);
     Buzz.error();
     digitalWrite(LEDpin, LOW); // onboard LED off
     screen.write_str(105, 39, " ERROR!", Font_Small, RED, BLACK);
    // while(1); //don't proceed if there's an error
     }
  bmp.setPresOversampling(OVERSAMPLING_X4);
  bmp.setTempOversampling(OVERSAMPLING_SKIP);
  bmp.setIIRFilter(IIR_FILTER_8);
  bmp.setTimeStandby(TIME_STANDBY_20MS); //This needs to match the total sampling time (eg: (# of pressure samples + # of temp samples) x5ms)
  bmp.startNormalConversion();
  for (int i = 0; i <= 5; i++) {
      bmp.getMeasurements(temperature, pressure, altitude); //take a few measurements to get a stable reading
      delay(40);
      }
  altioffset = round(altitude * 100) / 100; //Set an offest for height above ground. There's a better way to do this probably.
  Debug.debugBMP(3, altioffset);
  digitalWrite(LEDpin, LOW); // onboard LED Off
  screen.write_str(105, 39, " OK!", Font_Small, GREEN, BLACK);
  delay(75);

/****************************************/
/* Start low range IMU
/****************************************/
  digitalWrite(LEDpin, HIGH); // onboard LED on
  screen.write_str(0, 52, "Starting IMU...", Font_Small, GREEN, BLACK);
  if (!icm.begin_I2C(0x68, &SensorWire)) { //ICM library is passed I2C address and alternate bus object
     Debug.debugACC(1);
     Buzz.error();
     digitalWrite(LEDpin, LOW); // onboard LED off
     screen.write_str(105, 52, " ERROR!", Font_Small, RED, BLACK);
     while(1); //don't proceed if there's an error
     }
  icm.setAccelRateDivisor(1);
  icm.setAccelRange(ICM20948_ACCEL_RANGE_16_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_2000_DPS);
  icm.setMagDataRate(AK09916_MAG_DATARATE_100_HZ);
  digitalWrite(LEDpin, LOW); // onboard LED Off
  screen.write_str(105, 52, " OK!", Font_Small, GREEN, BLACK);
  delay(75);

/****************************************/
/* Start high range accelerometer
/****************************************/
digitalWrite(LEDpin, HIGH); // onboard LED on
screen.write_str(0, 64, "Starting ACC...", Font_Small, GREEN, BLACK);
if(!adxl.begin()) {
     Debug.debugACC(1);
     Buzz.error();
     digitalWrite(LEDpin, LOW); // onboard LED off
     screen.write_str(105, 64, " ERROR!", Font_Small, RED, BLACK);
     while(1); //don't proceed if there's an error
     }
  Debug.debugACC(2);
  digitalWrite(LEDpin, LOW); // onboard LED off
  screen.write_str(105, 64, " OK!", Font_Small, GREEN, BLACK);
  delay(75);

/****************************************/
/* Start SD
 * Check for existing flight logs
 * Create new file with unique number
 * Print header to the file
/****************************************/
  screen.fill_screen(BLACK);
  screen.write_str(0, 0, "Running setup", Font_Small, GREEN, BLACK);  
  digitalWrite(LEDpin, HIGH); // onboard LED on
  screen.write_str(0, 15, "Starting SD....", Font_Small, GREEN, BLACK);
  if (!SD.begin(SD_CONFIG)) {
     Debug.debugSD(1);
     Buzz.error();
     digitalWrite(LEDpin, LOW); // onboard LED off
     screen.write_str(105, 15, " ERROR!", Font_Small, RED, BLACK);
     //while (1); //Stopper. Won't continue without the SD card.
     }
  #define COMMA logfile.print(",");
  while (SD.exists("FltLog" + String(logNumber) + ".csv")) {
        logNumber++;
        }
  char filename[20];
  snprintf(filename, sizeof(filename), "FltLog%d.csv", logNumber);
  logfile.open(filename, O_WRITE | O_CREAT);
  delay(250); // Issues seem to happen without this short delay.
  logfile.println("~ HELTEC LOGGER v1.0 ~");
  logfile.println("~ Last compiled: 21-02-25 ~");
  logfile.println();
  logfile.println("ms,ax,ay,az,gx,gy,gz,mx,my,mz,hrax,hray,hraz,pres,alt,rel. alt,GPSalt,lat,long,");
  logfile.sync();
  Debug.debugSD(2);
  digitalWrite(LEDpin, LOW); // onboard LED off
  screen.write_str(105, 15, " OK!", Font_Small, GREEN, BLACK);
  delay(75);
  
/****************************************/
/* Check battery
 * Display a battery reading for a second or so
/****************************************/
  int rawADC = analogRead(1);
  int bat = (rawADC / 4.095 * 3.3 * ADC_MULTIPLIER);  
  screen.write_str(0, 27, "vbat = ", Font_Small, GREEN, BLACK);
  screen.write_str(49, 27, (String)bat, Font_Small, GREEN, BLACK);
  screen.write_str(77, 27, "mV", Font_Small, GREEN, BLACK);
  for (int i = 0; i <= 30; i++) {
      delay(100);   
      int rawADC = analogRead(1);
      int bat = (rawADC / 4.095 * 3.3 * ADC_MULTIPLIER);
      screen.write_str(49, 27, (String)bat, Font_Small, GREEN, BLACK);
      } 

/****************************************/
/* Indicate setup is done
 * Kick off pinned tasks
/****************************************/
  digitalWrite(LEDpin, HIGH); // onboard LED on
  Buzz.success();
  screen.fill_screen(BLACK);
  screen.write_str(20, 0, "Setup success!", Font_Small, GREEN, BLACK);
  delay(1500);
  screen.fill_screen(BLACK);
  digitalWrite(LEDpin, LOW); // onboard LED off
  digitalWrite(TFTpin, LOW); //turn off the screen to save power
  screenstate = false;
  delay(2000);
  xTaskCreatePinnedToCore(commsTask, "Comms Task", 8192, NULL, 10, &commsTaskHandle, 0);
  xTaskCreatePinnedToCore(loggingTask, "Logging Task", 8192, NULL, 10, &loggingTaskHandle, 1);
}

/****************************************/
/* Read IMU and add or subtract a calibration offset (not currently added in for this logger)
 *** Calibration values need to be adjusted for each logger.
 *** There is a better way to calibrate but this method works for testing.

 * Read high range accelerometer
 *** ADXL375 has the option to save offset in it's registers.
 *** run the ADXL375 offset calibration example to update.

 * Read barometer if it's time
 *** call ControlInstance.Deployment and pass altitude and Z acceleration for deployment checks
 *** Set flag to notify loop to print barometer readings
/****************************************/
void readsensors (void) {
  timer = millis();
  sensors_event_t a, g, temp, m;
  icm.getEvent(&a, &g, &temp, &m);
  ax = (round(a.acceleration.x * 100) / 100) - 0.19;
  ay = (round(a.acceleration.y * 100) / 100) + 0.04;
  az = (round(a.acceleration.z * 100) / 100) + 0.01;
  gx = (round(g.gyro.x * 100) / 100) + 0;
  gy = (round(g.gyro.y * 100) / 100) + 0.01;
  gz = (round(g.gyro.z * 100) / 100) + 0;
  mx = (round(m.magnetic.x * 100) / 100) + 0;
  my = (round(m.magnetic.y * 100) / 100) + 0;
  mz = (round(m.magnetic.z * 100) / 100) + 0;

  sensors_event_t event;
  adxl.getEvent(&event);
  hrax = (round(event.acceleration.x * 100) / 100) - 11.57;
  hray = (round(event.acceleration.y * 100) / 100) - 11.94;
  hraz = (round(event.acceleration.z * 100) / 100) + 4.32;

  if (bmp.getMeasurements(temperature, pressure, altitude)){
   pressure = round(pressure * 100) / 100; 
   altitude = round(altitude *100) / 100;   
   correctedalt = altitude - altioffset;
     ControlInstance.Deployment(correctedalt, az); //although not currently implemented in the ControlInstance library, function it set up to accept it so that it can be implemented in future, e.g detecting lift-off, recording maxx acceleration during boost, etc
     bmpready = true; //sets flag to say that there is some baro data to be written
     }
}

/****************************************/
/* Record a footer for log file.
 * Would like to move this in to a separate flight summary file instead at some point.
 * Need to add in an end of flight summary displayed on screen at some point.
 * Would be nice to at least see what apogee it hit when you collect the rocket.
/****************************************/
void endlog (void) {
  //TTA = apogeetime - liftofftime;
  logfile.println();
  logfile.print("max altitude: "); COMMA;
  logfile.print(ControlInstance.maxalt);
  logfile.print(" m, at: "); COMMA;
  logfile.println(ControlInstance.apogeetime);
  //  the following is not currently in use.
  //  logfile.print("liftoff: "); COMMA;
  //  logfile.print(liftofftime);
  //  logfile.println(" ms");
  //  logfile.print("Time to apogee:"); COMMA;
  //  logfile.print(TTA);
  //  logfile.println(" ms");
  logfile.println();
  logfile.print(" ~END OF LOG~");
  logfile.println();
  logfile.sync();
  logfile.close();
  digitalWrite(LEDpin, HIGH); //onboard LED on
  digitalWrite(TFTpin, HIGH); //turn off the screen to save power
  screen.write_str(0, 0, "Flight summary:", Font_Small, GREEN, BLACK);
  screen.write_str(0, 15, "max alt:", Font_Small, GREEN, BLACK);
  screen.write_str(63, 15, (String)ControlInstance.maxalt, Font_Small, GREEN, BLACK);
  screen.write_str(105, 15, "m", Font_Small, GREEN, BLACK);
  screen.write_str(0, 27, "at time:", Font_Small, GREEN, BLACK);
  screen.write_str(63, 27, (String)(ControlInstance.apogeetime / 1000), Font_Small, GREEN, BLACK);
  screen.write_str(105, 27, "sec", Font_Small, GREEN, BLACK);
  screen.write_str(0, 39, "max z:", Font_Small, GREEN, BLACK);
  screen.write_str(63, 39, "not logged", Font_Small, GREEN, BLACK);
  while(1) {
           Buzz.ended(); //ending tone
           }
}

/****************************************/
/* This is the function that is passed to the control library as the function to call back to on state change.
 * Allows user to set up some optional extra actions, like recording a note in the log file when apogee is detected or when the ControlInstance function has disarmed the pyro channels.
 * Knowing when there was a state change in the control logic is useful for understanding what happened and figuring out why certain logic was triggered on a flight if something didn't work as expected.
/****************************************/
void events (int state) {
  switch (state){
    case 0: COMMA; logfile.println("armed"); logfile.sync(); break;
    case 1: COMMA; logfile.println("drogue fired"); logfile.sync(); break;
    case 2: COMMA; logfile.println("main fired"); logfile.sync(); break;
    case 3: COMMA; logfile.println("disarmed"); logfile.sync(); break;
    default: break; //nothing to do here for states 4 and 5.
    }
}

/****************************************/
/* Function for screen toggle
 * Work in progress
 * Turns the screen on and off by holding the user button.
/****************************************/
void screencontrol() {
  if (digitalRead(BOOT_BUTTON) == LOW) {
     if (buttonPressTime == 0) {
        buttonPressTime = millis();
        }
     // Check if button has been held long enough
     if (millis() - buttonPressTime >= HoldDuration && screenstate) {
        digitalWrite(TFTpin, LOW); //turn off the screen to save power
        screenstate = false;
        } else if (millis() - buttonPressTime >= HoldDuration && !screenstate) {
                  digitalWrite(TFTpin, HIGH); //turn off the screen to save power
                  screenstate = true;
                  }
  } else {
         buttonPressTime = 0; // Reset if button is released
         }
}

/****************************************/
/* Run through the sensor reads and deployment checks

 * Log the readings to file.
 *** Occasionally flushes the SDfat data buffer to ensure data isn't lost where possible
 *** Note: this method might be inefficient, probably better to log in binary.

 * Also has a cool flashy LED light, yeehaa.
/****************************************/
void loggingTask(void *parameter) {
  for (;;) {
      if (millis() - timer >= logtime){
         readsensors();
         logfile.println();
         logfile.print(timer); COMMA;
         logfile.print(ax); COMMA;
         logfile.print(ay); COMMA;
         logfile.print(az); COMMA;
         logfile.print(gx); COMMA;
         logfile.print(gy); COMMA;
         logfile.print(gz); COMMA;
         logfile.print(mx); COMMA;
         logfile.print(my); COMMA;
         logfile.print(mz); COMMA;
         logfile.print(hrax); COMMA;
         logfile.print(hray); COMMA;
         logfile.print(hraz);

         if (bmpready){ //if baro data was collected, print it.
            COMMA;
            logfile.print(pressure); COMMA;
            logfile.print(altitude); COMMA;
            logfile.print(correctedalt);
            bmpready = false; //set the flag back until the next baro data is collected

            if (GPSready){ //if we're printing BMP data, check for GPS data and whack it on the end too
               COMMA;
               logfile.print(lastLatitude); COMMA;
               logfile.print(lastLongitude); COMMA;
               GPSready = false; //set the flag back until the next GPS data is collected
               }
            #ifdef datadebugging //Serial data printed only if the option was defined
            Debug.debugdata(timer, pressure, altitude, correctedalt, ax, ay, az, gx, gy, gz, mx, my, mz, ControlInstance.controlstate);
            #endif
            }


         //do this stuff only once per whatever flushtime is set to.
         if ((timer - flushclock >= flushtime)){
            logfile.sync(); //sync in the SDfat library does the same thing as flush in the standard Arduino SD library
            Buzz.running(); //tone to say that logging is running
            digitalWrite(LEDpin, HIGH); //onboard LED on
            LEDstate = true;
            flushclock = timer;
            LEDtimer = timer;
            
            //end of flight timeout
            if (ControlInstance.controlstate == 5){
               endlog();
               }
            }

        //Turn the LED off again after the set interval
        if (LEDstate && timer - LEDtimer >= LEDduration) {
           digitalWrite(LEDpin, LOW); // onboard LED off
           LEDtimer = timer;
           LEDstate = false;
           }
         }
      vTaskDelay(pdMS_TO_TICKS(1));
      }
}

/****************************************/
/* Log and display GPS data at defined intervals
 * Also send out packet via LoRa
/****************************************/
void commsTask(void *parameter) {
  for (;;) {
      screencontrol(); //Let this run before the rest    
      //If there's GPS data, read it
      while (Serial1.available() > 0) {
         GPS.encode(Serial1.read());
         }

      if ((millis() - lastGPSUpdate >= gpsUpdateInterval) && GPS.location.isValid()) {
         msgstring = String(msgID);
         latstring = String (GPS.location.lat(), 6);
         longstring = String (GPS.location.lng(), 6);
         altstring = String(GPS.altitude.meters(), 1);
         dataToSend = "MSG" + msgstring + ",LT" + latstring + ",LG" + longstring + ",AL" + altstring;
         msgID++;
         lastGPSUpdate = millis();
         }
         else if ((millis() - lastGPSUpdate >= gpsUpdateInterval) && !GPS.location.isValid()){ //This allows the message ID for the LoRa string to be updated but keeps the last valid location, this way the receiver can see if it's still transmitting without losing last known location.
                 msgstring = String(msgID);
                 dataToSend = "MSG" + msgstring + ",LT" + latstring + ",LG" + longstring + ",AL" + altstring;
                 msgID++;
                 lastGPSUpdate = millis();
                 }
      vTaskDelay(pdMS_TO_TICKS(10));

      //transmit location
      if (millis() - radiotimer >= transmitrate) {
         //Serial.print("send start: ");
         //Serial.println(millis());
         radio.standby();
         int state = radio.transmit(dataToSend);
         if (state == RADIOLIB_ERR_NONE) {
            Serial.println("Transmission successful!");
            } else {
              Serial.print("Transmission failed: ");
              Serial.println(state);
              }
         radio.sleep();
         radiotimer = millis();
         //Serial.print("send end: ");
         //Serial.println(millis());
         }
      }
      
}

void loop(){}

/* 29/01/2024 - this version is set up to run on a Heltec Wireless Tracker cand was ported from the working ESP32-S3 based data logger.

-Uses GY-912 breakout board with BMP388 for the barometer and ICM20948 for the IMU.
-Control library handles arming/disarming ejection charge pins, drogue deployment, main deployment and timeout notification on landing through the use of numbered "states"
-Each state corresponds with a different set of checks, when the parameters in one state are met, will change state and perform the new checks
-Control library can be given the deployment pins, arming altitude, main deployment altitude and a callback function that will be called by the logic of the control state before each state change.
-Serial debug library has a few different custom functions to assist with breadboard testing and diagnosis.
-logging rates for accelerometer and gyro should be able to reach over 500hz, Magnetometer is max 100hz, and at least 50hz for the baro although is currently not fully tested.
-Trying to implement GPS, LoRa and tasks


To do:
-Servo control should be possible, but might roll that in to a separate version.
-would like to make use of dual core processing
-Would like to switch over to recording microseconds rather than milliseconds for mor accuracy but had issues on last attempt.
-Heltec Wireless Tracker has a SX1262 LoRa module and a UC6580 GNSS module, making it perfect for use as a tracker. Would like to have it work as a transmitter visible to the Meshtastic network but not sure what's involved yet.

Known issues:
-don't think accelerometer or gyro output rates are set properly as this version is outputting at much lower rates than a prior version
-magnetometer maxes out at 100hz but it's still printed at 500hz
-there is an extra 5-6ms gap between recorded data rows occuring roughly every 20-24ms - this is likely due to reaching the limits of SdFat's single 512 byte buffer. multiple buffers can be used but this would probably only delay the problem, different method of logging or SPI memory might help
*/