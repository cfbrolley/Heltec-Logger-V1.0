#ifndef Flight_Config_h
#define Flight_Config_h

//Define constants and logger settings
#define SEALEVELPRESSURE_HPA 1013.25  
#define mainalti 100 //altitude in metres to activate mainpin on descent
#define lockout 20
#define flushtime 2500 //time between calling the sync() function from SdFat, LED flashes at this interval while logging as well 
#define batterytime 3000
#define bmprate 20 //time in ms between reading bmp sensor. Allows IMU and BMP to read and log at different rates.
#define gpsUpdateInterval 1000 // gps check rate
#define transmitrate 5000
#define LEDduration 100 //how long in ms for the LED to stay on for when it flashes during logging
//#define datadebugging //option to output sensor readings to serial monitor while logging. uncomment to activate

#endif