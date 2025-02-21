#include <arduino.h>
#include "Serial_Debug.h"

Serial_Debug::Serial_Debug(int baud) {
_baud = baud;
  Serial.begin(_baud);
  Serial.println();
  Serial.println("debugging running");
}

void Serial_Debug::debugdata(unsigned long _timer, float _pressure, float _altitude, float _correctedalt, float _ax, float _ay, float _az, float _gx, float _gy, float _gz, float _mx, float _my, float _mz, int _state) {
    #define COMMA Serial.print(", ");
    Serial.println();
    Serial.print(_timer); COMMA;
    Serial.print(_pressure); COMMA;
    Serial.print(_altitude); COMMA;
    Serial.print(_correctedalt); COMMA;
    Serial.print(_ax); COMMA;
    Serial.print(_ay); COMMA;
    Serial.print(_az); COMMA;
    Serial.print(_gx); COMMA;
    Serial.print(_gy); COMMA;
    Serial.print(_gz); COMMA;
    Serial.print(_mx); COMMA;
    Serial.print(_my); COMMA;
    Serial.print(_mz); COMMA;
    Serial.print(_state); COMMA;
}

void Serial_Debug::debugBMP(int _code, float _altioffset) {
  switch (_code) {
  case 1:
      Serial.println("BMP error!");
      Serial.println("Data bus!");
      break;
  case 2:
      Serial.println("BMP error!");
      Serial.println("Chip version!");
      break;
  case 3:
      Serial.println("BMP OK!");
      Serial.println("starting altitude: ");
      Serial.print(_altioffset);
      Serial.print("m");
      Serial.println();
      break;
  default:
      Serial.println("BMP error!");
      Serial.println("unknown!");
      break;
  }    
}

void Serial_Debug::debugIMU(int _code) {
  switch (_code) {
  case 1: Serial.println("IMU error!"); break;
  case 2: Serial.println("IMU OK!"); break;
  default: Serial.println("IMU error!"); break;
  }
}

void Serial_Debug::debugACC(int _code) {
  switch (_code) {
  case 1: Serial.println("ACC error!"); break;
  case 2: Serial.println("ACC OK!"); break;
  default: Serial.println("ACC error!"); break;
  }
}

void Serial_Debug::debugSD(int _code) {
  switch (_code) {
  case 1: Serial.println("SD error!"); break;
  case 2 : Serial.println("SD OK!"); break;
  default: Serial.println("SD error!"); break;
  } 
}

void Serial_Debug::debuggeneric() {
  Serial.println("test");
}