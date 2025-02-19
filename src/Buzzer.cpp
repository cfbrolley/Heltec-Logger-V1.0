#include <arduino.h>
#include "Buzzer.h"
#include "Flight_Config.h"

Buzzer::Buzzer(int pin)
{
  _pin = pin;
  pinMode(_pin, OUTPUT);
}

void Buzzer::startup()
{
  #ifndef mute
  tone(_pin, 4500, 160);
  delay(260);
  tone(_pin, 4500, 100);
  delay(150);
  tone(_pin, 4500, 100);
  delay(150);
  tone(_pin, 5800, 500);
  delay(1100);
  #endif 
}

void Buzzer::error()
{
  #ifndef mute
  tone(_pin, 5700, 200);
  delay(250);
  tone(_pin, 3800, 500);
  #endif
}

void Buzzer::success()
{
  #ifndef mute
  tone(_pin, 3700, 220);
  delay(200);
  tone(_pin, 5700, 120);
  #endif
}

void Buzzer::running()
{
  #ifndef mute
  tone(_pin, 4500, 300);
  #endif
}

void Buzzer::ended()
{
 #ifndef mute 
 tone(_pin, 5000, 300);
 delay(500);
 tone(_pin, 5000, 300);
 delay(3000);
 #endif
}