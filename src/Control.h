//Library used for deployment logic.

#ifndef Control_h
#define Control_h
#include <arduino.h>

typedef void (*ControlCallback)(int state);

class Control {
  public:
    Control(int drogue, int main, int armcheck, ControlCallback callback); //Allows user defined function. See control.cpp for more detailed comment
    Control(int drogue, int main, int armcheck);
    Control(int drogue, int main);
    void begin();
    void SetSafetyLock(int armcheck);
    void SetMainAltitude(int mainalti);
    void Deployment (float CurrentAlt, float az); //Consider a second call without the accelerometer for baro-only loggers
    void Override (int overrideCMD);    
    int controlstate = 0;
    unsigned long apogeetime;
    float maxaccel = 0;
    float maxalt = 0;

  private:
    ControlCallback _callback;
    int _drogue;
    int _main; 
    int _armcheck;
    int _controlstate;
    int _timeout; //needed?
    int _overrideCMD;
    float _correctedalt; 
    float _CurrentAlt;
    float _alticheck = 0;
    float _mainalti; //check this!! - wait... why? Thanks Past Rolley... super unhelpful, man.
    float _az;
    byte _droguecount = 0;
    byte _maincount = 0;
};

#endif