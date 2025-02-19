#include <arduino.h>
#include "Control.h"

/*
Pass the drogue trigger pin, main trigger pin, arming lockout altitude, and a callback function.
This one is special, because when you pass the constructor a function, the Deployment function will call that function and pass the current control state before switching the control state. 
This allows the user to place a function in the main code that's ONLY called right before a state change
for example, when the parameters to fire the drogue are met, the Deployment function sets the drogue pin high calls the calls the function it that has been passed to the class and passes the current state to it.
the function that was passed to the class runs, and does whatever the user wants based on the control state (let's say it's passed a '2' - based on '2', it runs serial.print("drogue fried")).
the Deploy function then sets control state to 3, which has different parameters before the callback is called again, that way it happens once and the callback won't keep repeating.
*/
Control::Control(int drogue, int main, int armcheck, ControlCallback callback) {
    _drogue = drogue;
    _main = main;
    _armcheck = armcheck;
    _controlstate = 0;
    _callback = callback;
}

//Pass the drogue trigger pin, main trigger pin and arming lockout altitude.
Control::Control(int drogue, int main, int armcheck) {
  _drogue = drogue;
  _main = main;
  _armcheck = armcheck;
  controlstate = 0;
}

//Option for default: Pass the drogue trigger pin, main trigger pin and set an arming lockout altitude to default 20m.
Control::Control(int drogue, int main) {
  _drogue = drogue;
  _main = main;
  _armcheck = 20;
  controlstate = 0;
}

void Control::begin() {
  pinMode(_drogue, OUTPUT);
  pinMode(_main, OUTPUT);
}

/*Set arming altitude. Ejection charges are disabled when the altitude above ground is below this height. 
This can just be set within one of the class definitions anyway, but having this as a function allows for user selectable height later on.*/
void Control::SetSafetyLock(int armcheck) {
 _armcheck = armcheck;
}

/*Set target altitude for main parachute deployment. Haven't set a default for this yet.
Having this as a function allows for user selectable height later on.*/
void Control::SetMainAltitude(int mainalti) {
 _mainalti = mainalti;
}

/*
This is the function that handles all of the arming and deployment checks.
checks different parameters and performs different actions based on current state.
Temporarily removed liftoff time recording and max acceleration recording. 
This is because these checks rely on accelerometer readings, but the accelerometer can take readings more frequently than the baro.
Calling it more frequently than the baro can update will result in duplicate altitude values being passed to the function.
Duplicate values received when the control state is >0 can trigger drogue deployment early.
Left in accepting acceleration value though as this can still be used as a safeguard against deploying while accelerating.
-------------------------------------------------------------------------------------------------------------------------------------------------
State 0 = checking for armed
State 1 = armed, checking for drogue
State 2 = drogue fired, checking for main
State 3 = main fired, checking for disarm
State 4 = disarmed, checking for timeout 
State 5 = timeout
-------------------------------------------------------------------------------------------------------------------------------------------------
*/
//Deployment function handles all of the apogee detection/deployment logic. changes "Control states" when the parameter for the prior case has been met.
//doubt this is the best way of doing things, but appears to work well for now.
void Control::Deployment (float CurrentAlt, float az) {
  _CurrentAlt = CurrentAlt;
  _az = az;
  switch (controlstate) {
  //control state 0: waiting for arming altitude
  case 0:
  if (_CurrentAlt > _armcheck) {
     _callback(controlstate);//callback armed print
     controlstate = 1;
     }
  break;
  //control state 1: armed and checking for apogee
  case 1:
  if (_CurrentAlt <= _alticheck){
     _droguecount++;
     _alticheck = _CurrentAlt;
        if (_alticheck > maxalt) {
           maxalt = _alticheck;
           apogeetime = millis();
           }
     }
     else {
          _droguecount = 0;
          _alticheck = _CurrentAlt;
          }   
  if (_droguecount >5) {
     _callback(controlstate);//callback function is called and passed the current state.
     digitalWrite(_drogue, HIGH); //fire drogue parachute ejection charge
     controlstate = 2;
     }
  break;
  //control state 2: past apogee and waiting to fall below main altitude
  case 2: 
  if (_CurrentAlt <= _mainalti) {
     _maincount++;
     }
     else {
          _maincount = 0;
          }
  if (_maincount > 5) {
     _callback(controlstate); //callback function is called and passed the current state.   
     digitalWrite(_main, HIGH); //fire main parachute ejection charge
     controlstate = 3;
     }
  break;
  //control state 3: all ejection charges fired and and waiting to disarm
  case 3:
  if (_CurrentAlt < _armcheck) {
     _callback(controlstate); //callback function is called and passed the current state.
     digitalWrite(_drogue, LOW); //both pyro channels are set low below the armcheck altitude
     digitalWrite(_main, LOW);
     controlstate = 4;
     }
  break;
  //control state 4: below disarm altitude and waiting to timeout. 
  case 4: 
  if (_CurrentAlt < 10) {
     _timeout = _timeout + 1;
     }
  if (_timeout >= 500) {
     _callback(controlstate); //callback function is called and passed the current state.
     controlstate = 5;
     }
  break;
  default:
  //do nothing
  break;
  }

  /* not currently implemented  
  if (controlstate > 0 && controlstate < 3 && _az > maxaccel){
    maxaccel = _az;
     }
    if (_az >= 3){
       //callback liftoff time
       controlstate = 1;
       }
*/ 
}

//override can be called to force an action regardless of what the current "control state" is, then sets the control state to where it should be after the action.
//e.g - Control.Overide(1) can be used to force the main parachute pyro channel to fire at any time.
void Control::Override(int overrideCMD) {
  _overrideCMD = overrideCMD;
  switch (_overrideCMD) {
  case 0:
      _callback(1);//callback apogee time and drogue print
      digitalWrite(_drogue, HIGH);
      controlstate = 2;
  break;
  case 1:
     _callback(2);//callback main print  
     digitalWrite(_main, HIGH);
     controlstate = 3;
  break;
  case 2:
     _callback(3);//callback disarmed print
     digitalWrite(_drogue, LOW);
     digitalWrite(_main, LOW);
     controlstate = 4;
  break;
  default:
  //do nothing
  break;
  }
} 