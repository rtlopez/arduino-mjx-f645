#ifndef MjxServo_h
#define MjxServo_h

#include "Arduino.h"
#include <Servo.h>

class MjxServo
{
  public:
    MjxServo();
    uint8_t attach(int pin);                       // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
    uint8_t attach(int pin, int min, int max);     // as above but also sets min and max values for writes. 
    void detach();
    void write(int value);                         // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
    void write(int value, double speed);           // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds, speed in degrees per second
    void update();                                 // update servo position

    double current() const { return _current; }
    int target() const { return _target; }
    
  private:
    double _current;
    int _target;
    double _speed;
    unsigned long _ts;
    Servo servo;
};

#endif
