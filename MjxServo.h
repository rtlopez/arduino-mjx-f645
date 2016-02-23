#ifndef MjxServo_h
#define MjxServo_h

#include "Arduino.h"
#include "MjxConfig.h"
#include <Servo.h>

class MjxServo
{
  public:
    MjxServo(unsigned int interval);
    uint8_t attach(int pin);                       // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
    uint8_t attach(int pin, int min, int max);     // as above but also sets min and max values for writes. 
    void detach();
    void write(double value);                      // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds
    void write(double value, double speed);        // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds, speed in degrees per second
    void write(double value, boolean force);       // write to servo immediatelly
    void update();                                 // update servo position

    double current() const { return _current; }
    double target()  const { return _target;  }
    
  private:
    double _current;
    double _target;
    double _speed;
    unsigned long _ts;
    unsigned int _interval;
    Servo servo;
};

#endif
