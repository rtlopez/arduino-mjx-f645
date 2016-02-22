#include "MjxServo.h"

MjxServo::MjxServo():
  _current(0), _target(0), _speed(0), _ts(0)
{
  _ts = millis();
}

uint8_t MjxServo::attach(int pin)
{
  _ts = millis();
  return servo.attach(pin);
}

uint8_t MjxServo::attach(int pin, int min, int max)
{
  _ts = millis();
  return servo.attach(pin, min, max);
}

void MjxServo::detach()
{
  servo.detach();
}

void MjxServo::write(double value)
{
  write(value, 0.0);
}

void MjxServo::write(double value, double speed)
{
  _target = value;
  _speed = speed;
}

void MjxServo::write(double value, boolean force)
{
  servo.write(value);
}

void MjxServo::update()
{
  unsigned long now = millis();
  unsigned long tm = now - _ts;
  if(tm >= MJX_UPDATE_INTERVAL)
  {
    if(_speed > 0)
    {
      double diff = _target - _current;
      double inc = _speed / 1000.0 * tm;
      inc = diff > 0 ? inc : -inc;
      if(abs(inc) < abs(diff))
      {
        _current += inc;
      }
      else
      {
        _current = _target;
      }
    }
    else
    {
      _current = _target;
    }
    write(_current, true);
    _ts = now;
  }
}

