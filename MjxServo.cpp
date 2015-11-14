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

void MjxServo::write(int value)
{
  write(value, 0);
}

void MjxServo::write(int value, double speed)
{
  _target = value;
  _speed = speed;
}

void MjxServo::update()
{
  unsigned long now = millis();
  unsigned long tm = now - _ts;
  if(tm >= 20)
  {
    if(_speed > 0)
    {
      double diff = _target - _current;
      double inc = _speed / 1000.0 * tm;
      inc = diff > 0 ? inc : -inc;
      /*Serial.print(tm);
      Serial.print(" "); Serial.print(_target);
      Serial.print(" "); Serial.print(_current);
      Serial.print(" "); Serial.print(_speed);
      Serial.print(" "); Serial.print(inc, 4);
      Serial.print(" "); Serial.print(diff, 4);
      Serial.println();*/
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
    servo.write(_current);
    _ts = now;
  }
}

