#ifndef MjxTelemetry_h
#define MjxTelemetry_h

#include "Arduino.h"
#include "MjxModel.h"

class MjxTelemetry
{
  public:
    MjxTelemetry(MjxModel& m);

    template<typename T>
    MjxTelemetry& operator<< (const T v)
    {
      Serial.print(v); Serial.print(" ");
      return *this;
    }
    
    MjxTelemetry& operator<< (const double v)
    {
      Serial.print(v, 4); Serial.print(" ");
      return *this;
    }

    MjxTelemetry& operator<< (const char* v)
    {
      Serial.print(v);
      return *this;
    }

  private:
    MjxModel& model;
};

#endif
