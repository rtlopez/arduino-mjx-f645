#ifndef MjxTelemetry_h
#define MjxTelemetry_h

#include "Arduino.h"
#include "MjxModel.h"

class MjxTelemetry
{
  public:
    MjxTelemetry(MjxModel& m);
    void print(double val, uint32_t flags);
    void reset();
    
  private:
    bool ready(uint32_t flags) const;
    
    MjxModel& model;
    uint32_t now;
    bool flush;   
};

#endif
