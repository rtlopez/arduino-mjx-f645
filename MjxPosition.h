#ifndef MjxPosition_h
#define MjxPosition_h

#include "Arduino.h"
#include "RTIMU.h"
#include "RTIMUMPU9150.h"
#include "RTIMUSettings.h"
#include "RTMath.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"
#include "MjxModel.h"

class MjxPosition
{
  public:
    MjxPosition(MjxModel& m);
    void begin();
    void update();
  private:
    MjxModel& model;
    RTIMUSettings settings;                               // the settings object
    RTIMUMPU9150 imu;                                     // the IMU object
    RTFusionRTQF fusion;                                  // the fusion object    
};

#endif
