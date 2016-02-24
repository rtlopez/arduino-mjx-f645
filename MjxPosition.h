#ifndef MjxPosition_h
#define MjxPosition_h

#include "Arduino.h"
#include "libs/RTIMULib/RTIMU.h"
#include "libs/RTIMULib/RTIMUMPU9150.h"
#include "libs/RTIMULib/RTIMUSettings.h"
#include "libs/RTIMULib/RTMath.h"
#include "libs/RTIMULib/RTFusionRTQF.h" 
#include "libs/CalLib/CalLib.h"
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
