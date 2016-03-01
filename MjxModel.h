#ifndef MjxModel_h
#define MjxModel_h

#include "Arduino.h"
#include "MjxConfig.h"
#include "libs/RTIMULib/RTMath.h"

class MjxInput
{
  public:
    MjxInput():
      throttle(0), yaw(0), pitch(0), roll(0), yaw_trim(0x40), pitch_trim(0x40), roll_trim(0x40), flags(0) {}
      
    MjxInput(int8_t throttle_, int8_t yaw_, int8_t pitch_, int8_t roll_, int8_t yaw_trim_, int8_t pitch_trim_, int8_t roll_trim_, int8_t flags_):
      throttle(throttle_), yaw(yaw_), pitch(pitch_), roll(roll_), yaw_trim(yaw_trim_), pitch_trim(pitch_trim_), roll_trim(roll_trim_), flags(flags_) {}
      
    uint8_t throttle;
    int8_t yaw;
    int8_t pitch; 
    int8_t roll;
    int8_t yaw_trim;
    int8_t pitch_trim;
    int8_t roll_trim;
    uint8_t flags;
};

class MjxModel
{
  public:
    const MjxInput& getInput() const;
    void updateInput(const MjxInput& value);

    const RTVector3& getGyro() const;
    void updateGyro(const RTVector3& value);

    const RTVector3& getPose() const;
    void updatePose(const RTVector3& value, float dt);
  
    MjxConfig config;
  
  private:
    MjxInput input;
    RTVector3 gyro;
    RTVector3 pose;
};

#endif
