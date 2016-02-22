#ifndef MjxModel_h
#define MjxModel_h

#include "MjxConfig.h"
#include "RTMath.h"

class MjxInput
{
  public:
    MjxInput():
      throttle(0), yaw(0), pitch(0), roll(0), yaw_trim(0x40), pitch_trim(0x40), roll_trim(0x40), flags(0) {}
      
    MjxInput(uint8_t throttle_, uint8_t yaw_, uint8_t pitch_, uint8_t roll_, uint8_t yaw_trim_, uint8_t pitch_trim_, uint8_t roll_trim_, uint8_t flags_):
      throttle(throttle_), yaw(yaw_), pitch(pitch_), roll(roll_), yaw_trim(yaw_trim_), pitch_trim(pitch_trim_), roll_trim(roll_trim_), flags(flags_) {}
      
    uint8_t throttle;
    uint8_t yaw;
    uint8_t pitch; 
    uint8_t roll;
    uint8_t yaw_trim;
    uint8_t pitch_trim;
    uint8_t roll_trim;
    uint8_t flags;
};

class MjxModel
{
  public:
    const MjxInput& getInput() const { return input; }
    const RTVector3& getGyro() const { return gyro; }
    const RTVector3& getPose() const { return pose; }

    void updateInput(const MjxInput& value) { input = value; }
    void updateGyro(const RTVector3& value) { gyro = value; }
    void updatePose(const RTVector3& value) { pose = value; }
    
  private:
    MjxInput input;
    RTVector3 gyro;
    RTVector3 pose;
};

#endif
