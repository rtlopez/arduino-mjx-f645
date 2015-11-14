#ifndef MjxPosition_h
#define MjxPosition_h

#include "Arduino.h"
#include "MPU6050.h"
#include "MjxConfig.h"

class MjxLocation
{
  public:
    MjxLocation():
     count(1), gyro_yaw(0.0), gyro_pitch(0.0), gyro_roll(0.0) {}
    MjxLocation(float gyro_yaw_, float gyro_pitch_, float gyro_roll_):
     count(1), gyro_yaw(gyro_yaw_), gyro_pitch(gyro_pitch_), gyro_roll(gyro_roll_) {}

    float gyroYaw()   const;
    float gyroPitch() const;
    float gyroRoll()  const;

    void gyroYaw(float v);
    void gyroPitch(float v);
    void gyroRoll(float v);

    void commit();
    void reset();
    
  private:
   int count;
   float gyro_yaw;
   float gyro_pitch;
   float gyro_roll;
};

class MjxPosition
{
  public:
    MjxPosition() {}
    void begin();
    void update(MjxLocation& location);
    void dumpSettings();
  private:
    MPU6050 mpu;
};

#endif
