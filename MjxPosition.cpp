#include "MjxPosition.h"

void MjxPosition::begin()
{
  settings.m_MPU9150GyroFsr = MPU9150_GYROFSR_2000;         // degr/sec: 250,500,1000,2000
  settings.m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;          // G:        2,4,8,16
  settings.m_MPU9150GyroAccelLpf = MPU9150_LPF_20;          // Hz:       5|10|20|42,98,188,256
  settings.m_MPU9150GyroAccelSampleRate = 50;
  settings.m_MPU9150CompassSampleRate = 50;

  Serial.print(F(" * Mjx IMU starting: ")); Serial.println(imu.IMUName());
  int errcode;
  if((errcode = imu.IMUInit()) < 0)
  {
    Serial.print(F(" * Failed to init IMU: ")); Serial.println(errcode);
    return;
  }

  Serial.print(F(" * Compass calibration: "));
  Serial.println(imu.getCalibrationValid());
  
  fusion.setSlerpPower(0.02);
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(true);
}

void MjxPosition::update(MjxModel& model)
{
  int loopCount = 0;
  while(imu.IMURead())
  {
    ++loopCount;
    if(loopCount >= 10) continue;
    fusion.newIMUData(imu.getGyro(), imu.getAccel(), imu.getCompass(), imu.getTimestamp());
    model.updateGyro(imu.getGyro());
  }
  if(loopCount)
  {
    model.updatePose(fusion.getFusionPose());
  }
}
