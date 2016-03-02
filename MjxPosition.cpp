#include "MjxPosition.h"

MjxPosition::MjxPosition(MjxModel& m): model(m), imu(&settings)
{
  settings.m_MPU9150GyroFsr = MPU9150_GYROFSR_2000;         // degr/sec: 250,500,1000,2000
  settings.m_MPU9150AccelFsr = MPU9150_ACCELFSR_8;          // G:        2,4,8,16
  settings.m_MPU9150GyroAccelLpf = MPU9150_LPF_10;          // Hz:       5|10|20|42,98,188,256
  settings.m_MPU9150GyroAccelSampleRate = 50;
  settings.m_MPU9150CompassSampleRate = 50;
}

void MjxPosition::begin()
{
  Wire.begin();
  Serial.print(F("IMU.start: ")); Serial.print(imu.IMUName());
  Serial.println();
  int errcode;
  if((errcode = imu.IMUInit()) < 0)
  {
    Serial.print(F("IMU.failed: ")); Serial.println(errcode);
    while(true);
  }

  bool compas = imu.getCalibrationValid();
  Serial.print(F("IMU.compas")); Serial.println(compas);
  
  fusion.setSlerpPower(0.02);
  fusion.setGyroEnable(true);
  fusion.setAccelEnable(true);
  fusion.setCompassEnable(compas);
  
  prev_tm = millis();
}

void MjxPosition::update()
{
  int loopCount = 0;
  unsigned long long tm;
  while(imu.IMURead())
  {
    if(++loopCount >= 10) continue;
    fusion.newIMUData(imu.getGyro(), imu.getAccel(), imu.getCompass(), imu.getTimestamp());
    tm = imu.getTimestamp();
  }
  if(loopCount && tm - prev_tm >= static_cast<uint16_t>(model.getUpdateInterval()))
  {
    float dt = tm - prev_tm / 1000.0;
    const RTVector3& pose = fusion.getFusionPose();
    model.updatePose(pose.x(), pose.y(), pose.z(), dt);
    prev_tm = tm;
  }
}
