#include "MjxPosition.h"

float MjxLocation::gyroYaw()   const { return gyro_yaw   / count; }
float MjxLocation::gyroPitch() const { return gyro_pitch / count; }
float MjxLocation::gyroRoll()  const { return gyro_roll  / count; }

void MjxLocation::gyroYaw(float v)   { gyro_yaw += v; }
void MjxLocation::gyroPitch(float v) { gyro_pitch += v; }
void MjxLocation::gyroRoll(float v)  { gyro_roll += v; }

void MjxLocation::commit() { count++; }
void MjxLocation::reset() { *this = MjxLocation(); }

void MjxPosition::begin()
{
  Serial.println(F(" * MPU6050 Initialization"));
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_8G))
  {
    Serial.println(F("ERROR: Could not find a valid MPU6050 sensor, check wiring!"));
    delay(500);
  }

  //mpu.setDLPFMode(MPU6050_DLPF_1);
  mpu.setDLPFMode(MJX_POSITION_DLPF);

  // If you want, you can set gyroscope offsets
  // mpu.setGyroOffsetX(155);
  // mpu.setGyroOffsetY(15);
  // mpu.setGyroOffsetZ(15);
  
  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  Serial.println(F(" * MPU6050 Gyro Calibration start"));
  int i = 50;
  int match = 0;
  float prev_z = 0;
  while(i--)
  {
    Vector gyro = mpu.readNormalizeGyro();
    float diff = prev_z - gyro.ZAxis;
    if(abs(diff) < 3.0) match++;
    if(match >= 5) break;
    prev_z = gyro.ZAxis;
    delay(100);
  }
  mpu.calibrateGyro();
  Serial.print(F(" * MPU6050 Gyro Calibration done: "));
  Serial.print(prev_z);
  Serial.println();
  
  // Set threshold sensivty. Default 3.
  // If you don't want use threshold, comment this line or set 0.
  mpu.setThreshold(0.05);
  //mpu.setThreshold(1);
  
  // Check settings
  dumpSettings();
}

void MjxPosition::update(MjxLocation& location)
{
  Vector gyro = mpu.readNormalizeGyro();
  location.gyroYaw(gyro.ZAxis);
  location.gyroPitch(gyro.YAxis);
  location.gyroRoll(gyro.XAxis);
  location.commit();
}

void MjxPosition::dumpSettings()
{
  Serial.print(F(" * MPU6050 Sleep Mode:   "));
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(F(" * MPU6050 Clock Source: "));
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println(F("Stops the clock and keeps the timing generator in reset")); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println(F("PLL with external 19.2MHz reference")); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println(F("PLL with external 32.768kHz reference")); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println(F("PLL with Z axis gyroscope reference")); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println(F("PLL with Y axis gyroscope reference")); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println(F("PLL with X axis gyroscope reference")); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println(F("Internal 8MHz oscillator")); break;
  }
  
  Serial.print(F(" * MPU6050 Gyro scale:   "));
  switch(mpu.getScale())
  {
    case MPU6050_SCALE_2000DPS:        Serial.println(F("2000 dps")); break;
    case MPU6050_SCALE_1000DPS:        Serial.println(F("1000 dps")); break;
    case MPU6050_SCALE_500DPS:         Serial.println(F("500 dps")); break;
    case MPU6050_SCALE_250DPS:         Serial.println(F("250 dps")); break;
  } 
  
  Serial.print(F(" * MPU6050 Gyro offsets: "));
  Serial.print(mpu.getGyroOffsetX());
  Serial.print(F(" / "));
  Serial.print(mpu.getGyroOffsetY());
  Serial.print(F(" / "));
  Serial.println(mpu.getGyroOffsetZ());
  
  //Serial.println();
}
