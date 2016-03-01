#include "MjxModel.h"
#include <EEPROM.h>

const uint8_t MjxModel::EEPROM_BASE_INT   = 0x00;
const uint8_t MjxModel::EEPROM_BASE_FLOAT = 0x40;

MjxModel::MjxModel()
{
}

void MjxModel::begin()
{
  load();
}

void MjxModel::updateInput(int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll, int16_t yaw_trim, int16_t pitch_trim, int16_t roll_trim, int16_t flags)
{
  idata[INPUT_THROTTLE] = throttle;
  idata[INPUT_YAW] = yaw;
  idata[INPUT_PITCH] = pitch;
  idata[INPUT_ROLL] = roll;
  idata[INPUT_YAW_TRIM] = yaw_trim;
  idata[INPUT_PITCH_TRIM] = pitch_trim;
  idata[INPUT_ROLL_TRIM] = roll_trim;
  idata[INPUT_FLAGS] = flags;
}

void MjxModel::updatePose(float x, float y, float z, float dt)
{
  float prev_x = fdata[POSE_X];
  float prev_y = fdata[POSE_Y];
  float prev_z = fdata[POSE_Z];
  fdata[POSE_X] = x;
  fdata[POSE_Y] = y;
  fdata[POSE_Z] = z;
  fdata[GYRO_X] = (x - prev_x) / dt;
  fdata[GYRO_Y] = (y - prev_y) / dt;
  fdata[GYRO_Z] = (z - prev_z) / dt; 
}

void MjxModel::load()
{
  bool valid = load(MAGIC, 0, true) == (int16_t)0xA5A5;
  idata[MAGIC]         = load(MAGIC, (int16_t)0xA5A5, valid);
  idata[RADIO_CE_PIN]  = load(RADIO_CE_PIN, 8, valid);
  idata[RADIO_CSN_PIN] = load(RADIO_CSN_PIN, 7, valid);
  idata[THROTTLE_PIN]  = load(THROTTLE_PIN, 3, valid);
  idata[YAW_PIN]       = load(YAW_PIN, 5, valid);
  idata[PITCH_PIN]     = load(PITCH_PIN, 10, valid);
  idata[ROLL_PIN]      = load(ROLL_PIN, 8, valid);
  idata[SERVO_PWM_MIN] = load(SERVO_PWM_MIN, 1000, valid);
  idata[SERVO_PWM_MAX] = load(SERVO_PWM_MAX, 1900, valid);
  idata[MOTOR_PWM_MIN] = load(MOTOR_PWM_MIN, 1000, valid);
  idata[MOTOR_PWM_MAX] = load(MOTOR_PWM_MAX, 1900, valid);
  idata[UPDATE_INTERVAL] = load(UPDATE_INTERVAL, 20, valid);

  fdata[MAG_MIN_X]  = load(MAG_MIN_X, -50.0, valid);
  fdata[MAG_MAX_X]  = load(MAG_MAX_X,  50.0, valid);
  fdata[MAG_MIN_Y]  = load(MAG_MIN_Y, -50.0, valid);
  fdata[MAG_MAX_Y]  = load(MAG_MAX_Y,  50.0, valid);
  fdata[MAG_MIN_Z]  = load(MAG_MIN_Z, -50.0, valid);
  fdata[MAG_MAX_Z]  = load(MAG_MAX_Z,  50.0, valid);
  
  fdata[YAW_PID_KP] = load(YAW_PID_KP, 1.50, valid);
  fdata[YAW_PID_KI] = load(YAW_PID_KI, 3.00, valid);
  fdata[YAW_PID_KD] = load(YAW_PID_KD, 0.05, valid);
}

void MjxModel::save()
{
  save(MAGIC, idata[MAGIC]);
  save(RADIO_CE_PIN, idata[RADIO_CE_PIN]);
  save(RADIO_CSN_PIN, idata[RADIO_CSN_PIN]);
  save(THROTTLE_PIN, idata[THROTTLE_PIN]);
  save(YAW_PIN, idata[YAW_PIN]);
  save(PITCH_PIN, idata[PITCH_PIN]);
  save(ROLL_PIN, idata[ROLL_PIN]);
  save(ROLL_PIN, idata[ROLL_PIN]);
  save(SERVO_PWM_MIN, idata[SERVO_PWM_MIN]);
  save(SERVO_PWM_MAX, idata[SERVO_PWM_MAX]);
  save(MOTOR_PWM_MIN, idata[MOTOR_PWM_MIN]);
  save(MOTOR_PWM_MAX, idata[MOTOR_PWM_MAX]);
  save(UPDATE_INTERVAL, idata[UPDATE_INTERVAL]);

  save(MAG_MIN_X, fdata[MAG_MIN_X]);
  save(MAG_MAX_X, fdata[MAG_MAX_X]);
  save(MAG_MIN_Y, fdata[MAG_MIN_Y]);
  save(MAG_MAX_Y, fdata[MAG_MAX_Y]);
  save(MAG_MIN_Z, fdata[MAG_MIN_Z]);
  save(MAG_MAX_Z, fdata[MAG_MAX_Z]);
  save(YAW_PID_KP, fdata[YAW_PID_KP]);
  save(YAW_PID_KI, fdata[YAW_PID_KI]);
  save(YAW_PID_KD, fdata[YAW_PID_KD]);
}

int16_t MjxModel::load(mjx_config_i idx, int16_t def, bool valid)
{
  if(!valid) return def;
  int16_t t = 0;
  EEPROM.get(EEPROM_BASE_INT * idx * sizeof(t), t);
  return t != 0 ? t : def;
}

float MjxModel::load(mjx_config_f idx, float def, bool valid)
{
  if(!valid) return def;
  int16_t t = 0;
  EEPROM.get(EEPROM_BASE_FLOAT * idx * sizeof(t), t);
  return t != 0 ? t : def;
}

void MjxModel::save(mjx_config_i idx, int16_t val)
{
  EEPROM.put(EEPROM_BASE_INT * idx * sizeof(val), val);
}

void MjxModel::save(mjx_config_f idx, float val)
{
  EEPROM.put(EEPROM_BASE_FLOAT * idx * sizeof(val), val);
}
