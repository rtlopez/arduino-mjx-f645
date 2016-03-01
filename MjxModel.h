#ifndef MjxModel_h
#define MjxModel_h

#include "Arduino.h"

enum mjx_config_i {
  MAGIC = 0,
  RADIO_CE_PIN,
  RADIO_CSN_PIN,
  THROTTLE_PIN,
  YAW_PIN,
  PITCH_PIN,
  ROLL_PIN,
  SERVO_PWM_MIN,
  SERVO_PWM_MAX,
  MOTOR_PWM_MIN,
  MOTOR_PWM_MAX,
  UPDATE_INTERVAL,
  
  INPUT_THROTTLE,
  INPUT_YAW,
  INPUT_PITCH,
  INPUT_ROLL,
  INPUT_YAW_TRIM,
  INPUT_PITCH_TRIM,
  INPUT_ROLL_TRIM,
  INPUT_FLAGS,
  OUTPUT_THROTTLE,
  OUTPUT_YAW,
  OUTPUT_PITCH,
  OUTPUT_ROLL,
};

enum mjx_config_f {
  MAG_MIN_X = 0,
  MAG_MAX_X,
  MAG_MIN_Y,
  MAG_MAX_Y,
  MAG_MIN_Z,
  MAG_MAX_Z,
  YAW_PID_KP,
  YAW_PID_KI,
  YAW_PID_KD,
  GYRO_X,
  GYRO_Y,
  GYRO_Z,
  POSE_X,
  POSE_Y,
  POSE_Z,
  YAW_PID_E,
  YAW_PID_P,
  YAW_PID_I,
  YAW_PID_D,
};

class MjxModel
{
  public:
    MjxModel();
    void begin();
    
    void updateInput(int16_t throttle, int16_t yaw, int16_t pitch, int16_t roll, int16_t yaw_trim, int16_t pitch_trim, int16_t roll_trim, int16_t flags);
    void updatePose(float x, float y, float z, float dt);
  
    int16_t getRadioCePin() const  { return idata[RADIO_CE_PIN]; }
    int16_t getRadioCsnPin() const { return idata[RADIO_CSN_PIN]; }
    int16_t getThrottlePin() const { return idata[THROTTLE_PIN]; }
    int16_t getYawPin() const      { return idata[YAW_PIN]; }
    int16_t getPitchPin() const    { return idata[PITCH_PIN]; }
    int16_t getRollPin() const     { return idata[ROLL_PIN]; }

    int16_t getServoPwmMin() const { return idata[SERVO_PWM_MIN]; }
    int16_t getServoPwmMax() const { return idata[SERVO_PWM_MAX]; }
    int16_t getMotorPwmMin() const { return idata[MOTOR_PWM_MIN]; }
    int16_t getMotorPwmMax() const { return idata[MOTOR_PWM_MAX]; }
    
    int16_t getUpdateInterval() const  { return idata[UPDATE_INTERVAL]; }
    
    int16_t getInputThrottle() const  { return idata[INPUT_THROTTLE]; }
    int16_t getInputYaw() const       { return idata[INPUT_YAW]; }
    int16_t getInputPitch() const     { return idata[INPUT_PITCH]; }
    int16_t getInputRoll() const      { return idata[INPUT_ROLL]; }
    int16_t getInputYawTrim() const   { return idata[INPUT_YAW_TRIM]; }
    int16_t getInputPitchTrim() const { return idata[INPUT_PITCH_TRIM]; }
    int16_t getInputRollTrim() const  { return idata[INPUT_ROLL_TRIM]; }
    int16_t getInputFlags() const     { return idata[INPUT_FLAGS]; }

    float getGyroX() const     { return fdata[GYRO_X]; }
    float getGyroY() const     { return fdata[GYRO_Y]; }
    float getGyroZ() const     { return fdata[GYRO_Z]; }

    float getYawPidKp() const     { return fdata[YAW_PID_KP]; }
    float getYawPidKi() const     { return fdata[YAW_PID_KI]; }
    float getYawPidKd() const     { return fdata[YAW_PID_KD]; }

    void setOutputThrottle(int16_t val)  { idata[OUTPUT_THROTTLE] = val; }
    void setOutputYaw(int16_t val)       { idata[OUTPUT_YAW] = val; }
    void setOutputPitch(int16_t val)     { idata[OUTPUT_PITCH] = val; }
    void setOutputRoll(int16_t val)      { idata[OUTPUT_ROLL] = val; }

    int16_t get(mjx_config_i i) const { return idata[i]; }
    float   get(mjx_config_f i) const { return fdata[i]; }
    void    set(mjx_config_i i, int16_t val) { idata[i] = val; }
    void    set(mjx_config_f i, float val)   { fdata[i] = val; }
    
  private:
    void load();
    int16_t load(mjx_config_i idx, int16_t def, bool valid);
    float   load(mjx_config_f idx, float def, bool valid);

    void save();
    void save(mjx_config_i idx, int16_t val);
    void save(mjx_config_f idx, float val);

    static const uint8_t EEPROM_BASE_INT;
    static const uint8_t EEPROM_BASE_FLOAT;

    int16_t idata[24];
    float   fdata[20];
};

#endif
