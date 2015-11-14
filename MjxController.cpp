#include "MjxController.h"

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
/*
int multiMap(int val, const int* _in, const int* _out, uint8_t size)
{
  // take care the value is within range
  val = constrain(val, _in[0], _in[size-1]);
  //if (val <= _in[0]) return _out[0];
  //if (val >= _in[size-1]) return _out[size-1];

  // search right interval
  uint8_t pos = 1;  // _in[0] allready tested
  while(val > _in[pos]) pos++;

  // this will handle all exact "points" in the _in array
  if (val == _in[pos]) return _out[pos];

  // interpolate in the right segment for the rest
  return (val - _in[pos-1]) * (_out[pos] - _out[pos-1]) / (_in[pos] - _in[pos-1]) + _out[pos-1];
}

static const int yaw_map_in[]  = { 0, 45,  90, 135, 180 };
static const int yaw_map_out[] = { 0, 90, 120, 150, 180 };
*/

MjxController::MjxController(int throttle_pin_, int yaw_pin_, int pitch_pin_, int roll_pin_):
  throttle_pin(throttle_pin_), yaw_pin(yaw_pin_), pitch_pin(pitch_pin_), roll_pin(roll_pin_), 
  pid_yaw(&gyro_yaw, &yaw_out, &yaw_in, 1, 0, 0, DIRECT)
{
  throttle_in = 0;
  yaw_in = 0;
  pitch_in = 0;
  roll_in = 0; 

  throttle_out = 0;
  yaw_out = 0;
  pitch_out = 90;
  roll_out = 90;

  gyro_yaw = 0;
  gyro_pitch = 0;
  gyro_roll = 0; 
}

void MjxController::begin()
{
  servo_throttle.attach(throttle_pin, 1000, 2000);
  servo_yaw.attach(yaw_pin, 1000, 2000);
  servo_pitch.attach(pitch_pin, 1000, 2000);
  servo_roll.attach(roll_pin, 1000, 2000);
  
  pid_yaw.SetMode(AUTOMATIC);
  pid_yaw.SetOutputLimits(0, 180, MJX_CONTROLLER_YAW_PID_I_RATE);
  pid_yaw.SetSampleTime(20);
  
  update();
}

#if MJX_DEBUG_CONTROLLER
static uint64_t disp_tm = 0;
#endif

void MjxController::update(const MjxControls& ctrl, MjxLocation& loc)
{
  // decode adjust input
  throttle_in = ctrl.throttle;

  yaw_in = ctrl.yaw;
  if(ctrl.yaw >= 0x80) yaw_in = 0x80 - ctrl.yaw;
  if(ctrl.flags == 0x10) yaw_in *= -1;
  yaw_in *= -180.0/127.0;
  
  pitch_in = ctrl.pitch;
  if(ctrl.pitch >= 0x80) pitch_in = 0x80 - ctrl.pitch;
  pitch_in *= -1.0;

  roll_in = ctrl.roll;
  if(ctrl.roll >= 0x80) roll_in = 0x80 - ctrl.roll;
  roll_in *= -1.0;

  // add trimming offsets
  //yaw_in += ctrl.yaw_trim - 64;
  //pitch_in += ctrl.pitch_trim - 64;
  //roll_in += ctrl.roll_trim - 64;

  gyro_yaw = loc.gyroYaw();
  gyro_pitch = loc.gyroPitch();
  gyro_roll = loc.gyroRoll();

  // calculate output
  const int loFrom = -127 - 64;
  const int hiFrom = -loFrom;
  const int loTo = 0;
  const int hiTo = 180;
  
  throttle_out = map(throttle_in, 0, 255, loTo, hiTo);
  pitch_out = map(pitch_in, loFrom, hiFrom, loTo, hiTo);
  roll_out = map(roll_in, loFrom, hiFrom, loTo, hiTo);
  //yaw_out = map(yaw_in, loFrom, hiFrom, loTo, hiTo);
  
  // tuning
  //float yaw_kp = 0.15;
  float yaw_kp = mapFloat(ctrl.roll_trim, 1, 127, 0, MJX_CONTROLLER_YAW_PID_KP);      // 0.15
  
  //float yaw_ki = 0.35;
  float yaw_ki = mapFloat(ctrl.yaw_trim, 1, 127, 0, MJX_CONTROLLER_YAW_PID_KI);       // 0.35
  
  //float yaw_kd = 0.05
  float yaw_kd = mapFloat(ctrl.pitch_trim, 1, 127, 0, MJX_CONTROLLER_YAW_PID_KD);     // 0.05

  float yaw_gyro_rate = MJX_CONTROLLER_YAW_PID_GYRO_RATE;                             // 0.1
  //float yaw_gyro_rate = mapFloat(ctrl.roll_trim, 1, 127, 0, 0.05);  // 0.05
  
  gyro_yaw *= yaw_gyro_rate;

  pid_yaw.SetTunings(yaw_kp, yaw_ki, yaw_kd);
  boolean updated = pid_yaw.Compute();
  
  //yaw_out = map(yaw_out, 0, 127, loTo, hiTo);
  //yaw_out = multiMap(yaw_out, yaw_map_in, yaw_map_out, sizeof(yaw_map_in));
  
  //if(ctrl.flags == 0x10) // yaw manual mode
  //{
  //  yaw_out = yaw_in > 0 ? yaw_in : 0;
  //}

  #if MJX_DEBUG_CONTROLLER > 0
  unsigned long now = millis();
  if(now - disp_tm > MJX_DEBUG_CONTROLLER)
  {
    //Serial.println();
    Serial.print(F(" * "));  Serial.print(yaw_in);
    Serial.print(F(" ")); Serial.print(gyro_yaw);
    Serial.print(F(" ")); Serial.print(yaw_out);
    Serial.print(F(" ")); Serial.print(yaw_kp);
    Serial.print(F(" ")); Serial.print(yaw_ki);
    Serial.print(F(" ")); Serial.print(yaw_kd);
    Serial.print(F(" ")); Serial.print(yaw_gyro_rate);
    Serial.println();
    disp_tm = now;
  }
  #endif
  
  update();
  if(updated) loc.reset();
}

#if MJX_DEBUG_SERVO
static uint64_t disp_tm2 = 0;
#endif

void MjxController::update()
{
  servo_throttle.write(throttle_out, MJX_SERVO_THROTTLE_SPEED);
  servo_yaw.write((throttle_out > 10 ? yaw_out : 0), MJX_SERVO_YAW_SPEED);
  servo_pitch.write(pitch_out, MJX_SERVO_PITCH_SPEED);
  servo_roll.write(roll_out, MJX_SERVO_ROLL_SPEED);

  servo_throttle.update();
  servo_yaw.update();
  servo_pitch.update();
  servo_roll.update();

  #if MJX_DEBUG_SERVO > 0
  unsigned long now = millis();
  if(false && now - disp_tm2 > MJX_DEBUG_SERVO)
  {
    //Serial.println();
    Serial.print(F(" * ")); Serial.print(servo_pitch.current());
    Serial.print(F(" "));   Serial.print(servo_pitch.target());
    Serial.print(F(" "));   Serial.print(servo_roll.current());
    Serial.print(F(" "));   Serial.print(servo_roll.target());
    Serial.println();
    disp_tm2 = now;
  }
  #endif
}

