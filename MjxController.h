#ifndef MjxController_h
#define MjxController_h

#include "MjxConfig.h"
#include "MjxRx.h"
#include "MjxPosition.h"
#include "MjxPID.h"
#include "MjxServo.h"

#include <Servo.h>

class MjxController
{
  public:
    MjxController(int throttle_pin, int yaw_pin, int pitch_pin, int roll_pin);
    void begin();
    void update();
    void update(const MjxControls& ctrl, MjxLocation& loc);
  private:
    double throttle_in, yaw_in, pitch_in, roll_in;
    double throttle_out, yaw_out, pitch_out, roll_out;
    double gyro_yaw, gyro_pitch, gyro_roll;
    uint8_t throttle_pin, yaw_pin, pitch_pin, roll_pin;
    
    MjxPID pid_yaw;
    MjxServo servo_throttle, servo_yaw, servo_pitch, servo_roll;
};

#endif
