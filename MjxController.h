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
    void update(const MjxModel& model);
  private:
    uint8_t throttle_pin, yaw_pin, pitch_pin, roll_pin;
    double throttle_in, yaw_in, pitch_in, roll_in;
    double throttle_out, yaw_out, pitch_out, roll_out;
    double yaw_gyro, pitch_gyro, roll_gyro;
    MjxPID yaw_pid;
    MjxServo throttle_servo, yaw_servo, pitch_servo, roll_servo;
};

#endif
