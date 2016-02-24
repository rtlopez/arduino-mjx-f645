#ifndef MjxController_h
#define MjxController_h

#include "MjxPID.h"
#include "MjxServo.h"
#include "MjxModel.h"
#include "MjxTelemetry.h"

#include <Servo.h>

class MjxController
{
  public:
    MjxController(const MjxModel& m, MjxTelemetry& t);
    void begin();
    void update();
    void execute();
    void output();
  private:
    const MjxModel& model;
    MjxTelemetry& telemetry;
    double throttle_in, yaw_in, pitch_in, roll_in;
    double throttle_out, yaw_out, pitch_out, roll_out;
    double yaw_gyro, pitch_gyro, roll_gyro;
    MjxPID yaw_pid;
    MjxServo throttle_servo, yaw_servo, pitch_servo, roll_servo;
};

#endif
