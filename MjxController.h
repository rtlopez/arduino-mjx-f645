#ifndef MjxController_h
#define MjxController_h

#include "Arduino.h"
#include "libs/PID/PID.h"
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
  private:
    const MjxModel& model;
    MjxTelemetry& telemetry;
    int16_t throttle_in, throttle_out;
    double yaw_in, yaw_out;
    int16_t pitch_in, pitch_out;
    int16_t roll_in, roll_out;
    double yaw_gyro;
    PID yaw_pid;
    Servo throttle_servo, yaw_servo, pitch_servo, roll_servo;
};

#endif
