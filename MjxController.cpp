#include "MjxController.h"
#include "libs/Tools.h"

MjxController::MjxController(const MjxModel& m, MjxTelemetry& t):
  model(m), telemetry(t),
  throttle_in(0), throttle_out(model.getMotorPwmMin()), 
  yaw_in(0), yaw_out(model.getMotorPwmMin()),
  pitch_in(0), pitch_out((model.getServoPwmMin() + model.getServoPwmMax()) / 2),
  roll_in(0), roll_out((model.getServoPwmMin() + model.getServoPwmMax()) / 2),
  yaw_gyro(0.0), yaw_pid(&yaw_gyro, &yaw_out, &yaw_in, 1, 0, 0, DIRECT)
{}

void MjxController::begin()
{
  // attach servos
  throttle_servo.attach(model.getThrottlePin(), model.getMotorPwmMin(), model.getMotorPwmMin());
  yaw_servo.attach(model.getYawPin(), model.getMotorPwmMin(), model.getMotorPwmMin());
  pitch_servo.attach(model.getPitchPin(), model.getServoPwmMin(), model.getServoPwmMax());
  roll_servo.attach(model.getRollPin(), model.getServoPwmMin(), model.getServoPwmMax());

  // write initial values
  execute();

  // init PID controllers
  yaw_pid.SetMode(AUTOMATIC);
  yaw_pid.SetOutputLimits(0, model.getMotorPwmMax() - model.getMotorPwmMin(), 0.3);
  yaw_pid.SetSampleTime(model.getUpdateInterval());
}

void MjxController::update()
{
  yaw_gyro   = model.getGyroZ();

  // ################# decode and adjust input ################# //
  throttle_in = model.getInputThrottle();
  yaw_in = -model.getInputYaw();
  pitch_in = -model.getInputPitch();
  roll_in = model.getInputRoll();

  // add trimming offsets
  //yaw_in += model.getInputYawTrim();
  //pitch_in += model.getInputPitchTrim();
  //roll_in += model.getInputRollTrim();

  // ################# calculate output ####################### // 
  throttle_out = map(throttle_in, 0,  255, model.getMotorPwmMin(), model.getMotorPwmMax());
  pitch_out    = map(pitch_in,  255, -255, model.getServoPwmMin(), model.getServoPwmMax());
  roll_out     = map(roll_in,   255, -255, model.getServoPwmMin(), model.getServoPwmMax());
  
  // ################# tuning ################# //
  //double yaw_kp = 0.15;
  double yaw_kp = Tools::map(model.getInputRollTrim(), -64.0, 64.0, 0.0, model.getYawPidKp());      // 0.15
  
  //double yaw_ki = 0.35;
  double yaw_ki = Tools::map(model.getInputYawTrim(), -64.0, 64.0, 0.0, model.getYawPidKi());       // 0.35
  
  //double yaw_kd = 0.05;
  double yaw_kd = Tools::map(model.getInputPitchTrim(), -64.0, 64.0, 0.0, model.getYawPidKd());     // 0.05

  double yaw_gyro_rate = 40.0;
  yaw_gyro *= yaw_gyro_rate;

  // ################# Compute PID ################# //
  yaw_pid.SetTunings(yaw_kp, yaw_ki, yaw_kd);
  if(yaw_pid.Compute())
  {
    yaw_out += model.getMotorPwmMin();

    //Serial.print(yaw_in); Serial.print(" ");
    //Serial.print(yaw_gyro); Serial.print(" ");
    //Serial.println();
    //telemetry << yaw_in << yaw_gyro << yaw_pid.getP() << yaw_pid.getI() << yaw_pid.getD() << "\n";
    //telemetry << throttle_in * 10 << throttle_out << "\n";
    
    execute();
  }
}

void MjxController::execute()
{
  throttle_servo.writeMicroseconds(throttle_out);
  yaw_servo.writeMicroseconds(throttle_out > 1050 ? yaw_out : model.getMotorPwmMin());
  pitch_servo.writeMicroseconds(pitch_out);
  roll_servo.writeMicroseconds(roll_out);
}

