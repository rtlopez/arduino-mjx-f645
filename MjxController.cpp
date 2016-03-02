#include "MjxController.h"
#include "libs/Tools.h"

MjxController::MjxController(MjxModel& m, MjxTelemetry& t):
  model(m), telemetry(t),
  throttle_in(0), throttle_out(0), 
  yaw_in(0), yaw_out(0),
  pitch_in(0), pitch_out((model.getServoPwmMax() - model.getServoPwmMin()) / 2),
  roll_in(0), roll_out((model.getServoPwmMax() - model.getServoPwmMin()) / 2),
  yaw_gyro(0.0), yaw_pid(&yaw_gyro, &yaw_out, &yaw_in, 1, 0, 0, DIRECT)
{}

void MjxController::begin()
{
  // attach servos
  throttle_servo.attach(model.getThrottlePin(), model.getMotorPwmMin(), model.getMotorPwmMax());
  yaw_servo.attach(model.getYawPin(), model.getMotorPwmMin(), model.getMotorPwmMax());
  pitch_servo.attach(model.getPitchPin(), model.getServoPwmMin(), model.getServoPwmMax());
  roll_servo.attach(model.getRollPin(), model.getServoPwmMin(), model.getServoPwmMax());

  // write initial values
  execute();

  // init PID loops
  yaw_pid.SetMode(AUTOMATIC);
  yaw_pid.SetOutputLimits(0, model.getMotorPwmMax() - model.getMotorPwmMin(), 0.3);
  yaw_pid.SetSampleTime(model.getUpdateInterval());

  prev_tm = millis();
}

void MjxController::update()
{
  unsigned long long tm = millis();
  if(tm - prev_tm >= static_cast<uint16_t>(model.getUpdateInterval()))
  {
    yaw_gyro   = model.getGyroZ();
  
    // ################# decode and adjust input ################# //
    throttle_in = model.getInputThrottle();
    yaw_in      = -model.getInputYaw();
    pitch_in    = -model.getInputPitch();
    roll_in     = model.getInputRoll();
  
    // add trimming offsets
    //yaw_in += model.getInputYawTrim();
    //pitch_in += model.getInputPitchTrim();
    //roll_in += model.getInputRollTrim();
  
    // ################# calculate output ####################### // 
    throttle_out = map(throttle_in, 0,  255, 0, model.getMotorPwmMax() - model.getMotorPwmMin());
    pitch_out    = map(pitch_in,  255, -255, 0, model.getServoPwmMax() - model.getServoPwmMin());
    roll_out     = map(roll_in,   255, -255, 0, model.getServoPwmMax() - model.getServoPwmMin());
    
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

    model.setOutputThrottle((int16_t)throttle_out);
    model.setOutputPitch((int16_t)pitch_out);
    model.setOutputRoll((int16_t)roll_out);
    prev_tm = tm;
  }
  
  if(yaw_pid.Compute())
  {
    model.setOutputYaw(yaw_out);
    model.setYawPidE(yaw_pid.getError());
    model.setYawPidP(yaw_pid.getP());
    model.setYawPidI(yaw_pid.getI());
    model.setYawPidD(yaw_pid.getD());
    execute();
  }
}

void MjxController::execute()
{
  throttle_servo.writeMicroseconds(model.getMotorPwmMin() + throttle_out);
  yaw_servo.writeMicroseconds(throttle_out > 50 ? model.getMotorPwmMin() + yaw_out : model.getMotorPwmMin());
  pitch_servo.writeMicroseconds(model.getServoPwmMin() + pitch_out);
  roll_servo.writeMicroseconds(model.getServoPwmMin() + roll_out);
}

