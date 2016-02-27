#include "MjxController.h"
#include "libs/Tools.h"

MjxController::MjxController(const MjxModel& m, MjxTelemetry& t):
  model(m), telemetry(t),
  yaw_pid(&yaw_gyro, &yaw_out, &yaw_in, 1, 0, 0, DIRECT),
  throttle_servo(model.config.update_interval), yaw_servo(model.config.update_interval), pitch_servo(model.config.update_interval), roll_servo(model.config.update_interval)
{
  throttle_in = 0;
  yaw_in = 0;
  pitch_in = 0;
  roll_in = 0; 

  throttle_out = 0;
  yaw_out = 0;
  pitch_out = 90;
  roll_out = 90;

  yaw_gyro = 0;
  pitch_gyro = 0;
  roll_gyro = 0; 
}

void MjxController::begin()
{
  // attach servos
  throttle_servo.attach(model.config.throttle_pin, 1000, 2000);
  yaw_servo.attach(model.config.yaw_pin, 1000, 2000);
  pitch_servo.attach(model.config.pitch_pin, 1000, 2000);
  roll_servo.attach(model.config.roll_pin, 1000, 2000);

  // write initial values
  throttle_servo.write(throttle_out, true);
  yaw_servo.write(yaw_out, true);
  pitch_servo.write(pitch_out, true);
  roll_servo.write(roll_out, true);

  // init PID controllers
  yaw_pid.SetMode(AUTOMATIC);
  yaw_pid.SetOutputLimits(0, 180, 0.6);
  yaw_pid.SetSampleTime(model.config.update_interval);
}

void MjxController::update()
{
  const MjxInput input = model.getInput();
  const RTVector3 gyro = model.getGyro();

  yaw_gyro   = gyro.z();
  pitch_gyro = gyro.x();
  roll_gyro  = gyro.y();

  // ################# decode and adjust input ################# //
  throttle_in = input.throttle;
  yaw_in = input.yaw;
  pitch_in = input.pitch;
  roll_in = input.roll;

  // add trimming offsets
  //yaw_in += input.yaw_trim;
  //pitch_in += input.pitch_trim;
  //roll_in += input.roll_trim;

  // ################# calculate output ####################### // 
  throttle_out = map(throttle_in, 0,  255, 0, 180);
  pitch_out    = map(pitch_in, -255,  255, 0, 180);
  roll_out     = map(roll_in,   255, -255, 0, 180);
  
  // ################# tuning ################# //
  //double yaw_kp = 0.15;
  double yaw_kp = Tools::map(input.roll_trim, -64.0, 64.0, 0.0, model.config.yaw_pid_kp);      // 0.15
  
  //double yaw_ki = 0.35;
  double yaw_ki = Tools::map(input.yaw_trim, -64.0, 64.0, 0.0, model.config.yaw_pid_ki);       // 0.35
  
  //double yaw_kd = 0.05;
  double yaw_kd = Tools::map(input.pitch_trim, -64.0, 64.0, 0.0, model.config.yaw_pid_kd);     // 0.05

  double yaw_gyro_rate = 40.0;
  //double yaw_gyro_rate = model.config.update_interval / 1000.0;                              // 0.05
  //double yaw_gyro_rate = Tools::map(input.roll_trim, -1, 1, 0, 0.05);                        // 0.05
  
  yaw_gyro *= yaw_gyro_rate;

  // ################# Compute PID ################# //
  yaw_pid.SetTunings(yaw_kp, yaw_ki, yaw_kd);
  if(yaw_pid.Compute())
  {
    //Serial.print(yaw_in); Serial.print(" ");
    //Serial.print(yaw_gyro); Serial.print(" ");
    //Serial.print(yaw_out); Serial.print(" ");
    //Serial.print(yaw_pid.getP()); Serial.print(" ");
    //Serial.print(yaw_pid.getI()); Serial.print(" ");
    //Serial.print(yaw_pid.getD()); Serial.print(" ");
    //Serial.print(yaw_pid.getError()); Serial.print(" ");
    //Serial.print(yaw_kp); Serial.print(" ");
    //Serial.print(yaw_ki); Serial.print(" ");
    //Serial.print(yaw_kd); Serial.print(" ");
    //Serial.println();
    execute();
    output();
  }
}

void MjxController::execute()
{
  throttle_servo.write(throttle_out, 0.0);
  yaw_servo.write((throttle_out > 10 ? yaw_out : 0), 0.0);
  pitch_servo.write(pitch_out, 300.0);
  roll_servo.write(roll_out, 300.0);
  
  throttle_servo.update();
  yaw_servo.update();
  pitch_servo.update();
  roll_servo.update();
}

void MjxController::output()
{
  //telemetry.print(throttle_in, MJX_DUMP_IN | MJX_DUMP_T);
  //telemetry.print(throttle_out, MJX_DUMP_OUT | MJX_DUMP_T);
  //telemetry.print(yaw_in, MJX_DUMP_IN | MJX_DUMP_Z);
  //telemetry.print(pitch_in, MJX_DUMP_IN | MJX_DUMP_X);
  //telemetry.print(roll_in, MJX_DUMP_IN | MJX_DUMP_Y);
  //telemetry.print(yaw_gyro, MJX_DUMP_GYRO | MJX_DUMP_Z);
  //telemetry.print(yaw_out, MJX_DUMP_OUT | MJX_DUMP_Z);
  //telemetry.print(yaw_pid.getError(), MJX_DUMP_OUT | MJX_DUMP_Z);
}

