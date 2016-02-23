#include "MjxController.h"

float mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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
  yaw_pid.SetOutputLimits(0, 180, 0.5);
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
  if(input.yaw >= 0x80) yaw_in = 0x80 - input.yaw;
  yaw_in *= -180.0/127.0;
  
  pitch_in = input.pitch;
  if(input.pitch >= 0x80) pitch_in = 0x80 - input.pitch;
  //pitch_in *= -1.0;

  roll_in = input.roll;
  if(input.roll >= 0x80) roll_in = 0x80 - input.roll;
  roll_in *= -1.0;

  // add trimming offsets
  //yaw_in += input.yaw_trim - 64;
  //pitch_in += input.pitch_trim - 64;
  //roll_in += input.roll_trim - 64;

  // ################# calculate output ####################### //
  const double loFrom = -127.0 - 64.0;
  const double hiFrom = -loFrom;
  const double loTo = 0.0;
  const double hiTo = 180.0;
  
  throttle_out = mapf(throttle_in, 0, 255, loTo, hiTo);
  pitch_out = mapf(pitch_in, loFrom, hiFrom, loTo, hiTo);
  roll_out = mapf(roll_in, loFrom, hiFrom, loTo, hiTo);
  //yaw_out = mapf(yaw_in, loFrom, hiFrom, loTo, hiTo);
  
  // ################# tuning ################# //
  //double yaw_kp = 0.15;
  double yaw_kp = mapf(input.roll_trim, 1, 127, 0, model.config.yaw_pid_kp);      // 0.15
  
  //double yaw_ki = 0.35;
  double yaw_ki = mapf(input.yaw_trim, 1, 127, 0, model.config.yaw_pid_ki);       // 0.35
  
  //double yaw_kd = 0.05
  double yaw_kd = mapf(input.pitch_trim, 1, 127, 0, model.config.yaw_pid_kd);     // 0.05

  double yaw_gyro_rate = 0.1;                                                       // 0.1
  //double yaw_gyro_rate = mapf(input.roll_trim, 1, 127, 0, 0.05);  // 0.05
  
  yaw_gyro *= yaw_gyro_rate;

  // ################# Compute PID ################# //
  yaw_pid.SetTunings(yaw_kp, yaw_ki, yaw_kd);
  if(yaw_pid.Compute())
  {
  
    //yaw_out = mapf(yaw_out, 0, 127, loTo, hiTo);
    //yaw_out = multiMap(yaw_out, yaw_map_in, yaw_map_out, sizeof(yaw_map_in));
    
    //if(input.flags == 0x10) // yaw manual mode
    //{
    //  yaw_out = yaw_in > 0 ? yaw_in : 0;
    //}

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
  telemetry.print(yaw_in, MJX_DUMP_IN | MJX_DUMP_Z);
  telemetry.print(yaw_gyro, MJX_DUMP_GYRO | MJX_DUMP_Z);
  telemetry.print(yaw_out, MJX_DUMP_OUT | MJX_DUMP_Z);
}

