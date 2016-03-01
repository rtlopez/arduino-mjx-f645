#include "MjxController.h"
#include "libs/Tools.h"

MjxController::MjxController(const MjxModel& m, MjxTelemetry& t):
  model(m), telemetry(t),
  throttle_in(0), throttle_out(model.config.motor_pwm_min), 
  yaw_in(0), yaw_out(model.config.motor_pwm_min),
  pitch_in(0), pitch_out((model.config.servo_pwm_min + model.config.servo_pwm_max) / 2),
  roll_in(0), roll_out((model.config.servo_pwm_min + model.config.servo_pwm_max) / 2),
  yaw_gyro(0.0), yaw_pid(&yaw_gyro, &yaw_out, &yaw_in, 1, 0, 0, DIRECT)
{}

void MjxController::begin()
{
  // attach servos
  throttle_servo.attach(model.config.throttle_pin, model.config.motor_pwm_min, model.config.motor_pwm_max);
  yaw_servo.attach(model.config.yaw_pin, model.config.motor_pwm_min, model.config.motor_pwm_max);
  pitch_servo.attach(model.config.pitch_pin, model.config.servo_pwm_min, model.config.servo_pwm_max);
  roll_servo.attach(model.config.roll_pin, model.config.servo_pwm_min, model.config.servo_pwm_max);

  // write initial values
  execute();

  // init PID controllers
  yaw_pid.SetMode(AUTOMATIC);
  yaw_pid.SetOutputLimits(0, model.config.motor_pwm_max - model.config.motor_pwm_min, 0.3);
  yaw_pid.SetSampleTime(model.config.update_interval);
}

void MjxController::update()
{
  const MjxInput input = model.getInput();
  const RTVector3 gyro = model.getGyro();

  yaw_gyro   = gyro.z();

  // ################# decode and adjust input ################# //
  throttle_in = input.throttle;
  yaw_in = -input.yaw;
  pitch_in = -input.pitch;
  roll_in = input.roll;

  // add trimming offsets
  //yaw_in += input.yaw_trim;
  //pitch_in += input.pitch_trim;
  //roll_in += input.roll_trim;

  // ################# calculate output ####################### // 
  throttle_out = map(throttle_in, 0,  255, model.config.motor_pwm_min, model.config.motor_pwm_max);
  pitch_out    = map(pitch_in,  255, -255, model.config.servo_pwm_min, model.config.servo_pwm_max);
  roll_out     = map(roll_in,   255, -255, model.config.servo_pwm_min, model.config.servo_pwm_max);
  
  // ################# tuning ################# //
  //double yaw_kp = 0.15;
  double yaw_kp = Tools::map(input.roll_trim, -64.0, 64.0, 0.0, model.config.yaw_pid_kp);      // 0.15
  
  //double yaw_ki = 0.35;
  double yaw_ki = Tools::map(input.yaw_trim, -64.0, 64.0, 0.0, model.config.yaw_pid_ki);       // 0.35
  
  //double yaw_kd = 0.05;
  double yaw_kd = Tools::map(input.pitch_trim, -64.0, 64.0, 0.0, model.config.yaw_pid_kd);     // 0.05

  double yaw_gyro_rate = 40.0;
  yaw_gyro *= yaw_gyro_rate;

  // ################# Compute PID ################# //
  yaw_pid.SetTunings(yaw_kp, yaw_ki, yaw_kd);
  if(yaw_pid.Compute())
  {
    yaw_out += model.config.motor_pwm_min;

    //Serial.print(yaw_in); Serial.print(" ");
    //Serial.print(yaw_gyro); Serial.print(" ");
    //telemetry << yaw_in << yaw_gyro << yaw_pid.getP() << yaw_pid.getI() << yaw_pid.getD() << "\n";
    //telemetry << throttle_in * 10 << throttle_out << "\n";
    
    execute();
    output();
  }
}

void MjxController::execute()
{
  throttle_servo.writeMicroseconds(throttle_out);
  yaw_servo.writeMicroseconds(throttle_out > 1050 ? yaw_out : model.config.motor_pwm_min);
  pitch_servo.writeMicroseconds(pitch_out);
  roll_servo.writeMicroseconds(roll_out);
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

