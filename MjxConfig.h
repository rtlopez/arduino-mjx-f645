#ifndef MjxConfig_h
#define MjxConfig_h

#include "Arduino.h"

enum mjx_dump_flags {
  MJX_DUMP_X    = 1 << 0,
  MJX_DUMP_Y    = 1 << 1,
  MJX_DUMP_Z    = 1 << 2,
  MJX_DUMP_T    = 1 << 3,
  MJX_DUMP_IN   = 1 << 4,
  MJX_DUMP_OUT  = 1 << 5,
  MJX_DUMP_GYRO = 1 << 6,
  MJX_DUMP_POSE = 1 << 7,
  MJX_DUMP_PID  = 1 << 8
};

class MjxConfig
{
  public:
    MjxConfig(): 
      dump_ts(0), 
      dump_flags(MJX_DUMP_T | MJX_DUMP_X | MJX_DUMP_Y | MJX_DUMP_Z | MJX_DUMP_IN | MJX_DUMP_OUT | MJX_DUMP_GYRO | MJX_DUMP_POSE | MJX_DUMP_PID),
      dump_interval(40),
      update_interval(20),
      radio_ce_pin(8),
      radio_csn_pin(7),
      throttle_pin(3),
      yaw_pin(5),
      pitch_pin(10),
      roll_pin(9),
      yaw_pid_kp(0.30),
      yaw_pid_ki(0.60),
      yaw_pid_kd(0.01)
      {}
      
    uint32_t dump_ts;         // telemetry last dump timestamp
    uint16_t dump_flags;      // telemetry flags
    uint16_t dump_interval;   // telemetry dump interval
    uint16_t update_interval; // PID and servo update interval
    uint8_t radio_ce_pin;     // radio CE pin
    uint8_t radio_csn_pin;    // radio CSN pin
    uint8_t throttle_pin;     // throttle
    uint8_t yaw_pin;          // tail rotate, (gyro_z)
    uint8_t pitch_pin;        // move forward/backward, (gyro_y)
    uint8_t roll_pin;         // move left/right, (gyro_x)
    double yaw_pid_kp;        // Yaw PID P param
    double yaw_pid_ki;        // Yaw PID I param
    double yaw_pid_kd;        // Yaw PID D param
};

#endif
