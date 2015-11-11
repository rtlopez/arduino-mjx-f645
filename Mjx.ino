#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#include "MjxRx.h"
#include "MjxPosition.h"
#include "MjxController.h"
#include "MjxProfiler.h"

// radio pins
#define CE_PIN 8
#define CSN_PIN 7

// PWM output pins
#define THROTLE_PIN 3   // gaz
#define YAW_PIN 5       // obrot gyro_z
#define PITCH_PIN 10    // przod/tyl gyro_y
#define ROLL_PIN 9      // prawo/lewo, gyro_x

nRF24 radio(CE_PIN, CSN_PIN);
MjxRx receiver(radio);

MjxControls controls;
MjxLocation location;

MjxPosition positioner;
MjxController controller(THROTLE_PIN, YAW_PIN, PITCH_PIN, ROLL_PIN);

MjxProfiler rec_p("rec", 5000);
MjxProfiler pos_p("pos", 5000);
MjxProfiler ctl_p("ctl", 5000);

void setup()
{
  //Serial.begin(57600);
  Serial.begin(115200);
  receiver.begin();
  controller.begin();
  positioner.begin();
}

void loop()
{
  rec_p.start();
  receiver.update(controls);                      // update transmitter data
  rec_p.stop();

  pos_p.start();
  positioner.update(location);                    // update actual orientation
  pos_p.stop();

  ctl_p.start();
  controller.update(controls, location);          // update servos and motors
  ctl_p.stop();
}

