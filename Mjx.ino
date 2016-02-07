#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#include "MjxConfig.h"
#include "MjxRx.h"
#include "MjxPosition.h"
#include "MjxController.h"
#include "MjxProfiler.h"

nRF24 radio(MJX_RADIO_CE_PIN, MJX_RADIO_CSN_PIN);
MjxRx receiver(radio);

MjxControls controls;
MjxLocation location;

MjxPosition positioner;
MjxController controller(MJX_CONTROLLER_THROTLE_PIN, MJX_CONTROLLER_YAW_PIN, MJX_CONTROLLER_PITCH_PIN, MJX_CONTROLLER_ROLL_PIN);

//MjxProfiler rec_p("rec", 5000);
//MjxProfiler pos_p("pos", 5000);
//MjxProfiler ctl_p("ctl", 5000);

void setup()
{
  //Serial.begin(57600);
  Serial.begin(115200);
  controller.begin();
  positioner.begin();
  receiver.begin();
}

void loop()
{
  //rec_p.start();
  receiver.update(controls);                      // update transmitter data
  //rec_p.stop();

  //pos_p.start();
  positioner.update(location);                    // update actual orientation
  //pos_p.stop();

  //ctl_p.start();
  controller.update(controls, location);          // update servos and motors
  //ctl_p.stop();
}

