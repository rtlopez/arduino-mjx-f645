#include <SPI.h>
#include <Servo.h>
#include <Wire.h>

#include "MjxConfig.h"

#include "RTIMU.h"
#include "RTMath.h"
#include "RTIMUSettings.h"
#include "RTFusionRTQF.h" 
#include "CalLib.h"

#include "MjxRx.h"
#include "MjxPosition.h"
#include "MjxController.h"

MjxModel model;
MjxRx receiver(MJX_RADIO_CE_PIN, MJX_RADIO_CSN_PIN);
MjxPosition positioner;
MjxController controller(MJX_CONTROLLER_THROTLE_PIN, MJX_CONTROLLER_YAW_PIN, MJX_CONTROLLER_PITCH_PIN, MJX_CONTROLLER_ROLL_PIN);

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  controller.begin();
  positioner.begin();
  receiver.begin();
}

void loop()
{
  receiver.update(model);            // update transmitter data
  positioner.update(model);          // update actual orientation
  controller.update(model);          // update servos and motors
}

