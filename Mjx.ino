#include "MjxModel.h"
#include "MjxRx.h"
#include "MjxPosition.h"
#include "MjxController.h"
#include "MjxTelemetry.h"

MjxModel model;
MjxTelemetry telemetry(model);
MjxRx receiver(model);
MjxPosition positioner(model);
MjxController controller(model, telemetry);

void setup()
{
  Serial.begin(115200);
  model.begin();
  controller.begin();
  positioner.begin();
  receiver.begin();
}

void loop()
{
  receiver.update();            // update transmitter data
  positioner.update();          // update actual orientation
  controller.update();          // update servos and motors
}

