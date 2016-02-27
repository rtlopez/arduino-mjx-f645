#include "MjxModel.h"

void MjxModel::updateInput(const MjxInput& value) { input = value; }
const MjxInput& MjxModel::getInput() const { return input; }

const RTVector3& MjxModel::getGyro() const { return gyro; }
void MjxModel::updateGyro(const RTVector3& value)
{
  static const double r = 0.8;
  gyro.setX(value.x() * r + gyro.x() * (1 - r));
  gyro.setY(value.y() * r + gyro.y() * (1 - r));
  gyro.setZ(value.z() * r + gyro.z() * (1 - r));
}

const RTVector3& MjxModel::getPose() const { return pose; }
void MjxModel::updatePose(const RTVector3& value) { pose = value; }

