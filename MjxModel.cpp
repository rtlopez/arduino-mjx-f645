#include "MjxModel.h"

void MjxModel::updateInput(const MjxInput& value) { input = value; }
const MjxInput& MjxModel::getInput() const { return input; }

const RTVector3& MjxModel::getGyro() const { return gyro; }
const RTVector3& MjxModel::getPose() const { return pose; }
void MjxModel::updatePose(const RTVector3& value, float dt)
{
  const RTVector3& prevPose = pose;
  pose = value;
  gyro.setX((pose.x() - prevPose.x()) / dt);
  gyro.setY((pose.y() - prevPose.y()) / dt);
  gyro.setZ((pose.z() - prevPose.z()) / dt);
}

