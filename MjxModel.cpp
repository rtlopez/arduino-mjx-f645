#include "MjxModel.h"

const MjxInput& MjxModel::getInput() const { return input; }
const RTVector3& MjxModel::getGyro() const { return gyro; }
const RTVector3& MjxModel::getPose() const { return pose; }

void MjxModel::updateInput(const MjxInput& value) { input = value; }
void MjxModel::updateGyro(const RTVector3& value) { gyro = value; }
void MjxModel::updatePose(const RTVector3& value) { pose = value; }

