#include "rrlib/motion/odometry.hpp"

namespace RRLib {
using namespace okapi;

void ManualPoseEstimator::update() {}

Pose ManualPoseEstimator::getPose() {
    lock.take(TIMEOUT_MAX);
    auto ret = currentPose;
    lock.give();
    return ret;
}

void ManualPoseEstimator::setPose(Pose p) {
    lock.take(TIMEOUT_MAX);
    currentPose = p;
    lock.give();
}

TwoWheelOdometry::TwoWheelOdometry(std::shared_ptr<ReadOnlyChassisModel> model, ChassisScales scales)
    : currentPose(0_m, 0_m, 0_rad)
    , model(model)
    , scales(scales) {}

void TwoWheelOdometry::update() {
    lock.take(TIMEOUT_MAX);
    auto newSensorValues = model->getSensorVals();

    double deltaLeft = (newSensorValues[0] - lastLeft) / scales.straight;
    double deltaRight = (newSensorValues[1] - lastRight) / scales.straight;

    lastLeft = newSensorValues[0];
    lastRight = newSensorValues[1];

    double deltaS = (deltaLeft + deltaRight) / 2.0;
    double deltaTheta = (deltaLeft - deltaRight) / scales.wheelTrack.convert(meter);
    
    double r = deltaTheta == 0 ? 0 : deltaS / deltaTheta;

    Vector2 positionUpdate(
        meter * (deltaTheta == 0 ? deltaS : (r * std::sin(deltaTheta))),
        meter * (deltaTheta == 0 ? 0 : (r * std::cos(deltaTheta) - r)));
    
    positionUpdate.rotateSelf(currentPose.heading);

    currentPose.position.addSelf(positionUpdate);
    currentPose.turn(radian * deltaTheta);
    lock.give();
}

Pose TwoWheelOdometry::getPose() {
    lock.take(TIMEOUT_MAX);
    auto ret = currentPose;
    lock.give();
    return ret;
}

void TwoWheelOdometry::reset(){
    lock.take(TIMEOUT_MAX);
    currentPose = {0_m, 0_m, 0_deg};
    lastLeft = model->getSensorVals()[0];
    lastRight = model->getSensorVals()[1];
    lock.give();
}

void TwoWheelOdometry::setPose(Pose p) {
    lock.take(TIMEOUT_MAX);
    currentPose = p;
    lock.give();
}
}