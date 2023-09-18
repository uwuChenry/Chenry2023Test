#pragma once

#include "rrlib/util/Vector2.hpp"
#include "rrlib/util/structs.hpp"
#include "rrlib/util/math.hpp"
#include "okapi/api/chassis/controller/chassisScales.hpp"
#include "okapi/api/chassis/model/chassisModel.hpp"
#include "okapi/api/device/rotarysensor/continuousRotarySensor.hpp"

namespace RRLib {
class PoseEstimator {
public:
    virtual Pose getPose() = 0;
    virtual void setPose(Pose pose) = 0;
    virtual void update() = 0;
};

class ManualPoseEstimator : public PoseEstimator {
    pros::Mutex lock;
    Pose currentPose{0_in, 0_in, 0_deg};

public:
    virtual Pose getPose() override;
    virtual void setPose(Pose pose) override;
    virtual void update() override;
};

class TwoWheelOdometry : public PoseEstimator {
    pros::Mutex lock;
    Pose currentPose;
    std::shared_ptr<okapi::ReadOnlyChassisModel> model;
    okapi::ChassisScales scales;

    double lastLeft = 0.0;
    double lastRight = 0.0;

public:
    TwoWheelOdometry(
        std::shared_ptr<okapi::ReadOnlyChassisModel> model,
        okapi::ChassisScales scales);

    virtual Pose getPose() override;
    virtual void setPose(Pose pose) override;
    virtual void update() override;
    void reset();
    double getLeftEnc();
    double getRightEnc();
};
}
