#pragma once
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "rrlib/util/feedforward.hpp"
#include "rrlib/motion/odometry.hpp"
#include "rrlib/util/DescretePath.hpp"
#include <memory>


namespace RRLib{
using namespace okapi;

class AdaptivePurePursuitController{
    FeedForwardv3 feedForwardLeft;
    FeedForwardv3 feedForwardRight;
    //TwoWheelOdometry odometry;
    std::shared_ptr<okapi::SkidSteerModel> chassis;

    public:
    struct Gains{
        double maxVelocity{0.0};
        double maxAcceleration{0.0};
        double maxTurnVelocity{0.0};
        QLength lookAhead{0.0};

        Gains() = default;
        ~Gains() = default;
        Gains(double maxVelocity, double maxAcceleration, double maxTurnVelocity, QLength lookAhead);
        bool operator==(const Gains& rhs) const;
        bool operator!=(const Gains& rhs) const;
    };

    AdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& ichassis, 
                                  const TimeUtil& timeUtil = okapi::TimeUtilFactory::createDefault()){
                                     chassis = std::static_pointer_cast<okapi::SkidSteerModel>(ichassis->getModel());
                                  }

    void followPath(DiscretePath& path, QTime timeout = 2_min, bool isReversed = false);

    void stop();

    void waitUntilSettled();

    private:

    std::optional<Vector2> getLookaheadPoint(DiscretePath& path, double& minIndex, Vector2 point, QLength radius);
    void generateKinematics();

    std::shared_ptr<OdomChassisController> chassis;
    std::shared_ptr<AbstractMotor> leftMotor;
    std::shared_ptr<AbstractMotor> rightMotor;
    
    Gains gains;

	TimeUtil timeUtil;

    pros::Task task;
};

}