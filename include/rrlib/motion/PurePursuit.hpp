#pragma once
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/impl/util/timeUtilFactory.hpp"
#include "rrlib/util/feedforward.hpp"
#include "rrlib/motion/odometry.hpp"
#include <memory>


namespace RRLib{
using namespace okapi;

class AdaptivePurePursuitController{
    FeedForwardv3 feedForwardLeft;
    FeedForwardv3 feedForwardRight;
    TwoWheelOdometry odometry;

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

    AdaptivePurePursuitController(const std::shared_ptr<OdomChassisController>& chassis, 
                                  const Gains& gains,
                                  FeedForwardv3 iFFLeft,
                                  FeedForwardv3 iFFRight,
                                  const TimeUtil& timeUtil = okapi::TimeUtilFactory::createDefault());

    void followPath(DiscretePath& path, QTime timeout = 2_min, bool isReversed = false);

    void stop();

    void waitUntilSettled();

    private:

    std::optional<Point> getLookaheadPoint(DiscretePath& path, double& minIndex, const Point& point, QLength radius);
    void generateKinematics();

    std::shared_ptr<OdomChassisController> chassis;
    std::shared_ptr<AbstractMotor> leftMotor;
    std::shared_ptr<AbstractMotor> rightMotor;
    
    Gains gains;

	TimeUtil timeUtil;

	std::unique_ptr<FeedforwardController<QLength>> leftController;
	std::unique_ptr<FeedforwardController<QLength>> rightController;

    pros::Task task;
};

}