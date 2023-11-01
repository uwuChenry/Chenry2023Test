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
    // FeedForwardv3 feedForwardLeft;
    // FeedForwardv3 feedForwardRight;
    //TwoWheelOdometry odometry;
    private:

    std::optional<Vector2> getLookaheadPoint(DiscretePath& path, int minIndex, Vector2 point, QLength radius);
    // void generateKinematics();
    std::shared_ptr<OdomChassisController> chassisController;
    std::shared_ptr<AbstractMotor> leftMotor;
    std::shared_ptr<AbstractMotor> rightMotor;
    std::shared_ptr<okapi::SkidSteerModel> chassis;
    // Gains gains;

    std::optional<int> prevClosest {std::nullopt};

	TimeUtil timeUtil;
    QLength lookAhead = 15_cm;

    pros::Task* task = nullptr;

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
                                  const TimeUtil& itimeUtil = okapi::TimeUtilFactory::createDefault())
                                  : chassisController (ichassis)
                                  , timeUtil(itimeUtil)
                                  {
                                     chassis = std::static_pointer_cast<okapi::SkidSteerModel>(ichassis->getModel());
                                     leftMotor = chassis->getLeftSideMotor();
                                     rightMotor = chassis->getRightSideMotor();
                                  }

    void followPath(DiscretePath& path, QTime timeout = 2_min, bool isReversed = false);
    int getClosestPoint(Pose currentPos, DiscretePath& path);
    void stop();
    

    // void waitUntilSettled();


};

}