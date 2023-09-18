#pragma once

#include "okapi/api/units/QLength.hpp"
#include "okapi/api/units/QAngle.hpp"
#include "okapi/api/units/QSpeed.hpp"
#include "okapi/api/units/QAcceleration.hpp"
#include "okapi/api/units/QJerk.hpp"

#include "rrlib/util/Vector2.hpp"

namespace RRLib {
struct KinematicConstraints {
    double maxVel;
    double maxAccel;
    double maxJerk;

public:
    KinematicConstraints(double maxVel, double maxAccel, double maxJerk = 0.0);
};

struct Pose {
    okapi::QAngle heading;
    Vector2 position;

    Pose();
    Pose(okapi::QLength x, okapi::QLength y, okapi::QAngle heading = 0_deg);
    Pose(Vector2 position, okapi::QAngle heading = 0_deg);

    void turn(okapi::QAngle headingChange); // TODO constrain this reee
};

//kP, kI, kD, limit, windup
struct pidGains{
    double kP;
    double kI;
    double kD;
    double limit;
    double windup;

    public:
    pidGains()
        : pidGains(0, 0, 0, 0, 0){}
    pidGains(double ikP, double ikD)
        : kP(ikP)
        , kI(0)
        , kD(ikD){}
    
    //kp, kp, kd, limit, windup
    pidGains(double ikP, double ikI, double ikD, double ilimit, double iwindup)
        : kP(ikP)
        , kI(ikI)
        , kD(ikD)
        , limit(ilimit)
        , windup(iwindup){}
};

struct FeedForwardGains{
    double kV;
    double kA;
    double kP_Pos;
    double kP_Vel;
    double kA_Down;
};


//kv, ka, kpPos, kpVel, KaDown, kS;
struct FeedForwardGains2{
    double kV;
    double kA;
    double kP_Pos;
    double kP_Vel;
    double kA_Down;
    double kS;
};

struct trajectory{
    double velocity;
    double acceleration;
    double position;
    double time;
};

struct distanceSet{
    okapi::QLength distance;
    bool isReversed;
};
}