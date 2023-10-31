#pragma once

#include "okapi/api/units/QLength.hpp"
#include <optional>
#include "rrlib/util/Vector2.hpp"



namespace RRLib {
namespace math {
using namespace okapi::literals;


okapi::QAngle constrainAngle(okapi::QAngle in, okapi::QAngle min = -180_deg, okapi::QAngle max = 180_deg);
double constrainAngleDouble(double angle);
double encoderTickToMeter(double encoderTicks);
std::optional<double> circleLineIntersection(Vector2& start, Vector2& end, Vector2& point, okapi::QLength radius);
okapi::QLength getCircumRadius(Vector2 a, Vector2 b, Vector2 c);

}

class Settled {
    private:
    double atTargetTime = 200;
    double atTargetError = 10;
    double atTargetDerivative = 5;
    double lastError = 0;
    int atTargetCounter;

    public:
    Settled (double targetTime, double targetError, double targetDerivative)
    : atTargetTime(targetTime)
    , atTargetError(targetError)
    , atTargetDerivative(targetDerivative){};

    bool isSettled (double error){
        if (fabs(error) <= atTargetError && fabs(error - lastError) <= atTargetDerivative){
            atTargetCounter++;
        }
        else atTargetCounter = 0;
        lastError = error;
        return (atTargetCounter*10) > atTargetTime;
    }
};
}