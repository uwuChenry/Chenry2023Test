#include "rrlib/util/math.hpp"

namespace RRLib {
using namespace okapi;

namespace math {
double constrainAngleDouble(double angle){
    return atan2(sin(angle / 180.0 * 3.14159265358979323846), cos(angle / 180.0 * 3.14159265358979323846))*180/3.14159265358979323846;
}

QAngle constrainAngle(QAngle in, QAngle min, QAngle max) {
    double out = in.convert(radian);

    while (out > max.convert(radian)) {
        out -= 2_pi;
    }

    while (out < min.convert(radian)) {
        out += 2_pi;
    }

    return out * radian;
}

double encoderTickToMeter(double encoderTicks){
        return encoderTicks/300 * (3.0 / 5.0) * 0.08255 * 3.141592653589793116;
    }
}
}