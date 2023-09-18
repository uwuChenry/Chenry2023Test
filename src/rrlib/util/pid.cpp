#include "rrlib/util/pid.hpp"
namespace RRLib
{
    PID::PID(pidGains igains)
    {
        gains.kP = igains.kP;
        gains.kI = igains.kI;
        gains.kD = igains.kD;
        gains.limit = igains.limit;
        gains.windup = igains.windup;
    }

    void PID::setGains(pidGains igains){
        gains.kP = igains.kP;
        gains.kI = igains.kI;
        gains.kD = igains.kD;
        gains.limit = igains.limit;
        gains.windup = igains.windup;
    }

    double PID::calculate (double error){
        integral = (fabs(error) <= gains.windup) ? integral += error : 0;
        integral *= (std::signbit(error) == std::signbit(lastError));
        integral = std::clamp(integral, -gains.limit, gains.limit);
        derivative = error - lastError;
        output = (error * gains.kP) + (integral * gains.kI) + (derivative * gains.kD);
        lastError = error;
        return output;
    }

    double PID::getError(){
        return error;
    }

    double PID::getIntegral(){
        return integral;
    }

    double PID::getOutput() {
        return output;
    }
} 