#pragma once
#include <algorithm>
#include <cmath>
#include "rrlib/util/structs.hpp"
namespace RRLib{

class PID {
    double error = 0, lastError = 0, derivative = 0, integral = 0;
    double output;
    pidGains gains;

    public:
    PID(pidGains igains);
    double calculate (double input);
    void setGains(pidGains igains);
    double getOutput();
    double getError();
    double getIntegral();
    
};
}
