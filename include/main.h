#pragma once
#define PROS_USE_SIMPLE_NAMES

#include "api.h"

#include <functional>
#include <vector>


#include "rrlib/rrlib.hpp"
#include "okapi/api.hpp"
#include "okapi/impl/util/configurableTimeUtilFactory.hpp"

// namespacing
using pros::Mutex;
using pros::Task;
using pros::Vision;

using namespace okapi;
using namespace okapi::literals;
using namespace RRLib;

namespace lcd {
using namespace pros::lcd;
}

namespace competition {
using namespace pros::competition;
static bool is_enabled() {
    return !is_disabled();
}

}
namespace battery {
using namespace pros::battery;
}

using pros::c::delay;
using pros::c::millis;
// end of namespacing

// subsystem includes
#include "systems/drive.hpp"

// end of subsystem includes


// pros comp control stuff
#ifdef __cplusplus
extern "C" {
#endif
void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);
#ifdef __cplusplus
}
#endif