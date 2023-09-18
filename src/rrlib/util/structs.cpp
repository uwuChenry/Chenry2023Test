#include "rrlib/util/structs.hpp"
#include "rrlib/util/math.hpp"

namespace RRLib {
using namespace okapi;

Pose::Pose(QLength x, QLength y, QAngle heading)
    : position(x, y)
    , heading(heading) {}
Pose::Pose()
    : Pose::Pose(0_m, 0_m, 0_deg) {}

Pose::Pose(Vector2 position, QAngle heading)
    : position(position)
    , heading(heading) {}

void Pose::turn(QAngle headingChange) {
    heading = math::constrainAngle(heading + headingChange);
}

KinematicConstraints::KinematicConstraints(double maxVel, double maxAccel, double maxJerk)
    : maxVel(maxVel)
    , maxAccel(maxAccel)
    , maxJerk(maxJerk) {}
}
