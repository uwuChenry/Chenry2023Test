#include "main.h"

void autonomous() {
    //using drive::profileController;
    using drive::turnTo;
    using drive::driveChassis;
    //using drive::linearProflie;
    using drive::linearScurveProfile;
    using namespace drive;
    
    RRLib::CubicBezier testPath({0, 0, 0}, {0.5, 0, 0});
    RRLib::DiscretePath testPathDiscrete(testPath.generatePathByLengthWithCurvature(0.01));
    ppController.followPath(testPathDiscrete);
}