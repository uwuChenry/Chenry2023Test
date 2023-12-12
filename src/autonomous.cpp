#include "main.h"

double rpmToLinear(double rpm){
    return 0.08255 * (3.0/5.0) / 2 * (rpm / 60 * 6.28318530718);
}

void autonomous() {
    //using drive::profileController;
    using drive::turnTo;
    using drive::driveChassis;
    //using drive::linearProflie;
    using drive::linearScurveProfile;
    using namespace drive;
    
    // RRLib::CubicBezier testPath({0, 0, 0}, {1, 0, 0});
    // RRLib::DiscretePath testPathDiscrete(testPath.generatePathByLengthWithCurvature(0.05));
    // ppController.followPath(testPathDiscrete);
    //linearScurveProfile.moveForwards(2_m);
    //linearScurveProfile.printVectorPosition(linearScurveProfile.pathTrajectory);
    //linearProflie.moveForwards(2_m);



    // int timeInMs = 2250;
    // leftMotors.tarePosition();
    // rightMotors.tarePosition();
    // printf("test");
    // double filteredVelocity = 0;
    // for (int i = 0; i < timeInMs/10; i++)
    // {
    //         leftMotors.moveVoltage(5000);
    //         rightMotors.moveVoltage(5000);
    //         delay(10);
    //         double leftcurrentvelocity = rpmToLinear(leftMotors.getActualVelocity());
    //         filteredVelocity = filteredVelocity * 0.75 + leftcurrentvelocity * 0.25;
    //         //printf("%f \n", filteredVelocity);
    //         printf("%f \n", math::encoderTickToMeter(leftMotors.getPosition()));
    // }
    // leftMotors.moveVoltage(0);
    // rightMotors.moveVoltage(0);
    // for (size_t i = 0; i < 100; i++)
    // {
    //     double leftcurrentvelocity = rpmToLinear(leftMotors.getActualVelocity());
    //     filteredVelocity = filteredVelocity * 0.75 + leftcurrentvelocity * 0.25;
    //     //printf("%f \n", filteredVelocity);
    //     printf("%f \n", math::encoderTickToMeter(leftMotors.getPosition()));
    //     delay(10);
    // }
    drivePID(2_m);
    turnTo(90_deg);
    
}