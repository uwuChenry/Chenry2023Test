#pragma once

#include "main.h"

namespace drive {
extern std::shared_ptr<OdomChassisController> driveChassis;
extern std::shared_ptr<ChassisModel> driveChassisPtr;
//extern ProfileController profileController;
extern scurveProfile linearScurveProfile;
extern RRLib::betterLinearProfile linearProflie;
extern RRLib::AdaptivePurePursuitController ppController;
extern void turnTo(QAngle target);

extern MotorGroup leftMotors;
extern MotorGroup rightMotors;

extern Controller controller;
extern void printEncoders();
extern void resetMotorEncoder();
extern void printOdomEncoders();
extern void printChassisOdomPos();
extern void resetOdomEncoders();
extern void printOdomPos();
extern void resetGyro();
extern void turnToOdom(QAngle target);


extern void waitUntilGyroRest();
extern void drivePID(QLength distance);

extern void waitUntilSettled();


extern void setMotors(double left, double right);
}