#pragma once

#include "main.h"

namespace drive {
extern std::shared_ptr<ChassisController> driveChassis;
extern std::shared_ptr<ChassisModel> driveChassisPtr;
//extern ProfileController profileController;
extern scurveProfile linearScurveProfile;
//extern RRLib::betterLinearProfile linearProflie;
extern void turnTo(QAngle target);
extern void turnToWithMogo(QAngle target);
extern void turnToHighMogo(QAngle target);
extern Controller controller;
extern void printMotorEncoders();
extern void resetMotorEncoder();
extern void printOdomEncoders();
extern void resetOdomEncoders();
extern void printOdomPos();
extern void resetGyro();
extern void turnToOdom(QAngle target);

extern void setHoldMode();

extern void waitUntilGyroRest();


extern void waitUntilSettled();

extern void movePID(QLength distance);

extern void setMotors(double left, double right);
}