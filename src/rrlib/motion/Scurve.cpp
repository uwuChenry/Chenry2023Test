#include "rrlib/motion/Scurve.hpp"


namespace RRLib
{
using namespace okapi;
using namespace okapi::literals;

double scurveProfile::rpmToLinear(double rpm){
        return 0.08255 * (3.0/5.0) / 2 * (rpm / 60 * 6.28318530718);
    }

double scurveProfile::encoderTickToMeter(double encoderTicks){
        return encoderTicks/300 * (3.0 / 5.0) * 0.08255 * 3.141592653589793116;
    }



trajectory scurveProfile::calculateTrajectory(double time)
{
    if (time < timePhase[0])
    {
        double pos1 = jerk * time * time * time / 6;
        return {(jerk * time * time / 2), (jerk * time), pos1, time};
    }
    else if (time < timePhase[1])
    {
        double dt2 = time - timePhase[0];
        double pos2 = fPosPhase[0] + fVelPhase[0] * dt2 + aPeak * dt2 * dt2 / 2;
        return {(fVelPhase[0] + (aPeak * dt2)), (aPeak), pos2, time};
    }
    else if (time < timePhase[2])
    {
        double dt3 = time - timePhase[1];
        double pos3 = fPosPhase[1] + fVelPhase[1] * dt3 + aPeak * dt3 * dt3 / 2 - jerk * dt3 * dt3 * dt3 / 6;
        return {(fVelPhase[1] + (aPeak * dt3) - (jerk * dt3 * dt3 / 2)), (aPeak - (jerk * dt3)), pos3, time};
    }
    else if (time < timePhase[3])
    {
        double dt4 = time - timePhase[2];
        double pos4 = fPosPhase[2] + fVelPhase[2] * dt4;
        return {(fVelPhase[2]), 0, pos4, time};
    }
    else if (time < timePhase[4])
    {
        double dt5 = time - timePhase[3];
        double pos5 = fPosPhase[3] + fVelPhase[3] * dt5 - jerk * dt5 * dt5 * dt5 / 6;
        return {(fVelPhase[3] - (jerk * dt5 * dt5 / 2)), (-jerk * dt5), pos5, time};
    }
    else if (time < timePhase[5])
    {
        double dt6 = time - timePhase[4];
        double pos6 = fPosPhase[4] + fVelPhase[4] * dt6 - aPeak * dt6 * dt6 / 2;
        return {(fVelPhase[4] - (aPeak * dt6)), (-aPeak), pos6, time};
    }
    else if (time < timePhase[6])
    {
        double dt7 = time - timePhase[5];
        double pos7 = fPosPhase[5] + fVelPhase[5] * dt7 - aPeak * dt7 * dt7 / 2 + jerk * dt7 * dt7 * dt7 / 6;
        return {(fVelPhase[5] + (-aPeak * dt7) + (jerk * dt7 * dt7 / 2)), (-aPeak + jerk * dt7), pos7, time};
    }
    else
    {
        return {0, 0, 0, 0};
    }
}

void scurveProfile::generateProfile(){
    double distance = currentDistanceSet.distance.convert(meter);
    bool isReversed = currentDistanceSet.isReversed;
    double taa = aMax / jerk;

    double velInStageThreeStart = vMax - (jerk * taa * taa / 2);
    double velInStageOneEnd = jerk * taa * taa / 2;
    double ta = (velInStageThreeStart - velInStageOneEnd) / aMax;
    double totalAccelTime = taa + taa + ta;
    double distanceInA = totalAccelTime * vMax;
    double tc = (distance - distanceInA) / vMax;
    bool isP4;

    double x = cbrt(((4 * distance * distance) / jerk) / distance);
    if (x < (aMax * 2 / jerk) && x >= 0) isP4 = true;
    
    if (tc > 0){ //7 phase
        std::cout << "7 phase" << std::endl;
        timePhase[0] = taa;
        timePhase[1] = taa + ta;
        timePhase[2] = totalAccelTime;
        timePhase[3] = tc + totalAccelTime;
        timePhase[4] = timePhase[3] + taa;
        timePhase[5] = timePhase[3] + taa + ta;
        timePhase[6] = timePhase[3] + totalAccelTime;
    }   
    if (tc <= 0 && isP4 == false){ //6 phase
        double dist = (distance / 2) - (jerk * taa * taa * taa);
        double a = (jerk * taa / 2);
        double b = ((jerk * taa * taa / 2) + (taa * taa * jerk));
        double c = -dist;
        double root1 = (-b + sqrt((b * b) - (4 * a * c))) / (2 * a);
        double root2 = (-b - sqrt((b * b) - (4 * a * c))) / (2 * a);
        double ta6;
        if (root1 > 0)
        {
            ta6 = root1;
        }
        else if (root2 > 0)
        {
            ta6 = root2;
        }

        std::cout << "6 phase" << std::endl;
        timePhase[0] = taa;
        timePhase[1] = ta6 + taa;
        timePhase[2] = taa + taa + ta6;
        timePhase[3] = timePhase[2];
        timePhase[4] = timePhase[2] + taa;
        timePhase[5] = timePhase[2] + ta6 + taa;
        timePhase[6] = timePhase[2] + taa + taa + ta6;
    }
    if (tc <= 0 && isP4 == true){ //4 phase
        double taa4 = cbrt((distance / 2) / jerk);
        std::cout << "4 Phase" << std::endl;
        timePhase[0] = taa4;
        timePhase[1] = taa4;
        timePhase[2] = 2 * taa4;
        timePhase[3] = 2 * taa4;
        timePhase[4] = 3 * taa4;
        timePhase[5] = 3 * taa4;
        timePhase[6] = 4 * taa4;
    }

    //final time for each segments
    fTimePhase[0] = timePhase[0];
    fTimePhase[1] = timePhase[1] - timePhase[0];
    fTimePhase[2] = timePhase[2] - timePhase[1];
    fTimePhase[3] = timePhase[3] - timePhase[2];
    fTimePhase[4] = timePhase[4] - timePhase[3];
    fTimePhase[5] = timePhase[5] - timePhase[4];
    fTimePhase[6] = timePhase[6] - timePhase[5];

    //peak accel
    aPeak = jerk * timePhase[0];

    //final velocity
    fVelPhase[0] = (jerk * fTimePhase[0] * fTimePhase[0] / 2);
    fVelPhase[1] = (fVelPhase[0] + (jerk * fTimePhase[0] * fTimePhase[1]));
    fVelPhase[2] = (fVelPhase[1] + (aPeak * fTimePhase[2]) - (jerk * fTimePhase[2] * fTimePhase[2] / 2));
    fVelPhase[3] = fVelPhase[2];
    fVelPhase[4] = fVelPhase[3] - (jerk * fTimePhase[4] * fTimePhase[4] / 2);
    fVelPhase[5] = fVelPhase[4] - (jerk * fTimePhase[4] * fTimePhase[5]);
    fVelPhase[6] = fVelPhase[5] - (jerk * fTimePhase[6]) + (jerk * fTimePhase[6] * fTimePhase[6] / 2);

    //final position for each time period
    fPosPhase[0] = jerk * fTimePhase[0] * fTimePhase[0] * fTimePhase[0] / 6;
    fPosPhase[1] = fPosPhase[0] + fVelPhase[0] * fTimePhase[1] + aPeak * fTimePhase[1] * fTimePhase[1] / 2;
    fPosPhase[2] = fPosPhase[1] + fVelPhase[1] * fTimePhase[2] + aPeak * fTimePhase[2] * fTimePhase[2] / 2 - jerk * fTimePhase[2] * fTimePhase[2] * fTimePhase[2] / 6;
    fPosPhase[3] = fPosPhase[2] + fVelPhase[2] * fTimePhase[3];
    fPosPhase[4] = fPosPhase[3] + fVelPhase[3] * fTimePhase[4] - jerk * fTimePhase[4] * fTimePhase[4] * fTimePhase[4] / 6;
    fPosPhase[5] = fPosPhase[4] + fVelPhase[4] * fTimePhase[5] - aPeak * fTimePhase[5] * fTimePhase[5] / 2;
    fPosPhase[6] = fPosPhase[5] + fVelPhase[5] * fTimePhase[6] - aPeak * fTimePhase[6] * fTimePhase[6] / 2 + jerk * fTimePhase[6] * fTimePhase[6] * fTimePhase[6] / 6;


    int stepAmount = timePhase[6] * 100 - 5;
    for (size_t i = 0; i < stepAmount; i++)
    {
        double currentTime = i * 10;
        double currentTimeInS = currentTime / 1000;
        auto traj = calculateTrajectory(currentTimeInS);
        if (isReversed == true)
        {
            traj.velocity *= -1;
            traj.acceleration *= -1;
            traj.position *= -1;
        }
        pathTrajectory.push_back(traj);
    }
}



void scurveProfile::trampoline(void* instance) {
    static_cast<scurveProfile*>(instance)->taskLoop();
}

void scurveProfile::taskLoop() {
    while (task->notify_take(true, TIMEOUT_MAX)) {

        pathTrajectory.clear();
        if(isPP.load()){
            executePlannerPath();
        }
        else {
            generateProfile();

            while (deferringMove.load()) {
                pros::delay(1);
            }

            executePath();

        }
        isExecuting = false;   

    }
}



void scurveProfile::deferNext(){
    deferringMove = true;
}

void scurveProfile::go(){
    deferringMove = false;
}

void scurveProfile::waitUntilSettled() {
    while (isExecuting.load()) {
        pros::delay(10);
    }
}


void scurveProfile::setTarget(QLength idistance, bool iisReversed) {
    if (!isExecuting.load()) {
        isPP = false;
        currentDistanceSet.distance = idistance;
        currentDistanceSet.isReversed = iisReversed;
        isExecuting = true;
        task->notify();
    }
}



void scurveProfile::moveForwards(QLength idistance){
    setTarget(idistance, false);
    waitUntilSettled();
}

void scurveProfile::moveBackwards(QLength idistance){
    setTarget(idistance, true);
    waitUntilSettled();
}

void scurveProfile::moveForwardsAsync(QLength idistance){
    setTarget(idistance, false);
}

void scurveProfile::moveBackwardsAsync(QLength idistance){
    setTarget(idistance, true);
}

void scurveProfile::movePlannerPath(std::vector<std::pair<trajectory, trajectory>>& path, bool ippIsReversed){
    if (!isExecuting.load()){
        isPP = true;
        isExecuting = true;
        ppIsReversed = ippIsReversed;
        ppPath = &path;
        task->notify();
    }
}

void scurveProfile::executePlannerPath(){
        auto leftSideMotor = chassis->getLeftSideMotor();
        auto rightSideMotor = chassis->getRightSideMotor();
        double startingLeftMotorPos = leftSideMotor->getPosition();
        double startingRightMotorPos = rightSideMotor->getPosition();
        double leftFilteredVelocity = 0;
        double rightFilteredVelocity = 0;
        double rightVoltage, leftVoltage;

        for (const auto& thing : *ppPath){
            
            double leftCurrentPositionMeter = encoderTickToMeter(leftSideMotor->getPosition() - startingLeftMotorPos);
            double rightCurrentPositionMeter = encoderTickToMeter(rightSideMotor->getPosition() - startingRightMotorPos);

            double leftPosError = thing.first.position - leftCurrentPositionMeter;
            double rightPosError = thing.second.position - rightCurrentPositionMeter;
            
            double leftCurrentVelocity = rpmToLinear(leftSideMotor->getActualVelocity());
            double rightCurrentVelocity = rpmToLinear(rightSideMotor->getActualVelocity());
            double leftFilteredVelocity = leftFilteredVelocity * 0.75 + leftCurrentVelocity * 0.25;
            double rightFilteredVelocity = rightFilteredVelocity * 0.75 + rightCurrentVelocity * 0.25;


            double leftVelError = thing.first.velocity - leftCurrentVelocity; //rpm convert to linear 
            double rightVelError = thing.second.velocity - rightCurrentVelocity; //rpm needs to be converted to linear
            if (!ppIsReversed.load()){
                leftVoltage = ppfeedForwardLeft.calculate(thing.first.velocity, thing.first.acceleration, leftPosError, leftVelError);
                rightVoltage = ppfeedForwardRight.calculate(thing.second.velocity, thing.second.acceleration, rightPosError, rightVelError);
            }
            else{
                leftVoltage = ppfeedForwardLeft.calculateIfReversed(thing.first.velocity, thing.first.acceleration, leftPosError, leftVelError);
                rightVoltage = ppfeedForwardRight.calculateIfReversed(thing.second.velocity, thing.second.acceleration, rightPosError, rightVelError);
            }
            
            leftSideMotor->moveVoltage(leftVoltage);
            rightSideMotor->moveVoltage(rightVoltage);
            
            printf("%f \n", rightFilteredVelocity);
            //printf("running, left %f, right %f, leftkv %f\n", leftVoltage, rightVoltage, feedForwardLeft.getKV());
            pros::delay(10);
        }
        leftSideMotor->moveVoltage(0);
        rightSideMotor->moveVoltage(0);
    }

void scurveProfile::executePath(){
    bool isReversed = currentDistanceSet.isReversed;
    auto leftSideMotor = chassis->getLeftSideMotor();
    auto rightSideMotor = chassis->getRightSideMotor();
    double startingLeftMotorPos = leftSideMotor->getPosition();
    double startingRightMotorPos = rightSideMotor->getPosition();
    double filteredVelocity = 0;
    double leftVoltage, rightVoltage;
    
        for (const auto& thing : pathTrajectory){
            double leftPosError = thing.position - encoderTickToMeter(leftSideMotor->getPosition() - startingLeftMotorPos);
            double rightPosError = thing.position - encoderTickToMeter(rightSideMotor->getPosition() - startingRightMotorPos);
            
            double leftcurrentvelocity = rpmToLinear(leftSideMotor->getActualVelocity());
            double rightCurrentVelocity = rpmToLinear(rightSideMotor->getActualVelocity());
            filteredVelocity = filteredVelocity * 0.75 + leftcurrentvelocity * 0.25;
            
            double leftVelError = thing.velocity - leftcurrentvelocity; //rpm convert to linear 
            double rightVelError = thing.velocity - rightCurrentVelocity; //rpm needs to be converted to linear
            
            if (!isReversed){
            leftVoltage = feedForwardLeft.calculate(thing.velocity, thing.acceleration, leftPosError, leftVelError);
            rightVoltage = feedForwardRight.calculate(thing.velocity, thing.acceleration, rightPosError, rightVelError);
            }
            if (isReversed){
            leftVoltage = feedForwardLeft.calculateIfReversed(thing.velocity, thing.acceleration, leftPosError, leftVelError);
            rightVoltage = feedForwardRight.calculateIfReversed(thing.velocity, thing.acceleration, rightPosError, rightVelError);
            }
            //leftSideMotor->moveVoltage(leftVoltage);
            //rightSideMotor->moveVoltage(rightVoltage);
            
            //printf("%f \n", filteredVelocity);
            //printf("%f \n", leftVoltage);
            //printf("%f \n", encoderTickToMeter(leftSideMotor->getPosition() - startingLeftMotorPos));
            //printf("running, left %f, right %f, leftkv %f\n", leftVoltage, rightVoltage, feedForwardLeft.getKV());
            pros::delay(10);
        }
    leftSideMotor->moveVoltage(0);
    rightSideMotor->moveVoltage(0);
}



void scurveProfile::printPPVelocity(std::vector<std::pair<trajectory, trajectory>>& path){
        for (const auto& yes : path){
            printf("%f \n", yes.second.velocity);
        }
    }

void scurveProfile::printVectorVelocity(const std::vector<trajectory>& v){
        for (const auto& yes : v){
            printf("%f \n", yes.velocity);
        }
    }

void scurveProfile::printVectorPosition(const std::vector<trajectory>& p){
        for (const auto& yes : p){
            printf("%f \n", yes.position);
        }
    }



}