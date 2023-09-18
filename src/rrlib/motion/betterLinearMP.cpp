#include "rrlib/motion/betterLinearMP.hpp"

namespace RRLib
{
using namespace okapi;

    void betterLinearProfile::generateTime(double distance){
        double tA = vMax / aMax ; 
        double tC = ((distance - (tA * vMax)) / vMax);
        timePhase[0] = tA;
        timePhase[1] = tC + tA;
        timePhase[2] = tA + tA + tC;
        
        if (tC < 0){
            double thalf = sqrt(distance / aMax);
            timePhase [0] = thalf;
            timePhase [1] = thalf;
            timePhase [2] = thalf + thalf;
        }
    }

    double betterLinearProfile::calculateVelocity(double time){
        if (time < timePhase[0]){
            return (time * aMax) ;
        }
        else if (time < timePhase[1]){
            return (vMax) ;
        }
        else if (time < timePhase[2]){
            return (vMax - (aMax * (time - timePhase[1])));
        }
        else return 0;
    }

    std::pair<double, double> betterLinearProfile::calculateVelocityAndAccleration(double time){
        if (time < timePhase[0]){
            return std::make_pair((time * aMax), aMax);
        }
        else if (time < timePhase[1]){
            return std::make_pair(vMax, 0);
        }
        else if (time < timePhase[2]){
            return std::make_pair(((timePhase[0] * aMax) - (aMax * (time - timePhase[1]))), -aMax);
        }
        else return std::make_pair(0, 0);
    }

    double betterLinearProfile::calculatePosition(double time){
        if (time < timePhase[0]){
            return (time * (time * aMax) / 2);
        }
        else if (time < timePhase[1]){
            return ((vMax * timePhase[0] / 2) + ((time - timePhase [0]) * vMax));
        }
        else if (time < timePhase[2]){
            return (((timePhase[0] * aMax) * timePhase[0] / 2) + ((timePhase[1]-timePhase[0]) * vMax) + (((timePhase[0] * aMax)
             * (time-timePhase[1])) - (aMax * (time-timePhase[1]) * (time-timePhase[1]) / 2)));
        }
        else return 0;
    }

    double betterLinearProfile::linearToRotational(double linearSpeed){
        return (linearSpeed * (360/ (3.25 * 3.141592653589793)) * 3/5);
    }

    void betterLinearProfile::generatePathLinear(std::string name, double distance){
        generateTime(distance);
        int stepAmount = timePhase[2] * 100 + 1;
        std::vector<trajectory> pathTrajectory;
        for (size_t i = 0; i < stepAmount; i++)
        {
            double currentTime = i * 10;
            double currentTimeInS = currentTime / 1000;
            auto velandaccel = calculateVelocityAndAccleration(currentTimeInS);
            trajectory trajectory{velandaccel.first, velandaccel.second, calculatePosition(currentTimeInS), currentTimeInS};
            pathTrajectory.push_back(trajectory);
        }
        generatedPaths.emplace(name, pathTrajectory);
    }
    
    //rpm to meterPerSecond
    double betterLinearProfile::rpmToLinear(double rpm){
        return 0.08255 * (3.0/5.0) / 2 * (rpm / 60 * 6.28318530718);
    }

    double betterLinearProfile::encoderTickToMeter(double encoderTicks){
        return encoderTicks/300 * (3.0 / 5.0) * 0.08255 * 3.141592653589793116;
    }

    void betterLinearProfile::printPlannerPosition(const std::vector<std::pair<trajectory, trajectory>>& path){
        for (const auto& yes : path){
            printf("%f \n", yes.second.position);
        }
    }

    void betterLinearProfile::reverseGeneratedPath(std::string name){
        std::vector<trajectory>& reversing = generatedPaths[name];
        for (size_t i = 0; i < reversing.size(); i++)
        {
            reversing[i].velocity *= -1;
            reversing[i].acceleration *= -1;
            reversing[i].position *= -1;
        }
        generatedPaths[name] = reversing;
    }
    void betterLinearProfile::moveBackwards(QLength idistance){
        double distance = idistance.convert(meter);
        generateTime(distance);
        int stepAmount = timePhase[2] * 100 + 1;
        std::vector<trajectory> pathTrajectory;
        for (size_t i = 0; i < stepAmount; i++)
        {
            double currentTime = i * 10;
            double currentTimeInS = currentTime / 1000;
            auto velandaccel = calculateVelocityAndAccleration(currentTimeInS);
            trajectory trajectory{velandaccel.first, velandaccel.second, calculatePosition(currentTimeInS), currentTimeInS};
            pathTrajectory.push_back(trajectory);
        }
        for (size_t i = 0; i < pathTrajectory.size(); i++)
        {
            pathTrajectory[i].velocity *= -1;
            pathTrajectory[i].acceleration *= -1;
            pathTrajectory[i].position *= -1;
        }
        auto leftSideMotor = chassis->getLeftSideMotor();
        auto rightSideMotor = chassis->getRightSideMotor();
        double startingLeftMotorPos = leftSideMotor->getPosition();
        double startingRightMotorPos = rightSideMotor->getPosition();
        double filteredVelocity = 0;
       
        for (const auto& thing : pathTrajectory){
            
            double leftCurrentPosition = leftSideMotor->getPosition() - startingLeftMotorPos;
            double rightCurrentPosition = rightSideMotor->getPosition() - startingRightMotorPos;
            
            double leftCurrentPositionMeter = encoderTickToMeter(leftCurrentPosition);
            double rightCurrentPositionMeter = encoderTickToMeter(rightCurrentPosition);    
            double leftPosError = thing.position - encoderTickToMeter(leftCurrentPosition);
            double rightPosError = thing.position - encoderTickToMeter(rightCurrentPosition);
            
            double leftcurrentvelocity = rpmToLinear(leftSideMotor->getActualVelocity());
            double rightCurrentVelocity = rpmToLinear(rightSideMotor->getActualVelocity());
            filteredVelocity = filteredVelocity * 0.75 + rightCurrentVelocity * 0.25;
            
            double leftVelError = thing.velocity - rpmToLinear(leftSideMotor->getActualVelocity()); //rpm convert to linear 
            double rightVelError = thing.velocity - rpmToLinear(rightSideMotor->getActualVelocity()); //rpm needs to be converted to linear
            
            
            double leftVoltage = feedForwardLeft.calculateIfReversed(thing.velocity, thing.acceleration, leftPosError, leftVelError);
            double rightVoltage = feedForwardRight.calculateIfReversed(thing.velocity, thing.acceleration, rightPosError, rightVelError);
            leftSideMotor->moveVoltage(leftVoltage);
            rightSideMotor->moveVoltage(rightVoltage);
            
            printf("%f \n", rightCurrentPositionMeter);
            //printf("running, left %f, right %f, leftkv %f\n", leftVoltage, rightVoltage, feedForwardLeft.getKV());
            pros::delay(10);
        }
        leftSideMotor->moveVoltage(0);
        rightSideMotor->moveVoltage(0);


    }

    void betterLinearProfile::moveForwards(QLength idistance){
        double distance = idistance.convert(meter);
        generateTime(distance);
        int stepAmount = timePhase[2] * 100 + 1;
        std::vector<trajectory> pathTrajectory;
        for (size_t i = 0; i < stepAmount; i++)
        {
            double currentTime = i * 10;
            double currentTimeInS = currentTime / 1000;
            auto velandaccel = calculateVelocityAndAccleration(currentTimeInS);
            trajectory trajectory{velandaccel.first, velandaccel.second, calculatePosition(currentTimeInS), currentTimeInS};
            pathTrajectory.push_back(trajectory);
        }
        auto leftSideMotor = chassis->getLeftSideMotor();
        auto rightSideMotor = chassis->getRightSideMotor();
        double startingLeftMotorPos = leftSideMotor->getPosition();
        double startingRightMotorPos = rightSideMotor->getPosition();
        double filteredVelocity = 0;
       
        for (const auto& thing : pathTrajectory){
            
            double leftCurrentPosition = leftSideMotor->getPosition() - startingLeftMotorPos;
            double rightCurrentPosition = rightSideMotor->getPosition() - startingRightMotorPos;
            
            double leftCurrentPositionMeter = encoderTickToMeter(leftCurrentPosition);
            double rightCurrentPositionMeter = encoderTickToMeter(rightCurrentPosition);    
            double leftPosError = thing.position - encoderTickToMeter(leftCurrentPosition);
            double rightPosError = thing.position - encoderTickToMeter(rightCurrentPosition);
            
            double leftcurrentvelocity = rpmToLinear(leftSideMotor->getActualVelocity());
            double rightCurrentVelocity = rpmToLinear(rightSideMotor->getActualVelocity());
            filteredVelocity = filteredVelocity * 0.75 + rightCurrentVelocity * 0.25;
            
            double leftVelError = thing.velocity - rpmToLinear(leftSideMotor->getActualVelocity()); //rpm convert to linear 
            double rightVelError = thing.velocity - rpmToLinear(rightSideMotor->getActualVelocity()); //rpm needs to be converted to linear
            
            
            double leftVoltage = feedForwardLeft.calculate(thing.velocity, thing.acceleration, leftPosError, leftVelError);
            double rightVoltage = feedForwardRight.calculate(thing.velocity, thing.acceleration, rightPosError, rightVelError);
            leftSideMotor->moveVoltage(leftVoltage);
            rightSideMotor->moveVoltage(rightVoltage);
            
            printf("%f \n", leftCurrentPositionMeter);
            //printf("%f \n", leftcurrentvelocity);
            //printf("%f \n", rightVoltage);
            //printf("running, left %f, right %f, leftkv %f\n", leftVoltage, rightVoltage, feedForwardLeft.getKV());
            pros::delay(10);
        }
        leftSideMotor->moveVoltage(0);
        rightSideMotor->moveVoltage(0);

        printf("\n \n \nprinting position thing \n \n \n");
        printVectorPosition(pathTrajectory);
    }
    

    void betterLinearProfile::executePlannerPath(const std::vector<std::pair<trajectory, trajectory>>& path){
        auto leftSideMotor = chassis->getLeftSideMotor();
        auto rightSideMotor = chassis->getRightSideMotor();
        double startingLeftMotorPos = leftSideMotor->getPosition();
        double startingRightMotorPos = rightSideMotor->getPosition();
        double filteredVelocity = 0;
        for (const auto& thing : path){
            
            double leftCurrentPosition = leftSideMotor->getPosition() - startingLeftMotorPos;
            double rightCurrentPosition = rightSideMotor->getPosition() - startingRightMotorPos;
            
            double leftCurrentPositionMeter = encoderTickToMeter(leftCurrentPosition);
            double rightCurrentPositionMeter = encoderTickToMeter(rightCurrentPosition);

            double leftPosError = thing.first.position - encoderTickToMeter(leftCurrentPosition);
            double rightPosError = thing.second.position - encoderTickToMeter(rightCurrentPosition);
            
            double leftcurrentvelocity = rpmToLinear(leftSideMotor->getActualVelocity());
            double rightCurrentVelocity = rpmToLinear(rightSideMotor->getActualVelocity());
            filteredVelocity = filteredVelocity * 0.75 + rightCurrentVelocity * 0.25;

            double leftVelError = thing.first.velocity - rpmToLinear(leftSideMotor->getActualVelocity()); //rpm convert to linear 
            double rightVelError = thing.second.velocity - rpmToLinear(rightSideMotor->getActualVelocity()); //rpm needs to be converted to linear
            
            
            double leftVoltage = feedForwardLeft.calculate(thing.first.velocity, thing.first.acceleration, leftPosError, leftVelError);
            double rightVoltage = feedForwardRight.calculate(thing.second.velocity, thing.second.acceleration, rightPosError, rightVelError);
            leftSideMotor->moveVoltage(leftVoltage);
            rightSideMotor->moveVoltage(rightVoltage);
            
            printf("%f \n", rightCurrentPositionMeter);
            //printf("running, left %f, right %f, leftkv %f\n", leftVoltage, rightVoltage, feedForwardLeft.getKV());
            pros::delay(10);
        }
        leftSideMotor->moveVoltage(0);
        rightSideMotor->moveVoltage(0);
    }

    void betterLinearProfile::executePath(std::string name){
        auto leftSideMotor = chassis->getLeftSideMotor();
        auto rightSideMotor = chassis->getRightSideMotor();
        const std::vector<trajectory>& yes = generatedPaths[name];
        double startingLeftMotorPos = leftSideMotor->getPosition();
        double startingRightMotorPos = rightSideMotor->getPosition();
        double filteredVelocity = 0;
       
        for (const auto& thing : yes){
            
            double leftCurrentPosition = leftSideMotor->getPosition() - startingLeftMotorPos;
            double rightCurrentPosition = rightSideMotor->getPosition() - startingRightMotorPos;
            
            double leftCurrentPositionMeter = encoderTickToMeter(leftCurrentPosition);
            double rightCurrentPositionMeter = encoderTickToMeter(rightCurrentPosition);    
            double leftPosError = thing.position - encoderTickToMeter(leftCurrentPosition);
            double rightPosError = thing.position - encoderTickToMeter(rightCurrentPosition);
            
            double leftcurrentvelocity = rpmToLinear(leftSideMotor->getActualVelocity());
            double rightCurrentVelocity = rpmToLinear(rightSideMotor->getActualVelocity());
            filteredVelocity = filteredVelocity * 0.75 + rightCurrentVelocity * 0.25;
            
            double leftVelError = thing.velocity - rpmToLinear(leftSideMotor->getActualVelocity()); //rpm convert to linear 
            double rightVelError = thing.velocity - rpmToLinear(rightSideMotor->getActualVelocity()); //rpm needs to be converted to linear
            
            
            double leftVoltage = feedForwardLeft.calculate(thing.velocity, thing.acceleration, leftPosError, leftVelError);
            double rightVoltage = feedForwardRight.calculate(thing.velocity, thing.acceleration, rightPosError, rightVelError);
            leftSideMotor->moveVoltage(leftVoltage);
            rightSideMotor->moveVoltage(rightVoltage);
            
            printf("%f \n", rightCurrentPositionMeter);
            //printf("running, left %f, right %f, leftkv %f\n", leftVoltage, rightVoltage, feedForwardLeft.getKV());
            pros::delay(10);
        }
        leftSideMotor->moveVoltage(0);
        rightSideMotor->moveVoltage(0);
    }

    void betterLinearProfile::executeBackwardsPath(std::string name){
        auto leftSideMotor = chassis->getLeftSideMotor();
        auto rightSideMotor = chassis->getRightSideMotor();
        const std::vector<trajectory>& yes = generatedPaths[name];
        double startingLeftMotorPos = leftSideMotor->getPosition();
        double startingRightMotorPos = rightSideMotor->getPosition();
        double filteredVelocity = 0;
       
        for (const auto& thing : yes){
            
            double leftCurrentPosition = leftSideMotor->getPosition() - startingLeftMotorPos;
            double rightCurrentPosition = rightSideMotor->getPosition() - startingRightMotorPos;
            
            double leftCurrentPositionMeter = encoderTickToMeter(leftCurrentPosition);
            double rightCurrentPositionMeter = encoderTickToMeter(rightCurrentPosition);    
            double leftPosError = thing.position - encoderTickToMeter(leftCurrentPosition);
            double rightPosError = thing.position - encoderTickToMeter(rightCurrentPosition);
            
            double leftcurrentvelocity = rpmToLinear(leftSideMotor->getActualVelocity());
            double rightCurrentVelocity = rpmToLinear(rightSideMotor->getActualVelocity());
            filteredVelocity = filteredVelocity * 0.75 + rightCurrentVelocity * 0.25;
            
            double leftVelError = thing.velocity - rpmToLinear(leftSideMotor->getActualVelocity()); //rpm convert to linear 
            double rightVelError = thing.velocity - rpmToLinear(rightSideMotor->getActualVelocity()); //rpm needs to be converted to linear
            
            
            double leftVoltage = feedForwardLeft.calculateIfReversed(thing.velocity, thing.acceleration, leftPosError, leftVelError);
            double rightVoltage = feedForwardRight.calculateIfReversed(thing.velocity, thing.acceleration, rightPosError, rightVelError);
            leftSideMotor->moveVoltage(leftVoltage);
            rightSideMotor->moveVoltage(rightVoltage);
            
            printf("%f \n", rightCurrentPositionMeter);
            //printf("running, left %f, right %f, leftkv %f\n", leftVoltage, rightVoltage, feedForwardLeft.getKV());
            pros::delay(10);
        }
        leftSideMotor->moveVoltage(0);
        rightSideMotor->moveVoltage(0);
    }



    void betterLinearProfile::printGeneratedPath(std::string name){
        std::vector<trajectory>& yes = generatedPaths[name];
        printVector(yes);
    }

    void betterLinearProfile::printGeneratedPathVelocity(std::string name){
        std::vector<trajectory>& yes = generatedPaths[name];
        printVectorVelocity(yes);
    }

    void betterLinearProfile::printGeneratedPathPosition(std::string name){
        std::vector<trajectory>& yes = generatedPaths[name];
        printVectorPosition(yes);
    }
    void betterLinearProfile::printVector(const std::vector<trajectory>& v){
        for (const auto& yes : v){
            printf("%f %f %f %f \n", yes.velocity, yes.acceleration, yes.position, yes.time);
    
        }
    }
    void betterLinearProfile::printVectorVelocity(const std::vector<trajectory>& v){
        for (const auto& yes : v){
            printf("%f \n", yes.velocity);
        }
    }
    void betterLinearProfile::printVectorPosition(const std::vector<trajectory>& p){
        for (const auto& yes : p){
            printf("%f \n", yes.position);
        }
    }
}
