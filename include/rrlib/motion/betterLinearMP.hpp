#pragma once
#include <string>
#include <vector>
#include <map>
#include "okapi/api.hpp"
#include "rrlib/util/feedforward.hpp"



namespace RRLib{
using namespace okapi;
class betterLinearProfile{
    private:
    double vMax;
    double aMax;
    bool isSettled;
    KinematicConstraints constraints;
    std::shared_ptr<okapi::ChassisController> chassisController;
    ChassisScales scales;
    std::shared_ptr<okapi::SkidSteerModel> chassis;
    double timePhase[3];
    FeedForwardv2 feedForwardLeft;
    FeedForwardv2 feedForwardRight;
    std::map<std::string, std::vector<trajectory>> generatedPaths;
    
    public:
    betterLinearProfile (
        KinematicConstraints iconstraints, 
        std::shared_ptr<okapi::ChassisController> ichassisController,  
        ChassisScales iscales, 
        okapi::AbstractMotor::GearsetRatioPair iratioPair,
        FeedForwardGains ileftffGains,
        FeedForwardGains irightffGains)
        : chassisController(ichassisController)
        , constraints(iconstraints)
        , scales(iscales)
        {
            chassis = std::static_pointer_cast<okapi::SkidSteerModel>(chassisController->getModel());
            vMax = iconstraints.maxVel;
            aMax = iconstraints.maxAccel;
            feedForwardLeft.setGains(ileftffGains);
            feedForwardRight.setGains(irightffGains);
        }
    
    void generateTime(double distance);

    double calculateVelocity(double time);
    std::pair<double, double> calculateVelocityAndAccleration(double time);
    double calculatePosition(double time);

    double linearToRotational(double linearSpeed);

    void generatePathLinear(std::string name, double distance);
    
    //rpm to meterPerSecond
    double rpmToLinear(double rpm);

    double encoderTickToMeter(double encoderTicks);

    void executePath(std::string name);
    void executeBackwardsPath(std::string name);
    void moveForwards(QLength idistance);

    void printGeneratedPath(std::string name);

    void printVector(const std::vector<trajectory>& v);
    void printGeneratedPathVelocity(std::string name);
    void printVectorVelocity(const std::vector<trajectory>& v);
    void printVectorPosition(const std::vector<trajectory>& p);
    void printGeneratedPathPosition(std::string name);
    void executePlannerPath(const std::vector<std::pair<trajectory, trajectory>>& path);
    void printPlannerPosition(const std::vector<std::pair<trajectory, trajectory>>& path);
    void moveBackwards(QLength distance);
    void reverseGeneratedPath(std::string name);
};
}