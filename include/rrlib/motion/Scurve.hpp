#pragma once
#include <string>
#include <vector>
#include <map>
#include "okapi/api.hpp"
#include "rrlib/util/feedforward.hpp"
#include <optional>


namespace RRLib{
using namespace okapi;

class scurveProfile{
    private:

    KinematicConstraints constraints;
    std::shared_ptr<okapi::ChassisController> chassisController;
    ChassisScales scales;
    std::shared_ptr<okapi::SkidSteerModel> chassis;
    FeedForwardv3 feedForwardLeft;
    FeedForwardv3 feedForwardRight;

    FeedForwardv2 ppfeedForwardLeft;
    FeedForwardv2 ppfeedForwardRight;


        // Internal things to kep track of
    pros::Task* task = nullptr;
    std::atomic_bool isExecuting = false;
    std::atomic_bool deferringMove = false;
    std::atomic_bool isPP = false;
    std::atomic_bool ppIsReversed = false;
        // Task trampoline and task code
    static void trampoline(void* instance);
    void taskLoop();
    
    void executePath();
    void executePlannerPath();
    void setTarget(QLength idistance, bool iisReversed);
    
    distanceSet currentDistanceSet;
    std::vector<std::pair<trajectory, trajectory>> *ppPath = nullptr; 

    double vMax = 1.5; //1.5
    double aMax = 2.5; //2.3
    double jerk = 15;  //5
    double timePhase[7];


    double fTimePhase[7];
    double fVelPhase[7];
    double fPosPhase[7];

    //peak accel
    double aPeak;

    void generateTime(double idistance);
    trajectory calculateTrajectory(double time);
    void calculateTrajectoryLinear2(double distance, bool isReversed);
    
    void generateProfile();
 
    //rpm to meterPerSecond
    double rpmToLinear(double rpm);
    double encoderTickToMeter(double encoderTicks);


    public:
    //constraints, chassis controller, scales, gear ratio, scurve ff left, scurve ff right, pp ff left, pp ff right
    scurveProfile (
        KinematicConstraints iconstraints, 
        std::shared_ptr<okapi::ChassisController> ichassisController,  
        ChassisScales iscales, 
        okapi::AbstractMotor::GearsetRatioPair iratioPair,
        FeedForwardGains2 ileftffGains,
        FeedForwardGains2 irightffGains,
        FeedForwardGains ippleftffGains,
        FeedForwardGains ipprightffGains)
        : chassisController(ichassisController)
        , constraints(iconstraints)
        , scales(iscales)
        , vMax(iconstraints.maxVel)
        , aMax(iconstraints.maxAccel)
        , jerk(iconstraints.maxJerk)
        {
            chassis = std::static_pointer_cast<okapi::SkidSteerModel>(chassisController->getModel());
            feedForwardLeft.setGains(ileftffGains);
            feedForwardRight.setGains(irightffGains);
            ppfeedForwardLeft.setGains(ippleftffGains);
            ppfeedForwardRight.setGains(ipprightffGains);
            task = new pros::Task(trampoline, this);
        }

    

    std::vector<trajectory> pathTrajectory;

    void deferNext();
    void go();

    void waitUntilSettled();

    void moveForwards(QLength idistance);
    void moveBackwards(QLength idistance);
    void moveForwardsAsync(QLength idistance);
    void moveBackwardsAsync(QLength idistance);
    void movePlannerPath(std::vector<std::pair<trajectory, trajectory>>& path, bool ippIsReversed);
    


    void printPPVelocity(std::vector<std::pair<trajectory, trajectory>>& path);
    void printVectorPosition(const std::vector<trajectory>& p);
    void printVectorVelocity(const std::vector<trajectory>& v);
};
}