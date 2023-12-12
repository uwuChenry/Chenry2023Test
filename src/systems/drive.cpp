#include "main.h"

namespace drive {
// Motors, measurements, encoders
MotorGroup leftMotors({-5, -7});
MotorGroup rightMotors({15, 16});

pros::Imu inertialSensor(1);

QLength movePidDistance = 0_cm;
bool PIDisSettled = true;

Controller controller;

ChassisScales trackingWheelsScales ({2.756_in, 17_cm}, 360);
ChassisScales drivenWheelsScales ({3.25_in, 23_cm}, (imev5BlueTPR*5/3));


ADIEncoder leftTrackingEncoder('A', 'B', true);
ADIEncoder rightTrackingEncoder('G', 'H', true);


// Gears
AbstractMotor::GearsetRatioPair gearing(AbstractMotor::gearset::blue, 5.0/3.0);

std::shared_ptr<OdomChassisController> driveChassis = ChassisControllerBuilder()
.withMotors(leftMotors, rightMotors)
.withSensors(leftTrackingEncoder, rightTrackingEncoder)
.withDimensions(AbstractMotor::gearset::blue, drivenWheelsScales)
.withOdometry(trackingWheelsScales)
.buildOdometry();

std::shared_ptr<ChassisModel> driveChassisPtr = driveChassis->getModel();

// Creates an odometry object
TwoWheelOdometry odometry(driveChassisPtr, trackingWheelsScales);
// Creates a pointer to the odometry object, some controllers want pointers
auto odometryPtr = std::shared_ptr<RRLib::PoseEstimator>(&odometry);

RRLib::AdaptivePurePursuitController ppController (driveChassis);


// this creats a task which constantly calculates the odometry
pros::Task odometryUpdateTask([] {
        while(1){
            odometry.update();
            pros::delay(10);
        }
    });

// calculate below:
// radius of a 4" omni is 2.0625"
// it's reccomend to use a value about 95% of the actual max velocity to allow room for feedback control to work.
// http://www.endmemo.com/physics/rpmlinear.php
KinematicConstraints constraints(
    1.45, // velocity in mps
    2.55, // accel in mps^2
    10 // jerk in mps^3
);

KinematicConstraints constraints2(
    1.45,
    2.55,
    6
);

//FeedForwardGains leftGains{6800, 1200, 0 ,0, 1200};
//FeedForwardGains rightGains{6800, 1200, 0, 0, 1200};



//FeedForwardGains leftGains2{6820, 1400, 192000, 0, 1000};
//FeedForwardGains rightGains2{6750, 1400, 192000, 0, 1000};

FeedForwardGains leftGains3{6980, 1650, 96000, 10000, 1000};
FeedForwardGains rightGains3{6950, 1650, 96000, 10000, 1000};


KinematicConstraints yes(1.2, 2.3, 10);

//FeedForwardGains leftScurveGain{7020, 1920, 0, 0, 850};
//FeedForwardGains rightScurveGain{7050, 1920, 0, 0, 900};

//FeedForwardGains leftScurveGain2{7020, 1600, 0, 0, 750};
//FeedForwardGains rightScurveGain2{7050, 1600, 0, 0, 750};

FeedForwardGains2 leftScurveGain3{6350, 1500, 96000, 0, 1000, 1000};
FeedForwardGains2 rightScurveGain3{6400, 1500, 96000, 0, 1000, 1000};

//betterLinearProfile linearProflie(yes, driveChassis, drivenWheelsScales, gearing, leftGains3, rightGains3);
betterLinearProfile linearProflie(constraints2, driveChassis, drivenWheelsScales, gearing, leftGains3, rightGains3);


scurveProfile linearScurveProfile(
    constraints2, 
    driveChassis, 
    drivenWheelsScales, 
    gearing, 
    leftScurveGain3, 
    rightScurveGain3, 
    leftGains3, 
    rightGains3);



void setMotors(double left, double right){
    leftMotors.moveVoltage(left);
    rightMotors.moveVoltage(right);
}


void turnTo(QAngle target){
    auto su = Settled(200, 1, 0.2);
    pidGains turnGains {2.2, 0.5, 20, 50, 15};
    PID turnPID(turnGains);
    double error, mappedError;
    do {
        error = target.convert(degree) - inertialSensor.get_heading();
        mappedError = math::constrainAngleDouble(error);
        double output = turnPID.calculate(mappedError)/200;
        pros::lcd::print(1, "error %f", error);
        pros::lcd::print(2, "output %f", output);
        pros::lcd::print(3, "integral %f", turnPID.getIntegral());
        driveChassisPtr->arcade(0, output);
        delay(10);
    } while (!su.isSettled(mappedError));
    driveChassisPtr->stop();
}

double rpmToLinear(double rpm){
    return 0.08255 * (3.0/5.0) / 2 * (rpm / 60 * 6.28318530718);
}

void drivePID(QLength distance){
    //error around 1to 1.5
    double filteredVelocity = 0;
    auto su = Settled(100, 0.04, 0.2);
    pidGains driveGains {350, 0, 0, 25, 0.2};
    PID drivePID(driveGains);
    double error, target;
    rightMotors.tarePosition();
    leftMotors.tarePosition();
    target = distance.convert(meter) + (math::encoderTickToMeter((rightMotors.getPosition() + leftMotors.getPosition())/2));
    do {
        error = target - (math::encoderTickToMeter((rightMotors.getPosition() + leftMotors.getPosition())/2));
        double output = drivePID.calculate(error)/200;
        pros::lcd::print(1, "error %f", error);
        pros::lcd::print(2, "output %f", output);
        pros::lcd::print(3, "integral %f", drivePID.getIntegral());
        driveChassisPtr->arcade(output, 0);
        //printf("%f \n", math::encoderTickToMeter((rightMotors.getPosition() + leftMotors.getPosition())/2));

        double leftcurrentvelocity = rpmToLinear(rightMotors.getActualVelocity());
        filteredVelocity = filteredVelocity * 0.75 + leftcurrentvelocity * 0.25;
        printf("%f \n", filteredVelocity);
        //printf("%f \n", math::encoderTickToMeter(leftMotors.getPosition()));
        delay(10);
    } while (!su.isSettled(error));
    //PIDisSettled = true;
    driveChassisPtr->stop();
}

// Task DriveTask{ [] {
//     while (Task::notify_take(true, TIMEOUT_MAX)) {
//         drivePID(movePidDistance);
//         movePidDistance = 0_m;
//     }
// } };

void waitUntilSettled(){
    while (!PIDisSettled)
    {
        delay(1);
    }
    
}

// void movePID(QLength distance){
//     movePidDistance = distance;
//     PIDisSettled = false;
//     DriveTask.notify();
// }


void resetMotorEncoder(){
    rightMotors.tarePosition();
    leftMotors.tarePosition();
}

void printEncoders(){
    pros::lcd::print(6, "left motor enc %f", leftTrackingEncoder.get());
    pros::lcd::print(7, "right motor enc %f", rightTrackingEncoder.get());
}


void printOdomPos(){
    printf("gyro angle %f", math::constrainAngleDouble(inertialSensor.get_heading()));
    printf(", x %f, y %f, a %f \n", odometry.getPose().position.getX().convert(centimeter), odometry.getPose().position.getY().convert(centimeter), odometry.getPose().heading.convert(degree));
}

void printChassisOdomPos(){
    printf("gyro angle %f", math::constrainAngleDouble(inertialSensor.get_heading()));
    printf(", x %f, y %f, a %f \n", 
        driveChassis->getState().x.convert(centimeter),
        driveChassis->getState().y.convert(centimeter), 
        driveChassis->getState().theta.convert(degree));
}


// void resetOdomEncoders(){
//     //driveChassisPtr->resetSensors();
//     odometry.reset();
// }


void resetGyro(){
    inertialSensor.reset();
}

void waitUntilGyroRest(){
    while (inertialSensor.is_calibrating())
    {
        delay(10);
    }
}

void setHoldMode(){
    leftMotors.setBrakeMode(AbstractMotor::brakeMode::brake);
    rightMotors.setBrakeMode(AbstractMotor::brakeMode::brake);
}

void setCoastMode(){
    leftMotors.setBrakeMode(AbstractMotor::brakeMode::coast);
    rightMotors.setBrakeMode(AbstractMotor::brakeMode::coast);
}

}
