#include "rrlib/util/feedforward.hpp"


namespace RRLib{
        double FeedForward::calculate(double vel, double accel, double velError){
            double pdOutput = velError * kP + (velError - lastVelError) * kD;
            lastVelError = velError;
            return (kV * vel + kA * accel + pdOutput);
            
        }

        double FeedForwardv2::calculate(double vel, double accel, double posError, double velError){
            double accelOut;
            if (accel < 0){
                accelOut = accel * kA_Decel;
            }
            else {
                accelOut = accel * kA;
            }
            return (kV * vel + accelOut + kP_Pos * posError + kP_Vel * velError);
        }
        double FeedForwardv2::calculateIfReversed(double vel, double accel,  double posError, double velError){
            double accelOut;
            if (accel > 0){
                accelOut = accel *  kA_Decel;
            }
            else {
                accelOut = accel * kA;
            }
            return (kV * vel + accelOut + kP_Pos * posError + kP_Vel * velError);
        }
        double FeedForwardv2::getKV(){
            return kV;
        }



        
        double FeedForwardv3::calculate(double vel, double accel, double posError, double velError){
            double accelOut;
            double kSOut;
            if (accel < 0){
                accelOut = accel * kA_Decel;
                kSOut = kS/2;
            }
            else {
                accelOut = accel * kA;
                kSOut = kS;
            }
            return (kV * vel + accelOut + kP_Pos * posError + kP_Vel * velError + kSOut);
        }
        double FeedForwardv3::calculateIfReversed(double vel, double accel,  double posError, double velError){
            double accelOut;
            double kSOut;
            if (accel > 0){
                accelOut = accel *  kA_Decel;
                kSOut = kS/4;
            }
            else {
                accelOut = accel * kA;
                kSOut = kS;
            }
            return (kV * vel + accelOut + kP_Pos * posError + kP_Vel * velError - kS);
        }
}
