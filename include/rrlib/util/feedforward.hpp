#pragma once
#include "rrlib/util/structs.hpp"

namespace RRLib{
    class FeedForward{
        private:
        double kA, kV, kP, kD, lastVelError;


        public:
        FeedForward(double ikA, double ikV, double ikP, double ikD)
        : kA(ikA), kV(ikV), kP(ikP), kD(ikD){}

        double calculate(double vel, double accel, double velError);
    };

    class FeedForwardv2{
        private:
        double kA, kV, kP_Pos, kP_Vel, kA_Decel;
        public:
        FeedForwardv2(double ika, double ikaDecel, double ikv, double ikP_pos, double ikP_vel)
        : kA(ika), kV(ikv), kP_Pos(ikP_pos), kP_Vel(ikP_vel), kA_Decel(ikaDecel){}
        FeedForwardv2(FeedForwardGains gains)
        : kA(gains.kA), kV(gains.kV), kP_Pos(gains.kP_Pos), kP_Vel(gains.kP_Vel), kA_Decel(gains.kA_Down){}
        FeedForwardv2(){
            setGains(0, 0, 0, 0, 0);
        }

        double calculate(double vel, double accel, double posError, double velError);
        double calculateIfReversed(double vel, double accel,  double posError, double velError);
        double getKV();
        void setGains(FeedForwardGains gains){
            kV = gains.kV;
            kA = gains.kA;
            kP_Pos = gains.kP_Pos;
            kP_Vel = gains.kP_Vel;
            kA_Decel = gains.kA_Down;
        }
        void setGains(double ikV, double ikA, double ikP_Pos, double ikP_Vel, double ikADown){
            kV = ikV;
            kA = ikA;
            kP_Pos = ikP_Pos;
            kP_Vel = ikP_Vel;
            kA_Decel = ikADown;
        }
    };

    class FeedForwardv3{
        private:
        double kA, kV, kP_Pos, kP_Vel, kA_Decel, kS;
        public:
        FeedForwardv3(double ika, double ikaDecel, double ikv, double ikP_pos, double ikP_vel, double ikS)
        : kA(ika), kV(ikv), kP_Pos(ikP_pos), kP_Vel(ikP_vel), kA_Decel(ikaDecel), kS(ikS){}
        FeedForwardv3(FeedForwardGains gains, double ikS)
        : kA(gains.kA), kV(gains.kV), kP_Pos(gains.kP_Pos), kP_Vel(gains.kP_Vel), kA_Decel(gains.kA_Down), kS(ikS){}
        FeedForwardv3(){
            setGains(0, 0, 0, 0, 0, 0);
        }

        double calculate(double vel, double accel, double posError, double velError);
        double calculateIfReversed(double vel, double accel,  double posError, double velError);
        double getKV();
        void setGains(FeedForwardGains gains, double iks){
            kV = gains.kV;
            kA = gains.kA;
            kP_Pos = gains.kP_Pos;
            kP_Vel = gains.kP_Vel;
            kA_Decel = gains.kA_Down;
            kS = iks;
        }
        void setGains(FeedForwardGains2 gains){
            kV = gains.kV;
            kA = gains.kA;
            kP_Pos = gains.kP_Pos;
            kP_Vel = gains.kP_Vel;
            kA_Decel = gains.kA_Down;
            kS = gains.kS;
        }
        void setGains(double ikV, double ikA, double ikP_Pos, double ikP_Vel, double ikADown, double ikS){
            kV = ikV;
            kA = ikA;
            kP_Pos = ikP_Pos;
            kP_Vel = ikP_Vel;
            kA_Decel = ikADown;
            kS = ikS;
        }
    };
}
