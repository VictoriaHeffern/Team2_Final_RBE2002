#pragma once
#include <Romi32U4.h>


class Chassis{
    public:
        //DriveBase Constants
        const float wheelDiameter = 70; //mm
        const int CPR = 1440;
        const float wheelCirc = wheelDiameter*PI;
        const int maxMotorEffort = 400;

        //Drivebase Variables
        float speedIPS = 0;

        //-----------------------------------------

        //DriveBase Control
        void driveAtSpeed();
        void driveAtSpeed(float speed);
        void driveDistance(float inches);
        void turnAngle(float angle);
        void halt();
        void setMotorEffort(int effortLeft, int effortRight);
        float getVelRight();
        float getVelLeft();

        //Variable Modifiers
        void setSpeed(float speed);
        float getSpeed();
        



    private:
        Romi32U4Motors motors;
        Romi32U4Encoders encoders;


};