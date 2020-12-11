#include <Romi32U4.h>
#include "Chassis.h"

//initial encoder counts old for velocity
int16_t oldEncoderRight = 0;
int16_t oldEncoderLeft = 0;
uint32_t lastTime = 0;
float velRight;
float velLeft;


void Chassis::setMotorEffort(int effortLeft, int effortRight){
    //directly sets the motor effort rather than using the speed PID
    motors.setEfforts(effortLeft, effortRight);
    if(millis() - lastTime >= 50){
        float encoderRightChange = encoders.getCountsRight() - oldEncoderRight;
        float encoderLeftChange = encoders.getCountsLeft() - oldEncoderLeft;

        velLeft = (encoderLeftChange * wheelCirc * 1000) / (CPR * 50);
        velRight = (encoderRightChange * wheelCirc * 1000) / (CPR * 50); 

        lastTime = millis();  
        oldEncoderLeft = encoders.getCountsLeft();
        oldEncoderRight = encoders.getCountsRight();


        Serial.print("Time: ");
        Serial.print(millis());
        Serial.print("ms            Right Velocity: ");
        Serial.print(velRight);
        Serial.print(" mm/s         Left Velocity: ");
        Serial.print(velLeft);
        Serial.println(" mm/s");


    }
}

float Chassis::getVelLeft(){
    return velLeft;
}

float Chassis::getVelRight(){
    return velRight;
}

//Modifiers
void Chassis::setSpeed(float speed){
    speedIPS = speed;
}

float Chassis::getSpeed(){
    return speedIPS;
}


//DriveBase Motion Functions
void Chassis::driveAtSpeed(){   //No speed passed, drive at current speedRPM
    //PID Constants
    float Kp = 0.3;
    float Ki = 0.1;
    float turnKp = 0;
    static float integralErrorLeft = 0;
    static float integralErrorRight = 0;

    //velocity calculation
    if(millis() - lastTime >= 50){
        float encoderRightChange = encoders.getCountsRight() - oldEncoderRight;
        float encoderLeftChange = encoders.getCountsLeft() - oldEncoderLeft;

        velLeft = (encoderLeftChange * wheelCirc * 20) / (1440);
        velRight = (encoderRightChange * wheelCirc * 20) / (1440);

        lastTime = millis();  
        oldEncoderLeft = encoders.getCountsLeft();
        oldEncoderRight = encoders.getCountsRight();

        //Error calculation
        float errorRight = speedIPS - velRight;
        float errorLeft = speedIPS - velLeft;
        integralErrorLeft += errorLeft;
        integralErrorRight += errorRight;

        float errorLR = velLeft - velRight;

        //effort calculation
        float effortRight = (errorRight * Kp) + (integralErrorRight * Ki) + (errorLR * turnKp);
        float effortLeft = (errorLeft * Kp) +(integralErrorLeft * Ki) - (errorLR * turnKp);

    
        //setting effort
        motors.setEfforts(effortLeft, effortRight);

        Serial.print("Time: ");
        Serial.print(millis());
        Serial.print("ms       ");

        Serial.print("Left Speed: ");
        Serial.print(velLeft);
        Serial.print("mm/s       ");

        Serial.print("Right Speed: ");
        Serial.print(velRight);
        Serial.print("mm/s       ");

        Serial.print("Left Error: ");
        Serial.print(errorLeft);
        Serial.print("mm/s       ");

        Serial.print("Right Error: ");
        Serial.print(errorRight);
        Serial.print("mm/s       ");

        Serial.print("Left Effort: ");
        Serial.print(effortLeft);
        Serial.print("       ");

        Serial.print("Right Effort: ");
        Serial.print(effortRight);
        Serial.print("       ");

        Serial.println("");
       
        
      
    }
    
}

void Chassis::driveAtSpeed(float newSpeed){    //specific speed passed, drive at new speedRPM
    setSpeed(newSpeed);
    driveAtSpeed();
}

void Chassis::halt(){
    motors.setEfforts(0,0);
}
