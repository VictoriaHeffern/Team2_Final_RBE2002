#include <Romi32U4.h>
#include "Encoders.h"
#include  "Speed_controller.h"
#include "Position_estimation.h"

Romi32U4Motors motors;
Encoder MagneticEncoder; 
Position odometry;

float time_track = 0;

void SpeedController::Init(void)
{
    MagneticEncoder.Init();
    odometry.Init();
}
void SpeedController::Run(float target_velocity_left, float target_velocity_right)
{
    if(MagneticEncoder.UpdateEncoderCounts()){
        time_track = time_track + 50/1000.0;
        float e_left = target_velocity_left - MagneticEncoder.ReadVelocityLeft();
        float e_right = target_velocity_right - MagneticEncoder.ReadVelocityRight();
        
        E_left += e_left;
        E_right += e_right;

        float u_left = Kp*e_left + Ki*E_left;
        float u_right = Kp*e_right + Ki*E_right;

        motors.setEfforts(u_left,u_right);
        odometry.UpdatePose(target_velocity_left,target_velocity_right);
    }
}

float SpeedController::getX(){return odometry.getX();}


boolean SpeedController::MoveToPosition(float target_x, float target_y){ 
//===========================================================================================
do{ 
    //----Forward Calculations----
    float robot_x = odometry.ReadPose().X; 
    float robot_y = odometry.ReadPose().Y; 
    float robot_theta = odometry.ReadPose().THETA;
    prev_theta = (robot_theta/(2*3.14159))*360;
    //----Error Calculations----
    error_distance = sqrt(pow((target_x-robot_x),2)+pow((target_y-robot_y),2));
    float target_theta = atan2((target_y-robot_y),(target_x-robot_x));
    float theta_to_turn = target_theta - robot_theta;
    error_theta = (theta_to_turn/(2*3.14159))*360;
    //----Speed Controller----
    float dist_part = (Kp_d*error_distance*100);
    float theta_part = (Kp_t*error_theta);
    int cons = 100;
    float constrained_dist_part = constrain(dist_part, -1* cons, cons);
    float constrained_theta_part = constrain(theta_part, -1*cons, cons);
    speed_left = (constrained_dist_part)-(constrained_theta_part);
    speed_right = (constrained_dist_part)+(constrained_theta_part);
        //Condition for turn (work in progress)
        /*
    if(target_theta-prev_theta > 180){
        //For right turn
        speed_left = (constrained_dist_part)+(constrained_theta_part);
        speed_right = (constrained_dist_part)-(constrained_theta_part);
    }else{
        //For left turn
        speed_left = (constrained_dist_part)-(constrained_theta_part);
        speed_right = (constrained_dist_part)+(constrained_theta_part);
    }
    */
    //----Add Constraints----
    speed_right = Constrain(speed_right, -1*cons, cons);
    speed_left = Constrain(speed_left, -1*cons, cons);
    //----Drive----
    Run(speed_left, speed_right);
    //----Exit Condition----
    }while(error_distance*100 >= 0.2);
    error_distance = 0;
    return 1;
}
//======================================================================================
float SpeedController::Constrain(float value, float min, float max)
{
    if(value < min){
        return min;
    }
    else if(value > max){
        return max;
    }
    return value;
}
boolean SpeedController::Turn(int degree, int direction)
{
    // motors.setEfforts(0, 0);
    // int turns = counts*(degree/180.0);
    // int count_turn = MagneticEncoder.ReadEncoderCountLeft();

    // while(abs(abs(count_turn) - abs(MagneticEncoder.ReadEncoderCountLeft())) <= turns)
    // {
    //     if(!direction) Run(50,-50);
    //     else Run(-50,50);
    // }
    // motors.setEfforts(0, 0);
    // return 1;
    motors.setEfforts(0,0);
    float robotAngle = 0;
    float initialCount = MagneticEncoder.ReadEncoderCountLeft();
    float tkP = 8;
    float tkI = 0;
    float tkD = 5;
    float sigma = 0;
    float delta = 0;
    float lastError = 0;
    while (abs(degree - robotAngle) > 1) {
        float elapsedCount = MagneticEncoder.ReadEncoderCountLeft() - initialCount;
        robotAngle = ((elapsedCount/1440)*70*M_PI) / (5.5*25.4*M_PI) * 360;

        float error = degree - robotAngle;
        sigma += error;
        delta = error - lastError;
        lastError = error;

        float power = tkP * error + tkI * sigma + tkD * delta;
        power = AccelerationCap(power);
        Run(power, -power);
    }
    Stop();
}

int SpeedController::AccelerationCap(float velocity) {
    float desiredAcceleration = (velocity - lastVelocity)/(millis() - lastAccelTime);
    if (abs(desiredAcceleration) < maxAcceleration || (velocity > 0 && desiredAcceleration < 0) || (velocity < 0 && desiredAcceleration > 0)) {
        //if the acceleration wont break shit
        lastVelocity = velocity;
        lastAccelTime = millis();
        return velocity;
    }
    else {
        //if desiredAcceleration is too high, do this
        float newVelocity = ((desiredAcceleration > 0)? 1 : -1) * maxAcceleration * (millis()-lastAccelTime) + lastVelocity;
        return newVelocity;
    }
}

void SpeedController::resetAcceleration() {
    lastVelocity = 0;
    lastAccelTime = millis() - 10;
}
boolean SpeedController::Straight(int target_velocity, int time)
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity,target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::AccelerateForward() {
    float power = 80;
    power = AccelerationCap(power);
    Run(power,power);
}
boolean SpeedController::Curved(int target_velocity_left, int target_velocity_right, int time) //in mm/s and s
{
    motors.setEfforts(0, 0);
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= time*1000){
        Run(target_velocity_left,target_velocity_right);
    }
    motors.setEfforts(0, 0);
    return 1;
}

boolean SpeedController::Reverse(int target_velocity, int distance) //in mm/s and cm
{
    motors.setEfforts(0, 0);
    
    uint32_t duration = 1000*((distance*10)/(float)target_velocity); //in ms
    unsigned long now = millis();

    while ((unsigned long)(millis() - now) <= duration){
        Run(-target_velocity,-target_velocity);
    }
    motors.setEfforts(0, 0);
    return 1;
}

void SpeedController::Stop()
{
    motors.setEfforts(0,0);
    odometry.Stop();
    time_track = 0;
}

void SpeedController::Accl(float speed, float time){
    static float initialtime = millis();
    while((millis()) < (initialtime + time)){
        int relative_time = millis() - initialtime;
        float step = speed/time;
        float run_speed = step * relative_time;
        Serial.println(run_speed);
        Run(run_speed,run_speed);
    }
}