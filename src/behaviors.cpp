#include <Romi32U4.h>
#include "Behaviors.h"
#include "Median_filter.h"
#include "IMU.h"
#include "Speed_controller.h"
#include "Chassis.h"
Chassis drivebase;

//sensors
IMU_sensor LSM6;
Romi32U4ButtonB buttonB;

//median filter
MedianFilter med_x;
MedianFilter med_y;
MedianFilter med_z;
MedianFilter med_yaw;
MedianFilter med_pitch;
MedianFilter med_roll;

//motor-speed controller
SpeedController PIcontroller;

void Behaviors::Init(void)
{
    LSM6.Init();
    med_x.Init();
    med_y.Init();
    med_z.Init();
    PIcontroller.Init();
}

boolean Behaviors::DetectCollision(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[0]) > threshold) || (abs(data[1]) > threshold)) return 1;
    else return 0;
}

boolean Behaviors::DetectBeingPickedUp(void)
{
    auto data_acc = LSM6.ReadAcceleration();
    data[0] = med_x.Filter(data_acc.X)*0.061;
    data[1] = med_y.Filter(data_acc.Y)*0.061;
    data[2] = med_z.Filter(data_acc.Z)*0.061;
    if((abs(data[2]) > threshold_pick_up)) return true;
    
    else return false;
}

void Behaviors::setBias() {
    gyroBias = (gyroBias+ReadGyro())/2;
}

//detects if gyroscope has reached threshold for gyro up
boolean Behaviors::DetectPitchDown() {
    auto data_acc = LSM6.ReadGyroscope();
    float pitch = med_pitch.Filter(data_acc.Y);
    Serial.println(pitch > -1000);
    return (pitch > 32000);
}

//detects if gyroscope has reached threshold for gyro up
boolean Behaviors::DetectPitchUp() {
    auto data_acc = LSM6.ReadGyroscope();
    dataGyro[0] = med_yaw.Filter(data_acc.X);
    dataGyro[1] = med_pitch.Filter(data_acc.Y);
    dataGyro[2] = med_roll.Filter(data_acc.Z);
    if(dataGyro[1] - gyroBias < -29000) return true;
    
    else return false;
}

float Behaviors::ReadGyro() {
    auto data_acc = LSM6.ReadGyroscope();
    dataGyro[0] = med_yaw.Filter(data_acc.X);
    dataGyro[1] = med_pitch.Filter(data_acc.Y);
    dataGyro[2] = med_roll.Filter(data_acc.Z);
    return dataGyro[1];
}

void Behaviors::Stop(void)
{
    PIcontroller.Stop();
}

void Behaviors::Run(void)
{
    //DATA OUTPUT
    static int oldMillis = 0;
    if(millis() - oldMillis >= 100){
        //if 100 ms have passed
        oldMillis = millis();

        auto data_acc = LSM6.ReadAcceleration();
        data[0] = med_x.Filter(data_acc.X)*0.061;
        data[1] = med_y.Filter(data_acc.Y)*0.061;
        data[2] = med_z.Filter(data_acc.Z)*0.061;
    }
    }
   
        
    