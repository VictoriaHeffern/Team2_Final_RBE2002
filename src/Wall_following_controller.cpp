#include <Romi32U4.h>
#include "Encoders.h"
#include "Wall_following_controller.h"
#include "IR_sensor.h"

IRsensor SharpIR;
Romi32U4Motors motors2; 


void WallFollowingController::Init(void)
{
    SharpIR.Init();
}

float WallFollowingController::Process(float target_distance)
{
 //outputs speed as a function of distance error
  error = target_distance - (SharpIR.ReadData());
  float speed_forwall = constrain((Kp * error) + (Kd *  (error-previousError)), -50, 50);
  float previousError = error; 

return speed_forwall;
}