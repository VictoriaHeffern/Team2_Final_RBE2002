#include <Arduino.h>
#include "Behaviors.h"
#include <Romi32U4.h>
//sensors
#include "IR_sensor.h"
#include "Encoders.h"
//behaviors
#include "Speed_controller.h"
#include "Wall_following_controller.h"
//position
#include "Position_estimation.h"

Romi32U4ButtonA button_A; 
Romi32U4ButtonC button_C;
SpeedController PI_controller;
WallFollowingController PDcontroller;
IRsensor ir;
Position odometry_1;
Behaviors collisionBehavior;

float endX;
float endX_Thresh = 5;
float timeforfollow;

void setup() {
  collisionBehavior.Init();
  PI_controller.Init();
  PDcontroller.Init();
  Serial.begin(115200);
}

//define states 
enum ROBOT_STATE {ROBOT_IDLE, ROBOT_DRIVING, ROBOT_IDLE2, ROBOT_DRIVING2, ROBOT_WALLFOLLOW, ROBOT_WALLFOLLOW_15, ROBOT_WALLFOLLOW2, ROBOT_WALLFOLLOW_25, ROBOT_WALLFOLLOW_3, ROBOT_WALLFOLLOW_4, ROBOT_FINISHED};
ROBOT_STATE robot_state = ROBOT_IDLE;

void loop() {
  unsigned long startTime = 0;
  switch (robot_state) 
  {
    case ROBOT_IDLE:
      if(button_A.getSingleDebouncedRelease()) { //doesnt allow program to start unless button is pressed
        delay(1000); //waits one second
        collisionBehavior.ReadGyro(); //read gyro statements eliminate any initial incorrect gyro readings
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
        PI_controller.resetAcceleration(); //reset the acceleration before begin
        robot_state = ROBOT_DRIVING; 
      }
      break;

    case ROBOT_DRIVING: //begins first part to program
      endX = PI_controller.getX();
      PI_controller.AccelerateForward(); //accelerates at set rate
      Serial.println("Driving");
      if(collisionBehavior.DetectCollision()) //when romi hits wall
      {  
        Serial.println("Collision Detected");
        PI_controller.Reverse(50,10);  //reverses robot distance of 10
        robot_state=ROBOT_IDLE2;
      }
      break;

    case ROBOT_IDLE2: //waits for button second time
      if(button_A.getSingleDebouncedRelease()) {
        robot_state = ROBOT_DRIVING2;
        PI_controller.resetAcceleration(); //acceleration reset
      }
      break;
      
    case ROBOT_DRIVING2: //turns, continues
      Serial.println("Robot Turning");
      PI_controller.Turn(-90, 1);
      robot_state= ROBOT_WALLFOLLOW;
      break;

    case ROBOT_WALLFOLLOW: //robot begins to follow wall
       Serial.println("Robot Wall Following");
      PI_controller.Run(50-PDcontroller.Process(30),50+PDcontroller.Process(30));
      if (collisionBehavior.DetectPitchDown()) //when robot reaches flat top base of ramp
      {
        PI_controller.Stop();
        robot_state = ROBOT_WALLFOLLOW_15;
      }
      break;
      
    case ROBOT_WALLFOLLOW_15: //(read as 1.5)
      Serial.println("wait 1"); 
      startTime = millis(); //small wait implemented during top ramp wall following to eliminate any erroneous gyro readings
      while (millis() - startTime < 1500) {
        PI_controller.Run(50-PDcontroller.Process(30),50+PDcontroller.Process(30));
      }
        collisionBehavior.ReadGyro(); //read gyro statements eliminate any initial incorrect gyro readings
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
      robot_state = ROBOT_WALLFOLLOW2;
      break;

    case ROBOT_WALLFOLLOW2: //while following the wall on the ramp, detect when pitch down
       Serial.println("Robot Wall Following 2");
      PI_controller.Run(50-PDcontroller.Process(30),50+PDcontroller.Process(30));
      if (collisionBehavior.DetectPitchDown()) //ramp end begins (downward)
      {
        PI_controller.Stop();
        robot_state = ROBOT_WALLFOLLOW_25;
      }
      break;

    case ROBOT_WALLFOLLOW_25: //(read as 2.5)
      Serial.println("wait 2");
      startTime = millis();
      while (millis() - startTime < 1000) { //small wait implemented during top ramp wall following to eliminate any erroneous gyro readings
        PI_controller.Run(50-PDcontroller.Process(30),50+PDcontroller.Process(30));
      }
      robot_state = ROBOT_WALLFOLLOW_3;
        collisionBehavior.ReadGyro(); //read gyro statements eliminate any initial incorrect gyro readings
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
        collisionBehavior.ReadGyro();
      break;

    case ROBOT_WALLFOLLOW_3: //robot runs down the ramp, until pitch up detected
       Serial.println("Robot Wall Following 3");
        PI_controller.Run(50,50 );
          if (collisionBehavior.DetectPitchUp()) 
          {
            PI_controller.Stop();
            robot_state = ROBOT_WALLFOLLOW_4;
          }
          break;

    case ROBOT_WALLFOLLOW_4: //state that drives final 10cm
      Serial.println("wait 2");
      startTime = millis(); //begin time
      while (millis() - startTime < 2000) {  //wall follows for 10 cm 
      // since velocity controller is used for wall following, we can accuratley use time for 10c
      //I.E. this value is based off velocity calculation 
        PI_controller.Run(50-PDcontroller.Process(30),50+PDcontroller.Process(30));
      }
      PI_controller.Stop();
      robot_state = ROBOT_FINISHED;
      break;

    case ROBOT_FINISHED: //robot stops
      break;
  
  }//close switch

}//close loop