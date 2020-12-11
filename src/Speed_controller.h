#ifndef SPEED_CONTROLLER
#define SPEED_CONTROLLER

#include <Romi32U4.h>

class SpeedController{
    private:
        const float Kp_d = 7; //Adapt the parameters until your robot moves at the speed you command it to drive
        const float Kp_t = 3;
        const float Kp = .5;
        const float Ki = 0.2; 
        float E_left = 0; 
        float E_right = 0;
        float e_d;
        float error_distance = 1;
        float error_theta_rad;
        float error_theta;
        float speed_left;
        float prev_theta = 0;
        float speed_right;
        float e_t;
        int counts = 1440; //number of counts for a 180 degree turn; you will likely have to change this
        float maxAcceleration = 0.025;
        unsigned long lastAccelTime = 0;
        float lastVelocity = 0;
    public:
        struct constrained_acceleration {
            float constrained_velocity_left;
            float constrained_velocity_right;

        };
        int direction;
        void Init(void);
        float Constrain(float, float, float);
        void Run(float, float); 
        void Accl(float, float);
        boolean Turn(int,int); //degrees, direction of rotation: 0->left, 1->right
        boolean Straight(int, int); //speed, duration
        boolean Curved(int,int,int); //speed left, speed right, duration
        boolean MoveToPosition(float,float); //target_x, target_y
        boolean Reverse(int, int);
        void AccelerateForward();
        void resetAcceleration();
        int AccelerationCap(float velocity);
        void Stop(void);
        float getX();
};

#endif