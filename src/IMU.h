#ifndef IMU
#define IMU

#include <Romi32U4.h>

class IMU_sensor{
    private:
        int data[3] = {0};
        char report[120];
        
    public:
        struct acceleration_data {
            int X;
            int Y;
            int Z;
        };
        struct gyro_data {
            float X;
            float Y;
            float Z;
        };
        void Init(void);
        void PrintAcceleration(void);
        acceleration_data ReadAcceleration(void);
        gyro_data ReadGyroscope(void);
};

#endif