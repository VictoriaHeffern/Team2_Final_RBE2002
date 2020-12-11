#include <Romi32U4.h>
#include "IR_sensor.h"

void IRsensor::Init(void)
{
    pinMode(pin_IR, INPUT);
}

float IRsensor::PrintData(void)
{
    Serial.println(ReadData());
    return 0;
}

float IRsensor::ReadData(void)
{
  adc = analogRead(pin_IR);
  voltage = (5.0/1024.0) *adc;
  float distance = 16.8271/(voltage-.43772);
  return distance;

}