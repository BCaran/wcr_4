#include "dxlMotorDriverSwerve.h"

dxlMotorDriverSwerve motor_driver;
int64_t zero_velocity[4] = {55,55, 55, 55};
double returnRobotSpeeds[3] = {0.0, 0.0, 0.0};
int32_t returnWheelEncoderTicks[4] = {0.0, 0.0, 0.0, 0.0};
double returnPositionMotorsRad[4] = {0.0, 0.0, 0.0, 0.0};
double returnMotorSpeeds[8] ={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
void setup()
{
    Serial.begin(57600);

    // Setting for Dynamixel motors
    motor_driver.init();

}

void loop()
{
    motor_driver.moveRobot(0.0,0.1,0);
    motor_driver.readRobotInfo(returnRobotSpeeds, returnWheelEncoderTicks, returnPositionMotorsRad, returnMotorSpeeds);
    for(int i = 0; i < 3; i++)
    {
        Serial.println(returnRobotSpeeds[i],5);
    }
    Serial.println("................");
}