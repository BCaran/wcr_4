#include "dxlMotorDriverSwerve_DM.h"

#define MOTOR_INFO_PRINT_RATE 1000 //Hz
#define MOTOR_INFO_DRIVE_RATE 100 //Hz
#define MAX_MOTOR_SPEED 460
dxlMotorDriverSwerve motor_driver;

static uint32_t tTime[2];
char temp;
int32_t motorSpeed = 0;
int32_t motorLoad = 0;
void setup()
{
    Serial.begin(57600);

    // Setting for Dynamixel motors
    motor_driver.init();

}

void loop()
{
    uint32_t t = millis();
    if (Serial.available()) //ako je korisnik nešto unio putem Serial monitora onda se izvršava daljnji kod
    {
        temp = Serial.read();
    }
    if((t - tTime[0] >= (1000 / MOTOR_INFO_PRINT_RATE)))
    {
        motor_driver.readMotorSpeed(motorSpeed);
        motor_driver.readMotorLoad(motorLoad);
        Serial.print("W,");
        Serial.println(float(motorSpeed) * 0.229); //RPM
        Serial.print("T,");
        Serial.println(float(motorLoad) * 0.1); //%
    }
    if((t - tTime[0] >= (1000 / MOTOR_INFO_DRIVE_RATE)) && temp == 's')
    {
        motor_driver.writeMotorSpeed(MAX_MOTOR_SPEED);
    }
}