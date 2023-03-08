#ifndef TEENSY_CONFIH_H
#define TEENSY_CONFIH_H
#include "Arduino.h"
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/Float32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>

#include <VescUart.h>
#include <PWMServo.h>
#include <HX711.h>

#include <math.h>

// Teensy 4.0 pins
#define ESC_pin1 3 // ESC1 pin
#define ESC_pin2 4 // ESC2 pin
#define ESC_pin3 5 // ESC3 pin
#define ESC_pin4 6 // ESC4 pin
#define DOUT_FRONT 9
#define CLOCK_FRONT 10
#define DOUT_BACK 12
#define CLOCK_BACK 11

// Calibration factors for force sensors
#define CALIB_FAC_FRONT 446780
#define CALIB_FAC_BACK 480880

#define MIN_DUTY_DRONE 0
#define MAX_DUTY_DRONE 180
#define MIN_DUTY_EDF 0
#define MAX_DUTY_EDF 1.0

#define BLDC_MOTORS_UPDATE_FREQUENCY 100 // HZ
#define CONTROL_MOTOR_TIMEOUT 1000       // ms
#define PUBLISH_SENSORS_INFO 50          // HZ
#define ZERO_DUTY 0
#define ZERO_THR_EDF 0.0

#define SLAVE_CURR_MSR_ADR 11

VescUart EDF;
PWMServo ESC1;
PWMServo ESC2;
PWMServo ESC3;
PWMServo ESC4;
HX711 forceSensor_front;
HX711 forceSensor_back;

const float g = 9.81; // m/s^2

void commandEDFCalllback(const std_msgs::Float32 &edf_throttle_in);
void commandThrustersCalllback(const std_msgs::Float32 &thrusters_throttle_in);
void tareForceSensorsCallback(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp);
void motorsEnableCallback(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp);

void publishSensorsInfo(void);
float vescCurrCorrection(float measured);

void initForceSensors();
void tareForceSensors();
void updateTFPrefix(bool isConnected);
void sendLogMsg(void);
void waitForSerialLink(bool isConnected);

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
 * Subscriber
 *******************************************************************************/
ros::Subscriber<std_msgs::Float32> dutyEDF_sub("throttle_EDF", commandEDFCalllback);
ros::Subscriber<std_msgs::Float32> dutyThrusters_sub("throttle_Thrusters", commandThrustersCalllback);

ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> enableMotors_srv("adhesion_motors_power", &motorsEnableCallback);
ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> forceTare_srv("tareForceSensor", &tareForceSensorsCallback);

/*******************************************************************************
 * Publisher
 *******************************************************************************/
std_msgs::Float32 EDF_force_pub;
ros::Publisher forceSensors_pub("edfFORCE", &EDF_force_pub);

std_msgs::Float32 powerEDF;
ros::Publisher powerEDF_pub("power_consumption_EDF", &powerEDF);
std_msgs::Float32 tempVESC;
ros::Publisher tempVESC_pub("temperature_VESC", &tempVESC);
/*******************************************************************************
 * SoftwareTimer
 *******************************************************************************/
static uint32_t tTime[10];

float edfDutyInput = 0;
int pwmThrustersInput = 0;
int32_t motorEnable = 0;

bool firstConnectFlag = false;

#endif