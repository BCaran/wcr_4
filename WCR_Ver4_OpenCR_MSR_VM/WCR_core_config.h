#ifndef WCR_CORE_CONFIG_H
#define WCR_CORE_CONFIG_H

#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <dxlMotorDriverSwerveVM.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#define CONTROL_MOTOR_SPEED_FREQUENCY 50 // hz
#define CONTROL_MOTOR_TIMEOUT 500        // ms
#define IMU_PUBLISH_FREQUENCY 100        // hz
#define ODOM_PUBLISH_FREQUENCY 50        // hz

#define LINEAR_X 0
#define LINEAR_Y 1
#define ANGULAR_Z 2
#define MOTOR_NUM 8

#define FL_WHEEL 0
#define FL_POSITION 1
#define FR_WHEEL 2
#define FR_POSITION 3
#define BL_WHEEL 4
#define BL_POSITION 5
#define BR_WHEEL 6
#define BR_POSITION 7

#define DEG2RAD(x) (x * 0.01745329252) // *PI/180
#define RAD2DEG(x) (x * 57.2957795131) // *180/PI

#define MAX_LINEAR_VELOCITY_MS 0.28195   // m/s
#define MAX_ANGULAR_VELOCITY_RADS 1.7722 // rad/s

#define TICK2RAD 0.001533981 // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f
// #define DEBUG
#define DEBUG_SERIAL SerialBT2

void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg);
void driveMotorsPowerCallback(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp);
void encoderOdomResetCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp);
void startVM_MSR(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp);
void publishDriveInformation(void);
void calculateEncoderOdometry(double diff_time);
void updateEncoderOdometry(void);
void calcAndUpdateJointStates(void);

void initEncoderOdom(void);
void initJointStates(void);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);
void updateVariable(bool isConnected);
void updateTFPrefix(bool isConnected);

/*******************************************************************************
 * ROS NodeHandle
 *******************************************************************************/
ros::NodeHandle nh;

/*******************************************************************************
 * ROS Parameter
 *******************************************************************************/
char encoder_odom_header_frame_id[30];
char encoder_odom_child_frame_id[30];
char joint_state_header_frame_id[30];

/*******************************************************************************
 * Subscriber
 *******************************************************************************/
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", commandVelocityCallback);

/*******************************************************************************
 * Publisher
 *******************************************************************************/
nav_msgs::Odometry encoder_odom;
ros::Publisher encoder_odom_pub("odom_encoder", &encoder_odom);
sensor_msgs::JointState joint_states;
ros::Publisher joint_states_pub("joint_states", &joint_states);

/*******************************************************************************
 * Service
 *******************************************************************************/
ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> motorPower_srv("drive_motors_power", &driveMotorsPowerCallback);
ros::ServiceServer<std_srvs::TriggerRequest, std_srvs::TriggerResponse> resetEncoderOdom_sub("reset_encoder_odom", encoderOdomResetCallback);
ros::ServiceServer<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse> startVM_msr_sub("start_vm_msr", startVM_MSR);

/*******************************************************************************
 * Transform Broadcaster
 *******************************************************************************/
// geometry_msgs::TransformStamped odom_tf;
// tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
 * SoftwareTimer
 *******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
 * Declaration for motor
 *******************************************************************************/
dxlMotorDriverSwerveVM motor_driver;
int64_t zero_velocity[4] = {0.0, 0.0, 0.0, 0.0};
float goal_velocity[3] = {0.0, 0.0, 0.0};
float goal_velocity_from_cmd[3] = {0.0, 0.0, 0.0};

/*******************************************************************************
 * Odometry, Joint State
 *******************************************************************************/
unsigned long prev_update_time;
double currentRobotSpeeds[3] = {0.0, 0.0, 0.0};                          // robot x_vel, y_vel, z_rot
int32_t currentWheelEncoderTicks[4] = {0, 0, 0, 0};                      // current encoder tick number from wheels
double currentPositionMotorsRad[4] = {0.0, 0.0, 0.0, 0.0};               // position of motors in rad
double currentMotorSpeeds[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // speed of every motor in rad/s
int32_t last_diff_tick[WHEELNUM] = {0, 0, 0, 0};
int32_t last_tick[WHEELNUM] = {0, 0, 0, 0};
double last_rad[WHEELNUM] = {0.0, 0.0, 0.0, 0.0};

float encoder_odom_pose[3] = {0.0, 0.0, 0.0};
float encoder_odom_vel[3] = {0.0, 0.0, 0.0};

/*******************************************************************************
 * Vladimir, Gijs Data Log
 *******************************************************************************/
bool startVMmsr = false;
float tempVMarr[8] = {0.0};
File logFile;
const int cs_pin = 8;
unsigned long passedTime = 0;
int32_t encoderTicks[MOTOR_NUM] = {0};
int32_t encoderTicks_start_offset[MOTOR_NUM] = {0};
void sendDataStreamToSD(int currStep);
Adafruit_BNO055 bno = Adafruit_BNO055(1, 0x28, &Wire);

#endif