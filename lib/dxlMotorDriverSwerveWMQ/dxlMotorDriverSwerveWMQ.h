#ifndef DXL_MOTOR_DRIVER_H
#define DXL_MOTOR_DRIVER_H

#include "variant.h"
#include <DynamixelSDK.h>
#include "swerveMath.h"

// Offset angles for rotation modules, calibrate
#define FL_ZERO_POS 2048
#define FR_ZERO_POS 2048
#define BL_ZERO_POS 2048
#define BR_ZERO_POS 2048

// Control table address (Dynamixel X-series)
#define ADDR_X_OPERATING_MODE 11
#define ADDR_X_TORQUE_ENABLE 64
#define ADDR_X_GOAL_VELOCITY 104
#define ADDR_X_GOAL_POSITION 116
#define ADDR_X_PRESENT_VELOCITY 128
#define ADDR_X_PRESENT_POSITION 132
#define ADDR_X_POSITION_P_GAIN 84

// Data Byte Length
#define LEN_X_OPERATING_MODE 1
#define LEN_X_TORQUE_ENABLE 1
#define LEN_X_GOAL_VELOCITY 4
#define LEN_X_GOAL_POSITION 4
#define LEN_X_REALTIME_TICK 2
#define LEN_X_PRESENT_VELOCITY 4
#define LEN_X_PRESENT_POSITION 4
#define LEN_X_POSITION_P_GAIN 2

// Limit values (XM430-W210-T and XM430-W350-T)
#define DXL_LIMIT_MAX_VELOCITY_INT 460 // 460 * 0.229 [rev/min]  = 105 RPM
#define RPM_FACTOR 0.229
#define DEG_PER_PULS 0.087890625 // 360/4096 DEG per PULS
#define PULS_PER_DEG 11.3788     // 360/4096 DEG per PULS
#define PULS_PER_ROT 4096

// motor IDs
#define DXL_ID_FL_WHEEL 69
#define DXL_ID_FL_POSITION 2
#define DXL_ID_FR_WHEEL 3
#define DXL_ID_FR_POSITION 4
#define DXL_ID_BL_WHEEL 5
#define DXL_ID_BL_POSITION 25
#define DXL_ID_BR_WHEEL 12
#define DXL_ID_BR_POSITION 8
// For indexing
#define FL_WHEEL 0
#define FL_POSITION 1
#define FR_WHEEL 2
#define FR_POSITION 3
#define BL_WHEEL 4
#define BL_POSITION 5
#define BR_WHEEL 6
#define BR_POSITION 7

#define BAUDRATE 1000000     // baurd rate of Dynamixel
#define PROTOCOL_VERSION 2.0 // Dynamixel protocol version 2.0

#define VELOCITY_MODE 1
#define POSITION_CONTROL_MODE 3

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque

#define WEAK_P_GAIN 100
#define STRONG_P_GAIN 3500

#define NAME ""
#define WHEELNUM 4
#define MOTORNUM 8
#define LENGTH 0.225                              // m
#define WIDTH 0.225                               // m
#define WHEEL_RAD 0.0254                          // m
#define WHEEL_POSITION_RADIUS 0.159               // m SQRT(WIDTH^2 + LENGTH^2)
#define VELOCITY_CONSTANT_VALUE_WHEEL 41.69988758 // V = r * w = r     *        (RPM             * 0.10472) \
                                                  //           = r     * (0.229 * Goal_Velocity) * 0.10472  \
                                                  //                                                        \
                                                  // Goal_Velocity = V / r * 41.69988757710309

#define MAX_LINEAR_VELOCITY_MS (WHEEL_RAD * 2 * 3.14159265359 * 106 / 60)          // m/s        0.28195 m/s
#define MAX_ANGULAR_VELOCITY_RADS (MAX_LINEAR_VELOCITY_MS / WHEEL_POSITION_RADIUS) // rad/s      1.7722 read/s

#define MIN_LINEAR_VELOCITY_MS -MAX_LINEAR_VELOCITY_MS
#define MIN_ANGULAR_VELOCITY_RADS -MAX_ANGULAR_VELOCITY_RADS

#define DEBUG_SERIAL SerialBT2

class dxlMotorDriverSwerveWMQ
{
public:
    dxlMotorDriverSwerveWMQ();
    ~dxlMotorDriverSwerveWMQ();
    bool init(void);
    bool setTorqueRotationMotors(bool onOff_wheels);
    bool setTorquePositionMotors(bool onOff_postion);
    bool setOperatingMode();
    bool setPgainPositionMotors(uint16_t Pgain);
    void setZeroPosition();

    bool readMotorsSpeed(int32_t motorsSpeedRead[]);
    bool readEncodersPosition(int32_t encodersPositionRead[]);

    bool writeWheelSpeed(int64_t write_Wheel_Speed[]);
    bool writeRotationPosition(int64_t write_Position_Motors[]);
    bool moveRobot(double x_vel, double y_vel, double z_rot);
    bool readRobotInfo(double returnRobotSpeeds[], int32_t returnWheelEncoderTicks[], double returnPositionMotorsDeg[], double returnMotorSpeeds[]);
    double mapf(double val, double in_min, double in_max, double out_min, double out_max);

private:
    uint32_t baudrate_;
    uint8_t FL_wheel_id_;
    uint8_t FL_position_id_;
    uint8_t FR_wheel_id_;
    uint8_t FR_position_id_;
    uint8_t BL_wheel_id_;
    uint8_t BL_position_id_;
    uint8_t BR_wheel_id_;
    uint8_t BR_position_id_;

    int32_t FL_zero_offset;
    int32_t FR_zero_offset;
    int32_t BL_zero_offset;
    int32_t BR_zero_offset;

    double wheel_radius;
    double puls_per_deg;
    double deg_per_puls;

    uint16_t dynamixel_limit_max_velocity_;
    double dynamixel_max_vel_ms;
    double dynamixel_min_vel_ms;
    double dynamixel_max_rot_vel_rads;
    double dynamixel_min_rot_vel_rads;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    dynamixel::GroupSyncRead *groupSyncReadMotorSpeed_;
    dynamixel::GroupSyncRead *groupSyncReadEncoderPosition_;

    dynamixel::GroupSyncWrite *groupSyncWriteWheelSpeed_;
    dynamixel::GroupSyncWrite *groupSyncWriteRotationPosition_;

    double length_, width_;
    SwerveMath *calcSwerve;
};

#endif