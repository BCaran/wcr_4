#ifndef DXL_MOTOR_DRIVER_DM_H
#define DXL_MOTOR_DRIVER_DM_H

#include "variant.h"
#include <DynamixelSDK.h>


// Control table address (Dynamixel X-series)
#define ADDR_X_OPERATING_MODE 11
#define ADDR_X_TORQUE_ENABLE 64
#define ADDR_X_GOAL_VELOCITY 104
#define ADDR_X_GOAL_POSITION 116
#define ADDR_X_PRESENT_LOAD 126
#define ADDR_X_PRESENT_VELOCITY 128
#define ADDR_X_PRESENT_POSITION 132
#define ADDR_X_POSITION_D_GAIN 80
#define ADDR_X_POSITION_I_GAIN 82
#define ADDR_X_POSITION_P_GAIN 84
#define ADDR_X_VELOCITY_I_GAIN 76
#define ADDR_X_VELOCITY_P_GAIN 78
#define ADDR_X_FEEDFORWARD_2ND_GAIN 88
#define ADDR_X_FEEDFORWARD_1ST_GAIN 90

// Data Byte Length
#define LEN_X_OPERATING_MODE 1
#define LEN_X_TORQUE_ENABLE 1
#define LEN_X_GOAL_VELOCITY 4
#define LEN_X_GOAL_POSITION 4
#define LEN_X_REALTIME_TICK 2
#define LEN_X_PRESENT_LOAD 2
#define LEN_X_PRESENT_VELOCITY 4
#define LEN_X_PRESENT_POSITION 4
#define LEN_X_POSITION_D_GAIN 2
#define LEN_X_POSITION_I_GAIN 2
#define LEN_X_POSITION_P_GAIN 2
#define LEN_X_VELOCITY_I_GAIN 2
#define LEN_X_VELOCITY_P_GAIN 2
#define LEN_X_FEEDFORWARD_2ND_GAIN 2
#define LEN_X_FEEDFORWARD_1ST_GAIN 2

// Limit values (XM430-W210-T and XM430-W350-T)
#define DXL_LIMIT_MAX_VELOCITY_INT 460 //460 * 0.229 [rev/min]  = 105 RPM
#define RPM_FACTOR 0.229
#define DEG_PER_PULS 0.087890625 // 360/4096 DEG per PULS
#define PULS_PER_DEG 11.3788 // 360/4096 DEG per PULS
#define PULS_PER_ROT 4096

//motor IDs
#define DXL_ID_MOTOR 7


#define BAUDRATE 1000000        // baurd rate of Dynamixel
#define PROTOCOL_VERSION 2.0 // Dynamixel protocol version 2.0

#define VELOCITY_MODE 1

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque


#define NAME ""
#define WHEELNUM 4
#define MOTORNUM 8
#define LENGTH 0.225 //m
#define WIDTH 0.225 //m
#define WHEEL_RAD 0.0254 //m
#define WHEEL_POSITION_RADIUS 0.159 //m SQRT(WIDTH^2 + LENGTH^2)
#define VELOCITY_CONSTANT_VALUE_WHEEL 41.69988758 // V = r * w = r     *        (RPM             * 0.10472) \
                                                  //           = r     * (0.229 * Goal_Velocity) * 0.10472  \
                                                  //                                                        \
                                                  // Goal_Velocity = V / r * 41.69988757710309

#define MAX_LINEAR_VELOCITY_MS (WHEEL_RAD * 2 * 3.14159265359 * 106 / 60) // m/s        0.28195 m/s
#define MAX_ANGULAR_VELOCITY_RADS (MAX_LINEAR_VELOCITY_MS / WHEEL_POSITION_RADIUS)       // rad/s      1.7722 read/s

#define MIN_LINEAR_VELOCITY_MS -MAX_LINEAR_VELOCITY_MS
#define MIN_ANGULAR_VELOCITY_RADS -MAX_ANGULAR_VELOCITY_RADS

#define DEBUG_SERIAL SerialBT2

class dxlMotorDriverSwerve
{
public:
    dxlMotorDriverSwerve();
    ~dxlMotorDriverSwerve();
    bool init(void);
    bool setTorque(bool onOff);
    bool setOperatingMode();
    bool setGainsToZero();

    bool readMotorSpeed(int32_t &readMotorSpeed);
    bool readMotorLoad(int32_t &readMotorLoad);

    bool writeMotorSpeed(uint32_t motorSpeed);

private:
    uint32_t baudrate_;
    uint8_t MOTOR_ID;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    dynamixel::GroupSyncRead *groupSyncReadMotorSpeed_;
    dynamixel::GroupSyncRead *groupSyncReadMotorLoad_;

    dynamixel::GroupSyncWrite *groupSyncWriteWheelSpeed_;

};

#endif