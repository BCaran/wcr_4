#ifndef DXL_MOTOR_DRIVER_H
#define DXL_MOTOR_DRIVER_H

#include "variant.h"
#include <DynamixelSDK.h>

// Control table address (Dynamixel X-series)
#define ADDR_X_TORQUE_ENABLE 64
#define ADDR_X_GOAL_VELOCITY 104
#define ADDR_X_GOAL_POSITION 116
#define ADDR_X_REALTIME_TICK 120
#define ADDR_X_PRESENT_VELOCITY 128
#define ADDR_X_PRESENT_POSITION 132

// Limit values (XM430-W210-T and XM430-W350-T)
#define DXL_LIMIT_MAX_VELOCITY 460

// Data Byte Length
#define LEN_X_TORQUE_ENABLE 1
#define LEN_X_GOAL_VELOCITY 4
#define LEN_X_GOAL_POSITION 4
#define LEN_X_REALTIME_TICK 2
#define LEN_X_PRESENT_VELOCITY 4
#define LEN_X_PRESENT_POSITION 4

#define PROTOCOL_VERSION 2.0 // Dynamixel protocol version 2.0

#define DXL_LEFT_ID 1  // ID of left motor
#define DXL_RIGHT_ID 2 // ID of right motor

#define BAUDRATE 1000000 // baurd rate of Dynamixel
#define DEVICENAME ""    // no need setting on OpenCR

#define TORQUE_ENABLE 1  // Value for enabling the torque
#define TORQUE_DISABLE 0 // Value for disabling the torque

#define WHEELNUM 2
#define LEFT 0
#define RIGHT 1

#define VELOCITY_CONSTANT_VALUE 41.69988758 // V = r * w = r     *        (RPM             * 0.10472) \
                                            //           = r     * (0.229 * Goal_Velocity) * 0.10472  \
                                            //                                                        \
                                            // Goal_Velocity = V / r * 41.69988757710309

#define DEBUG_SERIAL SerialBT2

class dxlMotorDriver
{
public:
    dxlMotorDriver();
    ~dxlMotorDriver();
    bool init(void);
    void close(void);
    bool setTorque(bool onoff);
    bool getTorque();
    bool readEncoder(int32_t &left_value, int32_t &right_value);
    bool readSpeed(int32_t &left_value, int32_t &right_value);
    bool writeVelocity(int64_t left_value, int64_t right_value);
    bool controlMotor(const float wheel_radius, const float wheel_separation, float *value);

private:
    uint32_t baudrate_;
    float protocol_version_;
    uint8_t left_wheel_id_;
    uint8_t right_wheel_id_;
    bool torque_;

    uint16_t dynamixel_limit_max_velocity_;

    dynamixel::PortHandler *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    dynamixel::GroupSyncWrite *groupSyncWriteVelocity_;
    dynamixel::GroupSyncRead *groupSyncReadEncoder_;
    dynamixel::GroupSyncRead *groupSyncReadSpeed_;
};

#endif