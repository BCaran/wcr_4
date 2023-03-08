#include "dxlMotorDriverSwerveWMQ.h"

dxlMotorDriverSwerveWMQ::dxlMotorDriverSwerveWMQ()
{
    baudrate_ = BAUDRATE;
    FL_wheel_id_ = DXL_ID_FL_WHEEL;
    FL_position_id_ = DXL_ID_FL_POSITION;
    FR_wheel_id_ = DXL_ID_FR_WHEEL;
    FR_position_id_ = DXL_ID_FR_POSITION;
    BL_wheel_id_ = DXL_ID_BL_WHEEL;
    BL_position_id_ = DXL_ID_BL_POSITION;
    BR_wheel_id_ = DXL_ID_BR_WHEEL;
    BR_position_id_ = DXL_ID_BR_POSITION;
    FL_zero_offset = FL_ZERO_POS;
    FR_zero_offset = FR_ZERO_POS;
    BL_zero_offset = BL_ZERO_POS;
    BR_zero_offset = BR_ZERO_POS;
    wheel_radius = WHEEL_RAD;
    puls_per_deg = PULS_PER_DEG;
    deg_per_puls = DEG_PER_PULS;
    dynamixel_limit_max_velocity_ = DXL_LIMIT_MAX_VELOCITY_INT;
    dynamixel_max_vel_ms = MAX_LINEAR_VELOCITY_MS;
    dynamixel_min_vel_ms = MIN_LINEAR_VELOCITY_MS;
    dynamixel_max_rot_vel_rads = MAX_ANGULAR_VELOCITY_RADS;
    dynamixel_min_rot_vel_rads = MIN_ANGULAR_VELOCITY_RADS;
    length_ = LENGTH;
    width_ = WIDTH;
    // Serial.begin(57600);
}

dxlMotorDriverSwerveWMQ::~dxlMotorDriverSwerveWMQ()
{
    setTorquePositionMotors(false);
    setTorqueRotationMotors(false);
    portHandler_->closePort();
    // Serial.end();
}

bool dxlMotorDriverSwerveWMQ::init(void)
{
    // Serial.begin(57600);

    portHandler_ = dynamixel::PortHandler::getPortHandler(NAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler_->openPort() == false)
    {
        // Serial.println("Failed to open port(Motor Driver)");
        return false;
    }

    // Set port baudrate
    if (portHandler_->setBaudRate(baudrate_) == false)
    {
        // Serial.println("Failed to set baud rate(Motor Driver)");
        return false;
    }

    groupSyncReadMotorSpeed_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    groupSyncReadEncoderPosition_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    groupSyncWriteWheelSpeed_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
    groupSyncWriteRotationPosition_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_POSITION, LEN_X_GOAL_POSITION);
    calcSwerve = new SwerveMath(length_, width_);
    // set operating mode for motors, postion and velocity
    setOperatingMode();
    setPgainPositionMotors(STRONG_P_GAIN);
    // Enable Dynamixel Torque
    setTorqueRotationMotors(true);
    setTorquePositionMotors(true);
    setZeroPosition();

    // Serial.println("Success to init Motor Driver");
    return true;
}
bool dxlMotorDriverSwerveWMQ::setOperatingMode()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // FRONT LEFT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FL_wheel_id_, ADDR_X_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FL_position_id_, ADDR_X_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // FRONT RIGHT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FR_wheel_id_, ADDR_X_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FR_position_id_, ADDR_X_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK LEFT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BL_wheel_id_, ADDR_X_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BL_position_id_, ADDR_X_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK RIGHT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BR_wheel_id_, ADDR_X_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BR_position_id_, ADDR_X_OPERATING_MODE, POSITION_CONTROL_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool dxlMotorDriverSwerveWMQ::setPgainPositionMotors(uint16_t Pgain)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // FRONT LEFT:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, FL_position_id_, ADDR_X_POSITION_P_GAIN, Pgain, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // FRONT RIGHT:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, FR_position_id_, ADDR_X_POSITION_P_GAIN, Pgain, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK LEFT:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, BL_position_id_, ADDR_X_POSITION_P_GAIN, Pgain, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK RIGHT:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, BR_position_id_, ADDR_X_POSITION_P_GAIN, Pgain, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}
bool dxlMotorDriverSwerveWMQ::setTorqueRotationMotors(bool onOff_wheels)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // FRONT LEFT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FL_wheel_id_, ADDR_X_TORQUE_ENABLE, onOff_wheels, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // FRONT RIGHT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FR_wheel_id_, ADDR_X_TORQUE_ENABLE, onOff_wheels, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK LEFT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BL_wheel_id_, ADDR_X_TORQUE_ENABLE, onOff_wheels, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK RIGHT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BR_wheel_id_, ADDR_X_TORQUE_ENABLE, onOff_wheels, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool dxlMotorDriverSwerveWMQ::setTorquePositionMotors(bool onOff_postion)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    // FRONT LEFT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FL_position_id_, ADDR_X_TORQUE_ENABLE, onOff_postion, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // FRONT RIGHT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, FR_position_id_, ADDR_X_TORQUE_ENABLE, onOff_postion, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //  Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        //  Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK LEFT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BL_position_id_, ADDR_X_TORQUE_ENABLE, onOff_postion, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        //  Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        //  Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    // BACK RIGHT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, BR_position_id_, ADDR_X_TORQUE_ENABLE, onOff_postion, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        // Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool dxlMotorDriverSwerveWMQ::readMotorsSpeed(int32_t motorsSpeedRead[])
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    bool dxl_addparam_result = false;   // addParam result
    bool dxl_getdata_result = false;    // GetParam result

    // Set parameter
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(FL_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(FL_position_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(FR_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(FR_position_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(BL_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(BL_position_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(BR_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(BR_position_id_);
    if (dxl_addparam_result != true)
        return false;

    // Syncread current speed
    dxl_comm_result = groupSyncReadMotorSpeed_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(FL_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(FL_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(FR_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(FR_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(BL_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(BL_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(BR_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(BR_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;

    // Get data
    motorsSpeedRead[FL_WHEEL] = groupSyncReadMotorSpeed_->getData(FL_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    motorsSpeedRead[FL_POSITION] = groupSyncReadMotorSpeed_->getData(FL_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    motorsSpeedRead[FR_WHEEL] = groupSyncReadMotorSpeed_->getData(FR_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    motorsSpeedRead[FR_POSITION] = groupSyncReadMotorSpeed_->getData(FR_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    motorsSpeedRead[BL_WHEEL] = groupSyncReadMotorSpeed_->getData(BL_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    motorsSpeedRead[BL_POSITION] = groupSyncReadMotorSpeed_->getData(BL_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    motorsSpeedRead[BR_WHEEL] = groupSyncReadMotorSpeed_->getData(BR_wheel_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    motorsSpeedRead[BR_POSITION] = groupSyncReadMotorSpeed_->getData(BL_position_id_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);

    groupSyncReadMotorSpeed_->clearParam();
    return true;
}
bool dxlMotorDriverSwerveWMQ::readEncodersPosition(int32_t encodersPositionRead[])
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    bool dxl_addparam_result = false;   // addParam result
    bool dxl_getdata_result = false;    // GetParam result

    // Set parameter
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(FL_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(FL_position_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(FR_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(FR_position_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(BL_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(BL_position_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(BR_wheel_id_);
    if (dxl_addparam_result != true)
        return false;
    dxl_addparam_result = groupSyncReadEncoderPosition_->addParam(BR_position_id_);
    if (dxl_addparam_result != true)
        return false;

    // Syncread current speed
    dxl_comm_result = groupSyncReadEncoderPosition_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }

    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(FL_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(FL_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(FR_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(FR_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(BL_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(BL_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(BR_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    dxl_getdata_result = groupSyncReadEncoderPosition_->isAvailable(BR_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    if (dxl_getdata_result != true)
        return false;
    // Get data
    encodersPositionRead[FL_WHEEL] = groupSyncReadEncoderPosition_->getData(FL_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    encodersPositionRead[FL_POSITION] = groupSyncReadEncoderPosition_->getData(FL_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    encodersPositionRead[FR_WHEEL] = groupSyncReadEncoderPosition_->getData(FR_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    encodersPositionRead[FR_POSITION] = groupSyncReadEncoderPosition_->getData(FR_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    encodersPositionRead[BL_WHEEL] = groupSyncReadEncoderPosition_->getData(BL_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    encodersPositionRead[BL_POSITION] = groupSyncReadEncoderPosition_->getData(BL_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    encodersPositionRead[BR_WHEEL] = groupSyncReadEncoderPosition_->getData(BR_wheel_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);
    encodersPositionRead[BR_POSITION] = groupSyncReadEncoderPosition_->getData(BR_position_id_, ADDR_X_PRESENT_POSITION, LEN_X_PRESENT_POSITION);

    groupSyncReadEncoderPosition_->clearParam();
    return true;
}

bool dxlMotorDriverSwerveWMQ::writeWheelSpeed(int64_t write_Wheel_Speed[])
{
    bool dxl_addparam_result;
    int8_t dxl_comm_result;
    uint8_t FL_data_byte[4] = {
        0,
    };
    uint8_t FR_data_byte[4] = {
        0,
    };
    uint8_t BL_data_byte[4] = {
        0,
    };
    uint8_t BR_data_byte[4] = {
        0,
    };
    // Front left wheel:
    FL_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Wheel_Speed[0]));
    FL_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Wheel_Speed[0]));
    FL_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Wheel_Speed[0]));
    FL_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Wheel_Speed[0]));
    dxl_addparam_result = groupSyncWriteWheelSpeed_->addParam(FL_wheel_id_, (uint8_t *)&FL_data_byte);
    if (dxl_addparam_result != true)
        return false;
    // Front right wheel:
    FR_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Wheel_Speed[1]));
    FR_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Wheel_Speed[1]));
    FR_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Wheel_Speed[1]));
    FR_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Wheel_Speed[1]));
    dxl_addparam_result = groupSyncWriteWheelSpeed_->addParam(FR_wheel_id_, (uint8_t *)&FR_data_byte);
    if (dxl_addparam_result != true)
        return false;
    // Back left wheel:
    BL_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Wheel_Speed[2]));
    BL_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Wheel_Speed[2]));
    BL_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Wheel_Speed[2]));
    BL_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Wheel_Speed[2]));
    dxl_addparam_result = groupSyncWriteWheelSpeed_->addParam(BL_wheel_id_, (uint8_t *)&BL_data_byte);
    if (dxl_addparam_result != true)
    {
        return false;
    }
    // Back right wheel:
    BR_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Wheel_Speed[3]));
    BR_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Wheel_Speed[3]));
    BR_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Wheel_Speed[3]));
    BR_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Wheel_Speed[3]));
    dxl_addparam_result = groupSyncWriteWheelSpeed_->addParam(BR_wheel_id_, (uint8_t *)&BR_data_byte);
    if (dxl_addparam_result != true)
    {
        return false;
    }
    dxl_comm_result = groupSyncWriteWheelSpeed_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    groupSyncWriteWheelSpeed_->clearParam();
    return true;
}
// bool dxlMotorDriverSwerve::writeRotationPosition(int64_t write_Position_Motors_FL, int64_t write_Position_Motors_FR, int64_t write_Position_Motors_BL, int64_t write_Position_Motors_BR)
bool dxlMotorDriverSwerveWMQ::writeRotationPosition(int64_t write_Position_Motors[])
{
    bool dxl_addparam_result;
    int8_t dxl_comm_result;
    uint8_t FL_data_byte[4] = {
        0,
    };
    uint8_t FR_data_byte[4] = {
        0,
    };
    uint8_t BL_data_byte[4] = {
        0,
    };
    uint8_t BR_data_byte[4] = {
        0,
    };
    // Front left wheel:
    FL_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Position_Motors[0]));
    FL_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Position_Motors[0]));
    FL_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Position_Motors[0]));
    FL_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Position_Motors[0]));
    dxl_addparam_result = groupSyncWriteRotationPosition_->addParam(FL_position_id_, (uint8_t *)&FL_data_byte);
    if (dxl_addparam_result != true)
        return false;
    // Front right wheel:
    FR_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Position_Motors[1]));
    FR_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Position_Motors[1]));
    FR_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Position_Motors[1]));
    FR_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Position_Motors[1]));
    dxl_addparam_result = groupSyncWriteRotationPosition_->addParam(FR_position_id_, (uint8_t *)&FR_data_byte);
    if (dxl_addparam_result != true)
        return false;
    // Back left wheel:
    BL_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Position_Motors[2]));
    BL_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Position_Motors[2]));
    BL_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Position_Motors[2]));
    BL_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Position_Motors[2]));
    dxl_addparam_result = groupSyncWriteRotationPosition_->addParam(BL_position_id_, (uint8_t *)&BL_data_byte);
    if (dxl_addparam_result != true)
        return false;
    // Back right wheel:
    BR_data_byte[0] = DXL_LOBYTE(DXL_LOWORD(write_Position_Motors[3]));
    BR_data_byte[1] = DXL_HIBYTE(DXL_LOWORD(write_Position_Motors[3]));
    BR_data_byte[2] = DXL_LOBYTE(DXL_HIWORD(write_Position_Motors[3]));
    BR_data_byte[3] = DXL_HIBYTE(DXL_HIWORD(write_Position_Motors[3]));
    dxl_addparam_result = groupSyncWriteRotationPosition_->addParam(BR_position_id_, (uint8_t *)&BR_data_byte);
    if (dxl_addparam_result != true)
        return false;
    dxl_comm_result = groupSyncWriteRotationPosition_->txPacket();
    if (dxl_comm_result != COMM_SUCCESS)
    {
        // Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    groupSyncWriteRotationPosition_->clearParam();
    return true;
}

bool dxlMotorDriverSwerveWMQ::moveRobot(double x_vel, double y_vel, double z_rot)
{
    bool dxl_comm_result = false;
    double returnValues[8];
    int64_t inputPositions[4];
    int64_t inputMotorSpeeds[4];

    // Scaling input values
    x_vel = mapf(x_vel, dynamixel_min_vel_ms, dynamixel_max_vel_ms, -1, 1);             // x_vel scale from dynamixel_min_vel_ms/dynamixel_max_vel_ms to -1/1
    y_vel = mapf(y_vel, dynamixel_min_vel_ms, dynamixel_max_vel_ms, -1, 1);             // y_vel scale from dynamixel_min_vel_ms/dynamixel_max_vel_ms to -1/1
    z_rot = mapf(z_rot, dynamixel_min_rot_vel_rads, dynamixel_max_rot_vel_rads, -1, 1); // z_rot scale from dynamixel_min_rot_vel_rads/dynamixel_max_rot_vel_rads to -1/1

    // Calculate speed and angle for each motor
    // returns wheel speed from -1 to 1 and wheel position from -90 to 90 deg
    calcSwerve->inverseKinematics(returnValues, x_vel, y_vel, z_rot);

    // angles --> transform from deg to pulses [4096] than add zero_offset from motor [180 deg]
    returnValues[FL_POSITION] = returnValues[FL_POSITION] * puls_per_deg + FL_zero_offset;
    returnValues[FR_POSITION] = returnValues[FR_POSITION] * puls_per_deg + FR_zero_offset;
    returnValues[BL_POSITION] = returnValues[BL_POSITION] * puls_per_deg + BL_zero_offset;
    returnValues[BR_POSITION] = returnValues[BR_POSITION] * puls_per_deg + BR_zero_offset;

    // Revers scaling from -1/1 to dynamixel_min_vel_ms/dynamixel_max_vel_ms
    returnValues[FL_WHEEL] = mapf(returnValues[FL_WHEEL], -1, 1, dynamixel_min_vel_ms, dynamixel_max_vel_ms);
    returnValues[FR_WHEEL] = mapf(returnValues[FR_WHEEL], -1, 1, dynamixel_min_vel_ms, dynamixel_max_vel_ms);
    returnValues[BL_WHEEL] = mapf(returnValues[BL_WHEEL], -1, 1, dynamixel_min_vel_ms, dynamixel_max_vel_ms);
    returnValues[BR_WHEEL] = mapf(returnValues[BR_WHEEL], -1, 1, dynamixel_min_vel_ms, dynamixel_max_vel_ms);
    // wheel speed --> transform from m/s to pulses while constraining for min/max velocity
    // Constraint speed to max of 0.28 m/s [105 rpm]
    returnValues[FL_WHEEL] = constrain(returnValues[FL_WHEEL] * VELOCITY_CONSTANT_VALUE_WHEEL / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
    returnValues[FR_WHEEL] = constrain(returnValues[FR_WHEEL] * VELOCITY_CONSTANT_VALUE_WHEEL / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
    returnValues[BL_WHEEL] = constrain(returnValues[BL_WHEEL] * VELOCITY_CONSTANT_VALUE_WHEEL / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);
    returnValues[BR_WHEEL] = constrain(returnValues[BR_WHEEL] * VELOCITY_CONSTANT_VALUE_WHEEL / wheel_radius, -dynamixel_limit_max_velocity_, dynamixel_limit_max_velocity_);

    inputPositions[0] = (int64_t)returnValues[FL_POSITION];
    inputPositions[1] = (int64_t)returnValues[FR_POSITION];
    inputPositions[2] = (int64_t)returnValues[BL_POSITION];
    inputPositions[3] = (int64_t)returnValues[BR_POSITION];

    inputMotorSpeeds[0] = (int64_t)returnValues[FL_WHEEL];
    inputMotorSpeeds[1] = (int64_t)returnValues[FR_WHEEL];
    inputMotorSpeeds[2] = (int64_t)returnValues[BL_WHEEL];
    inputMotorSpeeds[3] = (int64_t)returnValues[BR_WHEEL];
    // Send final values to motors
    dxl_comm_result = writeRotationPosition(inputPositions);
    if (dxl_comm_result == false)
        return 0;
    dxl_comm_result = writeWheelSpeed(inputMotorSpeeds);
    if (dxl_comm_result == false)
        return 0;
}

bool dxlMotorDriverSwerveWMQ::readRobotInfo(double returnRobotSpeeds[], int32_t returnWheelEncoderTicks[], double returnPositionMotorsRad[], double returnMotorSpeeds[])
{
    double estimatedRobotSpeed[3] = {0.0, 0.0, 0.0};
    int32_t encoderTicks[8] = {0, 0, 0, 0, 0, 0, 0};
    int32_t motorSpeeds[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    double inputValuesForwardKinematics[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // fill array with current encoder ticks
    readEncodersPosition(encoderTicks);
    // fill array with current motor speeds
    readMotorsSpeed(motorSpeeds);
    // transform from motor position angles to robot position angles in DEG
    inputValuesForwardKinematics[FL_POSITION] = ((double)encoderTicks[FL_POSITION] - (double)FL_zero_offset) * deg_per_puls;
    inputValuesForwardKinematics[FR_POSITION] = ((double)encoderTicks[FR_POSITION] - (double)FR_zero_offset) * deg_per_puls;
    inputValuesForwardKinematics[BL_POSITION] = ((double)encoderTicks[BL_POSITION] - (double)BL_zero_offset) * deg_per_puls;
    inputValuesForwardKinematics[BR_POSITION] = ((double)encoderTicks[BR_POSITION] - (double)BR_zero_offset) * deg_per_puls;
    // fill returnMotorSpeeds[] in rad/s
    returnMotorSpeeds[FL_WHEEL] = (((double)motorSpeeds[FL_WHEEL] * RPM_FACTOR) / 60) * 2 * PI;
    returnMotorSpeeds[FL_POSITION] = (((double)motorSpeeds[FL_POSITION] * RPM_FACTOR) / 60) * 2 * PI;
    returnMotorSpeeds[FR_WHEEL] = (((double)motorSpeeds[FR_WHEEL] * RPM_FACTOR) / 60) * 2 * PI;
    returnMotorSpeeds[FR_POSITION] = (((double)motorSpeeds[FR_POSITION] * RPM_FACTOR) / 60) * 2 * PI;
    returnMotorSpeeds[BL_WHEEL] = (((double)motorSpeeds[BL_WHEEL] * RPM_FACTOR) / 60) * 2 * PI;
    returnMotorSpeeds[BL_POSITION] = (((double)motorSpeeds[BL_POSITION] * RPM_FACTOR) / 60) * 2 * PI;
    returnMotorSpeeds[BR_WHEEL] = (((double)motorSpeeds[BR_WHEEL] * RPM_FACTOR) / 60) * 2 * PI;
    returnMotorSpeeds[BR_POSITION] = (((double)motorSpeeds[BR_POSITION] * RPM_FACTOR) / 60) * 2 * PI;
    // transform from wheel rpm to linear velocity in m/s
    inputValuesForwardKinematics[FL_WHEEL] = returnMotorSpeeds[FL_WHEEL] * WHEEL_RAD;
    inputValuesForwardKinematics[FR_WHEEL] = returnMotorSpeeds[FR_WHEEL] * WHEEL_RAD;
    inputValuesForwardKinematics[BL_WHEEL] = returnMotorSpeeds[BL_WHEEL] * WHEEL_RAD;
    inputValuesForwardKinematics[BR_WHEEL] = returnMotorSpeeds[BR_WHEEL] * WHEEL_RAD;
    // Scaling from dynamixel_min_vel_ms/dynamixel_max_vel_ms to -1/1
    inputValuesForwardKinematics[FL_WHEEL] = mapf(inputValuesForwardKinematics[FL_WHEEL], dynamixel_min_vel_ms, dynamixel_max_vel_ms, -1, 1);
    inputValuesForwardKinematics[FR_WHEEL] = mapf(inputValuesForwardKinematics[FR_WHEEL], dynamixel_min_vel_ms, dynamixel_max_vel_ms, -1, 1);
    inputValuesForwardKinematics[BL_WHEEL] = mapf(inputValuesForwardKinematics[BL_WHEEL], dynamixel_min_vel_ms, dynamixel_max_vel_ms, -1, 1);
    inputValuesForwardKinematics[BR_WHEEL] = mapf(inputValuesForwardKinematics[BR_WHEEL], dynamixel_min_vel_ms, dynamixel_max_vel_ms, -1, 1);
    calcSwerve->forwardKinematics(estimatedRobotSpeed, inputValuesForwardKinematics);
    // Scaling from [-1,1] to m/s or rad/s and filling returnRobotSpeedsArray[]
    returnRobotSpeeds[0] = mapf(estimatedRobotSpeed[0], -1, 1, dynamixel_min_vel_ms, dynamixel_max_vel_ms);             // x_vel scale from dynamixel_min_vel_ms/dynamixel_max_vel_ms to -1/1
    returnRobotSpeeds[1] = mapf(estimatedRobotSpeed[1], -1, 1, dynamixel_min_vel_ms, dynamixel_max_vel_ms);             // y_vel scale from dynamixel_min_vel_ms/dynamixel_max_vel_ms to -1/1
    returnRobotSpeeds[2] = mapf(estimatedRobotSpeed[2], -1, 1, dynamixel_min_rot_vel_rads, dynamixel_max_rot_vel_rads); // z_rot scale from dynamixel_min_rot_vel_rads/dynamixel_max_rot_vel_rads to -1/1
    // fill returnWheelEncoderTicks[] in ticks
    returnWheelEncoderTicks[0] = encoderTicks[FL_WHEEL];
    returnWheelEncoderTicks[1] = encoderTicks[FR_WHEEL];
    returnWheelEncoderTicks[2] = encoderTicks[BL_WHEEL];
    returnWheelEncoderTicks[3] = encoderTicks[BR_WHEEL];
    // fill returnPositionMotorsRad[] in rad
    returnPositionMotorsRad[0] = inputValuesForwardKinematics[FL_POSITION] * PI / 180;
    returnPositionMotorsRad[1] = inputValuesForwardKinematics[FR_POSITION] * PI / 180;
    returnPositionMotorsRad[2] = inputValuesForwardKinematics[BL_POSITION] * PI / 180;
    returnPositionMotorsRad[3] = inputValuesForwardKinematics[BR_POSITION] * PI / 180;
}

void dxlMotorDriverSwerveWMQ::setZeroPosition()
{
    int64_t zeroPositon[4] = {FL_ZERO_POS, FR_ZERO_POS, BL_ZERO_POS, BR_ZERO_POS};
    writeRotationPosition(zeroPositon);
}

double dxlMotorDriverSwerveWMQ::mapf(double val, double in_min, double in_max, double out_min, double out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
