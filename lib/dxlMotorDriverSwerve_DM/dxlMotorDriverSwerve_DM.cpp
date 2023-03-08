#include "dxlMotorDriverSwerve_DM.h"

dxlMotorDriverSwerve::dxlMotorDriverSwerve()
{
    baudrate_ = BAUDRATE;
    MOTOR_ID = DXL_ID_MOTOR;
    Serial.begin(57600);
}

dxlMotorDriverSwerve::~dxlMotorDriverSwerve()
{
    setTorque(false);
    portHandler_->closePort();
    Serial.end();
}

bool dxlMotorDriverSwerve::init(void)
{
    Serial.begin(57600);

    portHandler_ = dynamixel::PortHandler::getPortHandler(NAME);
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    // Open port
    if (portHandler_->openPort() == false)
    {
        Serial.println("Failed to open port(Motor Driver)");
        return false;
    }

    // Set port baudrate
    if (portHandler_->setBaudRate(baudrate_) == false)
    {
        Serial.println("Failed to set baud rate(Motor Driver)");
        return false;
    }

    groupSyncReadMotorSpeed_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    groupSyncReadMotorLoad_ = new dynamixel::GroupSyncRead(portHandler_, packetHandler_, ADDR_X_PRESENT_LOAD, LEN_X_PRESENT_LOAD);
    groupSyncWriteWheelSpeed_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_X_GOAL_VELOCITY, LEN_X_GOAL_VELOCITY);
    //set operating mode for motors, postion and velocity
    setOperatingMode();
    //setGainsToZero();
    // Enable Dynamixel Torque
    setTorque(true);

    Serial.println("Success to init Motor Driver");
    return true;
}
bool dxlMotorDriverSwerve::setOperatingMode()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    //FRONT LEFT:
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_OPERATING_MODE, VELOCITY_MODE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool dxlMotorDriverSwerve::setGainsToZero()
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    //POSITION D GAIN:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_POSITION_D_GAIN, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    //POSITION I GAIN:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_POSITION_I_GAIN, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    //POSITION P GAIN:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_POSITION_P_GAIN, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    //VELOCITY I GAIN:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_VELOCITY_I_GAIN, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    //VELOCITY P GAIN:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_VELOCITY_P_GAIN, 1000, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    //FEEDF 2ND GAIN:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_FEEDFORWARD_2ND_GAIN, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    //FEEDF 1ST GAIN:
    dxl_comm_result = packetHandler_->write2ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_FEEDFORWARD_1ST_GAIN, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}


bool dxlMotorDriverSwerve::setTorque(bool onOff)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_TORQUE_ENABLE, onOff, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}

bool dxlMotorDriverSwerve::readMotorSpeed(int32_t &readMotorSpeed)
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    bool dxl_addparam_result = false;   // addParam result
    bool dxl_getdata_result = false;    // GetParam result

    // Set parameter
    dxl_addparam_result = groupSyncReadMotorSpeed_->addParam(MOTOR_ID);
    if (dxl_addparam_result != true)
        return false;
    
    // Syncread current speed
    dxl_comm_result = groupSyncReadMotorSpeed_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadMotorSpeed_->isAvailable(MOTOR_ID, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    if (dxl_getdata_result != true)
        return false;
    // Get data
    readMotorSpeed = groupSyncReadMotorSpeed_->getData(MOTOR_ID, ADDR_X_PRESENT_VELOCITY, LEN_X_PRESENT_VELOCITY);
    groupSyncReadMotorSpeed_->clearParam();
    return true;
    
}
bool dxlMotorDriverSwerve::readMotorLoad(int32_t &readMotorLoad)
{
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    bool dxl_addparam_result = false;   // addParam result
    bool dxl_getdata_result = false;    // GetParam result

    // Set parameter
    dxl_addparam_result = groupSyncReadMotorLoad_->addParam(MOTOR_ID);
    if (dxl_addparam_result != true)
        return false;


    // Syncread current speed
    dxl_comm_result = groupSyncReadMotorLoad_->txRxPacket();
    if (dxl_comm_result != COMM_SUCCESS)
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));

    // Check if groupSyncRead data of Dynamixels are available
    dxl_getdata_result = groupSyncReadMotorLoad_->isAvailable(MOTOR_ID, ADDR_X_PRESENT_LOAD, LEN_X_PRESENT_LOAD);
    if (dxl_getdata_result != true)
        return false;

    // Get data
    readMotorLoad = groupSyncReadMotorLoad_->getData(MOTOR_ID, ADDR_X_PRESENT_LOAD, LEN_X_PRESENT_LOAD);

    groupSyncReadMotorLoad_->clearParam();
    return true;
    
}

bool dxlMotorDriverSwerve::writeMotorSpeed(uint32_t writeMotorSpeed)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_comm_result = packetHandler_->write4ByteTxRx(portHandler_, MOTOR_ID, ADDR_X_GOAL_VELOCITY, writeMotorSpeed, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        Serial.println(packetHandler_->getTxRxResult(dxl_comm_result));
        return false;
    }
    else if (dxl_error != 0)
    {
        Serial.println(packetHandler_->getRxPacketError(dxl_error));
        return false;
    }
    return true;
}
