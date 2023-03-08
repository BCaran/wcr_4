// Trenutno odometrija uzima rotacije iz IMU-a, uzeti rotaciju iz odometrije i proslijediti posebno IMU i odometriju EKF-u (robot_localization)
// velocity iz motora uzeti, ne svoj raditi u odometrji
#include "WCR_core_config.h"

/*******************************************************************************
 * Setup function
 *******************************************************************************/
void setup()
{
    nh.initNode();
    nh.getHardware()->setBaud(115200);

    nh.subscribe(cmd_vel_sub);

    nh.advertise(encoder_odom_pub);
    nh.advertise(joint_states_pub);

    nh.advertiseService(motorPower_srv);
    nh.advertiseService(resetEncoderOdom_sub);


    nh.advertiseService(startVM_msr_sub);


    // Setting for Dynamixel motors
    motor_driver.init();

    SD.begin(cs_pin);
    bno.begin(adafruit_bno055_opmode_t::OPERATION_MODE_IMUPLUS);
    bno.setExtCrystalUse(true);

    prev_update_time = millis();
}

/*******************************************************************************
 * Loop function
 *******************************************************************************/
void loop()
{
    uint32_t t = millis();
    updateVariable(nh.connected()); // init odomtrey after StartUp
    updateTFPrefix(nh.connected()); // init header's after StartUp

    if ((t - tTime[0]) >= (1000 / CONTROL_MOTOR_SPEED_FREQUENCY))
    {
        goal_velocity[LINEAR_X] = goal_velocity_from_cmd[LINEAR_X];
        goal_velocity[LINEAR_Y] = goal_velocity_from_cmd[LINEAR_Y];
        goal_velocity[ANGULAR_Z] = goal_velocity_from_cmd[ANGULAR_Z];
        if ((t - tTime[6]) > CONTROL_MOTOR_TIMEOUT)
        {
            motor_driver.writeWheelSpeed(zero_velocity);
        }
        else
        {
            motor_driver.moveRobot(goal_velocity[0], goal_velocity[1], goal_velocity[2]);
        }
        tTime[0] = t;
    }

    if ((t - tTime[2]) >= (1000 / LOG_FREQUENCY) && startVMmsr == true)
    {
        motor_driver.readEncodersPosition(encoderTicks);
        sendDataStreamToSD();
        tTime[2] = t;
    }

    sendLogMsg(); // send log connection after StartUp
    nh.spinOnce();
    waitForSerialLink(nh.connected());
}

void sendDataStreamToSD()
{
    imu::Vector<3> angular_velocity = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> linear_acceleration = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Quaternion quat = bno.getQuat();

    logFile.print(millis() - passedTime);
    logFile.print(", [");
    logFile.print(encoderTicks[FL_POSITION] - encoderTicks_start_offset[FL_POSITION]);
    logFile.print(", ");
    logFile.print(encoderTicks[BL_POSITION] - encoderTicks_start_offset[BL_POSITION]);
    logFile.print(", ");
    logFile.print(encoderTicks[BR_POSITION] - encoderTicks_start_offset[BR_POSITION]);
    logFile.print(", ");
    logFile.print(encoderTicks[FR_POSITION] - encoderTicks_start_offset[FR_POSITION]);
    logFile.print(", ");
    logFile.print(encoderTicks[FL_WHEEL] - encoderTicks_start_offset[FL_WHEEL]);
    logFile.print(", ");
    logFile.print(encoderTicks[BL_WHEEL] - encoderTicks_start_offset[BL_WHEEL]);
    logFile.print(", ");
    logFile.print(encoderTicks[BR_WHEEL] - encoderTicks_start_offset[BR_WHEEL]);
    logFile.print(", ");
    logFile.print(encoderTicks[FR_WHEEL] - encoderTicks_start_offset[FR_WHEEL]);
    logFile.print("], [");
    logFile.print(angular_velocity.x());
    logFile.print(", ");
    logFile.print(angular_velocity.y());
    logFile.print(", ");
    logFile.print(angular_velocity.z());
    logFile.print(", ");
    logFile.print(linear_acceleration.x());
    logFile.print(", ");
    logFile.print(linear_acceleration.y());
    logFile.print(", ");
    logFile.print(linear_acceleration.z());
    logFile.print(", ");
    logFile.print(quat.w());
    logFile.print(", ");
    logFile.print(quat.x());
    logFile.print(", ");
    logFile.print(quat.y());
    logFile.print(", ");
    logFile.print(quat.z());
    logFile.println("]");
}
/*******************************************************************************
 * Callback functions
 *******************************************************************************/
void commandVelocityCallback(const geometry_msgs::Twist &cmd_vel_msg)
{
    goal_velocity_from_cmd[LINEAR_X] = cmd_vel_msg.linear.x;
    goal_velocity_from_cmd[LINEAR_Y] = cmd_vel_msg.linear.y;
    goal_velocity_from_cmd[ANGULAR_Z] = cmd_vel_msg.angular.z;

    goal_velocity_from_cmd[LINEAR_X] = constrain(goal_velocity_from_cmd[LINEAR_X], MIN_LINEAR_VELOCITY_MS, MAX_LINEAR_VELOCITY_MS);
    goal_velocity_from_cmd[LINEAR_Y] = constrain(goal_velocity_from_cmd[LINEAR_Y], MIN_LINEAR_VELOCITY_MS, MAX_LINEAR_VELOCITY_MS);
    goal_velocity_from_cmd[ANGULAR_Z] = constrain(goal_velocity_from_cmd[ANGULAR_Z], MIN_ANGULAR_VELOCITY_RADS, MAX_ANGULAR_VELOCITY_RADS);
    tTime[6] = millis();
}
void driveMotorsPowerCallback(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp)
{
    bool dxl_power_rotation = req.data;
    if (motor_driver.setTorqueRotationMotors(dxl_power_rotation) && motor_driver.setTorquePositionMotors(dxl_power_rotation))
    {
        resp.success = true;
    }
    else
        resp.success = false;
}

void encoderOdomResetCallback(const std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &resp)
{
    char log_msg[50];
    initEncoderOdom();
    sprintf(log_msg, "Reset encoder odometry");
    nh.loginfo(log_msg);
    resp.success = true;
}

void startVM_MSR(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp)
{

    char log_msg[50];
    startVMmsr = req.data;
    if (startVMmsr == true)
    {
        logFile = SD.open("testLog.txt", FILE_WRITE);
        motor_driver.readEncodersPosition(encoderTicks_start_offset);
        logFile.println("*****************Start of measurement*****************");
        logFile.println("Current Step, Passed Time[ms], Encoders[Steering encoders, Wheel encoders], IMU[Gyro, Accel, Quat]");
        logFile.println("currStep, passedTime, [FL_S, BL_S, BR_S, FR_S, FL_W, BL_W, BR_W, FR_W], [rx, ry, rz, ax, ay, az, w, x, y, z]");
        passedTime = millis();
        motor_driver.readEncodersPosition(encoderTicks);
        sendDataStreamToSD();
    }
    if (startVMmsr == false)
    {
        logFile.println("*****************End of measurement*****************");
        logFile.close();
    }

    sprintf(log_msg, "Starting VM Process");
    nh.loginfo(log_msg);
    resp.success = true;
}


/*******************************************************************************
 * Publish msgs (odometry, joint states)
 *******************************************************************************/
void publishDriveInformation(void)
{
    unsigned long time_now = millis();
    unsigned long step_time = time_now - prev_update_time;
    bool dxl_comm_result = false;
    prev_update_time = time_now;
    ros::Time stamp_now = nh.now();


    // Read all info from robot
    motor_driver.readRobotInfo(currentRobotSpeeds, currentWheelEncoderTicks, currentPositionMotorsRad, currentMotorSpeeds);

    // Calculate encoder odometry
    calculateEncoderOdometry(step_time * 0.001);
    // Update encoder odometry
    updateEncoderOdometry();
    encoder_odom.header.stamp = stamp_now;
    encoder_odom_pub.publish(&encoder_odom);

    // Calculate and update joint states
    calcAndUpdateJointStates();
    joint_states.header.stamp = stamp_now;
    joint_states_pub.publish(&joint_states);

    // //Encoder odometry tf
    // updateTF(odom_tf);
    // odom_tf.header.stamp = stamp_now;
    // tf_broadcaster.sendTransform(odom_tf);
}
/*******************************************************************************
 * Calculate encoder odometry
 *******************************************************************************/
void calculateEncoderOdometry(double diff_time)
{
    double v_wx, v_wy, rot_wz;
    double delta_x, delta_y, delta_theta;

    v_wx = currentRobotSpeeds[0] * cos(encoder_odom_pose[2]) - currentRobotSpeeds[1] * sin(encoder_odom_pose[2]); // robot vx speed in odom frame
    v_wy = currentRobotSpeeds[0] * sin(encoder_odom_pose[2]) + currentRobotSpeeds[1] * cos(encoder_odom_pose[2]); // robot vy speed in odom frame
    rot_wz = currentRobotSpeeds[2];                                                                               // robot rotz speed in odom frame

    delta_x = v_wx * diff_time;
    delta_y = v_wy * diff_time;
    delta_theta = rot_wz * diff_time;

    encoder_odom_pose[0] += float(delta_x);
    encoder_odom_pose[1] += float(delta_y);
    encoder_odom_pose[2] += float(delta_theta);

    encoder_odom_vel[0] = float(v_wx);
    encoder_odom_vel[1] = float(v_wy);
    encoder_odom_vel[2] = float(rot_wz);
}

/*******************************************************************************
 * Update TF Prefix
 *******************************************************************************/
void updateTFPrefix(bool isConnected)
{
    static bool isChecked = false;
    char log_msg[50];

    if (isConnected)
    {
        if (isChecked == false)
        {
            sprintf(encoder_odom_header_frame_id, "odom_encoder");
            sprintf(encoder_odom_child_frame_id, "base_link");

            // sprintf(opticalFlow_odom_header_frame_id, "odom_OF");
            // sprintf(opticalFlow_odom_child_frame_id, "base_link");

            sprintf(joint_state_header_frame_id, "base_link");

            sprintf(log_msg, "Setup TF on Encoder Odometry [%s]", encoder_odom_header_frame_id);
            nh.loginfo(log_msg);
            // sprintf(log_msg, "Setup TF on Optical flow Odometry [%s]", opticalFlow_odom_header_frame_id);
            // nh.loginfo(log_msg);
            sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
            nh.loginfo(log_msg);
            isChecked = true;
        }
    }
    else
    {
        isChecked = false;
    }
}

/*******************************************************************************
 * Update the Encoder odometry
 *******************************************************************************/
void updateEncoderOdometry(void)
{
    encoder_odom.header.frame_id = encoder_odom_header_frame_id;
    encoder_odom.child_frame_id = encoder_odom_child_frame_id;

    encoder_odom.pose.pose.position.x = encoder_odom_pose[0];
    encoder_odom.pose.pose.position.y = encoder_odom_pose[1];
    encoder_odom.pose.pose.position.z = 0;
    encoder_odom.pose.pose.orientation = tf::createQuaternionFromYaw(encoder_odom_pose[2]);

    encoder_odom.twist.twist.linear.x = encoder_odom_vel[0];
    encoder_odom.twist.twist.linear.y = encoder_odom_vel[1];
    encoder_odom.twist.twist.angular.z = encoder_odom_vel[2];

    encoder_odom.pose.covariance[0] = 0.001;      // x position, NOT USED **** 0.01
    encoder_odom.pose.covariance[7] = 0.001;      // y position, NOT USED **** 0.01
    encoder_odom.pose.covariance[14] = 1000000.0; // z position, NOT USED
    encoder_odom.pose.covariance[21] = 1000000.0; // roll orientation, NOT USED
    encoder_odom.pose.covariance[28] = 1000000.0; // pitch orientation, NOT USED
    encoder_odom.pose.covariance[35] = 1000.0;    // yaw orientation, NOT USED **** 0.03

    encoder_odom.twist.covariance[0] = 0.01;       // x velocity, USED
    encoder_odom.twist.covariance[7] = 0.01;       // y velocity, USED
    encoder_odom.twist.covariance[14] = 1000000.0; // z velocity, NOT USED
    encoder_odom.twist.covariance[21] = 1000000.0; // roll orientation vel, NOT USED
    encoder_odom.twist.covariance[28] = 1000000.0; // pitch orientation vel, NOT USED
    encoder_odom.twist.covariance[35] = 10;        // yaw angular vel, USED
}

/*******************************************************************************
 * Update the joint states
 *******************************************************************************/
void calcAndUpdateJointStates(void)
{
    int32_t current_tick = 0;
    static float joint_states_pos[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static float joint_states_vel[MOTOR_NUM] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    joint_states_pos[FL_POSITION] = currentPositionMotorsRad[0];
    joint_states_pos[FR_POSITION] = currentPositionMotorsRad[1];
    joint_states_pos[BL_POSITION] = currentPositionMotorsRad[2];
    joint_states_pos[BR_POSITION] = currentPositionMotorsRad[3];

    current_tick = currentWheelEncoderTicks[0];
    last_diff_tick[0] = current_tick - last_tick[0];
    last_tick[0] = current_tick;
    last_rad[0] += TICK2RAD * (double)last_diff_tick[0];

    current_tick = currentWheelEncoderTicks[1];
    last_diff_tick[1] = current_tick - last_tick[1];
    last_tick[1] = current_tick;
    last_rad[1] += TICK2RAD * (double)last_diff_tick[1];

    current_tick = currentWheelEncoderTicks[2];
    last_diff_tick[2] = current_tick - last_tick[2];
    last_tick[2] = current_tick;
    last_rad[2] += TICK2RAD * (double)last_diff_tick[2];

    current_tick = currentWheelEncoderTicks[3];
    last_diff_tick[3] = current_tick - last_tick[3];
    last_tick[3] = current_tick;
    last_rad[3] += TICK2RAD * (double)last_diff_tick[3];

    joint_states_pos[FL_WHEEL] = float(last_rad[0]);
    joint_states_pos[FR_WHEEL] = float(last_rad[1]);
    joint_states_pos[BL_WHEEL] = float(last_rad[2]);
    joint_states_pos[BR_WHEEL] = float(last_rad[3]);

    joint_states_vel[FL_WHEEL] = float(currentMotorSpeeds[FL_WHEEL]);
    joint_states_vel[FL_POSITION] = float(currentMotorSpeeds[FL_POSITION]);
    joint_states_vel[FR_WHEEL] = float(currentMotorSpeeds[FR_WHEEL]);
    joint_states_vel[FR_POSITION] = float(currentMotorSpeeds[FR_POSITION]);
    joint_states_vel[BL_WHEEL] = float(currentMotorSpeeds[BL_WHEEL]);
    joint_states_vel[BL_POSITION] = float(currentMotorSpeeds[BL_POSITION]);
    joint_states_vel[BR_WHEEL] = float(currentMotorSpeeds[BR_WHEEL]);
    joint_states_vel[BR_POSITION] = float(currentMotorSpeeds[BR_POSITION]);

    joint_states.position = joint_states_pos;
    joint_states.velocity = joint_states_vel;
}

/*******************************************************************************
 * CalcUpdateulate the TF
 *******************************************************************************/
void updateTF(geometry_msgs::TransformStamped &odom_tf)
{
    odom_tf.header = encoder_odom.header;
    odom_tf.child_frame_id = encoder_odom.child_frame_id;
    odom_tf.transform.translation.x = encoder_odom.pose.pose.position.x;
    odom_tf.transform.translation.y = encoder_odom.pose.pose.position.y;
    odom_tf.transform.translation.z = encoder_odom.pose.pose.position.z;
    odom_tf.transform.rotation = encoder_odom.pose.pose.orientation;
}

/*******************************************************************************
 * Update variable (initialization)
 *******************************************************************************/
void updateVariable(bool isConnected)
{
    static bool variable_flag = false;
    if (isConnected)
    {
        if (variable_flag == false)
        {
            initEncoderOdom();
            // initOpticalFlowOdom();
            initJointStates();
            variable_flag = true;
        }
    }
    else
    {
        variable_flag = false;
    }
}

/*******************************************************************************
 * Wait for Serial Link
 *******************************************************************************/
void waitForSerialLink(bool isConnected)
{
    static bool wait_flag = false;
    if (isConnected)
    {
        if (wait_flag == false)
        {
            motor_driver.setTorqueRotationMotors(true);
            motor_driver.setTorquePositionMotors(true);
            delay(10);

            wait_flag = true;
        }
    }
    else
    {
        wait_flag = false;
    }
}
/*******************************************************************************
 * Send log message
 *******************************************************************************/
void sendLogMsg(void)
{
    static bool log_flag = false;
    char log_msg[100];

    if (nh.connected())
    {
        if (log_flag == false)
        {
            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);
            sprintf(log_msg, "Connected to OpenCR board!");
            nh.loginfo(log_msg);
            sprintf(log_msg, "--------------------------");
            nh.loginfo(log_msg);
            log_flag = true;
        }
    }
    else
    {
        log_flag = false;
    }
}

/*******************************************************************************
 * Initialization of encoder odometry data
 *******************************************************************************/
void initEncoderOdom(void)
{
    for (int index = 0; index < 3; index++)
    {
        encoder_odom_pose[index] = 0.0;
        encoder_odom_vel[index] = 0.0;
    }

    encoder_odom.pose.pose.position.x = 0.0;
    encoder_odom.pose.pose.position.y = 0.0;
    encoder_odom.pose.pose.position.z = 0.0;

    encoder_odom.pose.pose.orientation.x = 0.0;
    encoder_odom.pose.pose.orientation.y = 0.0;
    encoder_odom.pose.pose.orientation.z = 0.0;
    encoder_odom.pose.pose.orientation.w = 0.0;

    encoder_odom.twist.twist.linear.x = 0.0;
    encoder_odom.twist.twist.angular.z = 0.0;
}

/*******************************************************************************
 * Initialization joint states data
 *******************************************************************************/
void initJointStates(void)
{
    static char *joint_states_name[] = {(char *)"fl_wheel_joint", (char *)"fl_position_joint", (char *)"fr_wheel_joint", (char *)"fr_position_joint",
                                        (char *)"bl_wheel_joint", (char *)"bl_position_joint", (char *)"br_wheel_joint", (char *)"br_position_joint"};
    joint_states.header.frame_id = joint_state_header_frame_id;
    joint_states.name = joint_states_name;

    joint_states.name_length = MOTOR_NUM;
    joint_states.position_length = MOTOR_NUM;
    joint_states.velocity_length = MOTOR_NUM;
    joint_states.effort_length = MOTOR_NUM;

    for (int index = 0; index < WHEELNUM; index++)
    {
        last_diff_tick[index] = 0;
        last_tick[index] = 0;
        last_rad[index] = 0.0;
    }
    last_tick[0] = currentWheelEncoderTicks[0];
    last_tick[1] = currentWheelEncoderTicks[1];
    last_tick[2] = currentWheelEncoderTicks[2];
    last_tick[3] = currentWheelEncoderTicks[3];
}
