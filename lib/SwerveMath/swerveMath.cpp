#include "swerveMath.h"

SwerveMath::SwerveMath(double length, double width):
length_(length), width_(width)
{
    D_ = sqrt((length_ * length_) + (width_ * width_));
    Rx_ = length_ / D_;
    Ry_ = width_ / D_;
}

//From x_vel, y_vel, z_rotVel to vel and angle for every module
//Working range from -90 to 90 deg with possibility of changing speed direction for every wheel
void SwerveMath::inverseKinematics(double returnValues[], double x_vel, double y_vel, double z_rot) 
{
    //Robot state
    double robotState[8][3] = {{1, 0, (-1 * Ry_)}, {0, 1, Rx_}, {1, 0, Ry_}, {0, 1, Rx_}, {1, 0, (-1 * Ry_)}, {0, 1, (-1 * Rx_)}, {1, 0, Ry_}, {0, 1, (-1 * Rx_)}};
    double outputSpeeds[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double returnValues_temp[2];
    double inputSpeeds[3] = {x_vel, y_vel, z_rot};
    //Matrix multiplication, outputSpeeds = robotState * inputSpeeds
    for(int i = 0; i < 8; i++)
        for(int j = 0; j < 3; j++)
            outputSpeeds[i] += robotState[i][j]*inputSpeeds[j];
    //OutputSpeeds(vx, vy) to wheel speed and angle
    //TODO: Use radians, not deg
    returnValues[0] = sqrt(outputSpeeds[0] * outputSpeeds[0]  + outputSpeeds[1] * outputSpeeds[1]); // v_FL
    returnValues[1] = atan2(outputSpeeds[1], outputSpeeds[0]) * 180 / PI; //angle_FL
    returnValues[2] = sqrt(outputSpeeds[2] * outputSpeeds[2] + outputSpeeds[3] * outputSpeeds[3]); // v_FR
    returnValues[3] = atan2(outputSpeeds[3], outputSpeeds[2]) * 180 / PI; //angle_FR
    returnValues[4] = sqrt(outputSpeeds[4] * outputSpeeds[4] + outputSpeeds[5] * outputSpeeds[5]); // v_BL
    returnValues[5] = atan2(outputSpeeds[5], outputSpeeds[4]) * 180 / PI; //angle_BL
    returnValues[6] = sqrt(outputSpeeds[6] * outputSpeeds[6] + outputSpeeds[7] * outputSpeeds[7]); // v_BR
    returnValues[7] = atan2(outputSpeeds[7], outputSpeeds[6]) * 180 / PI; //angle_BR
    //Max speed scaling
    double maxSpeed = returnValues[0];
    if (returnValues[2] > maxSpeed) maxSpeed = returnValues[2];
    if (returnValues[4] > maxSpeed) maxSpeed = returnValues[4];
    if (returnValues[6] > maxSpeed) maxSpeed = returnValues[6];    
    if (maxSpeed > 1)
    {
        returnValues[0] /= maxSpeed;
        returnValues[2] /= maxSpeed;
        returnValues[4] /= maxSpeed;
        returnValues[6] /= maxSpeed;
    }
    //Speed and angle optimization
    optimazeSpeedAngle(returnValues_temp, returnValues[0], returnValues[1]);
    returnValues[0] = returnValues_temp[0]; //v_FL
    returnValues[1] = returnValues_temp[1]; //angle_FL
    optimazeSpeedAngle(returnValues_temp, returnValues[2], returnValues[3]);
    returnValues[2] = returnValues_temp[0]; //v_FR
    returnValues[3] = returnValues_temp[1]; //angle_FR
    optimazeSpeedAngle(returnValues_temp, returnValues[4], returnValues[5]);
    returnValues[4] = returnValues_temp[0]; //v_BL
    returnValues[5] = returnValues_temp[1]; //angle_BL
    optimazeSpeedAngle(returnValues_temp, returnValues[6], returnValues[7]);
    returnValues[6] = returnValues_temp[0]; //v_BR
    returnValues[7] = returnValues_temp[1]; //angle_BR
}

//From vel and angle for every module to x_vel, y_vel and z_rotVel for robot
//Every module could have +/-(vx, vy), using principles from  optimazeSpeedAngle() to decide if vx and vy is + or - for every module
void SwerveMath::forwardKinematics(double returnValues[], double inputValues[])
{
    double robotSpeed[3] = {0.0, 0.0, 0.0};
    //OutputSpeeds(vx, vy) for every module, calculated from module speed, angle and wheel rotation direction
    double outputSpeeds[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //Inverse of robotState from inverseKinematics()
    // float inv_robotState[3][8] = {{0.25, 0, 0.25, 0, 0.25, 0, 0.25, 0}, {0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 0.0, 0.25}, 
    //                             {(-1 * Ry_) / ( 4 * Rx_ * Rx_ + 4 * Ry_ * Ry_),
    //                             Rx_ / ( 4 * Rx_ * Rx_ + 4 * Ry_ * Ry_),
    //                             Ry_ / (4 * Rx_ * Rx_ + 4 * Ry_ * Ry_),
    //                             Rx_ / (4 * Rx_ * Rx_ + 4 * Ry_ * Ry_),
    //                             (-1 * Ry_) / (4 * Rx_ * Rx_ + 4 * Ry_ * Ry_),
    //                             (-1 * Rx_) / (4 * Rx_ * Rx_ + 4 * Ry_ * Ry_),
    //                             Ry_ / ( 4 * Rx_ * Rx_ + 4 * Ry_ * Ry_),
    //                             (-1 * Rx_) / (4 * Rx_ * Rx_ + 4 * Ry_ * Ry_)}};
    double inv_robotState[3][8] = {{0.25, 0, 0.25, 0, 0.25, 0, 0.25, 0}, 
                                  {0.0, 0.25, 0.0, 0.25, 0.0, 0.25, 0.0, 0.25}, 
                                  {-0.1767767,  0.1767767,  0.1767767,  0.1767767, -0.1767767, -0.1767767, 0.1767767, -0.1767767}};
    double returnValues_temp[2];
    calculateVxVy(returnValues_temp, inputValues[0], inputValues[1]);
    outputSpeeds[0] = returnValues_temp[0];
    outputSpeeds[1] = returnValues_temp[1];
    calculateVxVy(returnValues_temp, inputValues[2], inputValues[3]);
    outputSpeeds[2] = returnValues_temp[0];
    outputSpeeds[3] = returnValues_temp[1];
    calculateVxVy(returnValues_temp, inputValues[4], inputValues[5]);
    outputSpeeds[4] = returnValues_temp[0];
    outputSpeeds[5] = returnValues_temp[1];
    calculateVxVy(returnValues_temp, inputValues[6], inputValues[7]);
    outputSpeeds[6] = returnValues_temp[0];
    outputSpeeds[7] = returnValues_temp[1];
    //Matrix multiplication, robotSpeed = inv_robotState * outputSpeeds
    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 8; j++)
            robotSpeed[i] += inv_robotState[i][j]*outputSpeeds[j];
    for(int i = 0; i < 3; i++)
        returnValues[i] = robotSpeed[i];
}
//TODO: Use radians, not deg
void SwerveMath::optimazeSpeedAngle(double returnSpeedAngle[], double speed, double angle)
{
    if(angle > 90)
    {
        returnSpeedAngle[0] = -1 * speed;
        returnSpeedAngle[1] = angle - 180;
    }
    else if(angle <= -90)
    {
        returnSpeedAngle[0] = -1 * speed;
        returnSpeedAngle[1] = angle + 180;
    }
    else
    {
        returnSpeedAngle[0] = speed;
        returnSpeedAngle[1] = angle;
    }
}
//TODO: Use radians, not deg
void SwerveMath::calculateVxVy(double retVxVy[], double currentSpeed, double currentAngle)
{
    double temp_vx, temp_vy;
    double temp_angle = currentAngle * PI / 180;
    temp_vx = sqrt((currentSpeed * currentSpeed)/(1 + tan(temp_angle) * tan(temp_angle)));
    temp_vy = tan(temp_angle) * temp_vx;
    if(temp_vy < 0) temp_vy *= -1;
    if(currentSpeed >= 0 && currentAngle >= 0) 
    {
        temp_vx = temp_vx;
        temp_vy = temp_vy;
    }
    else if (currentSpeed > 0 && currentAngle < 0)
    {
        temp_vx = temp_vx;
        temp_vy = temp_vy * -1;  //vy
    }
    else if (currentSpeed < 0 && currentAngle < 0)
    {
        temp_vx = temp_vx * -1; //vx
        temp_vy = temp_vy;  //vy
    }
    else
    {
        temp_vx = temp_vx * -1;
        temp_vy = temp_vy * -1;
    }
    retVxVy[0] = temp_vx;
    retVxVy[1] = temp_vy;
}
