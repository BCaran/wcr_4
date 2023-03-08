#ifndef SWERVE_MATH_H
#define SWERVE_MATH_H

#include <math.h>
#include <stdlib.h>
#define PI 3.1415926535897932384626433832795

class SwerveMath
{
public:
	SwerveMath(double length, double width);
	void inverseKinematics(double returnValues[], double x_vel, double y_vel, double z_rot);
	void forwardKinematics(double returnValues[], double inputValues[]);
	void optimazeSpeedAngle(double returnSpeedAngle[], double currentSpeed, double currentAngle);
	void calculateVxVy(double retVxVy[], double currentSpeed, double currentAngle);
private:
	double length_, width_; // m
	double D_, Rx_, Ry_; // m
};

#endif