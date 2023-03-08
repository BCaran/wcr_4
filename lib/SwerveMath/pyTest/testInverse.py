import math
import numpy as np

inputSpeed = [[-0.32], [0.55], [0.25]]


def optimazeSpeedAngle(speed, angle):
    if(angle > 90):
        return -1 * speed, angle - 180
    elif(angle <= -90):
        return -1 * speed, angle + 180
    else:
        return speed, angle

def calculateVxVy(currentSpeed, currentAngle):
    retVx = math.sqrt((currentSpeed * currentSpeed)/(1 + math.tan(currentAngle * math.pi / 180) * math.tan(currentAngle * math.pi / 180)))
    retVy = abs(retVx * math.tan(currentAngle * math.pi / 180))
    if(currentSpeed >= 0 and currentAngle >= 0):
        return retVx, retVy
    elif (currentSpeed > 0 and currentAngle < 0):
        return retVx, retVy * -1
    elif(currentSpeed < 0 and currentAngle < 0):
        return retVx * -1, retVy
    else:
        return retVx * -1, retVy * -1

length_ = 0.255
width_ = 0.255
R_ = math.sqrt((length_ * length_) + (width_ * width_))

Rx_ = length_/R_
Ry_ = width_/R_


M = [[1, 0, Ry_ * -1],
    [0, 1, Rx_],
    [1, 0, Ry_],
    [0, 1, Rx_],
    [1, 0, Ry_ * -1],
    [0, 1, Rx_ * -1],
    [1, 0, Ry_],
    [0, 1, Rx_ * -1]]


result = [[0], [0], [0], [0], [0], [0], [0], [0]]

result = np.dot(M,inputSpeed)
returnValues = [0, 0, 0, 0, 0, 0, 0, 0]

returnValues[0] = math.sqrt(result[0] * result[0] + result[1] * result[1]) #v_FL
returnValues[1] = math.atan2(result[1], result[0]) * 180 / math.pi #angle_FL
returnValues[2] = math.sqrt(result[2] * result[2] + result[3] * result[3]) #v_FR
returnValues[3] = math.atan2(result[3], result[2]) * 180 / math.pi #angle_FR
returnValues[4] = math.sqrt(result[4] * result[4] + result[5] * result[5]) #v_BL
returnValues[5] = math.atan2(result[5], result[4]) * 180 / math.pi #angle_BL
returnValues[6] = math.sqrt(result[6] * result[6] + result[7] * result[7]) #v_BR
returnValues[7] = math.atan2(result[7], result[6]) * 180 / math.pi #angle_BR

maxSpeed = returnValues[0]
if(returnValues[2] > maxSpeed):
    maxSpeed = returnValues[2]
if(returnValues[4] > maxSpeed):
    maxSpeed = returnValues[4]
if (returnValues[6] > maxSpeed):
    maxSpeed = returnValues[6]
    
if (maxSpeed > 1):
    returnValues[0] /= maxSpeed
    returnValues[2] /= maxSpeed
    returnValues[4] /= maxSpeed
    returnValues[6] /= maxSpeed

returnValues[0], returnValues[1] = optimazeSpeedAngle(returnValues[0], returnValues[1])
returnValues[2], returnValues[3] = optimazeSpeedAngle(returnValues[2], returnValues[3])
returnValues[4], returnValues[5] = optimazeSpeedAngle(returnValues[4], returnValues[5])
returnValues[6], returnValues[7] = optimazeSpeedAngle(returnValues[6], returnValues[7])

for r in returnValues:
   print(r)

print("******************************")

currentMotorSpeed = [returnValues[0], returnValues[2], returnValues[4], returnValues[6]]
currentMotorAngle = [returnValues[1], returnValues[3], returnValues[5], returnValues[7]]
outputSpeeds = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
M_INV = np.linalg.pinv(M)
outputSpeeds[0], outputSpeeds[1] = calculateVxVy(currentMotorSpeed[0], currentMotorAngle[0])
outputSpeeds[2], outputSpeeds[3] = calculateVxVy(currentMotorSpeed[1], currentMotorAngle[1])
outputSpeeds[4], outputSpeeds[5] = calculateVxVy(currentMotorSpeed[2], currentMotorAngle[2])
outputSpeeds[6], outputSpeeds[7] = calculateVxVy(currentMotorSpeed[3], currentMotorAngle[3])
resutl_INV = [[0], [0], [0], [0], [0], [0], [0], [0]]
resutl_INV = np.dot(M_INV,outputSpeeds)
for r in resutl_INV:
   print(r)
print("******************************")
