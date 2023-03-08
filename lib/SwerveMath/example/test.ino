#include "swerveMath.h"

double inputInv[3] = {0, 0, 0.55};
double returnInv[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

double returnForward[3] = {0.0, 0.0, 0.0};
SwerveMath swerveMath(0.225, 0.225);
void setup()
{
    Serial.begin(115200);
}

void loop() {
    
    uint32_t t = millis();

    swerveMath.inverseKinematics(returnInv, inputInv);
    for(int i = 0; i < 8; i++)
    {
        Serial.println(returnInv[i]);
    }
    Serial.println("**************");
    swerveMath.forwardKinematics(returnForward, returnInv);
    for(int i = 0; i < 3; i++)
    {
        Serial.println(returnForward[i]);
    }
    Serial.println("**************");
    Serial.println("**************");

    delay(50);
}
