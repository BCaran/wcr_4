/*
Teensy 4.0 pins:
  T-Motor ESC:
    Digital 3(PWM)  - BLDC1
    Digital 4(PWM)  - BLDC2
    Digital 5(PWM)  - BLDC3
    Digital 6(PWM)  - BLDC4
    A9[23] (Analog) - Current Sensor ESC -- Ne valja pin A9 na teensy
    A8[22] (Analog) - Current Sensor HALL
  VESC Left (UART):
    0 (RX1)
    1 (TX1)
  VESC Right (UART):
    7 (RX2)
    8 (TX2)
  HX711 Left (S.f.):
    Digital 9 (DAT)
    Digital 10 (SCK)
  HX711 Right (S.f.):
    Digital 11 (SCK)
    Digital 12 (DAT)
*/
#include "t4_config.h"

void setup()
{
  nh.initNode();
  nh.getHardware()->setBaud(115200);

  nh.subscribe(dutyEDF_sub);
  nh.subscribe(dutyThrusters_sub);

  nh.advertise(forceSensors_pub);
  nh.advertise(powerEDF_pub);
  nh.advertise(tempVESC_pub);

  nh.advertiseService(enableMotors_srv);
  nh.advertiseService(forceTare_srv);

  Serial2.begin(115200);
  EDF.setSerialPort(&Serial2);
  ESC1.attach(ESC_pin1, 1000, 2000);
  ESC2.attach(ESC_pin2, 1000, 2000);
  ESC3.attach(ESC_pin3, 1000, 2000);
  ESC4.attach(ESC_pin4, 1000, 2000);
  ESC1.write(0);
  ESC2.write(0);
  ESC3.write(0);
  ESC4.write(0);
  delay(1000);
  initForceSensors();
  tareForceSensors();
}
void loop()
{
  uint32_t t = millis();
  updateTFPrefix(nh.connected());
  if ((t - tTime[0]) >= (1000 / BLDC_MOTORS_UPDATE_FREQUENCY))
  {
    if ((t - tTime[4]) > CONTROL_MOTOR_TIMEOUT)
    {
      EDF.setDuty(ZERO_THR_EDF);
    }
    else
    {
      if (motorEnable == 1)
      {
        EDF.setDuty(edfDutyInput);
      }
      else
        EDF.setDuty(ZERO_THR_EDF);
    }
    tTime[0] = t;
  }
  if ((t - tTime[1]) >= (1000 / BLDC_MOTORS_UPDATE_FREQUENCY))
  {
    if ((t - tTime[5]) > CONTROL_MOTOR_TIMEOUT)
    {
      if (pwmThrustersInput != 0)
      {
        for (int i = pwmThrustersInput; i >= 0; --i)
        {
          ESC1.write(i);
          ESC2.write(i);
          ESC3.write(i);
          ESC4.write(i);
          delay(100);
        }
        pwmThrustersInput = 0;
      }
      else
      {
        ESC1.write(ZERO_DUTY);
        ESC2.write(ZERO_DUTY);
        ESC3.write(ZERO_DUTY);
        ESC4.write(ZERO_DUTY);
        delay(100);
      }
    }
    else
    {
      if (motorEnable == 1)
      {
        ESC1.write(pwmThrustersInput);
        ESC2.write(pwmThrustersInput);
        ESC3.write(pwmThrustersInput);
        ESC4.write(pwmThrustersInput);
      }
      else
      {
        ESC1.write(ZERO_DUTY);
        ESC2.write(ZERO_DUTY);
        ESC3.write(ZERO_DUTY);
        ESC4.write(ZERO_DUTY);
      }
    }
    tTime[1] = t;
  }

  if ((t - tTime[3]) >= (1000 / PUBLISH_SENSORS_INFO))
  {
    publishSensorsInfo();
    tTime[3] = t;
  }
  sendLogMsg();
  nh.spinOnce();
  waitForSerialLink(nh.connected());

  // check if not connected, write 0 to pwm of thrusters ESC, problem occurring when sudo shutdown -h now RPI
  if (!nh.connected() && firstConnectFlag == true)
  {
    /*
    ESC1.write(ZERO_DUTY);
    ESC2.write(ZERO_DUTY);
    ESC3.write(ZERO_DUTY);
    ESC4.write(ZERO_DUTY);
    delay(10);

    if(nh.connected())
    {
      connectFlag == false;
    }
    */

    // send reboot command
    SCB_AIRCR = 0x05FA0004;
  }
}
void commandEDFCalllback(const std_msgs::Float32 &edf_throttle_in)
{
  edfDutyInput = edf_throttle_in.data;
  edfDutyInput = constrain(edfDutyInput, MIN_DUTY_EDF, MAX_DUTY_EDF);
  tTime[4] = millis();
}
void commandThrustersCalllback(const std_msgs::Float32 &thrusters_throttle_in)
{
  pwmThrustersInput = int(thrusters_throttle_in.data);
  if (pwmThrustersInput < MIN_DUTY_DRONE)
    pwmThrustersInput = MIN_DUTY_DRONE;
  if (pwmThrustersInput > MAX_DUTY_DRONE)
    pwmThrustersInput = MAX_DUTY_DRONE;
  tTime[5] = millis();
}
void motorsEnableCallback(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp)
{
  bool tempEnableVal = req.data;
  if (tempEnableVal == true)
  {
    if (pwmThrustersInput == 0 && edfDutyInput == 0.0)
    {
      motorEnable = 1;
      resp.success = true;
    }
  }
  else
  {
    motorEnable = 0;
    resp.success = false;
  }
}

void tareForceSensorsCallback(const std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &resp)
{
  if (req.data == true)
  {
    char log_msg[50];
    tareForceSensors();
    sprintf(log_msg, "Force sensor tare");
    nh.loginfo(log_msg);
    resp.success = true;
  }
}

void publishSensorsInfo()
{
  EDF.getVescValues();
  float vescPower = EDF.data.inpVoltage * EDF.data.avgInputCurrent;
  float vescTemp = EDF.data.tempMosfet;
  float forceSensor_sum = (forceSensor_front.get_units() + forceSensor_back.get_units()) * g;
  powerEDF.data = vescPower;
  tempVESC.data = vescTemp;
  EDF_force_pub.data = forceSensor_sum;
  powerEDF_pub.publish(&powerEDF);
  tempVESC_pub.publish(&tempVESC);
  forceSensors_pub.publish(&EDF_force_pub);
}

void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  if (isConnected)
  {
    if (isChecked == false)
    {
      // IMU REMOVED
      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;
  if (isConnected)
  {
    if (wait_flag == false)
    {
      wait_flag = true;
      firstConnectFlag = true;
    }
  }
  else
  {
    ESC1.write(ZERO_DUTY);
    ESC2.write(ZERO_DUTY);
    ESC3.write(ZERO_DUTY);
    ESC4.write(ZERO_DUTY);
    delay(10);
    wait_flag = false;
  }
}
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
      sprintf(log_msg, "Connected to Teensy 4.0 board!");
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

void initForceSensors()
{
  forceSensor_front.begin(DOUT_FRONT, CLOCK_FRONT);
  forceSensor_back.begin(DOUT_BACK, CLOCK_BACK);
  forceSensor_front.set_scale(CALIB_FAC_FRONT);
  forceSensor_back.set_scale(CALIB_FAC_BACK);
}
void tareForceSensors()
{
  forceSensor_front.tare();
  forceSensor_back.tare();
}
