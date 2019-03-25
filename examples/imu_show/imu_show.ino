/*!
  * imu_show.ino
  *
  * Download this demo to show attitude on [imu_show](https://github.com/DFRobot/DFRobot_IMU_Show)
  * Attitude will show on imu_show
  *
  * Product: http://www.dfrobot.com.cn/goods-1860.html
  * Copyright   [DFRobot](http://www.dfrobot.com), 2016
  * Copyright   GNU Lesser General Public License
  *
  * version  V1.0
  * date  07/03/2019
  */

#include "DFRobot_BNO055.h"
#include "Wire.h"

typedef DFRobot_BNO055_IIC    BNO;    // ******** use abbreviations instead of full names ********

BNO   bno(&Wire, 0x28);    // input TwoWire interface and IIC address

// show last sensor operate status
void printLastOperateStatus(BNO::eStatus_t eStatus)
{
  switch(eStatus) {
  case BNO::eStatusOK:    Serial.println("everything ok"); break;
  case BNO::eStatusErr:   Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:    Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut: Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:       Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup()
{
  Serial.begin(115200);
  bno.reset();
  while(bno.begin() != BNO::eStatusOK) {
    Serial.println("bno begin faild");
    printLastOperateStatus(bno.lastOperateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");
}

void loop()
{
  BNO::sEulAnalog_t   sEul;
  sEul = bno.getEul();
  Serial.print("pitch:");
  Serial.print(sEul.pitch, 3);
  Serial.print(" ");
  Serial.print("roll:");
  Serial.print(sEul.roll, 3);
  Serial.print(" ");
  Serial.print("yaw:");
  Serial.print(sEul.head, 3);
  Serial.println(" ");
  delay(80);
}
