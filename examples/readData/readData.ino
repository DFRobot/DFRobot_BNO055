/*!
  * read_data.ino
  *
  * Download this demo to test read data from bno055
  * Data will print on your serial monitor
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
  case BNO::eStatusOK:   Serial.println("everything ok"); break;
  case BNO::eStatusErr:  Serial.println("unknow error"); break;
  case BNO::eStatusErrDeviceNotDetect:   Serial.println("device not detected"); break;
  case BNO::eStatusErrDeviceReadyTimeOut:    Serial.println("device ready time out"); break;
  case BNO::eStatusErrDeviceStatus:    Serial.println("device internal status error"); break;
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

#define printAxisData(sAxis) \
  Serial.print(" x: "); \
  Serial.print(sAxis.x); \
  Serial.print(" y: "); \
  Serial.print(sAxis.y); \
  Serial.print(" z: "); \
  Serial.println(sAxis.z)

void loop()
{
  BNO::sAxisAnalog_t   sAccAnalog, sMagAnalog, sGyrAnalog, sLiaAnalog, sGrvAnalog;
  BNO::sEulAnalog_t    sEulAnalog;
  BNO::sQuaAnalog_t    sQuaAnalog;
  sAccAnalog = bno.getAxis(BNO::eAxisAcc);    // read acceleration
  sMagAnalog = bno.getAxis(BNO::eAxisMag);    // read geomagnetic
  sGyrAnalog = bno.getAxis(BNO::eAxisGyr);    // read gyroscope
  sLiaAnalog = bno.getAxis(BNO::eAxisLia);    // read linear acceleration
  sGrvAnalog = bno.getAxis(BNO::eAxisGrv);    // read gravity vector
  sEulAnalog = bno.getEul();                  // read euler angle
  sQuaAnalog = bno.getQua();                  // read quaternion
  Serial.println();
  Serial.println("======== analog data print start ========");
  Serial.print("acc analog: (unit mg)       "); printAxisData(sAccAnalog);
  Serial.print("mag analog: (unit ut)       "); printAxisData(sMagAnalog);
  Serial.print("gyr analog: (unit dps)      "); printAxisData(sGyrAnalog);
  Serial.print("lia analog: (unit mg)       "); printAxisData(sLiaAnalog);
  Serial.print("grv analog: (unit mg)       "); printAxisData(sGrvAnalog);
  Serial.print("eul analog: (unit degree)   "); Serial.print(" head: "); Serial.print(sEulAnalog.head); Serial.print(" roll: "); Serial.print(sEulAnalog.roll);  Serial.print(" pitch: "); Serial.println(sEulAnalog.pitch);
  Serial.print("qua analog: (no unit)       "); Serial.print(" w: "); Serial.print(sQuaAnalog.w); printAxisData(sQuaAnalog);
  Serial.println("========  analog data print end  ========");

  delay(1000);
}
