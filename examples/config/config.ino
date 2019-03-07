/*!
  * config.ino
  *
  * Download this demo to test config to bno055
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

BNO   bno(&Wire, BNO::eCom3Low);    // pin com3 is low

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
    printLastOperateStatus(bno.lastOpreateStatus);
    delay(2000);
  }
  Serial.println("bno begin success");

  bno.setOprMode(BNO::eOprModeConfig);    // must set sensor to config-mode before configure

  // accelerometer normal configure
  bno.setAccRange(BNO::eAccRange_4G);   // set range
  bno.setAccBandWidth(BNO::eAccBandWidth_62_5);   // set band width
  bno.setAccPowerMode(BNO::eAccPowerModeNormal);  // set power mode

  // magnetometer normal configure
  bno.setMagDataRate(BNO::eMagDataRate_20);   // set output data rate
  bno.setMagPowerMode(BNO::eMagPowerModeForce);   // set power mode
  bno.setMagOprMode(BNO::eMagOprModeRegular); // set operate mode

  // gyroscope normal configure
  bno.setGyrRange(BNO::eGyrRange_2000);   // set range
  bno.setGyrBandWidth(BNO::eGyrBandWidth_32);   // set band width
  bno.setGyrPowerMode(BNO::eGyrPowerModeNormal);    // set power mode

  BNO::sAxisAnalog_t    sOffsetAcc;   // unit mg, members can't out of acc range
  BNO::sAxisAnalog_t    sOffsetMag;   // unit ut, members can't out of mag range
  BNO::sAxisAnalog_t    sOffsetGyr;   // unit dps, members can't out of gyr range
  sOffsetAcc.x = 1;
  sOffsetAcc.y = 1;
  sOffsetAcc.z = 1;
  sOffsetMag.x = 1;
  sOffsetMag.y = 1;
  sOffsetMag.z = 1;
  sOffsetGyr.x = 1;
  sOffsetGyr.y = 1;
  sOffsetGyr.z = 1;
  bno.setAxisOffset(BNO::eAxisAcc, sOffsetAcc);   // set offset
  bno.setAxisOffset(BNO::eAxisMag, sOffsetMag);
  bno.setAxisOffset(BNO::eAxisGyr, sOffsetGyr);

  bno.setOprMode(BNO::eOprModeNdof);   // shift to other operate mode, reference datasheet for more detail
  delay(50);    // wait before operate mode shift done
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
  BNO::sAxisAnalog_t   sAccAnalog, sMagAnalog, sGyrAnalog;
  sAccAnalog = bno.getAxis(BNO::eAxisAcc);
  sMagAnalog = bno.getAxis(BNO::eAxisMag);
  sGyrAnalog = bno.getAxis(BNO::eAxisGyr);
  Serial.println();
  Serial.println("======== analog data print start ========");
  Serial.print("acc analog: (unit mg)       "); printAxisData(sAccAnalog);
  Serial.print("mag analog: (unit ut)       "); printAxisData(sMagAnalog);
  Serial.print("gyr analog: (unit dps)      "); printAxisData(sGyrAnalog);
  Serial.println("========  analog data print end  ========");

  delay(1000);
}
