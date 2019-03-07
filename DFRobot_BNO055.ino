#include "DFRobot_BNO055.h"

DFRobot_BNO055_IIC    bno055(&Wire, DFRobot_BNO055_IIC::eCom3Low);

// show last sensor operate status
void printLastOperateStatus(DFRobot_BNO055::eStatus_t eStatus)
{
  switch(eStatus) {
  case DFRobot_BNO055::eStatusOK:   Serial.println("everything ok"); break;
  case DFRobot_BNO055::eStatusErr:  Serial.println("unknow error"); break;
  case DFRobot_BNO055::eStatusErrDeviceNotDetect:   Serial.println("device not detected"); break;
  case DFRobot_BNO055::eStatusErrDeviceReadyTimeOut:    Serial.println("device ready time out"); break;
  case DFRobot_BNO055::eStatusErrDeviceStatus:    Serial.println("device internal status error"); break;
  default: Serial.println("unknow status"); break;
  }
}

void setup()
{
  Serial.begin(115200);
  while(bno055.begin() != DFRobot_BNO055::eStatusOK) {
    Serial.println("bno055 begin faild");
    printLastOperateStatus(bno055.lastOpreateStatus);
    delay(2000);
  }
  Serial.println("bno055 begin success");
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
  DFRobot_BNO055::sAxisData_t   sAccData, sMagData, sGyrData, sLiaData, sGrvData;
  DFRobot_BNO055::sEulData_t    sEulData;
  DFRobot_BNO055::sQuaData_t    sQuaData;
  sAccData = bno055.getAxisRaw(DFRobot_BNO055::eAxisAcc);
  sMagData = bno055.getAxisRaw(DFRobot_BNO055::eAxisMag);
  sGyrData = bno055.getAxisRaw(DFRobot_BNO055::eAxisGyr);
  sLiaData = bno055.getAxisRaw(DFRobot_BNO055::eAxisLia);
  sGrvData = bno055.getAxisRaw(DFRobot_BNO055::eAxisGrv);
  sEulData = bno055.getEulRaw();
  sQuaData = bno055.getQuaRaw();
  Serial.println();
  Serial.println("======== raw data print start ========");
  Serial.print("acc raw:"); printAxisData(sAccData);
  Serial.print("mag raw:"); printAxisData(sMagData);
  Serial.print("gyr raw:"); printAxisData(sGyrData);
  Serial.print("lia raw:"); printAxisData(sLiaData);
  Serial.print("grv raw:"); printAxisData(sGrvData);
  Serial.print("eul raw:"); Serial.print(" head: "); Serial.print(sEulData.head); Serial.print(" roll: "); Serial.print(sEulData.roll);  Serial.print(" pitch: "); Serial.println(sEulData.pitch);
  Serial.print("qua raw:"); Serial.print("w: "); Serial.print(sQuaData.w); printAxisData(sQuaData);
  Serial.println("========  raw data print end  ========");

  DFRobot_BNO055::sAxisAnalog_t   sAccAnalog, sMagAnalog, sGyrAnalog, sLiaAnalog, sGrvAnalog;
  DFRobot_BNO055::sEulAnalog_t    sEulAnalog;
  DFRobot_BNO055::sQuaAnalog_t    sQuaAnalog;
  sAccAnalog = bno055.getAxisAnalog(DFRobot_BNO055::eAxisAcc);
  sMagAnalog = bno055.getAxisAnalog(DFRobot_BNO055::eAxisMag);
  sGyrAnalog = bno055.getAxisAnalog(DFRobot_BNO055::eAxisGyr);
  sLiaAnalog = bno055.getAxisAnalog(DFRobot_BNO055::eAxisLia);
  sGrvAnalog = bno055.getAxisAnalog(DFRobot_BNO055::eAxisGrv);
  sEulAnalog = bno055.getEulAnalog();
  sQuaAnalog = bno055.getQuaAnalog();
  Serial.println();
  Serial.println("======== analog data print start ========");
  Serial.print("acc analog: (unit g/s^2)    "); printAxisData(sAccAnalog);
  Serial.print("mag analog: (unit ut)       "); printAxisData(sMagAnalog);
  Serial.print("gyr analog: (unit dps)      "); printAxisData(sGyrAnalog);
  Serial.print("lia analog: (unit g/s^2)    "); printAxisData(sLiaAnalog);
  Serial.print("grv analog: (unit g/s^2)    "); printAxisData(sGrvAnalog);
  Serial.print("eul analog: (unit degree)   "); Serial.print(" head: "); Serial.print(sEulAnalog.head); Serial.print(" roll: "); Serial.print(sEulAnalog.roll);  Serial.print(" pitch: "); Serial.println(sEulAnalog.pitch);
  Serial.print("qua analog: (no unit)       "); Serial.print("w: "); Serial.print(sQuaAnalog.w); printAxisData(sQuaAnalog);
  Serial.println("========  analog data print end  ========");

  delay(1000);
}
