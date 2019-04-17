/*!
  * interrupt.ino
  *
  * Download this demo to test bno055 interrupt
  * Connect bno055 int pin to arduino pin 2
  * If there occurs interrupt, it will printr on you serial monitor, more detail please reference comment
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

bool intFlag = false;

void intHandle()
{
  intFlag = true;
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

  bno.setOprMode(BNO::eOprModeConfig);    // set to config mode

  bno.setIntMaskEnable(BNO::eIntAll);    // set interrupt mask enable, signal to int pin when interrupt
  // bno.setIntMaskDisable(BNO::eIntAccAm | BNO::eIntAccNm);    // set interrupt mask disable, no signal to int pin when interrupt

  bno.setIntEnable(BNO::eIntAll);   // set interrupt enable
  // bno.setIntDisable(BNO::eIntAccAm | BNO::eIntAccNm);    // set interrupt disable

  bno.setAccIntEnable(BNO::eAccIntSetAll);    // set accelerometer interrupt enable
  // bno.setAccIntDisable(BNO::eAccIntSetAmnmXAxis | BNO::eAccIntSetHgXAxis);   // set accelerometer interrupt disable

  /* accelerometer any motion threshold to set, unit mg, value is dependent on accelerometer range selected,
   * case 2g, no more than 1991
   * case 4g, no more than 3985
   * case 8g, no more than 7968
   * case 16g, no more than 15937
   * attenion: The set value will be slightly biased according to datasheet
   * tips: default accelerometer range is 4g
   */
  // how to trig this: still --> fast move
  bno.setAccAmThres(200);
  // any motion interrupt triggers if duration consecutive data points are above the any motion interrupt
  // threshold define in any motion threshold
  bno.setAccIntAmDur(1);
  // set high-g duration, value from 2ms to 512ms
  bno.setAccHighGDuration(80);
  /*
   * accelerometer high-g threshold to set, unit mg, value is dependent on accelerometer range selected,
   * case 2g, no more than 1991
   * case 4g, no more than 3985
   * case 8g, no more than 7968
   * case 16g, no more than 15937
   * Attenion: The set value will be slightly biased according to datasheet
   */
  // how to trig this: still --> (very) fast move
  bno.setAccHighGThres(900);
  // accelerometer (no motion) / (slow motion) settings, 2nd parameter unit seconds, no more than 344
  bno.setAccNmSet(BNO::eAccNmSmnmNm, 4);
  /*
   * accelerometer no motion threshold to set, unit mg, value is dependent on accelerometer range selected,
   * case 2g, no more than 1991
   * case 4g, no more than 3985
   * case 8g, no more than 7968
   * case 16g, no more than 15937
   * Attenion: The set value will be slightly biased according to datasheet
   */
  // hot to trig this: any motion --> still --> still
  bno.setAccNmThres(100);

  bno.setGyrIntEnable((BNO::eGyrIntSet_t) (BNO::eGyrIntSetHrXAxis | BNO::eGyrIntSetHrYAxis | BNO::eGyrIntSetHrZAxis));    // set gyroscope interrupt enable, in most cases, this is enough.
  // bno.setGyrIntEnable(BNO::eGyrIntSetAmYAxis | BNO::eGyrIntSetAmYAxis | BNO::eGyrIntSetAmZAxis);    // set gyroscope interrupt enable
  // bno.setGyrIntDisable(BNO::eGyrIntSetHrXAxis | BNO::eGyrIntSetAmXAxis);    // set gyroscope interrupt disable

  /*
   * 2nd parameter, high rate threshold to set, unit degree/seconds, value is dependent on gyroscope range selected,
   * case 2000, no more than 1937
   * case 1000, no more than 968
   * case 500, no more than 484
   * case 250, no more than 242
   * case 125, no more than 121
   * Attenion: The set value will be slightly biased according to datasheet
   * 3rd parameter, high rate duration to set, unit ms, duration from 2.5ms to 640ms
   * Attenion: The set value will be slightly biased according to datasheet
   */
  // how to trigger this: still --> fast tilt
  bno.setGyrHrSet(BNO::eSingleAxisX, 300, 80);
  bno.setGyrHrSet(BNO::eSingleAxisY, 300, 80);
  bno.setGyrHrSet(BNO::eSingleAxisZ, 300, 80);
  /*
   * gyroscope any motion threshold to set, unit mg, value is dependent on accelerometer range selected,
   * case 2000, no more than 128
   * case 1000, no more than 64
   * case 500, no more than 32
   * case 250, no more than 16
   * case 125, no more than 8
   * Attenion: The set value will be slightly biased according to datasheet
   * tips: default range is 2000
   */
  // how to trigger this: still --> fast tilt
  bno.setGyrAmThres(20);

  bno.setOprMode(BNO::eOprModeNdof);    // configure done

  attachInterrupt(0, intHandle, RISING);   // attach interrupt
  bno.getIntState();    // clear unexpected interrupt
  intFlag = false;
}

void loop()
{
  if(intFlag) {
    intFlag = false;
    uint8_t   intSta = bno.getIntState();   // interrupt auto clear after read

    Serial.println("interrupt detected");
    if(intSta & BNO::eIntAccAm)
      Serial.println("accelerometer any motion detected");
    if(intSta & BNO::eIntAccNm)
      Serial.println("accelerometer no motion detected");
    if(intSta & BNO::eIntAccHighG)
      Serial.println("acceleromter high-g detected");
    if(intSta & BNO::eIntGyrHighRate)
      Serial.println("gyroscope high rate detected");
    if(intSta & BNO::eIntGyrAm)
      Serial.println("gyroscope any motion detected");
  }
}
