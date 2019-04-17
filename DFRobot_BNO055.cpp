/*
 MIT License

 Copyright (C) <2019> <@DFRobot Frank>

　Permission is hereby granted, free of charge, to any person obtaining a copy of this
　software and associated documentation files (the "Software"), to deal in the Software
　without restriction, including without limitation the rights to use, copy, modify,
　merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
　permit persons to whom the Software is furnished to do so.

　The above copyright notice and this permission notice shall be included in all copies or
　substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
 PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
 FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
#include "DFRobot_BNO055.h"

// use data struct to map register address
const DFRobot_BNO055::sRegsPage0_t PROGMEM    sRegsPage0 = DFRobot_BNO055::sRegsPage0_t();
const DFRobot_BNO055::sRegsPage1_t PROGMEM    sRegsPage1 = DFRobot_BNO055::sRegsPage1_t();

#ifdef __AVR__
typedef uint16_t    platformBusWidth_t;
#else
typedef uint32_t    platformBusWidth_t;
#endif

// use regOffset0 to get register offset in reg page0, regOffset1 similar
const platformBusWidth_t    regsPage0Addr = (platformBusWidth_t) & sRegsPage0;
const platformBusWidth_t    regsPage1Addr = (platformBusWidth_t) & sRegsPage1;
#define regOffset0(reg) ( (platformBusWidth_t) (& (reg)) - regsPage0Addr )
#define regOffset1(reg) ( (platformBusWidth_t) (& (reg)) - regsPage1Addr )

#define __DBG   0
#if __DBG
# define __DBG_CODE(x)   Serial.print("__DBG_CODE: "); Serial.print(__FUNCTION__); Serial.print(" "); Serial.print(__LINE__); Serial.print(" "); x; Serial.println()
#else
# define __DBG_CODE(x)
#endif

#define writeRegBitsHelper(pageId, reg, flied, val) \
  setToPage(pageId); \
  writeRegBits(regOffset##pageId(reg), *(uint8_t*) &(flied), *(uint8_t*) &(val))

// main class start ----------------------------------------------------------------

DFRobot_BNO055::DFRobot_BNO055() { lastOperateStatus = eStatusOK; _currentPage = 0xff; }

DFRobot_BNO055::eStatus_t DFRobot_BNO055::begin()
{
  uint8_t   temp = getReg(regOffset0(sRegsPage0.CHIP_ID), 0);  // get chip id
  __DBG_CODE(Serial.print("CHIP_ID: "); Serial.print(temp, HEX));
  if((lastOperateStatus == eStatusOK) && (temp == BNO055_REG_CHIP_ID_DEFAULT)) {
    uint8_t   timeOut = 0;
    reset();
    do {
      temp = getReg(regOffset0(sRegsPage0.SYS_STATUS), 0);
      delay(10);
      timeOut ++;
    } while((temp != 0) && (timeOut < 50));
    if(timeOut == 50)
      lastOperateStatus = eStatusErrDeviceReadyTimeOut;
    else {
      setOprMode(eOprModeConfig);
      delay(50);
      setUnit();
      setAccRange(eAccRange_4G);
      setGyrRange(eGyrRange_2000);
      setPowerMode(ePowerModeNormal);
      setOprMode(eOprModeNdof);
      delay(50);
    }
  } else
    lastOperateStatus = eStatusErrDeviceNotDetect;
  return lastOperateStatus;
}

// get data functions ----------------------------------------------------------------

// get register offset of raw data
uint8_t getOffsetOfData(DFRobot_BNO055::eAxis_t eAxis)
{
  switch(eAxis) {
  case DFRobot_BNO055::eAxisAcc: return regOffset0(sRegsPage0.ACC_DATA);
  case DFRobot_BNO055::eAxisMag: return regOffset0(sRegsPage0.MAG_DATA);
  case DFRobot_BNO055::eAxisGyr: return regOffset0(sRegsPage0.GYR_DATA);
  case DFRobot_BNO055::eAxisLia: return regOffset0(sRegsPage0.LIA_DATA);
  case DFRobot_BNO055::eAxisGrv: return regOffset0(sRegsPage0.GRV_DATA);
  default: return 0;
  }
}

DFRobot_BNO055::sAxisAnalog_t DFRobot_BNO055::getAxis(eAxis_t eAxis)
{
  sAxisData_t   sRaw = getAxisRaw(eAxis);
  sAxisAnalog_t sAnalog = {0};
  float   factor = 1.0f;

  switch (eAxis) {
  // shift accelerometer, gravity vector and linear acceleration data: 1m/s2 = 100lsb, 1mg = 1lsb
  case eAxisAcc: factor = 1.0f; break;
  case eAxisLia: factor = 1.0f; break;
  case eAxisGrv: factor = 1.0f; break;
  // shift magnetometer data: 1ut = 16lsb
  case eAxisMag: factor = 16.0f; break;
  // shift gyroscope data: 1dps = 16lsb, 1rps = 900lsb
  case eAxisGyr: factor = 16.0f; break;
  default: lastOperateStatus = eStatusErrParameter; break;
  }
  sAnalog.x = sRaw.x / factor;
  sAnalog.y = sRaw.y / factor;
  sAnalog.z = sRaw.z / factor;
  return sAnalog;
}

DFRobot_BNO055::sEulAnalog_t DFRobot_BNO055::getEul()
{
  sEulData_t    sEul = getEulRaw();
  sEulAnalog_t  sEulAnalog = {0};
  // shift euler data: 1degree = 16lsb, 1radians = 900lsb
  sEulAnalog.head = sEul.head / 16.0f;
  sEulAnalog.roll = sEul.roll / 16.0f;
  sEulAnalog.pitch = sEul.pitch / 16.0f;
  return sEulAnalog;
}

DFRobot_BNO055::sQuaAnalog_t DFRobot_BNO055::getQua()
{
  sQuaData_t    sQua = getQuaRaw();
  sQuaAnalog_t  sQuaAnalog = {0};
  // shift quaternion data: 1qua = 2^14 lsb
  sQuaAnalog.w = sQua.w / 16384.0f;
  sQuaAnalog.x = sQua.x / 16384.0f;
  sQuaAnalog.y = sQua.y / 16384.0f;
  sQuaAnalog.z = sQua.z / 16384.0f;
  return sQuaAnalog;
}

// get register offset of offset
uint8_t getOffsetOfOffset(DFRobot_BNO055::eAxis_t eAxis)
{
  switch(eAxis) {
  case DFRobot_BNO055::eAxisAcc: return regOffset0(sRegsPage0.ACC_OFFSET);
  case DFRobot_BNO055::eAxisMag: return regOffset0(sRegsPage0.MAG_OFFSET);
  case DFRobot_BNO055::eAxisGyr: return regOffset0(sRegsPage0.GYR_OFFSET);
  default: return 0;
  }
}

// set data functions ----------------------------------------------------------------

void DFRobot_BNO055::setAxisOffset(eAxis_t eAxis, sAxisAnalog_t sOffset)
{
  uint8_t   offset = getOffsetOfOffset(eAxis);
  float     factor = 0;
  uint16_t  maxValue = 0;

  switch(eAxis) {
  // shift accelerometer data: 1m/s2 = 100lsb, 1mg = 1lsb
  case eAxisAcc: {
    factor = 1.0f;
    switch(_eAccRange) {
    case eAccRange_2G: maxValue = 2000; break;
    case eAccRange_4G: maxValue = 4000; break;
    case eAccRange_8G: maxValue = 8000; break;
    case eAccRange_16G: maxValue = 16000; break;
    }
  } break;
  // shift magnetometer data: 1ut = 16lsb
  case eAxisMag: factor = 16.0f; maxValue = 1300; break;
  // shift gyroscope data: 1dps = 16lsb, 1rps = 900lsb
  case eAxisGyr: {
    factor = 16.0f;
    switch(_eGyrRange) {
    case eGyrRange_2000: maxValue = 2000; break;
    case eGyrRange_1000: maxValue = 1000; break;
    case eGyrRange_500: maxValue = 500; break;
    case eGyrRange_250: maxValue = 250; break;
    case eGyrRange_125: maxValue = 125; break;
    }
  } break;
  default: lastOperateStatus = eStatusErrParameter; break;
  }
  if(eAxis == eAxisGyr) {
    if((offset == 0) || (abs(sOffset.x) > maxValue) || (abs(sOffset.y) > maxValue) || (abs(sOffset.z) > 2500)) {
      lastOperateStatus = eStatusErrParameter;
      return;
    }
  } else if((offset == 0) || (abs(sOffset.x) > maxValue) || (abs(sOffset.y) > maxValue) || (abs(sOffset.z) > maxValue)) {
    lastOperateStatus = eStatusErrParameter;
    return;
  }
  sAxisData_t   sAxisData;
  sAxisData.x = sOffset.x * factor;
  sAxisData.y = sOffset.y * factor;
  sAxisData.z = sOffset.z * factor;
  setToPage(0);
  writeReg(offset, (uint8_t*) &sAxisData, sizeof(sAxisData));
}

void DFRobot_BNO055::setOprMode(eOprMode_t eMode)
{
  sRegOprMode_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.mode = 0xff; sRegVal.mode = eMode;
  writeRegBitsHelper(0, sRegsPage0.OPR_MODE, sRegFlied, sRegVal);
  delay(50);    // wait before operate mode shift done
}

void DFRobot_BNO055::setPowerMode(ePowerMode_t eMode)
{
  sRegPowerMode_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.mode = 0xff; sRegVal.mode = eMode;
  writeRegBitsHelper(0, sRegsPage0.PWR_MODE, sRegFlied, sRegVal);
}

void DFRobot_BNO055::reset()
{
  sRegSysTrigger_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.RST_SYS = 0xff; sRegVal.RST_SYS = 1;
  writeRegBitsHelper(0, sRegsPage0.SYS_TRIGGER, sRegFlied, sRegVal);
}

void DFRobot_BNO055::setAccRange(eAccRange_t eRange)
{
  sRegAccConfig_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.ACC_RANGE = 0xff; sRegVal.ACC_RANGE = eRange;
  writeRegBitsHelper(1, sRegsPage1.ACC_CONFIG, sRegFlied, sRegVal);
  if(lastOperateStatus == eStatusOK)
    _eAccRange = eRange;
}

void DFRobot_BNO055::setAccBandWidth(eAccBandWidth_t eBand)
{
  sRegAccConfig_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.ACC_BW = 0xff; sRegVal.ACC_BW = eBand;
  writeRegBitsHelper(1, sRegsPage1.ACC_CONFIG, sRegFlied, sRegVal);
}

void DFRobot_BNO055::setAccPowerMode(eAccPowerMode_t eMode)
{
  sRegAccConfig_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.ACC_PWR_MODE = 0xff; sRegVal.ACC_PWR_MODE = eMode;
  writeRegBitsHelper(1, sRegsPage1.ACC_CONFIG, sRegFlied, sRegVal);
}

void DFRobot_BNO055::setMagDataRate(eMagDataRate_t eRate)
{
  sRegMagConfig_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.MAG_DATA_OUTPUT_RATE = 0xff; sRegVal.MAG_DATA_OUTPUT_RATE = eRate;
  writeRegBitsHelper(1, sRegsPage1.MAG_CONFIG, sRegFlied, sRegVal);
}

void DFRobot_BNO055::setMagOprMode(eMagOprMode_t eMode)
{
  sRegMagConfig_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.MAG_OPR_MODE = 0xff; sRegVal.MAG_OPR_MODE = eMode;
  writeRegBitsHelper(1, sRegsPage1.MAG_CONFIG, sRegFlied, sRegVal);
}

void DFRobot_BNO055::setMagPowerMode(eMagPowerMode_t eMode)
{
  sRegMagConfig_t   sRegFlied = {0}, sRegVal = {0};
  sRegFlied.MAG_POWER_MODE = 0xff; sRegVal.MAG_POWER_MODE = eMode;
  writeRegBitsHelper(1, sRegsPage1.MAG_CONFIG, sRegFlied, sRegVal);
}

void DFRobot_BNO055::setGyrRange(eGyrRange_t eRange)
{
  sRegGyrConfig0_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.GYR_RANGE = 0xff; sRegVal.GYR_RANGE = eRange;
  writeRegBitsHelper(1, sRegsPage1.GYR_CONFIG0, sRegFlied, sRegVal);
  if(lastOperateStatus == eStatusOK)
    _eGyrRange = eRange;

#if __DBG
  readReg(regOffset1(sRegsPage1.GYR_CONFIG0), (uint8_t*) &sRegFlied, sizeof(sRegFlied));
  __DBG_CODE(Serial.print("gyr range: "); Serial.print(sRegFlied.GYR_RANGE, HEX));
#endif
}

void DFRobot_BNO055::setGyrBandWidth(eGyrBandWidth_t eBandWidth)
{
  sRegGyrConfig0_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.GYR_BANDWIDTH = 0xff; sRegVal.GYR_BANDWIDTH = eBandWidth;
  writeRegBitsHelper(1, sRegsPage1.GYR_CONFIG0, sRegFlied, sRegVal);
}

void DFRobot_BNO055::setGyrPowerMode(eGyrPowerMode_t eMode)
{
  sRegGyrConfig1_t    sRegFlied = {0}, sRegVal = {0};
  sRegFlied.GYR_POWER_MODE = 0xff; sRegVal.GYR_POWER_MODE = eMode;
  writeRegBitsHelper(1, sRegsPage1.GYR_CONFIG1, sRegFlied, sRegVal);
}

uint8_t DFRobot_BNO055::getIntState()
{
  sRegIntSta_t    sInt;
  sRegSysTrigger_t    sTrigFlied, sTrigVal;
  setToPage(0);
  readReg(regOffset0(sRegsPage0.INT_STA), (uint8_t*) &sInt, sizeof(sInt));
  __DBG_CODE(Serial.print("int state: "); Serial.print(*(uint8_t*) &sInt, HEX));

  sTrigFlied.RST_INT = 0xff; sTrigVal.RST_INT = 1;
  writeRegBitsHelper(0, sRegsPage0.SYS_TRIGGER, sTrigFlied, sTrigVal);
  return *(uint8_t*) &sInt;
}

void DFRobot_BNO055::setIntMaskEnable(eInt_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.INT_MASK), (uint8_t*) &temp, sizeof(temp));
  temp |= eInt;
  writeReg(regOffset1(sRegsPage1.INT_MASK), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setIntMaskDisable(eInt_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.INT_MASK), (uint8_t*) &temp, sizeof(temp));
  temp &= ~ eInt;
  writeReg(regOffset1(sRegsPage1.INT_MASK), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setIntEnable(eInt_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.INT_EN), (uint8_t*) &temp, sizeof(temp));
  temp |= eInt;
  writeReg(regOffset1(sRegsPage1.INT_EN), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setIntDisable(eInt_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.INT_EN), (uint8_t*) &temp, sizeof(temp));
  temp &= ~ eInt;
  writeReg(regOffset1(sRegsPage1.INT_EN), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setAccAmThres(uint16_t thres)
{
  uint8_t   temp = mapAccThres(thres);
  if(lastOperateStatus != eStatusOK)
    return;
  writeReg(regOffset1(sRegsPage1.ACC_AM_THRES), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setAccIntAmDur(uint8_t dur)
{
  sRegAccIntSet_t    sRegFleid = {0}, sRegVal = {0};
  if((dur > 4) || (dur << 1)) {
    lastOperateStatus = eStatusErrParameter;
    return;
  }
  sRegFleid.AM_DUR = 0xff; sRegVal.AM_DUR = dur - 1;
  writeRegBitsHelper(1, sRegsPage1.ACC_INT_SETTINGS, sRegFleid, sRegVal);
}

void DFRobot_BNO055::setAccIntEnable(eAccIntSet_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.ACC_INT_SETTINGS), (uint8_t*) &temp, sizeof(temp));
  temp |= eInt;
  writeReg(regOffset1(sRegsPage1.ACC_INT_SETTINGS), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setAccIntDisable(eAccIntSet_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.ACC_INT_SETTINGS), (uint8_t*) &temp, sizeof(temp));
  temp &= ~ eInt;
  writeReg(regOffset1(sRegsPage1.ACC_INT_SETTINGS), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setAccHighGDuration(uint16_t dur)
{
  if((dur < 2) || (dur > 512)) {
    lastOperateStatus = eStatusErrParameter;
    return;
  }
  uint8_t   temp = dur / 2;
  setToPage(1);
  writeReg(regOffset1(sRegsPage1.ACC_HG_THRES), (uint8_t*) temp, sizeof(temp));
}

void DFRobot_BNO055::setAccHighGThres(uint16_t thres)
{
  uint8_t   temp = mapAccThres(thres);
  if(lastOperateStatus != eStatusOK)
    return;
  writeReg(regOffset1(sRegsPage1.ACC_HG_THRES), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setAccNmThres(uint16_t thres)
{
  uint8_t   temp = mapAccThres(thres);
  if(lastOperateStatus != eStatusOK)
    return;
  writeReg(regOffset1(sRegsPage1.ACC_NM_THRES), (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setAccNmSet(eAccNmSmnm_t eSmnm, uint16_t dur)
{
  sRegAccNmSet_t    sReg;
  uint8_t   temp = 0;
  if(dur > 336) {
    lastOperateStatus = eStatusErrParameter;
    return;
  }
  sReg.SMNM = eSmnm;
  /*
   * more detail in datasheet page 85
   *
   * ACC_NM_SET
   * -----------------------------------------
   * | b7 | b6 | b5 | b4 | b3 | b2 | b1 | b0 |
   * -----------------------------------------
   * |    |  slo_no_mot_dur <5:0>       |Smnm|
   * -----------------------------------------
   *
   * slow / no motion duration = snmd (unit seconds)
   * if slo_no_mot_dur<5:4> == 0b00, then snmd = slo_no_mot_dur<3:0> + 1
   * if slo_no_mot_dur<5:4> == 0b01, then snmd = slo_no_mot_dur<3:0> * 4 + 20
   * if slo_no_mot_dur<5:5> == 0b1, then snmd = slo_no_mot_dur<4:0> * 8 + 88
   */
  if(dur < 17) {
    temp = dur - 1;
  } else if(dur < 80) {
    temp |= (0x01 << 4);
    if(dur > 20)
      temp |= (dur - 20) / 4;
  } else {
    temp |= (0x01 << 5);
    if(dur > 88)
      temp |= (dur - 88) / 8;
  }
  sReg.NO_SLOW_MOTION_DURATION = temp;
  setToPage(1);
  writeReg(regOffset1(sRegsPage1.ACC_NM_SET), (uint8_t*) &sReg, sizeof(sReg));
}

void DFRobot_BNO055::setGyrIntEnable(eGyrIntSet_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.GYR_INT_SETTING), (uint8_t*) temp, sizeof(temp));
  temp |= eInt;
  writeReg(regOffset1(sRegsPage1.GYR_INT_SETTING), (uint8_t*) temp, sizeof(temp));
}

void DFRobot_BNO055::setGyrIntDisable(eGyrIntSet_t eInt)
{
  uint8_t   temp;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.GYR_INT_SETTING), (uint8_t*) temp, sizeof(temp));
  temp &= ~ eInt;
  writeReg(regOffset1(sRegsPage1.GYR_INT_SETTING), (uint8_t*) temp, sizeof(temp));
}

void DFRobot_BNO055::setGyrHrSet(eSingleAxis_t eSingleAxis, uint16_t thres, uint16_t dur)
{
  uint8_t   hysteresis = 1;
  mapGyrHrThres(&hysteresis, &thres, &dur);
  hysteresis = 0;   // function not yet understood, temporarily used 1
  if(lastOperateStatus != eStatusOK)
    return;
  uint8_t   reg = regOffset1(sRegsPage1.GYR_HR_X_SET);    // get reg offset head
  reg += eSingleAxis * 2;                                 // calculate offset
  sRegGyrHrSet_t    sReg;
  uint8_t   temp = dur;
  sReg.HR_THRESHOLD = thres;
  sReg.HR_THRES_HYST = hysteresis;
  __DBG_CODE(Serial.print("thresHold: "); Serial.print(sReg.HR_THRESHOLD); Serial.print(" reg addr: "); Serial.print(reg, HEX));
  writeReg(reg, (uint8_t*) &sReg, sizeof(sReg));
  writeReg(reg + 1, (uint8_t*) &temp, sizeof(temp));
}

void DFRobot_BNO055::setGyrAmThres(uint8_t thres)
{
  mapGyrAmThres(&thres);
  if(lastOperateStatus != eStatusOK)
    return;
  setToPage(1);
  writeReg(regOffset1(sRegsPage1.GYR_AM_THRES), (uint8_t*) &thres, sizeof(thres));
}

// protected functions ----------------------------------------------------------------

uint8_t DFRobot_BNO055::getReg(uint8_t reg, uint8_t pageId)
{
  uint8_t   temp;
  setToPage(pageId);
  readReg(reg, &temp, sizeof(temp));
  return temp;
}

void DFRobot_BNO055::setToPage(uint8_t pageId)
{
  if(_currentPage != pageId) {
    writeReg(regOffset0(sRegsPage0.PAGE_ID), &pageId, sizeof(pageId));
    if(lastOperateStatus == eStatusOK) {
      _currentPage = pageId;
    }
  }
}

void DFRobot_BNO055::setUnit()
{
  sRegUnitSel_t   sReg;
  sReg.ACC = 1;   // 0: m/s^2, 1: mg
  sReg.EUL = 0;   // 0: degrees, 1: radians
  sReg.GYR = 0;   // 0: dps, 1: rps
  sReg.ORI_ANDROID_WINDOWS = 0;   // 0: windows, 1: android
  sReg.TEMP = 0;  // 0: celsius, 1: fahrenheit
  setToPage(0);
  writeReg(regOffset0(sRegsPage0.UNIT_SEL), (uint8_t*) &sReg, sizeof(sReg));
}

void DFRobot_BNO055::writeRegBits(uint8_t reg, uint8_t flied, uint8_t val)
{
  uint8_t   regVal;
  readReg(reg, &regVal, sizeof(regVal));
  regVal &= ~flied;
  regVal |= val;
  writeReg(reg, &regVal, sizeof(regVal));
}

/*
 * value is dependent on accelerometer range selected
 * --------------------------
 * |  range  |  1lsb = ?mg  |
 * --------------------------
 * |  2g     |  3.91        |
 * |  4g     |  7.81        |
 * |  8g     |  15.63       |
 * |  16g    |  31.25       |
 * --------------------------
 */
uint16_t DFRobot_BNO055::mapAccThres(uint16_t thres)
{
  sRegAccConfig_t   sReg;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.ACC_CONFIG), (uint8_t*) &sReg, sizeof(sReg));
  if(lastOperateStatus != eStatusOK)
    return 0;
  switch(sReg.ACC_RANGE) {
  case eAccRange_2G: {
    if(thres > (255.0f * 3.91f)) { lastOperateStatus = eStatusErrParameter; }
    else { thres /= 3.91f; }
  } break;
  case eAccRange_4G: {
    if(thres > (255.0f * 7.81f)) { lastOperateStatus = eStatusErrParameter; }
    else { thres /= 7.81f; }
  } break;
  case eAccRange_8G: {
    if(thres > (255.0f * 15.63f)) { lastOperateStatus = eStatusErrParameter; }
    else { thres /= 15.63f; }
  } break;
  case eAccRange_16G: {
    if(thres > (255.0f * 31.25f)) { lastOperateStatus = eStatusErrParameter; }
    else { thres /= 31.25f; }
  } break;
  default: lastOperateStatus = eStatusErrParameter; break;
  }
  if(lastOperateStatus == eStatusErrParameter)
    return 0;
  return thres;
}

/*
 * hysteresis value is dependent on gyroscope range selected
 * -----------------------------------------
 * |  range  |  1lsb = ? degree / seconds  |
 * -----------------------------------------
 * |  2000   |  62.26                      |
 * |  1000   |  31.13                      |
 * |  500    |  15.56                      |
 * |  250    |  7.78                       |
 * |  125    |  3.89                       |
 * -----------------------------------------
 *
 * threshold value is dependent on gyroscope range selected
 * -----------------------------------------
 * |  range  |  1lsb = ? degree / seconds  |
 * -----------------------------------------
 * |  2000   |  62.5                       |
 * |  1000   |  31.25                      |
 * |  500    |  15.625                     |
 * |  250    |  7.8125                     |
 * |  125    |  3.90625                    |
 * -----------------------------------------
 *
 * High rate duration to set, unit ms, duration from 2.5ms to 640ms
 */
void DFRobot_BNO055::mapGyrHrThres(uint8_t *pHysteresis, uint16_t *pThres, uint16_t *pDur)
{
  sRegGyrConfig0_t    sReg;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.GYR_CONFIG0), (uint8_t*) &sReg, sizeof(sReg));
  if(lastOperateStatus != eStatusOK)
    return;
  if((*pDur < 3) || (*pDur > 640)) {
    lastOperateStatus = eStatusErrParameter;
    return;
  }
  switch(sReg.GYR_RANGE) {
  case eGyrRange_2000: {
    if((*pHysteresis > (62.26f * 3.0f)) || (*pThres > (62.5f * 31.0f)))
      lastOperateStatus = eStatusErrParameter;
    else {
      *pHysteresis /= 62.26f;
      *pThres /= 62.5f;
    }
  } break;
  case eGyrRange_1000: {
    if((*pHysteresis > (31.13f * 3.0f)) || (*pThres > (31.25f * 31.0f)))
      lastOperateStatus = eStatusErrParameter;
    else {
      *pHysteresis /= 31.13f;
      *pThres /= 31.25f;
    }
  } break;
  case eGyrRange_500: {
    if((*pHysteresis > (15.56f * 3.0f)) || (*pThres > (15.625f * 31.0f)))
      lastOperateStatus = eStatusErrParameter;
    else {
      *pHysteresis /= 15.56f;
      *pThres /= 3.0f;
    }
  } break;
  case eGyrRange_250: {
    if((*pHysteresis > (7.78f * 3.0f)) || (*pThres > (7.8125f * 31.0f)))
      lastOperateStatus = eStatusErrParameter;
    else {
      *pHysteresis /= 7.78f;
      *pThres /= 7.8125f;
    }
  } break;
  case eGyrRange_125: {
    if((*pHysteresis > (3.89f * 3.0f)) || (*pThres > (3.90625f * 31.0f)))
      lastOperateStatus = eStatusErrParameter;
    else {
      *pHysteresis /= 3.89f;
      *pThres /= 3.0f;
    }
  } break;
  default: lastOperateStatus = eStatusErrParameter; break;
  }
  if(lastOperateStatus != eStatusErrParameter)
    *pDur /= 2.5f;
}

/*
 * threshold value is dependent on gyroscope range selected
 * -----------------------------------------
 * |  range  |  1lsb = ? degree / seconds  |
 * -----------------------------------------
 * |  2000   |  1                          |
 * |  1000   |  0.5                        |
 * |  500    |  0.25                       |
 * |  250    |  0.125                      |
 * |  125    |  0.0625                     |
 * -----------------------------------------
 */
void DFRobot_BNO055::mapGyrAmThres(uint8_t *pThres)
{
  sRegGyrConfig0_t    sReg;
  setToPage(1);
  readReg(regOffset1(sRegsPage1.GYR_CONFIG0), (uint8_t*) &sReg, sizeof(sReg));
  if(lastOperateStatus != eStatusOK)
    return;
  switch(sReg.GYR_RANGE) {
  case eGyrRange_2000: if(*pThres < (128.0f * 1.0f)) { *pThres /= 1.0f; } break;
  case eGyrRange_1000: if(*pThres < (128.0f * 0.5f)) { *pThres /= 0.5f; } break;
  case eGyrRange_500: if(*pThres < (128.0f * 0.25f)) { *pThres /= 0.25f; } break;
  case eGyrRange_250: if(*pThres < (128.0f * 0.125f)) { *pThres /= 0.125f; } break;
  case eGyrRange_125: if(*pThres < (128.0f * 0.0625f)) { *pThres /= 0.625f; } break;
  }
}

DFRobot_BNO055::sAxisData_t DFRobot_BNO055::getAxisRaw(eAxis_t eAxis)
{
  uint8_t   offset = getOffsetOfData(eAxis);
  sAxisData_t   sAxis = {0};
  setToPage(0);
  if(offset == 0)
    lastOperateStatus = eStatusErrParameter;
  else
    readReg(offset, (uint8_t*) &sAxis, sizeof(sAxis));
  return sAxis;
}

DFRobot_BNO055::sEulData_t DFRobot_BNO055::getEulRaw()
{
  sEulData_t    sEul = {0};
  setToPage(0);
  readReg(regOffset0(sRegsPage0.EUL_DATA), (uint8_t*) &sEul, sizeof(sEul));
  return sEul;
}

DFRobot_BNO055::sQuaData_t DFRobot_BNO055::getQuaRaw()
{
  sQuaData_t    sQua;
  setToPage(0);
  readReg(regOffset0(sRegsPage0.QUA_DATA), (uint8_t*) &sQua, sizeof(sQua));
  return sQua;
}

// main class end ----------------------------------------------------------------

// utils class start ----------------------------------------------------------------

DFRobot_BNO055_IIC::DFRobot_BNO055_IIC(TwoWire *pWire, uint8_t addr)
{
  _pWire = pWire;
  _addr = addr;
}

void DFRobot_BNO055_IIC::readReg(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetect;
  _pWire->begin();
  _pWire->beginTransmission(_addr);
  _pWire->write(reg);
  if(_pWire->endTransmission() != 0)
    return;

  _pWire->requestFrom(_addr, len);
  for(uint8_t i = 0; i < len; i ++)
    pBuf[i] = _pWire->read();
  lastOperateStatus = eStatusOK;
}

void DFRobot_BNO055_IIC::writeReg(uint8_t reg, uint8_t *pBuf, uint8_t len)
{
  lastOperateStatus = eStatusErrDeviceNotDetect;
  _pWire->begin();
  _pWire->beginTransmission(_addr);
  _pWire->write(reg);
  for(uint8_t i = 0; i < len; i ++)
    _pWire->write(pBuf[i]);
  if(_pWire->endTransmission() != 0)
    return;
  lastOperateStatus = eStatusOK;
}

// utils class end ----------------------------------------------------------------
