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

#ifndef DFROBOT_BNO055_H
#define DFROBOT_BNO055_H

#include "Arduino.h"
#include "Wire.h"

#ifndef PROGMEM
#define PROGMEM
#endif

// main class ----------------------------------------------------------------
class DFRobot_BNO055 {

// defines
public:
  /**
   * @brief global axis declare (excepet eular and quaternion)
   */
  typedef enum {
    eAxisAcc,
    eAxisMag,
    eAxisGyr,
    eAxisLia,
    eAxisGrv
  } eAxis_t;

  /**
   * @brief global single axis declare
   */
  typedef enum {
    eSingleAxisX,
    eSingleAxisY,
    eSingleAxisZ
  } eSingleAxis_t;

  // registers ----------------------------------------------------------------
  typedef struct {
    uint8_t   MAG: 2;
    uint8_t   ACC: 2;
    uint8_t   GYR: 2;
    uint8_t   SYS: 2;
  } sRegCalibState_t;

  typedef enum {
    eStResultFaild,
    eStResultPassed
  } eStResult_t;

  typedef struct {
    uint8_t   ACC: 1;
    uint8_t   MAG: 1;
    uint8_t   GYR: 1;
    uint8_t   MCU: 1;
  } sRegStResult_t;

  /**
   * @brief enum interrupt
   */
  typedef enum {
    eIntGyrAm = 0x04,
    eIntGyrHighRate = 0x08,
    eIntAccHighG = 0x20,
    eIntAccAm = 0x40,
    eIntAccNm = 0x80,
    eIntAll = 0xec
  } eInt_t;

  typedef struct {
    uint8_t   reserved1: 2;
    uint8_t   GYRO_AM: 1;
    uint8_t   HYR_HIGH_RATE: 1;
    uint8_t   reserved2: 1;
    uint8_t   ACC_HIGH_G: 1;
    uint8_t   ACC_AM: 1;
    uint8_t   ACC_NM: 1;
  } sRegIntSta_t;

  typedef struct {
    uint8_t   ST_MAIN_CLK: 1;
  } sRegSysClkStatus_t;

  typedef struct {
    uint8_t   ACC: 1;
    uint8_t   GYR: 1;
    uint8_t   EUL: 1;
    uint8_t   reserved1: 1;
    uint8_t   TEMP: 1;
    uint8_t   reserved2: 2;
    uint8_t   ORI_ANDROID_WINDOWS: 1;
  } sRegUnitSel_t;

  /**
   * @brief Operation mode enum
   */
  typedef enum {
    eOprModeConfig,
    eOprModeAccOnly,
    eOprModeMagOnly,
    eOprModeGyroOnly,
    eOprModeAccMag,
    eOprModeAccGyro,
    eOprModeMagGyro,
    eOprModeAMG,
    eOprModeImu,
    eOprModeCompass,
    eOprModeM4G,
    eOprModeNdofFmcOff,
    eOprModeNdof
  } eOprMode_t;

  typedef struct {
    uint8_t   mode: 4;
  } sRegOprMode_t;

  /**
   * @brief Poewr mode enum
   */
  typedef enum {
    ePowerModeNormal,
    ePowerModeLowPower,
    ePowerModeSuspend
  } ePowerMode_t;

  typedef struct {
    uint8_t   mode: 2;
  } sRegPowerMode_t;

  typedef struct {
    uint8_t   SELF_TEST: 1;
    uint8_t   reserved1: 4;
    uint8_t   RST_SYS: 1;
    uint8_t   RST_INT: 1;
    uint8_t   CLK_SEL: 1;
  } sRegSysTrigger_t;

  typedef struct {
    uint8_t   TEMP_SOURCE: 2;
  } sRegTempSource_t;

  typedef struct {
    uint8_t   remappedXAxisVal: 2;
    uint8_t   remappedYAxisVal: 2;
    uint8_t   remappedZAxisVal: 2;
  } sRegAxisMapConfig_t;

  typedef struct {
    uint8_t   remappedZAxisSign: 1;
    uint8_t   remappedYAxisSign: 1;
    uint8_t   remappedXAxisSign: 1;
  } sRegAxisMapSign_t;

  typedef struct {
    int16_t   x, y, z;
  } sAxisData_t;

  /**
   * @brief axis analog data struct
   */
  typedef struct {
    float   x, y, z;
  } sAxisAnalog_t;

  /**
   * @brief eular analog data struct
   */
  typedef struct {
    float   head, roll, pitch;
  } sEulAnalog_t;

  typedef struct {
    int16_t   head, roll, pitch;
  } sEulData_t;

  /**
   * @brief qua analog data struct
   */
  typedef struct {
    float   w, x, y, z;
  } sQuaAnalog_t;

  typedef struct {
    int16_t   w, x, y, z;
  } sQuaData_t;

  typedef struct {
    uint8_t   CHIP_ID;  // 0x00
    #define   BNO055_REG_CHIP_ID_DEFAULT   0xa0
    uint8_t   ACC_ID;
    #define   BNO055_REG_ACC_ID_DEFAULT    0xfb
    uint8_t   MAG_ID;
    #define   BNO055_REG_MAG_ID_DEFAULT    0x32
    uint8_t   GYR_ID;
    #define   BNO055_REG_GYR_ID_DEFAULT    0x0f
    uint16_t  SW_REV_ID;
    #define   BNO055_REG_SW_REV_ID_DEFAULT 0x0308
    uint8_t   BL_REV;
    uint8_t   PAGE_ID;
    sAxisData_t   ACC_DATA;
    sAxisData_t   MAG_DATA;  // 0x0f
    sAxisData_t   GYR_DATA;
    sEulData_t    EUL_DATA;
    sQuaData_t    QUA_DATA;  // 0x20
    sAxisData_t   LIA_DATA;
    sAxisData_t   GRV_DATA;  // 0x2f
    uint8_t   TEMP;
    sRegCalibState_t    CALIB_STATE;
    sRegStResult_t      ST_RESULT;
    sRegIntSta_t        INT_STA;
    sRegSysClkStatus_t    SYS_CLK_STATUS;
    uint8_t   SYS_STATUS;
    uint8_t   SYS_ERR;
    sRegUnitSel_t       UNIT_SEL;
    uint8_t   reserved1;
    sRegOprMode_t       OPR_MODE;
    sRegPowerMode_t     PWR_MODE;
    sRegSysTrigger_t    SYS_TRIGGER;
    sRegTempSource_t    TEMP_SOURCE;  // 0x40
    sRegAxisMapConfig_t AXIS_MAP_CONFIG;
    sRegAxisMapSign_t   AXIS_MAP_SIGN;
    uint8_t   reserved2[(0x54 - 0x43 + 1)];
    sAxisData_t   ACC_OFFSET;  // 0x55
    sAxisData_t   MAG_OFFSET;
    sAxisData_t   GYR_OFFSET;  // 0x61
    uint16_t  ACC_RADIUS;
    uint16_t  MAG_RADIUS;
  } sRegsPage0_t;

  /**
   * @brief enum accelerometer range, unit G
   */
  typedef enum {
    eAccRange_2G,
    eAccRange_4G,
    eAccRange_8G,
    eAccRange_16G
  } eAccRange_t;

  /**
   * @brief enum accelerometer band width, unit HZ
   */
  typedef enum {
    eAccBandWidth_7_81,    // 7.81HZ
    eAccBandWidth_15_63,   // 16.63HZ
    eAccBandWidth_31_25,
    eAccBandWidth_62_5,
    eAccBandWidth_125,
    eAccBandWidth_250,
    eAccBandWidth_500,
    eAccBandWidth_1000
  } eAccBandWidth_t;

  /**
   * @brief enum accelerometer power mode
   */
  typedef enum {
    eAccPowerModeNormal,
    eAccPowerModeSuspend,
    eAccPowerModeLowPower1,
    eAccPowerModeStandby,
    eAccPowerModeLowPower2,
    eAccPowerModeDeepSuspend
  } eAccPowerMode_t;

  typedef struct {
    uint8_t   ACC_RANGE: 2;
    uint8_t   ACC_BW: 3;
    uint8_t   ACC_PWR_MODE: 3;
  } sRegAccConfig_t;

  /**
   * @brief enum magnetometer data output rate, unit HZ
   */
  typedef enum {
    eMagDataRate_2,
    eMagDataRate_6,
    eMagDataRate_8,
    eMagDataRate_10,
    eMagDataRate_15,
    eMagDataRate_20,
    eMagDataRate_25,
    eMagDataRate_30
  } eMagDataRate_t;

  /**
   * @brief enum magnetometer operation mode
   */
  typedef enum {
    eMagOprModeLowPower,
    eMagOprModeRegular,
    eMagOprModeEnhancedRegular,
    eMagOprModeHighAccuracy
  } eMagOprMode_t;

  /**
   * @brief enum magnetometer power mode
   */
  typedef enum {
    eMagPowerModeNormal,
    eMagPowerModeSleep,
    eMagPowerModeSuspend,
    eMagPowerModeForce
  } eMagPowerMode_t;

  typedef struct {
    uint8_t   MAG_DATA_OUTPUT_RATE: 3;
    uint8_t   MAG_OPR_MODE: 2;
    uint8_t   MAG_POWER_MODE: 2;
  } sRegMagConfig_t;

  /**
   * @brief enum gyroscope range, unit dps
   */
  typedef enum {
    eGyrRange_2000,
    eGyrRange_1000,
    eGyrRange_500,
    eGyrRange_250,
    eGyrRange_125
  } eGyrRange_t;

  /**
   * @brief enum gyroscope band width, unit HZ
   */
  typedef enum {
    eGyrBandWidth_523,
    eGyrBandWidth_230,
    eGyrBandWidth_116,
    eGyrBandWidth_47,
    eGyrBandWidth_23,
    eGyrBandWidth_12,
    eGyrBandWidth_64,
    eGyrBandWidth_32
  } eGyrBandWidth_t;

  typedef struct {
    uint8_t   GYR_RANGE: 3;
    uint8_t   GYR_BANDWIDTH: 3;
  } sRegGyrConfig0_t;

  /**
   * @brief enum gyroscope power mode
   */
  typedef enum {
    eGyrPowerModeNormal,
    eGyrPowerModeFastPowerUp,
    eGyrPowerModeDeepSuspend,
    eGyrPowerModeSuspend,
    eGyrPowerModeAdvancedPowersave
  } eGyrPowerMode_t;

  typedef struct {
    uint8_t   GYR_POWER_MODE: 3;
  } sRegGyrConfig1_t;

  typedef enum {
    eAccSleepModeEventDriven,
    eAccSleepModeEquidstantSampling
  } eAccSleepMode_t;

  typedef enum {
    eAccSleepDuration_0_5 = 5,    // 0.5 ms
    eAccSleepDuration_1,
    eAccSleepDuration_2,
    eAccSleepDuration_4,
    eAccSleepDuration_6,
    eAccSleepDuration_10,
    eAccSleepDuration_25,
    eAccSleepDuration_50,
    eAccSleepDuration_100,
    eAccSleepDuration_500,
    eAccSleepDuration_1000
  } eAccSleepDuration_t;

  typedef struct {
    uint8_t   SLP_MODE: 1;
    uint8_t   SLP_DURATION: 4;
  } sRegAccSleepConfig_t;

  typedef enum {
    eGyrSleepDuration_2,
    eGyrSleepDuration_4,
    eGyrSleepDuration_5,
    eGyrSleepDuration_8,
    eGyrSleepDuration_10,
    eGyrSleepDuration_15,
    eGyrSleepDuration_18,
    eGyrSleepDuration_20
  } eGyrSleepDuration_t;

  typedef enum {
    eGyrAutoSleepDuration_No,
    eGyrAutoSleepDuration_4,
    eGyrAutoSleepDuration_5,
    eGyrAutoSleepDuration_8,
    eGyrAutoSleepDuration_10,
    eGyrAutoSleepDuration_15,
    eGyrAutoSleepDuration_20,
    eGyrAutoSleepDuration_40
  } eGyrAutoSleepDuration_t;

  typedef struct {
    uint8_t   SLP_DURATION: 3;
    uint8_t   AUTO_SLP_DURATION: 3;
  } sRegGyrSleepConfig_t;

  typedef struct {
    uint8_t   reserved1: 2;
    uint8_t   GYRO_AM: 1;
    uint8_t   GYR_HIGH_RATE: 1;
    uint8_t   reserved2: 1;
    uint8_t   ACC_HIGH_G: 1;
    uint8_t   ACC_AM: 1;
    uint8_t   ACC_NM: 1;
  } sRegIntMask_t;

  typedef struct {
    uint8_t   reserved1: 2;
    uint8_t   GYRO_AM: 1;
    uint8_t   GYR_HIGH_RATE: 1;
    uint8_t   reserved2: 1;
    uint8_t   ACC_HIGH_G: 1;
    uint8_t   ACC_AM: 1;
    uint8_t   ACC_NM: 1;
  } sRegIntEn_t;

  /**
   * @brief Enum accelerometer interrupt settings
   */
  typedef enum {
    eAccIntSetAmnmXAxis = (0x01 << 2),
    eAccIntSetAmnmYAxis = (0x01 << 3),
    eAccIntSetAmnmZAxis = (0x01 << 4),
    eAccIntSetHgXAxis = (0x01 << 5),
    eAccIntSetHgYAxis = (0x01 << 6),
    eAccIntSetHgZAxis = (0x01 << 7),
    eAccIntSetAll = 0xfc
  } eAccIntSet_t;

  typedef struct {
    uint8_t   AM_DUR: 2;
    uint8_t   AMNM_X_AXIS: 1;
    uint8_t   AMNM_Y_AXIS: 1;
    uint8_t   AMNM_Z_AXIS: 1;
    uint8_t   HG_X_AXIS: 1;
    uint8_t   HG_Y_AXIS: 1;
    uint8_t   HG_Z_AXIS: 1;
  } sRegAccIntSet_t;

  /**
   * @brief Enum accelerometer slow motion mode or no motion mode
   */
  typedef enum {
    eAccNmSmnmSm,  // slow motion mode
    eAccNmSmnmNm   // no motion mode
  } eAccNmSmnm_t;

  typedef struct {
    uint8_t   SMNM: 2;
    uint8_t   NO_SLOW_MOTION_DURATION: 5;
  } sRegAccNmSet_t;

  /**
   * @brief Enum gyroscope interrupt settings
   */
  typedef enum {
    eGyrIntSetAmXAxis = (0x01 << 0),
    eGyrIntSetAmYAxis = (0x01 << 1),
    eGyrIntSetAmZAxis = (0x01 << 2),
    eGyrIntSetHrXAxis = (0x01 << 3),
    eGyrIntSetHrYAxis = (0x01 << 4),
    eGyrIntSetHrZAxis = (0x01 << 5),
    eGyrIntSetAmFilt = (0x01 << 6),
    eGyrIntSetHrFilt = (0x01 << 7),
    eGyrIntSetAll = 0x3f
  } eGyrIntSet_t;

  typedef struct {
    uint8_t   AM_X_AXIS: 1;
    uint8_t   AM_Y_AXIS: 1;
    uint8_t   AM_Z_AXIS: 1;
    uint8_t   HR_X_AXIS: 1;
    uint8_t   HR_Y_AXIS: 1;
    uint8_t   HR_Z_AXIS: 1;
    uint8_t   AM_FILT: 1;
    uint8_t   HR_FILT: 1;
  } sRegGyrIntSetting_t;

  typedef struct {
    uint8_t   HR_THRESHOLD: 5;
    uint8_t   HR_THRES_HYST: 2;
  } sRegGyrHrSet_t;

  typedef struct {
    uint8_t   GYRO_ANY_MOTION_THRESHOLD: 7;
  } sRegGyrAmThres_t;

  typedef struct {
    uint8_t   SLOPE_SAMPLES: 2;
    uint8_t   AWAKE_DURATION: 2;
  } sRegGyrAmSet_t;

  typedef uint8_t   UniqueId_t[(0x5f - 0x50 + 1)];

  typedef struct {
    uint8_t   reserved1[(0x06 - 0x00 + 1)];
    uint8_t   PAGE_ID;  // 0x07
    sRegAccConfig_t   ACC_CONFIG;
    sRegMagConfig_t   MAG_CONFIG;
    sRegGyrConfig0_t  GYR_CONFIG0;
    sRegGyrConfig1_t  GYR_CONFIG1;
    sRegAccSleepConfig_t    ACC_SLEEP;
    sRegGyrSleepConfig_t    GYR_SLEEP;
    uint8_t   reserved2;
    sRegIntMask_t     INT_MASK;
    sRegIntEn_t       INT_EN;  // 0x10
    uint8_t   ACC_AM_THRES;
    sRegAccIntSet_t    ACC_INT_SETTINGS;
    uint8_t   ACC_HG_DURATION;
    uint8_t   ACC_HG_THRES;
    uint8_t   ACC_NM_THRES;
    sRegAccNmSet_t    ACC_NM_SET;
    sRegGyrIntSetting_t     GYR_INT_SETTING;
    sRegGyrHrSet_t    GYR_HR_X_SET;
    uint8_t   GYR_DUR_X;
    sRegGyrHrSet_t    GYR_HR_Y_SET;
    uint8_t   GYR_DUR_Y;
    sRegGyrHrSet_t    GYR_HR_Z_SET;
    uint8_t   GYR_DUR_Z;
    sRegGyrAmThres_t  GYR_AM_THRES;
    sRegGyrAmSet_t    GYR_AM_SET;  // 0x1f
    uint8_t   reserved3[(0x4f - 0x20 + 1)];
    UniqueId_t    UNIQUE_ID;  // 0x5f
  } sRegsPage1_t;

  /**
   * @brief Declare sensor status
   */
  typedef enum {
    eStatusOK,    // everything OK
    eStatusErr,   // unknow error
    eStatusErrDeviceNotDetect,    // device not detected
    eStatusErrDeviceReadyTimeOut, // device ready time out
    eStatusErrDeviceStatus,       // device internal status error
    eStatusErrParameter           // function parameter error
  } eStatus_t;

// functions
public:

  DFRobot_BNO055();

  /**
   * @brief begin Sensor begin
   * @return Sensor status
   */
  eStatus_t   begin();

  /**
   * @brief getAxisAnalog Get axis analog data
   * @param eAxis One axis type from eAxis_t
   * @return Struct sAxisAnalog_t, contains axis analog data, members unit depend on eAxis:
   *                case eAxisAcc, unit mg
   *                case eAxisLia, unit mg
   *                case eAxisGrv, unit mg
   *                case eAxisMag, unit ut
   *                case eAxisGyr, unit dps
   */
  sAxisAnalog_t getAxis(eAxis_t eAxis);

  /**
   * @brief getEulAnalog Get euler analog data
   * @return Struct sEulAnalog_t, contains euler analog data
   */
  sEulAnalog_t  getEul();

  /**
   * @brief getQuaAnalog Get quaternion analog data
   * @return Struct sQuaAnalog_t, contains quaternion analog data
   */
  sQuaAnalog_t  getQua();

  /**
   * @brief setAccOffset Set axis offset data
   * @param eAxis One axis type from eAxis_t, only support accelerometer, magnetometer and gyroscope
   * @param sOffset Struct sAxisAnalog_t, contains axis analog data, members unit depend on eAxis:
   *                case eAxisAcc, unit mg, members can't out of acc range
   *                case eAxisMag, unit ut, members can't out of mag range
   *                case eAxisGyr, unit dps, members can't out of gyr range
   */
  void    setAxisOffset(eAxis_t eAxis, sAxisAnalog_t sOffset);

  /**
   * @brief setOprMode Set operation mode
   * @param eOpr One operation mode from eOprMode_t
   */
  void    setOprMode(eOprMode_t eMode);

  /**
   * @brief setPowerMode Set power mode
   * @param eMode One power mode from ePowerMode_t
   */
  void    setPowerMode(ePowerMode_t eMode);

  /**
   * @brief Reset sensor
   */
  void    reset();

  /**
   * @brief setAccRange Set accelerometer measurement range, default value is 4g
   * @param eRange One range enum from eAccRange_t
   */
  void    setAccRange(eAccRange_t eRange);

  /**
   * @brief setAccBandWidth Set accelerometer band width, default value is 62.5hz
   * @param eBand One band enum from eAccBandWidth_t
   */
  void    setAccBandWidth(eAccBandWidth_t eBand);

  /**
   * @brief setAccPowerMode Set accelerometer power mode, default value is eAccPowerModeNormal
   * @param eMode One mode enum from eAccPowerMode_t
   */
  void    setAccPowerMode(eAccPowerMode_t eMode);

  /**
   * @brief setMagDataRate Set magnetometer data output rate, default value is 20hz
   * @param eRate One rate enum from eMagDataRate_t
   */
  void    setMagDataRate(eMagDataRate_t eRate);

  /**
   * @brief setMagOprMode Set magnetometer operation mode, default value is eMagOprModeRegular
   * @param eMode One mode enum from eMagOprMode_t
   */
  void    setMagOprMode(eMagOprMode_t eMode);

  /**
   * @brief setMagPowerMode Set magnetometer power mode, default value is eMagePowerModeForce
   * @param eMode One mode enum from eMagPowerMode_t
   */
  void    setMagPowerMode(eMagPowerMode_t eMode);

  /**
   * @brief setGyrRange Set gyroscope range, default value is 2000
   * @param eRange One range enum from eGyrRange_t
   */
  void    setGyrRange(eGyrRange_t eRange);

  /**
   * @brief setGyrBandWidth Set gyroscope band width, default value is 32HZ
   * @param eBandWidth One band width enum from eGyrBandWidth_t
   */
  void    setGyrBandWidth(eGyrBandWidth_t eBandWidth);

  /**
   * @brief setGyrPowerMode Set gyroscope power mode, default value is eGyrPowerModeNormal
   * @param eMode One power mode enum from eGyrPowerMode_t
   */
  void    setGyrPowerMode(eGyrPowerMode_t eMode);

  /**
   * @brief getIntState Get interrupt state, interrupt auto clear after read
   * @return If result > 0, at least one interrupt triggered. Result & eIntXXX (from eInt_t) to test is triggered
   */
  uint8_t   getIntState();

  /**
   * @brief setIntMask Set interrupt mask enable, there will generate a interrupt signal (raising) on INT pin if corresponding interrupt enabled
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setIntMaskEnable(eInt_t eInt);

  /**
   * @brief setIntMaskDisable Set corresponding interrupt mask disable
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setIntMaskDisable(eInt_t eInt);

  /**
   * @brief setIntEnEnable Set corresponding interrupt enable
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setIntEnable(eInt_t eInt);

  /**
   * @brief setIntEnDisable Set corresponding interrupt disable
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setIntDisable(eInt_t eInt);

  /**
   * @brief setAccAmThres Set accelerometer any motion threshold
   * @param thres Threshold to set, unit mg, value is dependent on accelerometer range selected,
   *        case 2g, no more than 1991
   *        case 4g, no more than 3985
   *        case 8g, no more than 7968
   *        case 16g, no more than 15937
   *        Attenion: The set value will be slightly biased according to datasheet
   */
  void    setAccAmThres(uint16_t thres);

  /**
   * @brief setAccIntDur Set accelerometer interrupt duration,
   *        any motion interrupt triggers if duration (dur + 1) consecutive data points are above the any motion interrupt
   *        threshold define in any motion threshold
   * @param dur Duration to set, range form 1 to 4
   */
  void    setAccIntAmDur(uint8_t dur);

  /**
   * @brief setAccIntEnable Set accelerometer interrupt enable
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setAccIntEnable(eAccIntSet_t eInt);

  /**
   * @brief setAccIntDisable Set accelerometer interrupt disable
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setAccIntDisable(eAccIntSet_t eInt);

  /**
   * @brief setAccHighGDuration Set accelerometer high-g interrupt, the high-g interrupt delay according to [dur + 1] * 2 ms
   * @param dur Duration from 2ms to 512ms
   */
  void    setAccHighGDuration(uint16_t dur);

  /**
   * @brief setAccHighGThres Set accelerometer high-g threshold
   * @param thres Threshold to set, unit mg, value is dependent on accelerometer range selected,
   *        case 2g, no more than 1991
   *        case 4g, no more than 3985
   *        case 8g, no more than 7968
   *        case 16g, no more than 15937
   *        Attenion: The set value will be slightly biased according to datasheet
   */
  void    setAccHighGThres(uint16_t thres);

  /**
   * @brief setAccNmThres Set accelerometer no motion threshold
   * @param thres Threshold to set, unit mg, value is dependent on accelerometer range selected,
   *        case 2g, no more than 1991
   *        case 4g, no more than 3985
   *        case 8g, no more than 7968
   *        case 16g, no more than 15937
   *        Attenion: The set value will be slightly biased according to datasheet
   */
  void    setAccNmThres(uint16_t thres);

  /**
   * @brief setAccNmSet Set accelerometer slow motion or no motion mode and duration
   * @param eSmnm Enum of eAccNmSmnm_t
   * @param dur Interrupt trigger delay (unit seconds), no more than 344.
   *            Attenion: The set value will be slightly biased according to datasheet
   */
  void    setAccNmSet(eAccNmSmnm_t eSmnm, uint16_t dur);

  /**
   * @brief setGyrIntEnable Set corresponding gyroscope interrupt enable
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setGyrIntEnable(eGyrIntSet_t eInt);

  /**
   * @brief setGyrIntDisable Set corresponding gyroscope interrupt disable
   * @param eInt One or more interrupt flags to set, input them through operate or
   */
  void    setGyrIntDisable(eGyrIntSet_t eInt);

  /**
   * @brief setGyrHrSet Set gyroscope high rate settings
   * @param eSingleAxis Single axis to set
   * @param thres High rate threshold to set, unit degree/seconds, value is dependent on gyroscope range selected,
   *        case 2000, no more than 1937
   *        case 1000, no more than 968
   *        case 500, no more than 484
   *        case 250, no more than 242
   *        case 125, no more than 121
   *        Attenion: The set value will be slightly biased according to datasheet
   * @param dur High rate duration to set, unit ms, duration from 2.5ms to 640ms
   *            Attenion: The set value will be slightly biased according to datasheet
   */
  void    setGyrHrSet(eSingleAxis_t eSingleAxis, uint16_t thres, uint16_t dur);

  /**
   * @brief setGyrAmThres Set gyroscope any motion threshold
   * @param thres Threshold to set, unit mg, value is dependent on accelerometer range selected,
   *        case 2000, no more than 128
   *        case 1000, no more than 64
   *        case 500, no more than 32
   *        case 250, no more than 16
   *        case 125, no more than 8
   *        Attenion: The set value will be slightly biased according to datasheet
   */
  void    setGyrAmThres(uint8_t thres);

protected:
  virtual void readReg(uint8_t reg, uint8_t *pBuf, uint8_t len) = 0;
  virtual void writeReg(uint8_t reg, uint8_t *pBuf, uint8_t len) = 0;

  uint8_t   getReg(uint8_t reg, uint8_t pageId);
  void      setToPage(uint8_t pageId);
  void      setUnit();
  void      writeRegBits(uint8_t reg, uint8_t flied, uint8_t val);
  uint16_t  mapAccThres(uint16_t thres);
  void      mapGyrHrThres(uint8_t *pHysteresis, uint16_t *pThres, uint16_t *pDur);
  void      mapGyrAmThres(uint8_t *pThres);

  sAxisData_t   getAxisRaw(eAxis_t eAxis);
  sEulData_t    getEulRaw();
  sQuaData_t    getQuaRaw();

// variables ----------------------------------------------------------------
public:
  /**
   * @brief lastOpreateStatus Show last operate status
   */
  eStatus_t   lastOperateStatus;

protected:
  uint8_t   _currentPage;
  eAccRange_t   _eAccRange;
  eGyrRange_t   _eGyrRange;

};


// utils class ----------------------------------------------------------------

class DFRobot_BNO055_IIC : public DFRobot_BNO055 {
public:
  /**
   * @brief The eCom3State enum, sensor address is according to pin(com3) state
   */
  typedef enum {
    eCom3Low,
    eCom3High
  } eCom3State_t;

  /**
   * @brief DFRobot_BNO055_IIC class constructor
   * @param pWire select One TwoWire peripheral
   * @param addr Sensor address
   */
  DFRobot_BNO055_IIC(TwoWire *pWire, uint8_t addr);

protected:
  void    readReg(uint8_t reg, uint8_t *pBuf, uint8_t len);
  void    writeReg(uint8_t reg, uint8_t *pBuf, uint8_t len);

protected:
  TwoWire   *_pWire;
  uint8_t   _addr;

};

#endif
