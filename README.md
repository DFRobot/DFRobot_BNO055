# DFRobot_BNO055 Intelligent 10DOF AHRS

The BNO055 is a System in Package (SiP), integrating a triaxial 14-bit accelerometer, <br>
a triaxial 16-bit gyroscope with a range of ±2000 degrees per second, a triaxial geomagnetic sensor <br>
and a 32-bit cortex M0+ microcontroller running Bosch Sensortec sensor fusion software, in a single package. <br>


## DFRobot_BNO055 Library for Arduino
---------------------------------------------------------
Provides an Arduino library for reading and interpreting Bosch BNO055 data over I2C.Used to read the current pose and calculate the Euler angles.

## Table of Contents

* [Installation](#installation)
* [Methods](#methods)
* [Compatibility](#compatibility)
* [History](#history)
* [Credits](#credits)

<snippet>
<content>

## Installation

To use this library download the zip file, uncompress it to a folder named DFRobot_BNO055. <br>
Download the zip file first to use this library and uncompress it to a folder named DFRobot_BNO055. <br>
If you want to see the graphical interface using imu_show.ino,<br>
you can download IMU_show.exe at https://github.com/DFRobot/DFRobot_IMU_Show/releases. <br>

## Methods

```C++

class DFRobot_BNO055 {

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

  /**
   * @brief Poewr mode enum
   */
  typedef enum {
    ePowerModeNormal,
    ePowerModeLowPower,
    ePowerModeSuspend
  } ePowerMode_t;

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

  /**
   * @brief qua analog data struct
   */
  typedef struct {
    float   w, x, y, z;
  } sQuaAnalog_t;

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

  /**
   * @brief Enum accelerometer slow motion mode or no motion mode
   */
  typedef enum {
    eAccNmSmnmSm,  // slow motion mode
    eAccNmSmnmNm   // no motion mode
  } eAccNmSmnm_t;

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

  /**
   * @brief lastOpreateStatus Show last operate status
   */
  eStatus_t   lastOpreateStatus;
};

class DFRobot_BNO055_IIC : public DFRobot_BNO055 {
public:

  /**
   * @brief DFRobot_BNO055_IIC class constructor
   * @param pWire select One TwoWire peripheral
   * @param addr Sensor address
   */
  DFRobot_BNO055_IIC(TwoWire *pWire, uint8_t addr);
};

```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266  |      √       |             |            | 
Arduino uno |       √      |             |            | 

## History

- 07/03/2019 - Version 0.2 released.
## About this Driver ##

Written by Frank [jiehan.guo@dfrobot.com](#jiehan.guo@dfrobot.com)
