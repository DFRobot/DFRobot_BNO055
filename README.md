# DFRobot_BNO055 Intelligent 10DOF AHRS

The BNO055 is a System in Package (SiP), integrating a triaxial 14-bit accelerometer, 
a triaxial 16-bit gyroscope with a range of ±2000 degrees per second, a triaxial geomagnetic sensor 
and a 32-bit cortex M0+ microcontroller running Bosch Sensortec sensor fusion software, in a single package. 


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

To use this library download the zip file, uncompress it to a folder named DFRobot_BNO055. 
Download the zip file first to use this library and uncompress it to a folder named DFRobot_BNO055. 

## Methods

```C++

/*
 * @brief init BNO055 device
 *
 * @return result
 *    ture : falid
 *    false : succussful
 */
    bool init();
/*
 * @brief set BNO055 mode
 *
 * @param  powerMode   Set power mode.
 *         dataRate    Set the data transfer rate.
 *
 */
    void setMode(eBNO055PowerModes_t powerMode, eBNO055DataRateMode_t dataRate);
/*
 * @brief  Read euler angles.
 *         The resulting data is stored in EulerAngles.
 *         For exmple: EulerAngles.x, EulerAngles.y, EulerAngles.z
 */
    void readEuler(void);
/*
 * @brief  Read linear acceleration.
 *         The resulting data is stored in LinAccData.
 *         For exmple: LinAccData.x, LinAccData.y, LinAccData.z
 */
    void readLinAcc(void);
/*
 * @brief read quaternion data
 *         The resulting data is stored in QuaData.
 *         For exmple: QuaData.w, QuaData.x, QuaData.y, QuaData.z
 */
    void readQua(void);
/*
 * @brief  Uses last loaded QuaData and LinAccData.
 *         The resulting data is stored in AbsLinAccData.
 *         For exmple: AbsLinAccData.x, AbsLinAccData.y, AbsLinAccData.z
 */
    void calcAbsLinAcc(void);
    
    typedef enum
    {
        eNORMAL_POWER_MODE     = 0b00000000,
        eLOW_POWER_MODE        = 0b00000001,
        eSUSPEND_POWER_MODE    = 0b00000010,
    } eBNO055PowerModes_t;
    
    typedef enum
    {
        eFASTEST_MODE    = 0b00100000,
        eGAME_MODE       = 0b01000000,
        eUI_MODE         = 0b01100000,
        eNORMAL_MODE     = 0b10000000,
    } eBNO055DataRateMode_t;

    typedef struct BNO055EulerData_s
    {
      float x;
      float y;
      float z;
    } BNO055EulerData;
    
    typedef struct BNO055LinAccData_s
    {
        float x;
        float y;
        float z;
    } BNO055LinAccData;
    
    typedef struct BNO055QuaData_s
    {
        float w;
        float x;
        float y;
        float z;
    } BNO055QuaData;
    
    typedef struct BNO055AbsLinAccData_s
    {
        float x;
        float y;
        float z;
    } BNO055AbsLinAccData;

    BNO055EulerData EulerAngles;
    BNO055LinAccData LinAccData;
    BNO055QuaData QuaData;
    BNO055AbsLinAccData AbsLinAccData;


```

## Compatibility

MCU                | Work Well | Work Wrong | Untested  | Remarks
------------------ | :----------: | :----------: | :---------: | -----
FireBeetle-ESP32  |      √       |             |            | 
FireBeetle-ESP8266  |      √       |             |            | 
Arduino uno |       √      |             |            | 

## History

- Nov 31, 2018 - Version 0.1 released.

## Credits

Written by DFRobot_YangYue, 2018. (Welcome to our [website](https://www.dfrobot.com/))
