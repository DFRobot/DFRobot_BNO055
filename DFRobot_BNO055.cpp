/***************************************************************************
  This is a library for the BNO055 orientation sensor
  Designed specifically to work with the Adafruit BNO055 Breakout.
  Pick one up today in the adafruit shop!
  ------> https://www.adafruit.com/product/2472
  These sensors use I2C to communicate, 2 pins are required to interface.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by KTOWN for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ***************************************************************************/
 
#include "DFRobot_BNO055.h"

bool DFRobot_BNO055::init()
{
  uint16_t timeOut = 0;
  address = BNO055_ADDRESS;
  /*  IIC init */
  Wire.begin();
  /* Make sure we have the correct chip ID. This checks
     for correct address and that the IC is properly connected */
  delay(100);
  
  if (readByte(eBNO055_REGISTER_CHIP_ID) != BNO055_ID) 
  {
    
    while((readByte(eBNO055_REGISTER_CHIP_ID) != BNO055_ID))
    {  
        delay(4);
        if(++timeOut == 500)
           return false;
      }
    /*wont get here now, it will just hang above */
    if (readByte(eBNO055_REGISTER_CHIP_ID) != BNO055_ID)
       return false;
  }
  
  timeOut = 0;
  /* Go to config mode if not there */
  writeByte(eBNO055_REGISTER_OPR_MODE, eCONFIGMODE); 
  delay(25); 
  readByte(eBNO055_REGISTER_CHIP_ID);
  /* reset the sensor */
  writeByte(eBNO055_REGISTER_SYS_TRIGGER, 0b00100000); 
  
  while (readByte(eBNO055_REGISTER_CHIP_ID) != BNO055_ID) 
  {
    
    delay(10);
    if(++timeOut == 200)
      return false;
  }
  delay(50);
  writeByte(eBNO055_REGISTER_PWR_MODE, eNORMAL_POWER_MODE);
  delay(20);
  writeByte(eBNO055_REGISTER_OPR_MODE, eNDOF|eFASTEST_MODE);
  delay(20);
  return true;
}

void DFRobot_BNO055::setMode(eBNO055PowerModes_t powerMode, eBNO055DataRateMode_t dataRate)
{
  /* set a base mode (normal power mode, fastest/NDOF, manually packed for now) and start! */
  writeByte(eBNO055_REGISTER_PWR_MODE, powerMode);
  delay(20);
  writeByte(eBNO055_REGISTER_OPR_MODE, eNDOF|dataRate);
  delay(20);
}

void DFRobot_BNO055::setOpMode(eBNO055Mode_t opMode)
{
  _mode = opMode;
  writeByte(eBNO055_REGISTER_OPR_MODE, _mode);
  delay(30);
}

void DFRobot_BNO055::setAxisRemap(eBNO055AxisRemap_config_t remapcode )
{
    eBNO055Mode_t modeback = _mode;
    setOpMode(eCONFIGMODE);
    delay(25);
    writeByte(eBNO055_REGISTER_AXIS_MAP_CONFIG, remapcode);
    delay(10);
    setOpMode(modeback);
    delay(20);
}

void DFRobot_BNO055::getRevInfo(DFRobotBNO055_ReInfo_t *info)
{
    uint8_t a, b;

    memset(info, 0, sizeof(DFRobotBNO055_ReInfo_t));

    /* Check the accelerometer revision */
    info->accel_rev = readByte(eBNO055_REGISTER_ACC_ID);

    /* Check the magnetometer revision */
    info->mag_rev = readByte(eBNO055_REGISTER_MAG_ID);

    /* Check the gyroscope revision */
    info->gyro_rev = readByte(eBNO055_REGISTER_GYR_ID);

    /* Check the SW revision */
    info->bl_rev = readByte(eBNO055_REGISTER_BL_REV_ID);

    a = readByte(eBNO055_REGISTER_SW_REV_ID_LSB);
    b = readByte(eBNO055_REGISTER_SW_REV_ID_MSB);
    info->sw_rev = (((uint16_t)b) << 8) | ((uint16_t)a);
}
void DFRobot_BNO055::setAxisSign(eBNO055AxisRemap_sign_t remapsign)
{
    eBNO055Mode_t modeback = _mode;

    setOpMode(eCONFIGMODE);
    delay(25);
    writeByte(eBNO055_REGISTER_AXIS_MAP_SIGN, remapsign);
    delay(10);
    setOpMode(modeback);
    delay(20);
}
void DFRobot_BNO055::getSystemStatus(uint8_t *system_status, uint8_t *self_result, uint8_t *system_error)
{
  writeByte(eBNO055_REGISTER_PAGE_ID, 0);

  /* System Status (see section 4.3.58)
     ---------------------------------
     0 = Idle
     1 = System Error
     2 = Initializing Peripherals
     3 = System Iniitalization
     4 = Executing Self-Test
     5 = Sensor fusio algorithm running
     6 = System running without fusion algorithms */

  if (system_status != 0)
    *system_status    = readByte(eBNO055_REGISTER_SYS_STATUS);

  /* Self Test Results (see section )
     --------------------------------
     1 = test passed, 0 = test failed

     Bit 0 = Accelerometer self test
     Bit 1 = Magnetometer self test
     Bit 2 = Gyroscope self test
     Bit 3 = MCU self test

     0x0F = all good! */

  if (self_result != 0)
    *self_result = readByte(eBNO055_REGISTER_ST_RESULT);

  /* System Error (see section 4.3.59)
     ---------------------------------
     0 = No error
     1 = Peripheral initialization error
     2 = System initialization error
     3 = Self test result failed
     4 = Register map value out of range
     5 = Register map address out of range
     6 = Register map write error
     7 = BNO low power mode not available for selected operat ion mode
     8 = Accelerometer power mode not available
     9 = Fusion algorithm configuration error
     A = Sensor configuration error */

  if (system_error != 0)
    *system_error = readByte(eBNO055_REGISTER_SYS_ERR);

  delay(200);
}

void DFRobot_BNO055::getCalibration( uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag)
{
  uint8_t calData = readByte(eBNO055_REGISTER_CALIB_STAT);
  if (sys != NULL) {
    *sys = (calData >> 6) & 0x03;
  }
  if (gyro != NULL) {
    *gyro = (calData >> 4) & 0x03;
  }
  if (accel != NULL) {
    *accel = (calData >> 2) & 0x03;
  }
  if (mag != NULL) {
    *mag = calData & 0x03;
  }
}
bool DFRobot_BNO055::isFullyCalibrated(void)
{
    uint8_t system, gyro, accel, mag;
    getCalibration(&system, &gyro, &accel, &mag);
    if (system < 3 || gyro < 3 || accel < 3 || mag < 3)
        return false;
    return true;
}

bool DFRobot_BNO055::getSensorOffsets(uint8_t* calibData)
{
    if (isFullyCalibrated())
    {
        eBNO055Mode_t modeback = _mode;
        setOpMode(eCONFIGMODE);
        readByteLen(eBNO055_REGISTER_ACC_OFFSET_X_LSB, calibData, NUM_BNO055_OFFSET_REGISTERS);
        setOpMode(modeback);
        return true;
    }
    return false;
}

bool DFRobot_BNO055::getSensorOffsets(DFRobotBNO055_offsets_t &offsets_type)
{
    if (isFullyCalibrated())
    {
        eBNO055Mode_t modeback = _mode;
        setOpMode(eCONFIGMODE);
        delay(25);
        /* Accel offset range depends on the G-range:
           +/-2g  = +/- 2000 mg
           +/-4g  = +/- 4000 mg
           +/-8g  = +/- 8000 mg
           +/-16g = +/- 16000 mg */
        offsets_type.accel_offset_x = (readByte(eBNO055_REGISTER_ACC_OFFSET_X_MSB) << 8) | (readByte(eBNO055_REGISTER_ACC_OFFSET_X_LSB));
        offsets_type.accel_offset_y = (readByte(eBNO055_REGISTER_ACC_OFFSET_Y_MSB) << 8) | (readByte(eBNO055_REGISTER_ACC_OFFSET_Y_LSB));
        offsets_type.accel_offset_z = (readByte(eBNO055_REGISTER_ACC_OFFSET_Z_MSB) << 8) | (readByte(eBNO055_REGISTER_ACC_OFFSET_Z_LSB));

        /* Magnetometer offset range = +/- 6400 LSB where 1uT = 16 LSB */
        offsets_type.mag_offset_x = (readByte(eBNO055_REGISTER_MAG_OFFSET_X_MSB) << 8) | (readByte(eBNO055_REGISTER_MAG_OFFSET_X_LSB));
        offsets_type.mag_offset_y = (readByte(eBNO055_REGISTER_MAG_OFFSET_Y_MSB) << 8) | (readByte(eBNO055_REGISTER_MAG_OFFSET_Y_LSB));
        offsets_type.mag_offset_z = (readByte(eBNO055_REGISTER_MAG_OFFSET_Z_MSB) << 8) | (readByte(eBNO055_REGISTER_MAG_OFFSET_Z_LSB));

        /* Gyro offset range depends on the DPS range:
          2000 dps = +/- 32000 LSB
          1000 dps = +/- 16000 LSB
           500 dps = +/- 8000 LSB
           250 dps = +/- 4000 LSB
           125 dps = +/- 2000 LSB
           ... where 1 DPS = 16 LSB */
        offsets_type.gyro_offset_x = (readByte(eBNO055_REGISTER_GYR_OFFSET_X_MSB) << 8) | (readByte(eBNO055_REGISTER_GYR_OFFSET_X_LSB));
        offsets_type.gyro_offset_y = (readByte(eBNO055_REGISTER_GYR_OFFSET_Y_MSB) << 8) | (readByte(eBNO055_REGISTER_GYR_OFFSET_Y_LSB));
        offsets_type.gyro_offset_z = (readByte(eBNO055_REGISTER_GYR_OFFSET_Z_MSB) << 8) | (readByte(eBNO055_REGISTER_GYR_OFFSET_Z_LSB));

        /* Accelerometer radius = +/- 1000 LSB */
        offsets_type.accel_radius = (readByte(eBNO055_REGISTER_ACC_RADIUS_MSB) << 8) | (readByte(eBNO055_REGISTER_ACC_RADIUS_LSB));

        /* Magnetometer radius = +/- 960 LSB */
        offsets_type.mag_radius = (readByte(eBNO055_REGISTER_MAG_RADIUS_MSB) << 8) | (readByte(eBNO055_REGISTER_MAG_RADIUS_LSB));

        setOpMode(modeback);
        return true;
    }
    return false;
}

void DFRobot_BNO055::setSensorOffsets(const uint8_t* calibData)
{
    eBNO055Mode_t modeback = _mode;
    setOpMode(eCONFIGMODE);
    delay(25);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    /* A writeLen() would make this much cleaner */
    writeByte(eBNO055_REGISTER_ACC_OFFSET_X_LSB, calibData[0]);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_X_MSB, calibData[1]);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Y_LSB, calibData[2]);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Y_MSB, calibData[3]);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Z_LSB, calibData[4]);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Z_MSB, calibData[5]);

    writeByte(eBNO055_REGISTER_MAG_OFFSET_X_LSB, calibData[6]);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_X_MSB, calibData[7]);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Y_LSB, calibData[8]);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Y_MSB, calibData[9]);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Z_LSB, calibData[10]);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Z_MSB, calibData[11]);

    writeByte(eBNO055_REGISTER_GYR_OFFSET_X_LSB, calibData[12]);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_X_MSB, calibData[13]);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Y_LSB, calibData[14]);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Y_MSB, calibData[15]);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Z_LSB, calibData[16]);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Z_MSB, calibData[17]);

    writeByte(eBNO055_REGISTER_ACC_RADIUS_LSB, calibData[18]);
    writeByte(eBNO055_REGISTER_ACC_RADIUS_MSB, calibData[19]);

    writeByte(eBNO055_REGISTER_MAG_RADIUS_LSB, calibData[20]);
    writeByte(eBNO055_REGISTER_MAG_RADIUS_MSB, calibData[21]);

    setOpMode(modeback);
}

void DFRobot_BNO055::setSensorOffsets(const DFRobotBNO055_offsets_t &offsets_type)
{
    eBNO055Mode_t modeback = _mode;
    setOpMode(eCONFIGMODE);
    delay(25);

    /* Note: Configuration will take place only when user writes to the last
       byte of each config data pair (ex. ACCEL_OFFSET_Z_MSB_ADDR, etc.).
       Therefore the last byte must be written whenever the user wants to
       changes the configuration. */

    writeByte(eBNO055_REGISTER_ACC_OFFSET_X_LSB, (offsets_type.accel_offset_x) & 0x0FF);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_X_MSB, (offsets_type.accel_offset_x >> 8) & 0x0FF);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Y_LSB, (offsets_type.accel_offset_y) & 0x0FF);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Y_MSB, (offsets_type.accel_offset_y >> 8) & 0x0FF);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Z_LSB, (offsets_type.accel_offset_z) & 0x0FF);
    writeByte(eBNO055_REGISTER_ACC_OFFSET_Z_MSB, (offsets_type.accel_offset_z >> 8) & 0x0FF);

    writeByte(eBNO055_REGISTER_MAG_OFFSET_X_LSB, (offsets_type.mag_offset_x) & 0x0FF);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_X_MSB, (offsets_type.mag_offset_x >> 8) & 0x0FF);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Y_LSB, (offsets_type.mag_offset_y) & 0x0FF);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Y_MSB, (offsets_type.mag_offset_y >> 8) & 0x0FF);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Z_LSB, (offsets_type.mag_offset_z) & 0x0FF);
    writeByte(eBNO055_REGISTER_MAG_OFFSET_Z_MSB, (offsets_type.mag_offset_z >> 8) & 0x0FF);

    writeByte(eBNO055_REGISTER_GYR_OFFSET_X_LSB, (offsets_type.gyro_offset_x) & 0x0FF);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_X_MSB, (offsets_type.gyro_offset_x >> 8) & 0x0FF);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Y_LSB, (offsets_type.gyro_offset_y) & 0x0FF);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Y_MSB, (offsets_type.gyro_offset_y >> 8) & 0x0FF);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Z_LSB, (offsets_type.gyro_offset_z) & 0x0FF);
    writeByte(eBNO055_REGISTER_GYR_OFFSET_Z_MSB, (offsets_type.gyro_offset_z >> 8) & 0x0FF);

    writeByte(eBNO055_REGISTER_ACC_RADIUS_LSB, (offsets_type.accel_radius) & 0x0FF);
    writeByte(eBNO055_REGISTER_ACC_RADIUS_MSB, (offsets_type.accel_radius >> 8) & 0x0FF);

    writeByte(eBNO055_REGISTER_MAG_RADIUS_LSB, (offsets_type.mag_radius) & 0x0FF);
    writeByte(eBNO055_REGISTER_MAG_RADIUS_MSB, (offsets_type.mag_radius >> 8) & 0x0FF);

    setOpMode(modeback);
}

void DFRobot_BNO055::readEuler()
{ 
  uint8_t xHigh=0, xLow=0, yLow, yHigh, zLow,zHigh;

    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit */
    Wire.write(eBNO055_REGISTER_EUL_DATA_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)6);
    
    xLow  = Wire.read();
    xHigh = Wire.read();
    yLow  = Wire.read();
    yHigh = Wire.read();
    zLow  = Wire.read();
    zHigh = Wire.read();

    /* Shift values to create properly formed integer (low byte first) */
    /* 1 degree = 16 LSB  1radian = 900 LSB   */
    EulerAngles.x = (int16_t)(xLow | (xHigh << 8)) / 15.800;
    EulerAngles.y = (int16_t)(yLow | (yHigh << 8)) / 15.800;
    EulerAngles.z = -(int16_t)(zLow | (zHigh << 8)) / 15.800;

    if(EulerAngles.x > 360)  EulerAngles.x =  360;
    if(EulerAngles.y < -90)  EulerAngles.y =  -90;
    if(EulerAngles.y > 90)   EulerAngles.y =   90;
    if(EulerAngles.z > 180)  EulerAngles.z =  180;
    if(EulerAngles.z < -180) EulerAngles.z = -180;
}

void DFRobot_BNO055::readAngularVelocity()
{ 
  uint8_t xHigh, xLow, yLow, yHigh, zLow,zHigh;

    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit */
    Wire.write(eBNO055_REGISTER_GYR_DATA_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)6);
    
    xLow  = Wire.read();
    xHigh = Wire.read();
    yLow  = Wire.read();
    yHigh = Wire.read();
    zLow  = Wire.read();
    zHigh = Wire.read();

    /* Shift values to create properly formed integer (low byte first) */
    /* 1 degree = 16 LSB  1radian = 900 LSB   */
    GyrData.x = (int16_t)(xLow | (xHigh << 8)) / 15.800;
    GyrData.y = (int16_t)(yLow | (yHigh << 8)) / 15.800;
    GyrData.z = (int16_t)(zLow | (zHigh << 8)) / 15.800;
}

void DFRobot_BNO055::readLinAcc()
{
    uint8_t xHigh, xLow, yLow, yHigh, zLow,zHigh;
    
    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit  */
    Wire.write(eBNO055_REGISTER_LIA_DATA_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)6);
    
    xLow = Wire.read();
    xHigh = Wire.read();
    yLow = Wire.read();
    yHigh = Wire.read();
    zLow = Wire.read();
    zHigh = Wire.read();
    
    /* Shift values to create properly formed integer (low byte first) */
    /*1m/s2=100LSB        1mg=1LSB*/
    LinAccData.x = (int16_t)(xLow | (xHigh << 8))/100.0;
    LinAccData.y = (int16_t)(yLow | (yHigh << 8))/100.0;
    LinAccData.z = (int16_t)(zLow | (zHigh << 8))/100.0;
    
}

void DFRobot_BNO055::readAcc()
{
    uint8_t xHigh, xLow, yLow, yHigh, zLow,zHigh;
    
    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit  */
    Wire.write(eBNO055_REGISTER_ACC_DATA_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)6);
    
    xLow = Wire.read();
    xHigh = Wire.read();
    yLow = Wire.read();
    yHigh = Wire.read();
    zLow = Wire.read();
    zHigh = Wire.read();
    
    /* Shift values to create properly formed integer (low byte first) */
    /*1m/s2=100LSB        1mg=1LSB*/
    AccData.x = (int16_t)(xLow | (xHigh << 8))/100.0;
    AccData.y = (int16_t)(yLow | (yHigh << 8))/100.0;
    AccData.z = (int16_t)(zLow | (zHigh << 8))/100.0;
    
}

void DFRobot_BNO055::readQua()
{
    uint8_t wHigh, wLow, xHigh, xLow, yLow, yHigh, zLow,zHigh;
    
    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit */
    Wire.write(eBNO055_REGISTER_QUA_DATA_W_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)8);
    
    wLow = Wire.read();
    wHigh = Wire.read();
    xLow = Wire.read();
    xHigh = Wire.read();
    yLow = Wire.read();
    yHigh = Wire.read();
    zLow = Wire.read();
    zHigh = Wire.read();
    
    /* Shift values to create properly formed integer (low byte first) then scale (1qua=2^14lsb) */
    QuaData.w = (int16_t)(wLow | (wHigh << 8))/16384.0;
    QuaData.x = (int16_t)(xLow | (xHigh << 8))/16384.0;
    QuaData.y = (int16_t)(yLow | (yHigh << 8))/16384.0;
    QuaData.z = (int16_t)(zLow | (zHigh << 8))/16384.0;

    
}
/* uses last loaded QuaData and LinAccData. */
void DFRobot_BNO055::calcAbsLinAcc()  
{
    float a1,b1,c1, d1, a2, b2, c2, d2, ra,rb,rc,rd,den; 
    
    /*  stuff my quaternions and acceleration vectors into some variables to operate on them
        specifically calc q^-1.  Note, sensor seems to send the inverse.  */
    readLinAcc();
    readQua();
    den=  pow(QuaData.w,2)+pow(QuaData.x,2)+pow(QuaData.y,2)+pow(QuaData.z,2); 

    if (den<1.01 && den>.99)  /*close enough lets save some processing  */
    {
        a1=QuaData.w;
        b1=QuaData.x;   
        c1=QuaData.y;   
        d1=QuaData.z;  
       
    }else{ 
        a1=QuaData.w/den;
        b1=QuaData.x/den;  
        c1=QuaData.y/den;   
        d1=QuaData.z/den;   
    }
    /* load accel vector V  */
    a2=0;
    b2=LinAccData.x;
    c2=LinAccData.y;
    d2=LinAccData.z;
    
    /* time to Hamilton it up! (q^-1 * V) */
    ra=a1*a2-b1*b2-c1*c2-d1*d2;
    rb=a1*b2 + b1*a2 + c1*d2 - d1*c2;
    rc=a1*c2 - b1*d2 + c1*a2 + d1*b2;
    rd=a1*d2 + b1*c2 - c1*b2 + d1*a2;
    
    /* swap some vars
       first invert q */
    a2=a1;
    b2=-b1;
    c2=-c1;
    d2=-d1;
    /* now move the result */
    a1=ra;
    b1=rb;
    c1=rc;
    d1=rd;

    /* Hamilton it up again! (result*q) */
    ra=a1*a2-b1*b2-c1*c2-d1*d2;
    rb=a1*b2 + b1*a2 + c1*d2 - d1*c2;
    rc=a1*c2 - b1*d2 + c1*a2 + d1*b2;
    rd=a1*d2 + b1*c2 - c1*b2 + d1*a2;

    AbsLinAccData.x = rb;
    AbsLinAccData.y = rc;
    AbsLinAccData.z = rd;
}

void DFRobot_BNO055::getInfo()
{
   SystemStatusCode = readByte(eBNO055_REGISTER_SYS_STATUS);
   SelfTestStatus = readByte(eBNO055_REGISTER_ST_RESULT);
   SystemError = readByte(eBNO055_REGISTER_SYS_ERR);
}

void DFRobot_BNO055::writeByte(eBNO055Registers_t reg, byte value)
{
    Wire.beginTransmission(address);
    Wire.write((byte)reg);
    Wire.write(value);
    Wire.endTransmission();
}

byte DFRobot_BNO055::readByte(eBNO055Registers_t reg)
{
  byte value;
  
  Wire.beginTransmission(address);
  Wire.write((byte)reg);
  Wire.endTransmission();

  Wire.requestFrom(address, (byte)1);
  value = Wire.read();

  return value;
}
bool DFRobot_BNO055::readByteLen(eBNO055Registers_t reg, byte * buffer, uint8_t len)
{
  Wire.beginTransmission(address);
  #if ARDUINO >= 100
    Wire.write((uint8_t)reg);
  #else
    Wire.send(reg);
  #endif
  Wire.endTransmission();
  Wire.requestFrom(address, (byte)len);

  for (uint8_t i = 0; i < len; i++)
  {
    #if ARDUINO >= 100
      buffer[i] = Wire.read();
    #else
      buffer[i] = Wire.receive();
    #endif
  }

  return true;
}



