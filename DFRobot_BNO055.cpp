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
  delay(20);
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

void DFRobot_BNO055::readEuler()
{ 
  uint8_t xHigh, xLow, yLow, yHigh, zLow,zHigh;

    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit */
    Wire.write(eBNO055_REGISTER_EUL_DATA_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)6);

    while (Wire.available() < 6);
    
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
    EulerAngles.z = (int16_t)(zLow | (zHigh << 8)) / 15.800;
}

void DFRobot_BNO055::readLinAcc()
{
    uint8_t xHigh, xLow, yLow, yHigh, zLow,zHigh;
    
    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit  */
    Wire.write(eBNO055_REGISTER_LIA_DATA_X_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)6);
    
    /* Wait around until enough data is available  */
    while (Wire.available() < 6);
    
    xLow = Wire.read();
    xHigh = Wire.read();
    yLow = Wire.read();
    yHigh = Wire.read();
    zLow = Wire.read();
    zHigh = Wire.read();
    
    /* Shift values to create properly formed integer (low byte first) */
    LinAccData.x = (int16_t)(xLow | (xHigh << 8));
    LinAccData.y = (int16_t)(yLow | (yHigh << 8));
    LinAccData.z = (int16_t)(zLow | (zHigh << 8));
    
}

void DFRobot_BNO055::readQua()
{
    uint8_t wHigh, wLow, xHigh, xLow, yLow, yHigh, zLow,zHigh;
    
    Wire.beginTransmission(address);
    /* Make sure to set address auto-increment bit */
    Wire.write(eBNO055_REGISTER_QUA_DATA_W_LSB);
    Wire.endTransmission();
    Wire.requestFrom(address, (byte)8);
    
    /* Wait around until enough data is available  */
    while (Wire.available() < 8);
    
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
  Wire.endTransmission(false);

  return value;
}



