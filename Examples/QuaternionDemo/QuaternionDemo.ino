/*
 * file EulerAngleDemo.ino
 *
 * @ https://github.com/DFRobot/DFRobot_BNO055
 *
 * connect BNO055 I2C interface with your board (please reference board compatibility)
 *
 * Gets the quaternions and prints it out through the serial port.
 *
 *
 * version  V0.1
 * date  2018-1-8
 */
 
#include <Wire.h> 
#include "DFRobot_BNO055.h"

DFRobot_BNO055 mpu;

void setup() 
{
   Serial.begin(115200);
   while (!mpu.init())
   {
     Serial.println("ERROR! Unable to initialize the chip!");
     delay(30);
   }
//   mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE);
   delay(100);
   Serial.println("Read quaternion...");
}

void loop() 
{
  mpu.readQua();  /* read quaternion */
  Serial.print("W: "); 
  Serial.print(mpu.QuaData.w,3); 
  Serial.print("  ");
  Serial.print("X: "); 
  Serial.print(mpu.QuaData.x,3); 
  Serial.print("  ");
  Serial.print("Y: "); 
  Serial.print(mpu.QuaData.y,3); 
  Serial.print("  ");
  Serial.print("Z: "); 
  Serial.print(mpu.QuaData.z,3); 
  Serial.println("  ");
  delay(200);
}

