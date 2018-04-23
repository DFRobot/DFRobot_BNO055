/*
 * file EulerAngleDemo.ino
 *
 * @ https://github.com/DFRobot/DFRobot_BNO055
 *
 * connect BNO055 I2C interface with your board (please reference board compatibility)
 *
 * Gets the Angular Velocity of the current sensor and prints it out through the serial port.
 *
 * Copyright   [DFRobot](http://www.dfrobot.com), 2016
 * Copyright   GNU Lesser General Public License
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
     delay(50);
   }
//   mpu.setMode(mpu.eNORMAL_POWER_MODE, mpu.eFASTEST_MODE);
   delay(100);
   Serial.println("Read Angular Velocity...");
}

void loop() 
{
  mpu.readAngularVelocity();  /* read Angular Velocity */
  
  Serial.print("X: "); 
  Serial.print(mpu.GyrData.x, 3); 
  Serial.print(" dps  "); 
  
  Serial.print("Y: "); 
  Serial.print(mpu.GyrData.y, 3); 
  Serial.print(" dps ");
  
  Serial.print("Z: "); 
  Serial.print(mpu.GyrData.z, 3); 
  Serial.println(" dps ");
  
  delay(200);
}

