/*
 * file EulerAngleDemo.ino
 *
 * @ https://github.com/DFRobot/DFRobot_BNO055
 *
 * connect BNO055 I2C interface with your board (please reference board compatibility)
 *
 * Gets the quaternions and prints it out through the serial port.
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
void displayCalStatus(void)
{
    /* Get the four calibration values (0..3) */
    /* Any sensor data reporting 0 should be ignored, */
    /* 3 means 'fully calibrated" */
    uint8_t system, gyro, accel, mag;
    system = gyro = accel = mag = 0;
    mpu.getCalibration(&system, &gyro, &accel, &mag);
    /* The data should be ignored until the system calibration is > 0 */
    if (!system)
    {
        Serial.print("! ");
    }

    /* Display the individual values */
    Serial.print("Sys:");
    Serial.print(system, DEC);
    Serial.print(" G:");
    Serial.print(gyro, DEC);
    Serial.print(" A:");
    Serial.print(accel, DEC);
    Serial.print(" M:");
    Serial.println(mag, DEC);
}
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
  displayCalStatus();
  Serial.println("##########################################");
  delay(1000);
}

