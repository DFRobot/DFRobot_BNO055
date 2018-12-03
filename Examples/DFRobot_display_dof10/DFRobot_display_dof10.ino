/*
 * file EulerAngleDemo.ino
 *
 * @ https://github.com/DFRobot/DFRobot_BNO055
 *
 * connect BNO055 I2C interface with your board (please reference board compatibility)
 *
 * This example can be used to display the gesture on dof10.exe
 *
 *
 * version  V0.1
 * date  2018-1-8
 */
 
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
   Serial.println("Read euler angles...");
}

void loop() 
{
  mpu.readEuler();  /* read euler angle */
  /* In order to match the API of the upper computer, X ----> pitch  */
  Serial.print("pitch:"); 
  Serial.print(mpu.EulerAngles.y, 3); 
  Serial.print(" "); 
  /* In order to match the API of the upper computer, Y ----> roll  */
  Serial.print("roll:"); 
  Serial.print(mpu.EulerAngles.z, 3); 
  Serial.print(" ");
  /* In order to match the API of the upper computer, Z ----> yaw  */
  Serial.print("yaw:"); 
  Serial.print(mpu.EulerAngles.x, 3); 
  Serial.println(" ");
  
  delay(80);
}

