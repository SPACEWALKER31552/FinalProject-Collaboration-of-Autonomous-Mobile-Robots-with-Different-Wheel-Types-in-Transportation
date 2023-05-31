/* Get tilt angles on X and Y, and rotation angle on Z
    Angles are given in degrees
 License: MIT
 */

 double startz = 0.0 ;
 double diff = 0.0 ;
 double r = 1.696827923 ;
 double rx = 0.0;
 double ry = 0.0;
 #include "Wire.h"
 #include <MPU6050_light.h>
 MPU6050 mpu(Wire);
 unsigned long timer = 0;
 void setup() {
   Serial.begin(115200);
   Serial.print("hello");
   Wire.begin();
 byte status = mpu.begin();
   Serial.print(F("MPU6050 status: "));
   Serial.println(status);
   while (status != 0) { } // stop everything if could not connect to MPU6050
 Serial.println(F("Calculating offsets, do not move MPU6050"));
   delay(1000);
   mpu.calcOffsets(); // gyro and accelero
   Serial.println("Done!\n");
   mpu.update();
   startz = mpu.getAngleZ();
 }
 void loop() {
   mpu.update();
 if ((millis() - timer) > 10) { // print data every 10ms
     //Serial.print("X : ");
     //Serial.print(mpu.getAngleX());
     //Serial.print("\tY : ");
     //Serial.print(mpu.getAngleY());
     Serial.print("Z : ");
     Serial.println(mpu.getAngleZ());
     diff = abs(mpu.getAngleZ() - startz);
     Serial.println(diff);
     rx = r*sin((diff+19.45)*3.14/180);
     ry = r*cos((diff+19.45)*3.14/180);
     Serial.print("rx = ");
     Serial.print(rx);
     Serial.print(" ry = ");
     Serial.println(ry);
     timer = millis();
   }
 }
