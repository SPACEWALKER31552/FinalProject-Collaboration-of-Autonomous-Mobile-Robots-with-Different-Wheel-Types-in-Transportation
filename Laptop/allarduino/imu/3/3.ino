/* Get tilt angles on X and Y, and rotation angle on Z
    Angles are given in degrees
 License: MIT
 */

 double startz = 0.0 ;
 double diff = 0.0 ;
 double realdiff = 0.0 ;
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
     //Serial.print("Z : ");
     //Serial.println(mpu.getAngleZ());
     
     if((mpu.getAngleZ() - startz+19.45)>0)
     {
      diff = mpu.getAngleZ() - startz + 19.45;
     Serial.println(diff);
     if(diff>360)
     {
      diff = diff - 360;
     }
     
     if(diff>=270)
     {
      //quotant 4
        diff = diff - 270 ;
        rx = r*cos(diff*3.14/180);
        ry = r*sin(diff*3.14/180);
        rx = -1*abs(rx);
        ry = -1*abs(ry);
     }
     else if(diff>=180)
     {
        //quotant 3
        diff = diff - 180 ;
        rx = r*sin(diff*3.14/180);
        ry = r*cos(diff*3.14/180);
        rx = -1*abs(rx);
        ry = abs(ry);
     }
     else if(diff>=90)
     {
        //quotant 2
        diff = diff - 90 ;
        rx = r*cos(diff*3.14/180);
        ry = r*sin(diff*3.14/180);
        rx = abs(rx);
        ry = abs(ry);
     }
     else if(diff<90)
     {
        //quotant 1
        rx = r*sin(diff*3.14/180);
        ry = r*cos(diff*3.14/180);
        rx = abs(rx);
        ry = -1*abs(ry);    
     }
     }
     else if((mpu.getAngleZ() - startz+19.45)<0)
     {
     diff = mpu.getAngleZ() - startz + 19.45;
     Serial.println(diff);
     if(diff<-360)
     {
      diff = diff + 360;
     }

     if(diff<=-270)
     {
        //quotant -4
        diff = diff + 270 ;
        rx = r*cos(diff*3.14/180);
        ry = r*sin(diff*3.14/180);
        rx = abs(rx);
        ry = -1*abs(ry);
     }
     else if(diff<=-180)
     {
        //quotant -3
        diff = diff + 180 ;
        rx = r*sin(diff*3.14/180);
        ry = r*cos(diff*3.14/180);
        rx = abs(rx);
        ry = abs(ry);
     }
     else if(diff<=-90)
     {
        //quotant -2
        diff = diff + 90 ;
        rx = r*cos(diff*3.14/180);
        ry = r*sin(diff*3.14/180);
        rx = -1*abs(rx);
        ry = abs(ry);
     }
     else if(diff>-90)
     {
        //quotant -1
        rx = r*sin(diff*3.14/180);
        ry = r*cos(diff*3.14/180);
        rx = -1*abs(rx);
        ry = -1*abs(ry);   
     }
     }
    
     Serial.print("rx = ");
     Serial.print(rx);
     Serial.print(" ry = ");
     Serial.println(ry);
     timer = millis();
 }
 }
