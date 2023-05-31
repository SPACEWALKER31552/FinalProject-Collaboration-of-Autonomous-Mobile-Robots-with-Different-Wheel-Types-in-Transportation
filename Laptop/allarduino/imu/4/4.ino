/* Get tilt angles on X and Y, and rotation angle on Z
    Angles are given in degrees
 License: MIT
 */

 double startz = 0.0 ;
 double slave2diff = 0.0 ;
 double slavediff = 0.0 ;
 double r = 1.696827923 ;
 double slave2x = 0.0;
 double slave2y = 0.0;
 double slavex = 0.0;
 double slavey = 0.0;
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
   while (status != 0) { } // stop eveslave2ything if could not connect to MPU6050
 Serial.println(F("Calculating offsets, do not move MPU6050"));
   delay(1000);
   mpu.calcOffsets(); // gyro and accelero
   Serial.println("Done!\n");
   mpu.update();
   startz = mpu.getAngleZ();
 }
 void loop() {
   mpu.update();
 if ((millis() - timer) > 10) { // print data eveslave2y 10ms
     //Serial.print("X : ");
     //Serial.print(mpu.getAngleX());
     //Serial.print("\tY : ");
     //Serial.print(mpu.getAngleY());
     //Serial.print("Z : ");
     //Serial.println(mpu.getAngleZ());
     // slave2 --------------------------------------------------------------------------------------
     if((mpu.getAngleZ() - startz+19.45)>0)
     {
      slave2diff = mpu.getAngleZ() - startz + 19.45;
     //Serial.println(slave2diff);
     if(slave2diff>360)
     {
      slave2diff = slave2diff - 360;
     }
     
     if(slave2diff>=270)
     {
      //quotant 4
        slave2diff = slave2diff - 270 ;
        slave2x = r*cos(slave2diff*3.14/180);
        slave2y = r*sin(slave2diff*3.14/180);
        slave2x = -1*abs(slave2x);
        slave2y = -1*abs(slave2y);
     }
     else if(slave2diff>=180)
     {
        //quotant 3
        slave2diff = slave2diff - 180 ;
        slave2x = r*sin(slave2diff*3.14/180);
        slave2y = r*cos(slave2diff*3.14/180);
        slave2x = -1*abs(slave2x);
        slave2y = abs(slave2y);
     }
     else if(slave2diff>=90)
     {
        //quotant 2
        slave2diff = slave2diff - 90 ;
        slave2x = r*cos(slave2diff*3.14/180);
        slave2y = r*sin(slave2diff*3.14/180);
        slave2x = abs(slave2x);
        slave2y = abs(slave2y);
     }
     else if(slave2diff<90)
     {
        //quotant 1
        slave2x = r*sin(slave2diff*3.14/180);
        slave2y = r*cos(slave2diff*3.14/180);
        slave2x = abs(slave2x);
        slave2y = -1*abs(slave2y);    
     }
     }
     else if((mpu.getAngleZ() - startz+19.45)<0)
     {
     slave2diff = mpu.getAngleZ() - startz + 19.45;
     //Serial.println(slave2diff);
     if(slave2diff<-360)
     {
      slave2diff = slave2diff + 360;
     }

     if(slave2diff<=-270)
     {
        //quotant -4
        slave2diff = slave2diff + 270 ;
        slave2x = r*cos(slave2diff*3.14/180);
        slave2y = r*sin(slave2diff*3.14/180);
        slave2x = abs(slave2x);
        slave2y = -1*abs(slave2y);
     }
     else if(slave2diff<=-180)
     {
        //quotant -3
        slave2diff = slave2diff + 180 ;
        slave2x = r*sin(slave2diff*3.14/180);
        slave2y = r*cos(slave2diff*3.14/180);
        slave2x = abs(slave2x);
        slave2y = abs(slave2y);
     }
     else if(slave2diff<=-90)
     {
        //quotant -2
        slave2diff = slave2diff + 90 ;
        slave2x = r*cos(slave2diff*3.14/180);
        slave2y = r*sin(slave2diff*3.14/180);
        slave2x = -1*abs(slave2x);
        slave2y = abs(slave2y);
     }
     else if(slave2diff>-90)
     {
        //quotant -1
        slave2x = r*sin(slave2diff*3.14/180);
        slave2y = r*cos(slave2diff*3.14/180);
        slave2x = -1*abs(slave2x);
        slave2y = -1*abs(slave2y);   
     }
     }

    // slave --------------------------------------------------------------------------------------
    if((mpu.getAngleZ() - startz-19.45)>0)
     {
      slavediff = mpu.getAngleZ() - startz - 19.45;
     //Serial.println(slavediff);
     if(slavediff>360)
     {
      slavediff = slavediff - 360;
     }
     
     if(slavediff>=270)
     {
      //quotant 4
        slavediff = slavediff - 270 ;
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = -1*abs(slavey);
     }
     else if(slavediff>=180)
     {
        //quotant 3
        slavediff = slavediff - 180 ;
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = abs(slavey);
     }
     else if(slavediff>=90)
     {
        //quotant 2
        slavediff = slavediff - 90 ;
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = abs(slavex);
        slavey = abs(slavey);
     }
     else if(slavediff<90)
     {
        //quotant 1
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        slavex = abs(slavex);
        slavey = -1*abs(slavey);    
     }
     }
     else if((mpu.getAngleZ() - startz-19.45)<0)
     {
     slavediff = mpu.getAngleZ() - startz - 19.45;
     //Serial.println(slavediff);
     if(slavediff<-360)
     {
      slavediff = slavediff + 360;
     }

     if(slavediff<=-270)
     {
        //quotant -4
        slavediff = slavediff + 270 ;
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = abs(slavex);
        slavey = -1*abs(slavey);
     }
     else if(slavediff<=-180)
     {
        //quotant -3
        slavediff = slavediff + 180 ;
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        slavex = abs(slavex);
        slavey = abs(slavey);
     }
     else if(slavediff<=-90)
     {
        //quotant -2
        slavediff = slavediff + 90 ;
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = abs(slavey);
     }
     else if(slavediff>-90)
     {
        //quotant -1
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = -1*abs(slavey);   
     }
     }


     Serial.print("slavex = ");
     Serial.print(slavex);
     Serial.print(" slavey = ");
     Serial.println(slavey);
     
     Serial.print("slave2x = ");
     Serial.print(slave2x);
     Serial.print(" slave2y = ");
     Serial.println(slave2y);
     
     timer = millis();
 }
 }
