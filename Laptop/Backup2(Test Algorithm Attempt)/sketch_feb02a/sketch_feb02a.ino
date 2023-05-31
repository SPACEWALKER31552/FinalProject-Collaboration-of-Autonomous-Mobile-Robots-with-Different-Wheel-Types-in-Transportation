
#include <Wire.h>
//#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include<Wire.h>
 
const int MPU_addr=0x68;
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
 
int minVal=265;
int maxVal=402;
 
double x;
double y;
double z;

//define varialble
unsigned long lastmillis = 0;
double currentzeta = 0 ;

double linx = 0;
double liny = 0;
double linz = 0;
double angx = 0;
double angy = 0;
double angz = 1;
double leaderturnicomponent = 0;
double leaderturnjcomponent = 0;
double leaderturnkcomponent = 0;

double slaveturnicomponent = 0;
double slaveturnjcomponent = 0;
double slaveturnkcomponent = 0;

double slave2turnicomponent = 0;
double slave2turnjcomponent = 0;
double slave2turnkcomponent = 0;


//coordinate for master
double leaderRx = 0;
double leaderRy = 0;
double leaderRz = 0;

//coordinate for slave
double slaveRx = -0.565;
double slaveRy = -1.6;
double slaveRz = 0;

//coordinate for slave2
double slave2Rx = 0.565;
double slave2Ry = -1.6;
double slave2Rz = 0;

double newzeta = 0.0 ;

#define Looptime 1000 //millisec
int sec = 0 ;
#define l 1.697 
#define Pi 3.14159 
#define alpha1 2.514
#define alpha2 3.768515
#define alpha3 0
#define d1 0.963133
#define d2 0.963133
#define d3 0.82

void handle_cmd(const geometry_msgs::TwistStamped& cmd_vel) 
{                                              //Reset the counter for number of main loops without communication
  
  linx = cmd_vel.twist.linear.x;
  liny = cmd_vel.twist.linear.y;
  linz = cmd_vel.twist.linear.z;
  angx = cmd_vel.twist.angular.x;
  angy = cmd_vel.twist.angular.y;
  angz = cmd_vel.twist.angular.z;

}
void handle_zeta(const geometry_msgs::Pose& zeta) 
{                                              //Reset the counter for number of main loops without communication
  
  currentzeta = zeta.orientation.x;

}


void setup() 
{
   Serial.begin(115200);
   Wire.begin();
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x6B);
   Wire.write(0);
   Wire.endTransmission(true);
}

void loop() 
{
  Wire.beginTransmission(MPU_addr);
Wire.write(0x3B);
Wire.endTransmission(false);
Wire.requestFrom(MPU_addr,14,true);
AcX=Wire.read()<<8|Wire.read();
AcY=Wire.read()<<8|Wire.read();
AcZ=Wire.read()<<8|Wire.read();
int xAng = map(AcX,minVal,maxVal,-90,90);
int yAng = map(AcY,minVal,maxVal,-90,90);
int zAng = map(AcZ,minVal,maxVal,-90,90);
 
x= RAD_TO_DEG * (atan2(-yAng, -zAng)+PI);
y= RAD_TO_DEG * (atan2(-xAng, -zAng)+PI);
z= RAD_TO_DEG * (atan2(-yAng, -xAng)+PI);
  
  
  if((millis()-lastmillis) >= Looptime)
  {
    lastmillis = millis();
    /*  if(sec<=3)
      {
        newzeta = 19.45 + (angz*sec*180/Pi);
        slave2Ry = -1*(l*cos(newzeta*Pi/180));
        slave2Rx = (l*sin(newzeta*Pi/180));
        sec = sec+1;  
      }
      else if(sec>3)
      {
        sec = 0;
        newzeta = 19.45;
        slave2Rx = 0.565;
        slave2Ry = -1.6;
      }*/
   
      
   Serial.print("zeta: ");
   Serial.println(newzeta);
   Serial.print("x: ");
   Serial.println(slave2Rx);
   Serial.print("y: ");
   Serial.println(slave2Ry);
  }
  
}
