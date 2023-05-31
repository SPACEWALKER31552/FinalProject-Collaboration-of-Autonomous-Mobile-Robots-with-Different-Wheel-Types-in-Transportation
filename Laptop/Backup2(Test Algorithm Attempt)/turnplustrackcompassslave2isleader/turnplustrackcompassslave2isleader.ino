/* Get tilt angles on X and Y, and rotation angle on Z
    Angles are given in degrees
 License: MIT
 */

#include <Wire.h>
//#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
//define varialble
unsigned long lastmillis = 0;
double currentzeta = 0 ;
double leaderzeta = 0.0 ;
double slavezeta = 0.0 ;
double slave2zeta = 0.0 ;
float uslave = 0.0 ;
float uslave2 = 0.0 ;
float uleader = 0.0 ;

double linx = 0;
double liny = 0;
double linz = 0;
double angx = 0;
double angy = 0;
double angz = 0;
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
double leaderRx = -0.565;
double leaderRy = 1.6;
double leaderRz = 0;

//coordinate for slave
double slaveRx = -1.13;
double slaveRy = 0;
double slaveRz = 0;

//coordinate for slave2
double slave2Rx = 0;
double slave2Ry = 0;
double slave2Rz = 0;

double newzeta = 0.0 ;

#define Looptime 100 //millisec
int sec = 1 ;
#define l 1.37 //1.697 
#define Pi 3.14159 
#define alpha1 2.514
#define alpha2 3.768515
#define alpha3 0
#define d1 0.963133
#define d2 0.963133
#define d3 0.82

 double startz = 0.0 ;
 double start2z = 0.0 ;
 double slave2diff = 0.0 ;
 double slavediff = 0.0 ;
 double r = 1.37; //1.696827923 ;
 //double r = 3 ;
 double slave2x = 0.0;
 double slave2y = 0.0;
 double slavex = 0.0;
 double slavey = 0.0;
 #include "Wire.h"
 #include <MPU6050_light.h>
 MPU6050 mpu(Wire);
 unsigned long timer = 0;
long prevT = 0;
float eprevslave = 0;
float eintegralslave = 0;
double goalangleslave = 0.0 ;

long prevTslave2 = 0;
float eprevslave2 = 0;
float eintegralslave2 = 0;
double goalangleslave2 = 0.0 ;

long prevTleader = 0;
float eprevleader = 0;
float eintegralleader = 0;
double goalangleleader = 0.0 ;

int desdegree = 0 ;
int desdegreeleaderonly = 0 ;

void handle_cmd(const geometry_msgs::Twist& cmd_vel) 
{                                              //Reset the counter for number of main loops without communication
  
  linx = cmd_vel.linear.x;
  liny = cmd_vel.linear.y;
  linz = cmd_vel.linear.z;
  angx = cmd_vel.angular.x;
  angy = cmd_vel.angular.y;
  angz = cmd_vel.angular.z;
  

}
void handle_leader(const geometry_msgs::Vector3Stamped& imu) 
{                                              //Reset the counter for number of main loops without communication
  
  leaderzeta = imu.vector.x;
  
}
void handle_slave(const geometry_msgs::Vector3Stamped& imu) 
{                                              //Reset the counter for number of main loops without communication
  
  slavezeta = imu.vector.x;
  
}
void handle_slave2(const geometry_msgs::Vector3Stamped& imu) 
{                                              //Reset the counter for number of main loops without communication
  
  slave2zeta = imu.vector.x;
  
}
void handle_degree(const geometry_msgs::Vector3Stamped& degree) 
{                                              //Reset the counter for number of main loops without communication
  
  desdegree = degree.vector.x;
  desdegreeleaderonly = degree.vector.y;
  
}
ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel",handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
ros::Subscriber<geometry_msgs::Vector3Stamped> imu("slaveIMU",handle_slave);
ros::Subscriber<geometry_msgs::Vector3Stamped> imuu("slave2IMU",handle_slave2);
ros::Subscriber<geometry_msgs::Vector3Stamped> imuleader("leaderIMU",handle_leader);
ros::Subscriber<geometry_msgs::Vector3Stamped> degree("destinationdegree",handle_degree);

geometry_msgs::Twist follower1velocity;                                
ros::Publisher follower1velocity_msg("holo1/cmd_vel", &follower1velocity);
geometry_msgs::Twist follower2velocity;                                
ros::Publisher follower2velocity_msg("holo2/cmd_vel", &follower2velocity);
geometry_msgs::Twist follower3velocity;                               
ros::Publisher follower3velocity_msg("holo3/cmd_vel", &follower3velocity);
geometry_msgs::Vector3Stamped masterkinematicleader;                                
ros::Publisher masterkinematicleader_msg("leaderposition", &masterkinematicleader); 
geometry_msgs::Vector3Stamped masterkinematicslave;                                
ros::Publisher masterkinematicslave_msg("slaveposition", &masterkinematicslave);
geometry_msgs::Vector3Stamped masterkinematicslave2;                                
ros::Publisher masterkinematicslave2_msg("slave2position", &masterkinematicslave2);
geometry_msgs::Vector3Stamped IMUprogram;                                
ros::Publisher IMUprogram_msg("IMUprogram", &IMUprogram);

void setup() 
 {
  //Serial.begin(115200);
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);
  nh.subscribe(degree);
  nh.subscribe(cmd);
  nh.subscribe(imu);
  nh.subscribe(imuu);
  nh.subscribe(imuleader);
  nh.advertise(IMUprogram_msg);
  nh.advertise(follower1velocity_msg);
  nh.advertise(follower2velocity_msg);
  nh.advertise(follower3velocity_msg);
  nh.advertise(masterkinematicleader_msg);
  nh.advertise(masterkinematicslave_msg);
  nh.advertise(masterkinematicslave2_msg);  
  //Serial.print("hello");
  //Wire.begin();
  //byte status = mpu.begin();
  //Serial.print(F("MPU6050 status: "));
  //Serial.println(status);
  //while (status != 0) { } // stop eveslave2ything if could not connect to MPU6050
  //Serial.println(F("Calculating offsets, do not move MPU6050"));
  //delay(1000);
  //mpu.calcOffsets(); // gyro and accelero
  //Serial.println("Done!\n");
  //mpu.update();
  //startz = mpu.getAngleZ();
 }
 void loop() {
   //mpu.update();
   nh.spinOnce();
   if((millis()-lastmillis) >= Looptime)
    {
      lastmillis = millis();
     //Serial.print("X : ");
     //Serial.print(mpu.getAngleX());
     //Serial.print("\tY : ");
     //Serial.print(mpu.getAngleY());
     //Serial.print("Z : ");
     //Serial.println(mpu.getAngleZ());
     
     if(angz != 0 && linx == 0)
     {
     if(startz == 0)
     {
        startz = slavezeta ;
        
     }
     if(start2z == 0)
     {
        start2z = slave2zeta ;
        
     }
     else
     {}

         // slave --------------------------------------------------------------------------------------
    if((slavezeta - startz)>0)
     {
      slavediff = slavezeta - startz;
     //Serial.println(slavediff);
     if(slavediff>360)
     {
      slavediff = slavediff - 360;
     }
     
     if(slavediff>=270)
     {
      //quotant 4
        slavediff = slavediff - 270 ;
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = -1*abs(slavey);
     }
     else if(slavediff>=180)
     {
        //quotant 3
        slavediff = slavediff - 180 ;
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = abs(slavex);
        slavey = -1*abs(slavey);
     }
     else if(slavediff>=90)
     {
        //quotant 2
        slavediff = slavediff - 90 ;
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        slavex = abs(slavex);
        slavey = abs(slavey);
     }
     else if(slavediff<90)
     {
        //quotant 1
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = abs(slavey);    
     }
     }
     else if((slavezeta - startz)<0)
     {
     slavediff = slavezeta - startz;
     //Serial.println(slavediff);
     if(slavediff<-360)
     {
      slavediff = slavediff + 360;
     }

     if(slavediff<=-270)
     {
        //quotant -4
        slavediff = slavediff + 270 ;
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        
        slavex = abs(slavex);
        slavey = -1*abs(slavey);
     }
     else if(slavediff<=-180)
     {
        //quotant -3
        slavediff = slavediff + 180 ;
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = abs(slavex);
        slavey = abs(slavey);
     }
     else if(slavediff<=-90)
     {
        //quotant -2
        slavediff = slavediff + 90 ;
        slavex = r*sin(slavediff*3.14/180);
        slavey = r*cos(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = abs(slavey);
     }
     else if(slavediff>-90)
     {
        //quotant -1
        slavex = r*cos(slavediff*3.14/180);
        slavey = r*sin(slavediff*3.14/180);
        slavex = -1*abs(slavex);
        slavey = -1*abs(slavey);   
     }
     }

   
    slaveturnicomponent = (slaveRz*angy)-(slavey*angz);
    slaveturnjcomponent = (slavex*angz)-(slaveRz*angx);
    slaveturnkcomponent = (slavey*angx)-(slavex*angy);

    // leader --------------------------------------------------------------------------------------
    

    
      


    /*Serial.print("slave2turnicomponent : ");
    Serial.print(slave2turnicomponent);
    Serial.print("  slave2turnjcomponent : ");
    Serial.print(slave2turnjcomponent);
    Serial.print("  slave2turnkcomponent : ");
    Serial.println(slave2turnkcomponent);*/
   
    //cross to find omega component master
    //leaderturnicomponent = (leaderRz*angy)-(leaderRy*angz);
    //leaderturnjcomponent = (leaderRx*angz)-(leaderRz*angx);
    //leaderturnkcomponent = (leaderRy*angx)-(leaderRx*angy);
    
    /*Serial.print("leaderturnicomponent : ");
    Serial.print(leaderturnicomponent);
    Serial.print("  leaderturnjcomponent : ");
    Serial.print(leaderturnjcomponent);
    Serial.print("  leaderturnkcomponent : ");
    Serial.println(leaderturnkcomponent);*/
    
    /*Serial.print("slavex = ");
    Serial.print(slavex);
    Serial.print(" slavey = ");
    Serial.println(slavey);
     
    Serial.print("slave2x = ");
    Serial.print(slave2x);
    Serial.print(" slave2y = ");
    Serial.println(slave2y);*/
    
     
    follower3velocity.linear.x = 0;
    follower3velocity.linear.y = 0;
    follower3velocity.angular.z = 0;


    //leader---------------------------------------------------------

    follower1velocity.linear.x = abs(angz*3); //1.7
    follower1velocity.linear.y = 0;
    follower1velocity.angular.z = angz;

    //slave---------------------------------------------------------
    
    
    
    if((slavezeta - startz)<=90 && (slavezeta - startz)>=0 ) 
    {
    follower2velocity.linear.x = linx + (slaveturnjcomponent*abs(cos((int((slavezeta - startz))%90)*3.14/180))) + (slaveturnicomponent*abs(sin((int((slavezeta - startz))%90)*3.14/180)));
    follower2velocity.linear.y = -(slaveturnicomponent*abs(cos((int((slavezeta - startz))%90)*3.14/180))) - (slaveturnjcomponent*abs(sin((int((slavezeta - startz))%90)*3.14/180)));
    follower2velocity.angular.z = angz/10;
    }
    else if((slavezeta - startz)<= 180 && (slavezeta - startz)>90 )
    {
    follower2velocity.linear.x = linx + (slaveturnicomponent*abs(cos((int((slavezeta - startz))%90)*3.14/180))) + (slaveturnjcomponent*abs(sin((int((slavezeta - startz))%90)*3.14/180)));
    follower2velocity.linear.y = -(slaveturnjcomponent*abs(cos((int((slavezeta - startz))%90)*3.14/180))) + (slaveturnicomponent*abs(sin((int((slavezeta - startz))%90)*3.14/180)));
    follower2velocity.angular.z = angz/10;
    }
    else
    {
    follower2velocity.linear.x = 0;
    follower2velocity.linear.y = 0;
    follower2velocity.angular.z = 0;  
    }
    }
    else if(linx != 0 && angz == 0)
    {
      
    if(linx != 0)
    { 
    if(goalangleslave == 0)
    {
     goalangleslave = slavezeta ;
      
    }
    if(goalangleslave2 == 0)
    {
     goalangleslave2 = slave2zeta ; 
    }
    if(goalangleleader == 0)
    {
     goalangleleader = leaderzeta ; 
    }
    }  
    else
    {}

    // PID constants
    float kp = 0.05;
    float kd = 0.000;
    float ki = 0.00;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

  
    // errorleader
    int eleader = goalangleleader - leaderzeta;

    // derivative
    float dedtleader = (eleader-eprevleader)/(deltaT);

    // integral
    eintegralleader = eintegralleader + eleader*deltaT;

    // control signal
    float uleader = 0.3*eleader + kd*dedtleader + 0*eintegralleader;

    // store previous error
    eprevleader = eleader;



    // error
    int eslave = goalangleslave - slavezeta;

    // derivative
    float dedtslave = (eslave-eprevslave)/(deltaT);

    // integral
    eintegralslave = eintegralslave + eslave*deltaT;

    // control signal
    float uslave = kp*eslave + kd*dedtslave + ki*eintegralslave;

    // store previous error
    eprevslave = eslave;



    // error2
    int eslave2 = goalangleslave2 - slave2zeta;

    // derivative2
    float dedtslave2 = (eslave2-eprevslave2)/(deltaT);

    // integral2
    eintegralslave2 = eintegralslave2 + eslave2*deltaT;

    // control signal2
    float uslave2 = kp*eslave2 + kd*dedtslave2 + ki*eintegralslave2;

    // store previous error
    eprevslave2 = eslave2;

    follower1velocity.linear.x = 1.5*linx ;
    follower1velocity.angular.z = uleader ;

  
    follower2velocity.linear.x = linx ;
    follower2velocity.angular.z = uslave ;

  
    follower3velocity.linear.x = linx ;
    follower3velocity.angular.z = uslave2 ;

    }
    else
    {
      startz = 0.0 ;
      start2z = 0.0 ;
      
      leaderturnicomponent = 0;
      leaderturnjcomponent = 0;
      leaderturnkcomponent = 0;

       slaveturnicomponent = 0;
       slaveturnjcomponent = 0;
       slaveturnkcomponent = 0;

       slave2turnicomponent = 0;
       slave2turnjcomponent = 0;
       slave2turnkcomponent = 0;        
    }

    if(desdegree != 0)
    {
        // PID constants
    float kp = 0.02;
    float kd = 0.0001;
    float ki = 0.00;

    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

  
    // errorleader
    int eleader = desdegree - leaderzeta;

    // derivative
    float dedtleader = (eleader-eprevleader)/(deltaT);

    // integral
    eintegralleader = eintegralleader + eleader*deltaT;

    // control signal
    float uleader = 0.2*eleader + 0.0*dedtleader + 0.0*eintegralleader;


    // store previous error
    eprevleader = eleader;


  
    // error
    int eslave = desdegree - slavezeta;

    // derivative
    float dedtslave = (eslave-eprevslave)/(deltaT);

    // integral
    eintegralslave = eintegralslave + eslave*deltaT;

    // control signal
    float uslave = kp*eslave + kd*dedtslave + ki*eintegralslave;

    // store previous error
    eprevslave = eslave;

 

    // error2
    int eslave2 = desdegree - slave2zeta;

    // derivative2
    float dedtslave2 = (eslave2-eprevslave2)/(deltaT);

    // integral2
    eintegralslave2 = eintegralslave2 + eslave2*deltaT;

    // control signal2
    float uslave2 = kp*eslave2 + kd*dedtslave2 + ki*eintegralslave2;

    // store previous error
    eprevslave2 = eslave2;

    follower1velocity.linear.x = 0 ;
    follower1velocity.linear.y = 0 ;
    follower1velocity.angular.z = uleader ;
    
    follower2velocity.linear.x = 0 ;
    follower2velocity.linear.y = 0 ;
    follower2velocity.angular.z = uslave ;
    
    follower3velocity.linear.x = 0 ;
    follower3velocity.linear.y = 0 ;
    follower3velocity.angular.z = uslave2 ;
    }
    else if(desdegreeleaderonly != 0)
    {
    // time difference
    long currT = micros();
    float deltaT = ((float) (currT - prevT))/( 1.0e6 );
    prevT = currT;

  
    // errorleader
    int eleader = desdegreeleaderonly - leaderzeta;

    // derivative
    float dedtleader = (eleader-eprevleader)/(deltaT);

    // integral
    eintegralleader = eintegralleader + eleader*deltaT;

    // control signal
    float uleader = 0.2*eleader + 0.0*dedtleader + 0.0*eintegralleader;


    // store previous error
    eprevleader = eleader;

    follower1velocity.linear.x = 0 ;
    follower1velocity.linear.y = 0 ;
    follower1velocity.angular.z = uleader ;
    
    }
    else if(linx == 0 && angz == 0 )
    {
      goalangleleader = 0 ;
      goalangleslave = 0 ;
      goalangleslave2 = 0 ;
      
       uslave = 0 ;
       uslave2 = 0 ;   
       uleader = 0 ;
      
       follower1velocity.linear.x = 0 ;
       follower1velocity.linear.y = 0 ;
       follower1velocity.angular.z = 0 ;

  
       follower2velocity.linear.x = 0 ;
       follower2velocity.linear.y = 0 ;
       follower2velocity.angular.z = 0 ;

  
       follower3velocity.linear.x = 0 ;
       follower3velocity.linear.y = 0 ;
       follower3velocity.angular.z = 0 ;
    }


    IMUprogram.vector.x = goalangleleader;
    IMUprogram.vector.y = goalangleslave; 
    IMUprogram.vector.z = desdegree;
    IMUprogram_msg.publish(&IMUprogram);     
    follower1velocity_msg.publish(&follower1velocity);
    follower2velocity_msg.publish(&follower2velocity);
    follower3velocity_msg.publish(&follower3velocity);
    masterkinematicslave_msg.publish(&masterkinematicslave);
    masterkinematicslave2_msg.publish(&masterkinematicslave2);
     
     }
    
 }
