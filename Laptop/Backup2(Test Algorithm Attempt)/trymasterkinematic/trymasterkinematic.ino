
#include <Wire.h>
//#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
//define varialble
unsigned long lastmillis = 0;

double linx = 0;
double liny = 0;
double linz = 0;
double angx = 0;
double angy = 0;
double angz = 0;

double VvL = 0; //m/s
double omegavL = 0;
#define Looptime 1000 //millisec

//double startk = 0; //(k at 0 or starting point)
//double positionk = 0; 
//double futurek = 0; 

//---------------------- Virtual Leader ----------------------
//double previousXvL = 0;
double currentXvL = 0;
double futureXvL = 0;

//double previousYvL = 0;
double currentYvL = 0;
double futureYvL = 0;

//double previouszetavL = 0;
double currentzetavL = 0;
double futurezetavL = 0;

//---------------------- Follower 1----------------------
double currentXF1 = 0;
double futureXF1 = 0;

double currentYF1 = 0;
double futureYF1 = 0;

double currentzetaF1 = 0;
double futurezetaF1 = 0;

//---------------------- Follower 2----------------------
double currentXF2 = 0;
double futureXF2 = 0;

double currentYF2 = 0;
double futureYF2 = 0;

double currentzetaF2 = 0;
double futurezetaF2 = 0;

//---------------------- Follower 3----------------------
double currentXF3 = 0;
double futureXF3 = 0;

double currentYF3 = 0;
double futureYF3 = 0;

double currentzetaF3 = 0;
double futurezetaF3 = 0;

//----------------------- Velocity ----------------------
double velocityXF1 = 0;
double velocityYF1 = 0;
double omegaF1 = 0;

double velocityXF2 = 0;
double velocityYF2 = 0;
double omegaF2 = 0;

double velocityXF3 = 0;
double velocityYF3 = 0;
double omegaF3 = 0;

double passzetavL = 0;

double passcurrentXF1 = 0;
double passcurrentYF1 = 0;
double passcurrentzetaF1 = 0;

double passcurrentXF2 = 0;
double passcurrentYF2 = 0;
double passcurrentzetaF2 = 0;

double passcurrentXF3 = 0;
double passcurrentYF3 = 0;
double passcurrentzetaF3 = 0;

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
  VvL = linx;;//sqrt((linx*linx)+(liny*liny));
  omegavL = angz ;

}
void handle_zeta(const geometry_msgs::Pose& zeta) 
{                                              //Reset the counter for number of main loops without communication
  
  currentzetavL = zeta.orientation.x;

}


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::TwistStamped> odomspeed("odomspeed",handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
ros::Subscriber<geometry_msgs::Twist> zetaodomsteelplate("zetaodomsteelplate",handle_zeta); 

geometry_msgs::Vector3Stamped virtualleader;                                
ros::Publisher virtualleader_msg("virtualleader", &virtualleader);
geometry_msgs::Vector3Stamped follower1;                                
ros::Publisher follower1_msg("follower1", &follower1);
geometry_msgs::Vector3Stamped follower2;                                
ros::Publisher follower2_msg("follower2", &follower2);
geometry_msgs::Vector3Stamped follower3;                                
ros::Publisher follower3_msg("follower3", &follower3);
geometry_msgs::Twist follower1velocity;                                
ros::Publisher follower1velocity_msg("holo1/cmd_vel", &follower1velocity);
geometry_msgs::Twist follower2velocity;                                
ros::Publisher follower2velocity_msg("holo2/cmd_vel", &follower2velocity);
geometry_msgs::Twist follower3velocity;                                
ros::Publisher follower3velocity_msg("holo3/cmd_vel", &follower3velocity);


geometry_msgs::Vector3Stamped report;                                
ros::Publisher report_msg("report", &report);
void changetodeg(double deg)
{
  double rad;
  rad = deg * Pi / 180  ;
  Serial.println(rad);
  currentzetavL = rad;
}
void setup() 
{
  //Serial.begin(115200);
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);
  nh.subscribe(odomspeed);
  nh.advertise(virtualleader_msg);                  //prepare to publish speed in ROS topic
  nh.advertise(follower1_msg); 
  nh.advertise(follower2_msg); 
  nh.advertise(follower3_msg);
  nh.advertise(follower1velocity_msg);
  nh.advertise(follower2velocity_msg);
  nh.advertise(follower3velocity_msg);  
  nh.advertise(report_msg);  
}

void loop() 
{
  nh.spinOnce();
  if((millis()-lastmillis) >= Looptime)
  {
    lastmillis = millis();
    //---------------- Virtual Leader ---------------
    if(omegavL == 0)
    {
      futurezetavL = currentzetavL;
      futureXvL = currentXvL + ((VvL*Looptime/1000)*cos(currentzetavL));
      futureYvL = currentYvL + ((VvL*Looptime/1000)*sin(currentzetavL));
  
      report.vector.x = VvL ;
      report.vector.y = currentzetavL ;
      report.vector.z = futurezetavL ;
      report_msg.publish(&report);
    
      passzetavL = currentzetavL ;
      //currentzetavL = futurezetavL; 
      currentXvL = futureXvL;
      currentYvL = futureYvL;
    
    }
    else if(omegavL != 0)
    {
      futurezetavL = currentzetavL + (omegavL*Looptime/1000);
      futureXvL = currentXvL + ((VvL/omegavL)*(sin(futurezetavL)-sin(currentzetavL)));
      futureYvL = currentYvL - ((VvL/omegavL)*(cos(futurezetavL)-cos(currentzetavL)));

      report.vector.x = VvL ;
      report.vector.y = currentzetavL ;
      report.vector.z = futurezetavL ;
      report_msg.publish(&report);
      
      passzetavL = currentzetavL ;
      //currentzetavL = futurezetavL;
      currentXvL = futureXvL;
      currentYvL = futureYvL;
    }

    
    //Serial.print("futurezeta : ");
    //Serial.print(futurezetavL);
    //Serial.print("  futureXvL : ");
    //Serial.print(futureXvL);
    //Serial.print("  futureYvL : ");
    //Serial.println(futureYvL);

    virtualleader.vector.x = futurezetavL ;
    virtualleader.vector.y = futureXvL ;
    virtualleader.vector.z = futureYvL ;
    virtualleader_msg.publish(&virtualleader);
    


  
    //------------ Follower Part -------------
    futureXF1 = futureXvL + (d1*cos(alpha1 + passzetavL));
    futureYF1 = futureYvL + (d1*sin(alpha1 + passzetavL));
  
    futureXF2 = futureXvL + (d2*cos(alpha2 + passzetavL));
    futureYF2 = futureYvL + (d2*sin(alpha2 + passzetavL));

    futureXF3 = futureXvL + (d3*cos(alpha3 + passzetavL));
    futureYF3 = futureYvL + (d3*sin(alpha3 + passzetavL));
  
    if(omegavL != 0)
    {
      futurezetaF1 = atan2(futureYF1-currentYF1,futureXF1-currentXF1); 
      futurezetaF2 = atan2(futureYF2-currentYF2,futureXF2-currentXF2); 
      futurezetaF3 = atan2(futureYF3-currentYF3,futureXF3-currentXF3);     
    }
    else if(omegavL == 0)
    {
      futurezetaF1 = futurezetavL ;
      futurezetaF2 = futurezetavL ;
      futurezetaF3 = futurezetavL ; 
    }

    currentzetavL = futurezetavL;
  
    passcurrentXF1 = currentXF1 ;
    currentXF1 = futureXF1 ;
    passcurrentYF1 = currentYF1 ;
    currentYF1 = futureYF1 ;
    passcurrentzetaF1 = currentzetaF1 ;
    currentzetaF1 = futurezetaF1 ;
  
    passcurrentXF2 = currentXF2 ;
    currentXF2 = futureXF2 ;
    passcurrentYF2 = currentYF2 ;
    currentYF2 = futureYF2 ;
    passcurrentzetaF2 = currentzetaF2 ;
    currentzetaF2 = futurezetaF2 ;

    passcurrentXF3 = currentXF3 ;
    currentXF3 = futureXF3 ;
    passcurrentYF3 = currentYF3 ;
    currentYF3 = futureYF3 ;
    passcurrentzetaF3 = currentzetaF3 ;
    currentzetaF3 = futurezetaF3 ;

    //Serial.print("futurezetaF1 : ");
    //Serial.print(futurezetaF1);
    //Serial.print("  futureXF1 : ");
    //Serial.print(futureXF1);
    //Serial.print("  futureYF1 : ");
    //Serial.println(futureYF1);
   
    follower1.vector.x = futureXF1 ;
    follower1.vector.y = futureYF1 ;
    follower1.vector.z = futurezetaF1 ;
     
    //Serial.print("futurezetaF2 : ");
    //Serial.print(futurezetaF2);
    //Serial.print("  futureXF2 : ");
    //Serial.print(futureXF2);
    //Serial.print("  futureYF2 : ");
    //Serial.println(futureYF2);

    follower2.vector.x = futureXF2 ;
    follower2.vector.y = futureYF2 ;
    follower2.vector.z = futurezetaF2 ;
    
    //Serial.print("futurezetaF3 : ");
    //Serial.print(futurezetaF3);
    //Serial.print("  futureXF3 : ");
    //Serial.print(futureXF3);
    //Serial.print("  futureYF3 : ");
    //Serial.println(futureYF3);

    follower3.vector.x = futureXF3 ;
    follower3.vector.y = futureYF3 ;
    follower3.vector.z = futurezetaF3 ;
    
    //Serial.println("------------------------------");


    velocityXF1 = ((futureXF1 - passcurrentXF1)*1000)/Looptime ;
    velocityYF1 = ((futureYF1 - passcurrentYF1)*1000)/Looptime ;
    omegaF1 = ((futurezetaF1 - passcurrentzetaF1)*1000)/Looptime;
    
    velocityXF2 = ((futureXF2 - passcurrentXF2)*1000)/Looptime ;
    velocityYF2 = ((futureYF2 - passcurrentYF2)*1000)/Looptime ;
    omegaF2 = ((futurezetaF2 - passcurrentzetaF2)*1000)/Looptime;

    velocityXF3 = ((futureXF3 - passcurrentXF3)*1000)/Looptime ;
    velocityYF3 = ((futureYF3 - passcurrentYF3)*1000)/Looptime ;
    omegaF3 = ((futurezetaF3 - passcurrentzetaF3)*1000)/Looptime;
    
    
    //Serial.print("velocityXF1 : ");
    //Serial.print(velocityXF1);
    //Serial.print("  velocityYF1 : ");
    //Serial.print(velocityYF1);
    //Serial.print("  omegaF1 : ");
    //Serial.println(omegaF1);

    follower1velocity.linear.x = velocityXF1 ;
    follower1velocity.linear.y = velocityYF1 ; // 0 ;
    follower1velocity.angular.z = omegaF1 ;
    
    //Serial.print("velocityXF2 : ");
    //Serial.print(velocityXF2);
    //Serial.print("  velocityYF2 : ");
    //Serial.print(velocityYF2);
    //Serial.print("  omegaF2 : ");
    //Serial.println(omegaF2);

    follower2velocity.linear.x = velocityXF2 ;
    follower2velocity.linear.y = velocityYF2 ; // 0 ;
    follower2velocity.angular.z = omegaF2 ;
    
    //Serial.print("velocityXF3 : ");
    //Serial.print(velocityXF3);
    //Serial.print("  velocityYF3 : ");
    //Serial.print(velocityYF3);
    //Serial.print("  omegaF3 : ");
    //Serial.println(omegaF3);

    follower3velocity.linear.x = velocityXF3 ;
    follower3velocity.linear.y = velocityYF3 ; // 0 ; 
    follower3velocity.angular.z = omegaF3 ;
    
    //Serial.println("------------------------------");

    follower1_msg.publish(&follower1);
    follower2_msg.publish(&follower2);
    follower3_msg.publish(&follower3);
    follower1velocity_msg.publish(&follower1velocity);
    follower2velocity_msg.publish(&follower2velocity);
    follower3velocity_msg.publish(&follower3velocity);
  }

  

}
