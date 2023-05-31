
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
double currentzeta = 0 ;

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
double VL = 0.0 ;
double VR = 0.0 ;

#define Looptime 1000 //millisec
int sec = 1 ;
#define l 1.697 
#define Pi 3.14159 
#define alpha1 2.514
#define alpha2 3.768515
#define alpha3 0
#define d1 0.963133
#define d2 0.963133
#define d3 0.82

void handle_cmd(const geometry_msgs::Twist& cmd_vel) 
{                                              //Reset the counter for number of main loops without communication
  
  linx = cmd_vel.linear.x;
  liny = cmd_vel.linear.y;
  linz = cmd_vel.linear.z;
  angx = cmd_vel.angular.x;
  angy = cmd_vel.angular.y;
  angz = cmd_vel.angular.z;

}
void handle_zeta(const geometry_msgs::Pose& zeta) 
{                                              //Reset the counter for number of main loops without communication
  
  currentzeta = zeta.orientation.x;

}


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> odomspeed("cmd_vel",handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
ros::Subscriber<geometry_msgs::Twist> zetaodomsteelplate("zetaodomsteelplate",handle_zeta); 

geometry_msgs::Twist follower1velocity;                                
ros::Publisher follower1velocity_msg("holo1/cmd_vel", &follower1velocity);
geometry_msgs::Twist follower2velocity;                                
ros::Publisher follower2velocity_msg("holo2/cmd_vel", &follower2velocity);
geometry_msgs::Twist follower3velocity;                                
ros::Publisher follower3velocity_msg("holo3/cmd_vel", &follower3velocity);


void changetodeg(double deg)
{
  double rad;
  rad = deg * Pi / 180  ;
  Serial.println(rad);
  currentzeta = rad;
}
void setup() 
{
  //Serial.begin(115200);
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);
  nh.subscribe(odomspeed);
  nh.advertise(follower1velocity_msg);
  nh.advertise(follower2velocity_msg);
  nh.advertise(follower3velocity_msg);   
}

void loop() 
{
  nh.spinOnce();
  if((millis()-lastmillis) >= Looptime)
  {
    lastmillis = millis();
    
    
    
    
    follower1velocity.linear.x = 1.5*linx;
    follower1velocity.linear.y = 0;
    follower1velocity.angular.z = angz;

    VL = linx - (1.22*angz/2);
    VR = linx + (1.22*angz/2);

    
    follower2velocity.linear.x = VL;
    follower2velocity.linear.y = 0;
    
    follower3velocity.linear.x = VR;
    follower3velocity.linear.y = 0;
    
    follower1velocity_msg.publish(&follower1velocity);
    follower2velocity_msg.publish(&follower2velocity);
    follower3velocity_msg.publish(&follower3velocity);

  }
}
