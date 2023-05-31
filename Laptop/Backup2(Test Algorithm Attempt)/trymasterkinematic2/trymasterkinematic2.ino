
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


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::TwistStamped> odomspeed("odomspeed",handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
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
    
    //cross to find omega component slave
    slaveturnicomponent = (slaveRz*angy)-(slaveRy*angz);
    slaveturnjcomponent = (slaveRx*angz)-(slaveRz*angx);
    slaveturnkcomponent = (slaveRy*angx)-(slaveRx*angy);

    /*Serial.print("slaveturnicomponent : ");
    Serial.print(slaveturnicomponent);
    Serial.print("  slaveturnjcomponent : ");
    Serial.print(slaveturnjcomponent);
    Serial.print("  slaveturnkcomponent : ");
    Serial.println(slaveturnkcomponent);*/
      
    //cross to find omega component slave2
    slave2turnicomponent = (slave2Rz*angy)-(slave2Ry*angz);
    slave2turnjcomponent = (slave2Rx*angz)-(slave2Rz*angx);
    slave2turnkcomponent = (slave2Ry*angx)-(slave2Rx*angy);

    /*Serial.print("slave2turnicomponent : ");
    Serial.print(slave2turnicomponent);
    Serial.print("  slave2turnjcomponent : ");
    Serial.print(slave2turnjcomponent);
    Serial.print("  slave2turnkcomponent : ");
    Serial.println(slave2turnkcomponent);*/
   
    //cross to find omega component master
    leaderturnicomponent = (leaderRz*angy)-(leaderRy*angz);
    leaderturnjcomponent = (leaderRx*angz)-(leaderRz*angx);
    leaderturnkcomponent = (leaderRy*angx)-(leaderRx*angy);
    
    /*Serial.print("leaderturnicomponent : ");
    Serial.print(leaderturnicomponent);
    Serial.print("  leaderturnjcomponent : ");
    Serial.print(leaderturnjcomponent);
    Serial.print("  leaderturnkcomponent : ");
    Serial.println(leaderturnkcomponent);*/
    
    follower1velocity.linear.x = linx + leaderturnjcomponent;
    follower1velocity.linear.y = -leaderturnicomponent;
    
    follower2velocity.linear.x = linx + slaveturnjcomponent;
    follower2velocity.linear.y = -slaveturnicomponent;
    
    follower3velocity.linear.x = linx + slave2turnjcomponent;
    follower3velocity.linear.y = -slave2turnicomponent;
    
    follower1velocity_msg.publish(&follower1velocity);
    follower2velocity_msg.publish(&follower2velocity);
    follower3velocity_msg.publish(&follower3velocity);

  }
}
