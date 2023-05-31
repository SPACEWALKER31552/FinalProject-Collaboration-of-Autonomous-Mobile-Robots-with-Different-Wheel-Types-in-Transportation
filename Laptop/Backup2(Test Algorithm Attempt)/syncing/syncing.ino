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
double leaderzeta = 0.0 ;
double slavezeta = 0.0 ;
double slave2zeta = 0.0 ;

double linx = 0;
double liny = 0;
double linz = 0;
double angx = 0;
double angy = 0;
double angz = 0;

double leaderlinx = 0;
double leaderliny = 0;
double leaderlinz = 0;
double leaderangx = 0;
double leaderangy = 0;
double leaderangz = 0;

double slavelinx = 0;
double slaveliny = 0;
double slavelinz = 0;
double slaveangx = 0;
double slaveangy = 0;
double slaveangz = 0;

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

#define Looptime 100 //millisec
int sec = 1 ;
#define l 1.697 
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
 double r = 1.696827923 ;
 //double r = 3 ;
 double slave2x = 0.0;
 double slave2y = 0.0;
 double slavex = 0.0;
 double slavey = 0.0;

 unsigned long timer = 0;

float eprevslavevelo = 0;
float eintegralslavevelo = 0;

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

ros::NodeHandle nh;

void handle_cmd(const geometry_msgs::Twist& cmd_vel) 
{                                              //Reset the counter for number of main loops without communication
  
  linx = cmd_vel.linear.x;
  liny = cmd_vel.linear.y;
  linz = cmd_vel.linear.z;
  angx = cmd_vel.angular.x;
  angy = cmd_vel.angular.y;
  angz = cmd_vel.angular.z;


}

void handle_leaderspeed(const geometry_msgs::Vector3Stamped& cmd_vel) 
{                                              //Reset the counter for number of main loops without communication
  
  leaderlinx = ((cmd_vel.vector.x)+(cmd_vel.vector.y))/2;
  
 

}

void handle_slavespeed(const geometry_msgs::Twist& cmd_vel) 
{                                              //Reset the counter for number of main loops without communication
  
  slavelinx = cmd_vel.linear.x;
  slaveliny = cmd_vel.linear.y;
  slavelinz = cmd_vel.linear.z;
  slaveangx = cmd_vel.angular.x;
  slaveangy = cmd_vel.angular.y;
  slaveangz = cmd_vel.angular.z;
 

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


ros::Subscriber<geometry_msgs::Twist> cmd("cmd_vel",handle_cmd);   //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
ros::Subscriber<geometry_msgs::Vector3Stamped> imuslave("slaveIMU",handle_slave);
ros::Subscriber<geometry_msgs::Vector3Stamped> imuslave2("slave2IMU",handle_slave2);
ros::Subscriber<geometry_msgs::Vector3Stamped> imuleader("leaderIMU",handle_leader);
ros::Subscriber<geometry_msgs::Vector3Stamped> speedleader("leaderspeed",handle_leaderspeed);
ros::Subscriber<geometry_msgs::Twist> speedslave("slavespeed",handle_slavespeed);

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

void setup() 
 {
  //Serial.begin(115200);
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);
  nh.subscribe(cmd);
  nh.subscribe(imuslave);
  nh.subscribe(imuslave2);
  nh.subscribe(imuleader); 
  nh.subscribe(speedleader);  
  nh.subscribe(speedslave);  
  
  nh.advertise(follower1velocity_msg);
  nh.advertise(follower2velocity_msg);
  nh.advertise(follower3velocity_msg);
  nh.advertise(masterkinematicleader_msg);
  nh.advertise(masterkinematicslave_msg);
  nh.advertise(masterkinematicslave2_msg); 
 }


 void loop()
 {
  
  nh.spinOnce();
  if((millis()-lastmillis) >= Looptime)
  {
  lastmillis = millis();
  if(linx != 0)
  { 
    goalangleleader = leaderzeta ;
    /*
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
    }*/
  }  
  else
  {
    goalangleleader = 0 ;
    goalangleslave = 0 ;
    goalangleslave2 = 0 ;
  }
  

  // PID constants
  float kp = 0.03;
  float kd = 0.001;
  float ki = 0.0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  // errorvelocity
  int eslavevelo = leaderlinx - slavelinx;

  // derivative
  float dedtslavevelo = (eslavevelo-eprevslavevelo)/(deltaT);

  // integral
  eintegralslavevelo = eintegralslavevelo + eslavevelo*deltaT;

  // control signal
  float uslavevelolinx = 1*eslavevelo + 0.1*dedtslavevelo + ki*eintegralslavevelo;

  // store previous error
  eprevslavevelo = eslavevelo;
  

  /*
  // errorleader
  int eleader = goalangleleader - leaderzeta;

  // derivative
  float dedtleader = (eleader-eprevleader)/(deltaT);

  // integral
  eintegralleader = eintegralleader + eleader*deltaT;

  // control signal
  float uleader = 0.7*eleader + kd*dedtleader + ki*eintegralleader; */


/*  
  // error
  int eslave = goalangleleader - slavezeta;

  // derivative
  float dedtslave = (eslave-eprevslave)/(deltaT);

  // integral
  eintegralslave = eintegralslave + eslave*deltaT;

  // control signal
  float uslave = kp*eslave + kd*dedtslave + ki*eintegralslave;



  // error2
  int eslave2 = goalangleslave2 - slave2zeta;

  // derivative2
  float dedtslave2 = (eslave2-eprevslave2)/(deltaT);

  // integral2
  eintegralslave2 = eintegralslave2 + eslave2*deltaT;

  // control signal2
  float uslave2 = kp*eslave2 + kd*dedtslave2 + ki*eintegralslave2;
*/
  
  if(linx == 0)
  {
    //uslave = 0 ;
    //uslave2 = 0 ;   
    //uleader = 0 ;
  }

  follower1velocity.linear.x = 3*linx ;
  follower1velocity.angular.z = (5*angz) ;
  follower1velocity_msg.publish(&follower1velocity);
  
  follower2velocity.linear.x = linx ;
  follower2velocity.angular.z = 0;//uslave ;
  follower2velocity_msg.publish(&follower2velocity);

  /*
  follower3velocity.linear.x = linx ;
  follower3velocity.angular.z = angz+uslave2 ;
  follower3velocity_msg.publish(&follower3velocity);
*/
  }
}
