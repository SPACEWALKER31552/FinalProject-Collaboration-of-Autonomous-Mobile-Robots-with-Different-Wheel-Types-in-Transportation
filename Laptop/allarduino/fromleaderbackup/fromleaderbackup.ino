#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <MPU6050_light.h>
 MPU6050 mpu(Wire);
 
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter

const int PIN_ENCOD_A_MOTOR_LEFT = 2;               //A channel for encoder of left motor                    
const int PIN_ENCOD_B_MOTOR_LEFT = 8;               //B channel for encoder of left motor

const int PIN_ENCOD_A_MOTOR_RIGHT = 3;              //A channel for encoder of right motor         
const int PIN_ENCOD_B_MOTOR_RIGHT = 9;              //B channel for encoder of right motor 


unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.06;                   //Wheel radius, in m
const double b = 0.057;
const double wheelbase = 0.57;               //Wheelbase, in m
const double encoder_cpr = 600;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.0393; //0.28   // 0.00235 Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.09;          //0.0882 (min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double speedlinx = 0;
double speedliny = 0;
double speedangz = 0;                 //Desired angular speed for the robot, in rad/s

double orientationX = 0;
double orientationY = 0;
double orientationZ = 0;
double orientationW = 0;

double siny_cosp = 0 ;
double cosy_cosp = 0 ;
double theta = 0 ;

double speed_req_left = 0;                    //Desired speed for left wheel in m/s
double speed_act_left = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_left = 0;                    //Command speed for left wheel in m/s 

double speed_req_right = 0;                   //Desired speed for right wheel in m/s
double speed_act_right = 0;                   //Actual speed for right wheel in m/s
double speed_cmd_right = 0;                   //Command speed for right wheel in m/s 

double forleftwheel = 0 ;
double forrightwheel = 0 ;
                        
const double max_speed = 0.4;                 //Max speed in m/s

//int PWM_leftMotor = 0;                     //PWM command for left motor
//int PWM_rightMotor = 0;                    //PWM command for right motor 
         
volatile float pos_left = 0;       //Left motor encoder position
volatile float pos_right = 0;      //Right motor encoder position

float I ;
float H ;
float EF ;
float BC ;

int dir1 , dir2  ; //wheel direction 1 = foward / 2 = backward

int ia ;

//PID

int PWM_leftMotor = 0;
int PWM_rightMotor = 0;

double goalangleleader = 0.0;
double goalangleleaderold = 0.0;
float eprevleader = 0;

const double PID_param[] = { 0.4, 0, 0 }; //Respectively Kp, Ki and Kd for left motor PID
PID PID_leftMotor(&speed_act_left,&speed_cmd_left, &speed_req_left, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_rightMotor(&speed_act_right,&speed_cmd_right, &speed_req_left, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor

//motor driver setup
/*
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);      //Create left motor object
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);     //Create right motor object
*/
class motor{
    public:
        int pinA;
        int pinB;
        int pinPWM;
    
    void motor_setup(){
        pinMode(pinA,OUTPUT);
        pinMode(pinB,OUTPUT);
        pinMode(pinPWM,OUTPUT);
    }

    void setSpeed(const int speedInput){
        analogWrite(pinPWM, speedInput);
    }

    void run(const String runInput){
       if(runInput == "BRAKE"){
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, HIGH);
       }
       else if(runInput == "FORWARD"){
            digitalWrite(pinA, HIGH);
            digitalWrite(pinB, LOW);
       }
       else if(runInput == "BACKWARD"){
            digitalWrite(pinA, LOW);
            digitalWrite(pinB, HIGH);
       }
    }
};

ros::NodeHandle nh;

geometry_msgs::Vector3Stamped encode;                                
ros::Publisher encode_msg("leaderencode", &encode);       

geometry_msgs::Vector3Stamped speedreq;                                
ros::Publisher speed_reqmsg("leaderspeedreq", &speedreq); 

geometry_msgs::Vector3Stamped PWMout;                                
ros::Publisher PWM_msg("leaderPWM", &PWMout); 

geometry_msgs::Vector3Stamped IMUvalue;                                
ros::Publisher IMUvalue_msg("leaderIMU", &IMUvalue); 

//function that will be called when receiving command from host
void handle_cmd(const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  
  speedlinx = cmd_vel.linear.x;
  /*if(speedlinx > 0.14 )
  {
    speedlinx = 0.14 ;
  }
  else
  {}*/
  speedliny = cmd_vel.linear.y;
  speedangz = 0.0; //cmd_vel.angular.z;
  /*if(speedangz > 0.07 )
  {
    speedangz = 0.07 ;
  }*/
}

void handle_odom(const geometry_msgs::Pose& leaderfortheta) 
{       
 
  //orientationX = odom.orientation.x;
  //orientationY = odom.orientation.y;
  //orientationZ = odom.orientation.z;
  //orientationW = odom.orientation.w;
  //orientationY = odom.pose.pose.orientation.y;
  //orientationZ = odom.pose.pose.orientation.z;
  //orientationW = odom.pose.pose.orientation.w;
  
  theta = leaderfortheta.orientation.x;
                        
}


void handle_holoodompy(const geometry_msgs::Twist& holoodompy) 
{       
 
  goalangleleader = holoodompy.angular.x ;
                        
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("cmd_vel",handle_cmd); //cmd_vel  //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
//ros::Subscriber<geometry_msgs::Twist> cmd_vel("nav1/cmd_vel",handle_cmd);
ros::Subscriber<geometry_msgs::Pose> leaderfortheta("leaderfortheta",handle_odom);
ros::Subscriber<geometry_msgs::Twist> holoodompy("holo/odompy",handle_holoodompy);
geometry_msgs::Vector3Stamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("leaderspeed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type

//motor driver setup
motor leftMotor, rightMotor;
long prevT = 0;
float eprevright = 0;
float eprevleft = 0;
float eintegralright = 0;
float eintegralleft = 0;

//__________________________________________________________________________

void setup() 
{
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets(); // gyro and accelero
  
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.subscribe(leaderfortheta);
  nh.subscribe(holoodompy);
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  nh.advertise(encode_msg);                  //prepare to publish speed in ROS topic
  nh.advertise(speed_reqmsg);                  //prepare to publish speed in ROS topic
  nh.advertise(PWM_msg);
  nh.advertise(IMUvalue_msg);

  H = radius/(2*b);
  I = -radius/(2*b);

  
  //AFMS.begin();
  leftMotor.pinA = 12;
  leftMotor.pinB = 11;
  leftMotor.pinPWM = 10;
  rightMotor.pinA = 6;
  rightMotor.pinB = 7;
  rightMotor.pinPWM = 5;
  leftMotor.motor_setup();
  rightMotor.motor_setup();
  //setting motor speeds to zero
  leftMotor.setSpeed(0);
  leftMotor.run("BRAKE");
  rightMotor.setSpeed(0);
  rightMotor.run("BRAKE");
 
  //setting PID parameters
    //set PID
  PID_leftMotor.SetSampleTime(95);
  PID_leftMotor.SetOutputLimits(-max_speed, max_speed);
  PID_leftMotor.SetMode(AUTOMATIC);

  PID_rightMotor.SetSampleTime(95);
  PID_rightMotor.SetOutputLimits(-max_speed, max_speed);
  PID_rightMotor.SetMode(AUTOMATIC);

   
  // Define the rotary encoder for left motor
  pinMode(PIN_ENCOD_A_MOTOR_LEFT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_LEFT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_LEFT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_LEFT, HIGH);
  attachInterrupt(0, encoderLeftMotor, RISING);

  // Define the rotary encoder for right motor
  pinMode(PIN_ENCOD_A_MOTOR_RIGHT, INPUT); 
  pinMode(PIN_ENCOD_B_MOTOR_RIGHT, INPUT); 
  digitalWrite(PIN_ENCOD_A_MOTOR_RIGHT, HIGH);                // turn on pullup resistor
  digitalWrite(PIN_ENCOD_B_MOTOR_RIGHT, HIGH);
  attachInterrupt(1, encoderRightMotor, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();
  if((millis()-lastMilli) >= LOOPTIME)   
  {             
    // enter timed loop
    lastMilli = millis();
    
    byte status = mpu.begin();
    //mpu.update();
   
    IMUvalue.vector.x = mpu.getAngleZ();
    IMUvalue.vector.y = int(mpu.getAngleZ())%360;
    
    
    if (abs(pos_left) < 5){                                                   //Avoid taking in account small disturbances
      speed_act_left = 0;
    }
    else {
      speed_act_left=((pos_left/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;           // calculate speed of left wheel
    }
    
    if (abs(pos_right) < 5){                                                  //Avoid taking in account small disturbances
      speed_act_right = 0;
    }
    else {
    speed_act_right=((pos_right/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius;          // calculate speed of right wheel
    }
    encode.vector.x = pos_left;    //left wheel speed (in m/s)
    encode.vector.y = pos_right;   //right wheel speed (in m/s)
    pos_left = 0;
    pos_right = 0;

    /*PWM_rightMotor = round(1100);
    rightMotor.setSpeed(abs(PWM_rightMotor));
    rightMotor.run("BACKWARD");
    leftMotor.setSpeed(abs(PWM_rightMotor));
    leftMotor.run("BACKWARD");*/
    
    if(goalangleleader != goalangleleaderold)
    {
    float kpp = 0.05;

    // errorleader
    int eleader = goalangleleader - mpu.getAngleZ();

    // control signal
    float uleader = kpp * eleader ;

    // store previous error
    eprevleader = eleader;
    goalangleleaderold = goalangleleader;
    speedangz = speedangz + uleader ;
    }
    
    theta = theta * 180 / 3.141592653589793238462643383279502 ;
    if(theta > 360)
    {
      theta = abs(theta - 360);
    }
    
    encode.vector.z = theta ;
    theta = theta * 3.141592653589793238462643383279502 / 180 ;

    BC = (radius*abs(cos(theta)))/2 ;
    EF = (radius*abs(sin(theta)))/2 ;

    //speed_req_right = (((speedlinx + speedliny)*I) - ((BC+EF)*(speedangz/0.25)))/(((BC+EF)*I)-((BC+EF)*H)) ;
    //speed_req_left = (((speedlinx + speedliny)*H) - ((BC+EF)*(speedangz/0.25)))/(((BC+EF)*H)-((BC+EF)*I)) ;
      
    speed_req_left = speedlinx - speedangz*(wheelbase/2);     //Calculate the required speed for the left motor to comply with commanded linear and angular speeds
    speed_req_right = speedlinx + speedangz*(wheelbase/2); 
    /*if(speedangz > 0)
    {
      forleftwheel = -1*(speedangz*(wheelbase/2));  
      forrightwheel = (speedangz*(wheelbase/2));
    }
    else if(speedangz < 0)
    {
      forleftwheel = (speedangz*(wheelbase/2));  
      forrightwheel = -1*(speedangz*(wheelbase/2));
    }
    else if(speedangz == 0)
    {
      forleftwheel = 0 ;  
      forrightwheel = 0 ;
    }     
    
    speed_req_right = speedlinx + forrightwheel ;
    speed_req_left = speedlinx + forleftwheel ;
    */

    speedreq.vector.x =  speed_req_left;
    speedreq.vector.y =  speed_req_right;
    speedreq.vector.z =  goalangleleader;

  
    /*speed_cmd_left = constrain(speed_cmd_left, -max_speed, max_speed);
    speed_cmd_right = constrain(speed_cmd_right, -max_speed, max_speed);
    PID_leftMotor.Compute();
    PID_rightMotor.Compute();                             
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_leftMotor = constrain(((speed_req_left+sgn(speed_req_left)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_left/speed_to_pwm_ratio), -255, 255); 
    PWM_rightMotor = constrain(((speed_req_right+sgn(speed_req_right)*min_speed_cmd)/speed_to_pwm_ratio) + (speed_cmd_right/speed_to_pwm_ratio), -255, 255);*/
     float kp = 0.9;
     float kd = 0.025;
     float ki = 0.0;
     long currT = micros();
     
     float deltaT = ((float) (currT - prevT))/( 1.0e6 );
     prevT = currT;

     float eright = speed_req_right - speed_act_right;
     eprevright = eright;

     // derivative
     float dedtright = (eright-eprevright)/(deltaT);

     // integral
     eintegralright = eintegralright + eright*deltaT;

      // control signal
     float uright = kp*eright + kd*dedtright + ki*eintegralright;

     PWM_rightMotor = uright*(255/0.3) ;
          
     PWM_rightMotor = constrain(PWM_rightMotor,-255,255);

     
     float eleft = speed_req_left - speed_act_left;
     eprevleft = eleft;

     // derivative
     float dedtleft = (eleft-eprevleft)/(deltaT);

     // integral
     eintegralleft = eintegralleft + eleft*deltaT;

      // control signal
     float uleft = kp*eleft + kd*dedtleft + ki*eintegralleft;

     PWM_leftMotor = uleft*(255/0.3) ;
          
     PWM_leftMotor = constrain(PWM_leftMotor,-255,255);  

    if(PWM_leftMotor >= 0) // forward left
    {
      leftMotor.run("FORWARD");
    }
    else if(PWM_leftMotor < 0)
    {
      leftMotor.run("BACKWARD");
    }

    if(PWM_rightMotor >= 0) // forward left
    {
      rightMotor.run("FORWARD");
    }
    else if(PWM_rightMotor < 0)
    {
      rightMotor.run("BACKWARD");
    }
    /*PWM_leftMotor = speed_req_left/0.3 ;//0.00235   //PWM command for left motor
    PWM_rightMotor = speed_req_right/0.3 ;  //0.00235 
        
    if(PWM_leftMotor >= 255)
    {
      PWM_leftMotor = 255 ;
      dir1 = 1 ;
    }
    else if(PWM_leftMotor < 0)
    {
      PWM_leftMotor = abs(PWM_leftMotor) ;
      dir1 = 2 ;
      if (PWM_leftMotor >= 255)
      {
        PWM_leftMotor = 255 ;
      }
    }
    else if(PWM_leftMotor == 0)
    {
      PWM_leftMotor = 0;
      dir1 = 0 ;
      //leftMotor.setSpeed(PWM_leftMotor);
      //leftMotor.run("BRAKE");
    }
    else
    {
      dir1 = 1 ;
    }

    if(PWM_rightMotor >= 255)
    {
      PWM_rightMotor = 255 ;
      dir2 = 1 ;
    }
    else if(PWM_rightMotor < 0)
    {
      PWM_rightMotor = abs(PWM_rightMotor) ;
      dir2 = 2 ;
      if (PWM_rightMotor >= 255)
      {
        PWM_rightMotor = 255 ;
      }
    }
    else if(PWM_rightMotor == 0)
    {
      PWM_rightMotor = 0;
      dir2 = 0 ;
      //rightMotor.setSpeed(PWM_rightMotor);
      //rightMotor.run("BRAKE");
    }
    else
    {
      dir2 = 1 ;
    }

    if(dir1 == 1) // forward left
    {
      leftMotor.run("FORWARD");
    }
    else if(dir1 == 2)
    {
      leftMotor.run("BACKWARD");
    }
    if(dir2 == 1) // forward left
    {
      rightMotor.run("FORWARD");
    }
    else if(dir2 == 2)
    {
      rightMotor.run("BACKWARD");
    }*/

    PWMout.vector.x = PWM_leftMotor ;
    PWMout.vector.y = PWM_rightMotor ; 
    rightMotor.setSpeed(abs(PWM_rightMotor));
    leftMotor.setSpeed(abs(PWM_leftMotor));
    nh.spinOnce();
     
    
    noCommLoops++;
    if (noCommLoops == 65535){
      noCommLoops = noCommLoopMax;
    }

   publishSpeed(LOOPTIME);   //Publish odometry on ROS topic
  }
 }
//Publish function for odometry, uses a vector type message to send the data (message type is not meant for that but that's easier than creating a specific message type)
void publishSpeed(double time) 
{
  speed_reqmsg.publish(&speedreq);
  encode_msg.publish(&encode);
  PWM_msg.publish(&PWMout);
  IMUvalue_msg.publish(&IMUvalue);
  nh.spinOnce();
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.vector.x = speed_act_left;    //left wheel speed (in m/s)
  speed_msg.vector.y = speed_act_right;   //right wheel speed (in m/s)
  speed_msg.vector.z = time/1000;         //looptime, should be the same as specified in LOOPTIME (in s)
  speed_pub.publish(&speed_msg);
  nh.loginfo("Publishing speed_msgs");
}

//Left motor encoder counter
void encoderLeftMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_LEFT) == digitalRead(PIN_ENCOD_B_MOTOR_LEFT)) pos_right--;
  else pos_right++;
}

//Right motor encoder counter
void encoderRightMotor() {
  if (digitalRead(PIN_ENCOD_A_MOTOR_RIGHT) == digitalRead(PIN_ENCOD_B_MOTOR_RIGHT)) pos_left++;
  else pos_left--;
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
