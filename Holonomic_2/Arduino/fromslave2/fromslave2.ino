
#include <Wire.h>
#include <PID_v1.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/time.h>
#include <nav_msgs/Odometry.h>
#include <MPU6050_light.h>
 MPU6050 mpu(Wire);
 
//initializing all the variables
#define LOOPTIME                      100     //Looptime in millisecond
const byte noCommLoopMax = 10;                //number of main loops the robot will execute without communication before stopping
unsigned int noCommLoops = 0;                 //main loop without communication counter


#define PWM_motor1 6 //2
#define PWM_motor2 4
#define PWM_motor3 12
#define PWM_motor4 13

#define Direction_motor1 14 //3
#define Direction_motor2 5
#define Direction_motor3 30
#define Direction_motor4 32

// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define encoder_encoder1A 2 //21
#define encoder_encoder2A 3 //20
#define encoder_encoder3A 19
#define encoder_encoder4A 18

// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define encoder_encoder1B 22
#define encoder_encoder2B 24
#define encoder_encoder3B 26
#define encoder_encoder4B 28

// True = Forward; False = Reverse
boolean dir1 = true;
boolean dir2 = true;
boolean dir3 = true;
boolean dir4 = true;

int dirreal1 = 0 ;
int dirreal2 = 0 ;
int dirreal3 = 0 ;
int dirreal4 = 0 ;

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
volatile int counter1 = 0;
volatile int counter2 = 0;
volatile int counter3 = 0;
volatile int counter4 = 0;

volatile int lastpos1 = 0 ;
volatile int lastpos2 = 0 ;
volatile int lastpos3 = 0 ;
volatile int lastpos4 = 0 ;

unsigned long lastMilli = 0;

//--- Robot-specific constants ---
const double radius = 0.076;                   //Wheel radius, in m
const double wheelbase = 0.55;               //Wheelbase, in m
const double encoder_cpr = 360;               //Encoder ticks or counts per rotation
const double speed_to_pwm_ratio = 0.0383;    // 0.00235 Ratio to convert speed (in m/s) to PWM value. It was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the slope of the linear function).
const double min_speed_cmd = 0.09;          //0.0882 (min_speed_cmd/speed_to_pwm_ratio) is the minimum command value needed for the motor to start moving. This value was obtained by plotting the wheel speed in relation to the PWM motor command (the value is the constant of the linear function).

double linx = 0;
double liny = 0;
double linz = 0;
double angx = 0;
double angy = 0;
double angz = 0;                 //Desired angular speed for the robot, in rad/s

double speed_req_motor1 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor1 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor1 = 0;                    //Command speed for left wheel in m/s 

double speed_req_motor2 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor2 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor2 = 0;                    //Command speed for left wheel in m/s 

double speed_req_motor3 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor3 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor3 = 0;                    //Command speed for left wheel in m/s 

double speed_req_motor4 = 0;                    //Desired speed for left wheel in m/s
double speed_act_motor4 = 0;                    //Actual speed for left wheel in m/s
double speed_cmd_motor4 = 0;                    //Command speed for left wheel in m/s 

                        
const double max_speed = 0.4;                 //Max speed in m/s

int PWM1 = 0;                     //PWM command for left motor
int PWM2 = 0;                    //PWM command for right motor 
int PWM3 = 0;                     //PWM command for left motor
int PWM4 = 0;                    //PWM command for right motor 
              
//PID

int PWM_Motor1 = 0;
int PWM_Motor2 = 0;
int PWM_Motor3 = 0;
int PWM_Motor4 = 0;


double goalangleslave2 = 0.0;
double goalangleslave2old = 0.0;
float eprevslave2 = 0;

const double PID_param[] = { 0.4, 0, 0.001 }; //Respectively Kp, Ki and Kd for left motor PID
PID PID_Motor1(&speed_act_motor1,&speed_cmd_motor1, &speed_req_motor1, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_Motor2(&speed_act_motor2,&speed_cmd_motor2, &speed_req_motor2, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_Motor3(&speed_act_motor3,&speed_cmd_motor3, &speed_req_motor3, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
PID PID_Motor4(&speed_act_motor4,&speed_cmd_motor4, &speed_req_motor4, PID_param[0], PID_param[1], PID_param[2], DIRECT);          //Setting up the PID for left motor
//motor driver setup
/*
Adafruit_MotorShield AFMS = Adafruit_MotorShield();  // Create the motor shield object with the default I2C address
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);      //Create left motor object
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2);     //Create right motor object
*/
class motor{
    public:
        int pinA;
        int pinPWM;
    
    void motor_setup(){
        pinMode(pinA,OUTPUT);
        pinMode(pinPWM,OUTPUT);
    }

    void setSpeed(const int speedInput){
        analogWrite(pinPWM, speedInput);
    }

    void run(const String runInput){
       if(runInput == "BRAKE"){
            digitalWrite(pinA, HIGH);
            analogWrite(pinPWM, 0);
       }
       else if(runInput == "FORWARD"){
            digitalWrite(pinA, HIGH);
       }
       else if(runInput == "BACKWARD"){
            digitalWrite(pinA, LOW);
       }
    }
};

ros::NodeHandle nh;

geometry_msgs::Twist encode;                                
ros::Publisher encode_msg("slave2encode", &encode);       

geometry_msgs::Twist speedreq;                                
ros::Publisher speed_reqmsg("slave2speedreq", &speedreq);   

geometry_msgs::Twist PWMout;                                
ros::Publisher PWM_msg("slave2PWM", &PWMout);      

geometry_msgs::Vector3Stamped IMUvalue;                                
ros::Publisher IMUvalue_msg("slave2IMU", &IMUvalue);

//function that will be called when receiving command from host
void handle_cmd(const geometry_msgs::Twist& cmd_vel) {
  noCommLoops = 0;                                                  //Reset the counter for number of main loops without communication
  linx = cmd_vel.linear.x;
  /*if(linx > 0.1)
  {
    linx = 0.1 ;
  }
  else
  {}*/
  liny = cmd_vel.linear.y;
  /*if(liny > 0.08)
  {
    liny = 0.08 ;
  }
  if(liny < -0.08)
  {
    liny = -0.08 ;
  }
  else
  {}*/
  linz = cmd_vel.linear.z;
  angx = cmd_vel.angular.x;
  angy = cmd_vel.angular.y;
  angz = cmd_vel.angular.z;
  /*if(angz > 0.07)
  {
    angz = 0.07 ;
  }
  if(angz < -0.07)
  {
    angz = -0.07 ;
  }*/
}
void handle_holoodompy(const geometry_msgs::Twist& holoodompy) 
{       
 
  goalangleslave2= holoodompy.angular.z ;
                        
}

ros::Subscriber<geometry_msgs::Twist> cmd_vel("holo3/cmd_vel",handle_cmd); //cmd_vel  //create a subscriber to ROS topic for velocity commands (will execute "handle_cmd" function when receiving data)
geometry_msgs::TwistStamped speed_msg;                                //create a "speed_msg" ROS message
ros::Publisher speed_pub("slave2speed", &speed_msg);                          //create a publisher to ROS topic "speed" using the "speed_msg" type
ros::Subscriber<geometry_msgs::Twist> holoodompy("holo/odompy",handle_holoodompy);
//motor driver setup
motor motor1, motor2 ,motor3 ,motor4;


//__________________________________________________________________________

void setup() 
{

  
  nh.initNode();                            //init ROS node
  nh.getHardware()->setBaud(57600);         //set baud for ROS serial communication
  nh.subscribe(cmd_vel);                    //suscribe to ROS topic for velocity commands
  nh.subscribe(holoodompy);
  nh.advertise(speed_pub);                  //prepare to publish speed in ROS topic
  nh.advertise(encode_msg);                  //prepare to publish speed in ROS topic
  nh.advertise(speed_reqmsg);                  //prepare to publish speed in ROS topic
  nh.advertise(PWM_msg);                  //prepare to publish speed in ROS topic
  nh.advertise(IMUvalue_msg);
  Wire.begin();
  byte status = mpu.begin();
  mpu.calcOffsets(); // gyro and accelero
  
  
  //AFMS.begin();
  motor1.pinA = Direction_motor1;
  motor1.pinPWM = PWM_motor1;  
  motor1.motor_setup();

  motor2.pinA = Direction_motor2;
  motor2.pinPWM = PWM_motor2;  
  motor2.motor_setup();

  motor3.pinA = Direction_motor3;
  motor3.pinPWM = PWM_motor3;  
  motor3.motor_setup();

  motor4.pinA = Direction_motor4;
  motor4.pinPWM = PWM_motor4;  
  motor4.motor_setup();

  //set PID
  PID_Motor1.SetSampleTime(95);
  PID_Motor1.SetOutputLimits(-max_speed, max_speed);
  PID_Motor1.SetMode(AUTOMATIC);

  PID_Motor2.SetSampleTime(95);
  PID_Motor2.SetOutputLimits(-max_speed, max_speed);
  PID_Motor2.SetMode(AUTOMATIC);

  PID_Motor3.SetSampleTime(95);
  PID_Motor3.SetOutputLimits(-max_speed, max_speed);
  PID_Motor3.SetMode(AUTOMATIC);

  PID_Motor4.SetSampleTime(95);
  PID_Motor4.SetOutputLimits(-max_speed, max_speed);
  PID_Motor4.SetMode(AUTOMATIC);

  
  //setting motor speeds to zero
  motor1.setSpeed(0);
  motor1.run("BRAKE");
  motor2.setSpeed(0);
  motor2.run("BRAKE");
  motor3.setSpeed(0);
  motor3.run("BRAKE");
  motor4.setSpeed(0);
  motor4.run("BRAKE");
  //setting PID parameters
   
  pinMode(encoder_encoder1A , INPUT_PULLUP);
  pinMode(encoder_encoder1B , INPUT_PULLUP); 
  pinMode(encoder_encoder2A , INPUT_PULLUP);
  pinMode(encoder_encoder2B , INPUT_PULLUP);
  pinMode(encoder_encoder3A , INPUT_PULLUP);
  pinMode(encoder_encoder3B , INPUT_PULLUP); 
  pinMode(encoder_encoder4A , INPUT_PULLUP);
  pinMode(encoder_encoder4B , INPUT_PULLUP);
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(encoder_encoder1A), encoder1_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_encoder2A), encoder2_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_encoder3A), encoder3_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(encoder_encoder4A), encoder4_tick, RISING);
}

//_________________________________________________________________________

void loop() {
  nh.spinOnce();

  if((millis()-lastMilli) >= LOOPTIME)   
  {
    lastMilli = millis();
    
    byte status = mpu.begin();
    //mpu.update();// enter timed loop
    
    IMUvalue.vector.x = mpu.getAngleZ();
    IMUvalue.vector.y = int(mpu.getAngleZ())%360; 
   
    if(abs(counter1 - lastpos1) < 5 )
    {
      speed_act_motor1 = 0;
    }
    else
    {
      speed_act_motor1 = ((((counter1-lastpos1)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos1 = counter1 ;
    }

    if(abs(counter2 - lastpos2) < 5 )
    {
      speed_act_motor2 = 0;
    }
    else
    {
      speed_act_motor2 = ((((counter2-lastpos2)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos2 = counter2 ;
    }

    if(abs(counter3 - lastpos3) < 5 )
    {
      speed_act_motor3 = 0;
    }
    else
    {
      speed_act_motor3 = ((((counter3-lastpos3)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos3 = counter3 ;
    }

    if(abs(counter4 - lastpos4) < 5 )
    {
      speed_act_motor4 = 0;
    }
    else
    {
      speed_act_motor4 = ((((counter4-lastpos4)/encoder_cpr)*2*PI)*(1000/LOOPTIME)*radius);
      lastpos4 = counter4 ;
    }
       
    encode.linear.x = counter1;    //left wheel speed (in m/s)
    encode.linear.y = counter2;   //right wheel speed (in m/s)
    encode.angular.x = counter3;
    encode.angular.y = counter4;

    if(goalangleslave2 != goalangleslave2old)
    {
    float kpp = 0.03;

    // errorleader
    int eslave2 = goalangleslave2 - mpu.getAngleZ();

    // control signal
    float uslave2 = kpp * eslave2 ;

    // store previous error
    eprevslave2 = eslave2;
    goalangleslave2old = goalangleslave2;
    angz = angz + uslave2 ;
    }

     
    speed_req_motor1 = (linx-liny-(wheelbase*angz));
    speed_req_motor2 = (linx+liny+(wheelbase*angz));
    speed_req_motor3 = (linx+liny-(wheelbase*angz));
    speed_req_motor4 = (linx-liny+(wheelbase*angz));

    speedreq.linear.x = speed_req_motor1;
    speedreq.linear.y = speed_req_motor2;
    speedreq.angular.x = speed_req_motor3;
    speedreq.angular.y = speed_req_motor4;


    //PWM1 = (speed_req_motor1/0.001) ;
    //PWM2 = (speed_req_motor2/0.001) ;
    //PWM3 = (speed_req_motor3/0.001) ;
    //PWM4 = (speed_req_motor4/0.001) ;

    speed_cmd_motor1 = constrain(speed_cmd_motor1, -max_speed, max_speed);
    speed_cmd_motor2 = constrain(speed_cmd_motor2, -max_speed, max_speed);
    speed_cmd_motor3 = constrain(speed_cmd_motor3, -max_speed, max_speed);
    speed_cmd_motor4 = constrain(speed_cmd_motor4, -max_speed, max_speed);
    PID_Motor1.Compute();
    PID_Motor2.Compute();
    PID_Motor3.Compute();
    PID_Motor4.Compute();                                                 
    // compute PWM value for left motor. Check constant definition comments for more information.
    PWM_Motor1 = constrain(speed_req_motor1*(255/0.29), -255, 255); 
    PWM_Motor2 = constrain(speed_req_motor2*(255/0.29), -255, 255);
    PWM_Motor3 = constrain(speed_req_motor3*(255/0.29), -255, 255);
    PWM_Motor4 = constrain(speed_req_motor4*(255/0.29), -255, 255);
    
    if(PWM_Motor1 >= 0) // forward left
    {
      motor1.run("FORWARD");
    }
    else if(PWM_Motor1 < 0)
    {
      motor1.run("BACKWARD");
      PWM_Motor1 = PWM_Motor1*(-1);
    }

    if(PWM_Motor2 >= 0) // forward left
    {
      //if(PWM_Motor2 >= 240)
      //{
      //  PWM_Motor2 = 240 ;
      //}
      motor2.run("FORWARD");
    }
    else if(PWM_Motor2 < 0)
    {
      motor2.run("BACKWARD");
      PWM_Motor2 = PWM_Motor2*(-1);
    }

    if(PWM_Motor3 >= 0) // forward left
    {
      motor3.run("FORWARD");
    }
    else if(PWM_Motor3 < 0)
    {
      //if(PWM_Motor3 <= -240)
      //{
      //  PWM_Motor3 = -240 ;
      //}
      motor3.run("BACKWARD");
      PWM_Motor3 = PWM_Motor3*(-1);
    }

    if(PWM_Motor4 >= 0) // forward left
    {
      motor4.run("FORWARD");
    }
    else if(PWM_Motor4 < 0)
    {
      motor4.run("BACKWARD");
      PWM_Motor4 = PWM_Motor4*(-1);
    }     


    PWMout.linear.x = PWM_Motor1 ;
    PWMout.linear.y = PWM_Motor2 ;
    PWMout.angular.x = PWM_Motor3 ;
    PWMout.angular.y = PWM_Motor4 ;

    motor1.setSpeed(PWM_Motor1);
    motor2.setSpeed(PWM_Motor2);
    motor3.setSpeed(PWM_Motor3);
    motor4.setSpeed(PWM_Motor4);
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
  
  IMUvalue_msg.publish(&IMUvalue);
  speed_reqmsg.publish(&speedreq);
  encode_msg.publish(&encode);
  PWM_msg.publish(&PWMout);
  nh.spinOnce();
  speed_msg.header.stamp = nh.now();      //timestamp for odometry data
  speed_msg.twist.linear.x = speed_act_motor1;    //left wheel speed (in m/s)
  speed_msg.twist.linear.y = speed_act_motor2;   //right wheel speed (in m/s)
  speed_msg.twist.angular.x = speed_act_motor3;
  speed_msg.twist.angular.y = speed_act_motor4;
  
 
  speed_pub.publish(&speed_msg);
  nh.loginfo("Publishing speed_msgs");
}

// Increment the number of ticks
void encoder1_tick() 
{

  // Read the value for the encoder for the right wheel
  int val = digitalRead(encoder_encoder1B);

  if(val == LOW) 
  {
    dir1 = false; // Reverse
  }
  else 
  {
    dir1 = true; // Forward
  }

  if (dir1) 
  {

    if (counter1 == encoder_maximum) 
    {
      counter1 = encoder_minimum;
    }
    else 
    {
      counter1++;  
    }    
  }
  else 
  {
    if (counter1 == encoder_minimum) 
    {   
      counter1 = encoder_maximum;
    }
    else 
    {
      counter1--;
    }   
  }
}   
void encoder2_tick() 
{

  // Read the value for the encoder for the right wheel
  int val2 = digitalRead(encoder_encoder2B);

  if(val2 == LOW) 
  {
    dir2 = false; // Reverse
  }
  else 
  {
    dir2 = true; // Forward
  }

  if (dir2) 
  {

    if (counter2 == encoder_maximum) 
    {
      counter2 = encoder_minimum;
    }
    else 
    {
      counter2++;  
    }    
  }
  else 
  {
    if (counter2 == encoder_minimum) 
    {   
      counter2 = encoder_maximum;
    }
    else 
    {
      counter2--;
    }   
  } 

}
void encoder3_tick() 
{

  // Read the value for the encoder for the right wheel
  int val3 = digitalRead(encoder_encoder3B);

  if(val3 == LOW) 
  {
    dir3 = false; // Reverse
  }
  else 
  {
    dir3 = true; // Forward
  }

  if (dir3) 
  {

    if (counter3 == encoder_maximum) 
    {
      counter3 = encoder_minimum;
    }
    else 
    {
      counter3++;  
    }    
  }
  else 
  {
    if (counter3 == encoder_minimum) 
    {   
      counter3 = encoder_maximum;
    }
    else 
    {
      counter3--;
    }   
  } 

}
void encoder4_tick() 
{

  // Read the value for the encoder for the right wheel
  int val4 = digitalRead(encoder_encoder4B);

  if(val4 == LOW) 
  {
    dir4 = false; // Reverse
  }
  else 
  {
    dir4 = true; // Forward
  }

  if (dir4) 
  {

    if (counter4 == encoder_maximum) 
    {
      counter4 = encoder_minimum;
    }
    else 
    {
      counter4++;  
    }    
  }
  else 
  {
    if (counter4 == encoder_minimum) 
    {   
      counter4 = encoder_maximum;
    }
    else 
    {
      counter4--;
    }   
  } 

}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
