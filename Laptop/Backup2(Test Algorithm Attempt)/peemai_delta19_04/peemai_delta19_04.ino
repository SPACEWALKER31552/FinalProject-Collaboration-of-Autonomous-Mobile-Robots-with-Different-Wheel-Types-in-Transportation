#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro
#include <SPI.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
geometry_msgs::Twist velocity_msg ;
ros::Publisher chatter("cmd_vel",&velocity_msg);

/* Serial rates for UART */
#define BAUDRATE        115200

/* SPI commands */
#define AMT22_NOP       0x00
#define AMT22_RESET     0x60
#define AMT22_ZERO      0x70

/* Define special ascii characters */
#define NEWLINE         0x0A
#define TAB             0x09

/* We will use these define macros so we can write code once compatible with 12 or 14 bit encoders */
#define RES12           12
#define RES14           14

/* SPI pins */
#define ENC_1           26
#define ENC_2           27
#define ENC_3           28
#define ENC_4           38
#define ENC_5           39
#define ENC_6           40
#define SPI_MOSI        51
#define SPI_MISO        50
#define SPI_SCLK        52
const int pwm1 = 44 ; 
const int in1_1 = 14 ;
const int in1_2 = 15 ;
const int pwm2 = 45 ;
const int in2_1 = 16 ;
const int in2_2 = 17 ;
const int pwm3 = 46 ; 
const int in3_1 = 18 ;
const int in3_2 = 19 ;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
volatile uint16_t encoderPosition1;
volatile uint16_t encoderPosition2;
volatile uint16_t encoderPosition3;
volatile uint16_t encoderPosition4;
volatile uint16_t encoderPosition5;
volatile uint16_t encoderPosition6;

volatile double encoderAngVel1;
volatile double encoderAngVel2;
volatile double encoderAngVel3;
volatile double encoderAngVel4;
volatile double encoderAngVel5;
volatile double encoderAngVel6;

volatile uint16_t encoderPos1_old;
volatile uint16_t encoderPos1_new;
volatile uint16_t encoderPos2_old;
volatile uint16_t encoderPos2_new;
volatile uint16_t encoderPos3_old;
volatile uint16_t encoderPos3_new;
volatile uint16_t encoderPos4_old;
volatile uint16_t encoderPos4_new;
volatile uint16_t encoderPos5_old;
volatile uint16_t encoderPos5_new;
volatile uint16_t encoderPos6_old;
volatile uint16_t encoderPos6_new;

//let's also create a variable where we can count how many times we've tried to obtain the position in case there are errors
int os1 = 0;
int os2 = 0;
int os3 = 0;
int dif1 = 0;
int dif2 = 0;
int dif3 = 0;
int difflist1[100];
int difflist2[100];
int difflist3[100];
int tg1;
int tg2;
int tg3;
int endiff1;
int endiff2;
int endiff3;
float v1;
float v2;
float v3;
float vx;
float vy;
float eprev1 = 0;
float eintegral1 = 0;
float eprev2 = 0;
float eintegral2 = 0;
float eprev3 = 0;
float eintegral3 = 0;
float deg1;
float deg2;
float deg3;
float deg4;
float deg5;
float deg6;
float actual_x, actual_y, actual_z;
float motor_x, motor_y, motor_z;
float t ;
float y1 ;
float y2 ;
float z1 ;
float z2 ;
float y3 ;
float x2 ;
float x3 ;
float z3 ;
float w1 ;
float w2 ;
float w3 ;
float D ;
double Dfinal ;
float A ;
float B ;
float C ;
float X ;
float Y ;
float Z ;
float a1 ;
float a2 ;
float b1 ;
float b2 ;

// robot geometry
// (look at pics above for explanation)
const float e = 115.0;     // end effector
const float f = 457.3;     // base
const float re = 232.0;
const float rf = 112.0;

// trigonometric constants
const float sqrt3 = sqrt(3.0);
const float pi = 3.141592653;    // PI
const float sin120 = sqrt3/2.0;   
const float cos120 = -0.5;        
const float tan60 = sqrt3;
const float sin30 = 0.5;
const float tan30 = 1/sqrt3;

float static_x,static_y;
unsigned long startMillis;  
unsigned long currentMillis;

void setCSLine (uint8_t encoder, uint8_t csLine)
{
  digitalWrite(encoder, csLine);
}
uint8_t spiWriteRead(uint8_t sendByte, uint8_t encoder, uint8_t releaseLine)
{
  //holder for the received over SPI
  uint8_t data;

  //set cs low, cs may already be low but there's no issue calling it again except for extra time
  setCSLine(encoder ,LOW);

  //There is a minimum time requirement after CS goes low before data can be clocked out of the encoder.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //send the command  
  data = SPI.transfer(sendByte);
  delayMicroseconds(3); //There is also a minimum time after clocking that CS should remain asserted before we release it
  setCSLine(encoder, releaseLine); //if releaseLine is high set it high else it stays low
  
  return data;
}
uint16_t getPositionSPI(uint8_t encoder, uint8_t resolution)
{
  uint16_t currentPosition;       //16-bit response from encoder
  bool binaryArray[16];           //after receiving the position we will populate this array and use it for calculating the checksum

  //get first byte which is the high byte, shift it 8 bits. don't release line for the first byte
  currentPosition = spiWriteRead(AMT22_NOP, encoder, false) << 8;   

  //this is the time required between bytes as specified in the datasheet.
  //We will implement that time delay here, however the arduino is not the fastest device so the delay
  //is likely inherantly there already
  delayMicroseconds(3);

  //OR the low byte with the currentPosition variable. release line after second byte
  currentPosition |= spiWriteRead(AMT22_NOP, encoder, true);        

  //run through the 16 bits of position and put each bit into a slot in the array so we can do the checksum calculation
  for(int i = 0; i < 16; i++) binaryArray[i] = (0x01) & (currentPosition >> (i));

  //using the equation on the datasheet we can calculate the checksums and then make sure they match what the encoder sent
  if ((binaryArray[15] == !(binaryArray[13] ^ binaryArray[11] ^ binaryArray[9] ^ binaryArray[7] ^ binaryArray[5] ^ binaryArray[3] ^ binaryArray[1]))
          && (binaryArray[14] == !(binaryArray[12] ^ binaryArray[10] ^ binaryArray[8] ^ binaryArray[6] ^ binaryArray[4] ^ binaryArray[2] ^ binaryArray[0])))
    {
      //we got back a good position, so just mask away the checkbits
      currentPosition &= 0x3FFF;
    }
  else
  {
    currentPosition = 0xFFFF; //bad position
  }

  //If the resolution is 12-bits, and wasn't 0xFFFF, then shift position, otherwise do nothing
  if ((resolution == RES12) && (currentPosition != 0xFFFF)) currentPosition = currentPosition >> 2;

  return currentPosition;
}

void setup() {
  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(chatter);

  //Serial.begin(BAUDRATE);
  pinMode(SPI_SCLK, OUTPUT);
  pinMode(SPI_MOSI, OUTPUT);
  pinMode(SPI_MISO, INPUT);
  pinMode(ENC_1, OUTPUT);
  pinMode(ENC_2, OUTPUT);
  pinMode(ENC_3, OUTPUT);
  pinMode(ENC_4, OUTPUT);
  pinMode(ENC_5, OUTPUT);
  pinMode(ENC_6, OUTPUT);
  pinMode(pwm1,OUTPUT);
  pinMode(pwm2,OUTPUT);
  pinMode(pwm3,OUTPUT);
  pinMode(in1_1,OUTPUT);
  pinMode(in1_2,OUTPUT);
  pinMode(in2_1,OUTPUT);
  pinMode(in2_2,OUTPUT);
  pinMode(in3_1,OUTPUT);
  pinMode(in3_2,OUTPUT);
  digitalWrite(ENC_1, HIGH);
  digitalWrite(ENC_2, HIGH);
  digitalWrite(ENC_3, HIGH);
  digitalWrite(ENC_4, HIGH);
  digitalWrite(ENC_5, HIGH);
  digitalWrite(ENC_6, HIGH);
  SPI.setClockDivider(SPI_CLOCK_DIV32);    // 500 kHz
  SPI.begin();
  //Serial.println("target pos");
  
  cli();//stop interrupts

//set timer4 interrupt at 1000Hz
 TCCR4A = 0;// set entire TCCR1A register to 0
 TCCR4B = 0;// same for TCCR1B
 TCNT4  = 0;//initialize counter value to 0
 // set compare match register for 1hz increments
 OCR4A = 15624/1000;// = (16*10^6) / (1*1024) - 1 (must be <65536)
 // turn on CTC mode
 TCCR4B |= (1 << WGM12);
 // Set CS12 and CS10 bits for 1024 prescaler
 TCCR4B |= (1 << CS12) | (1 << CS10);  
 // enable timer compare interrupt
 TIMSK4 |= (1 << OCIE4A);

sei();//allow interrupts

encoderPos1_old = getPositionSPI(ENC_1, RES14);
encoderPos2_old = getPositionSPI(ENC_2, RES14);
encoderPos3_old = getPositionSPI(ENC_3, RES14);
encoderPos4_old = getPositionSPI(ENC_4, RES14);
encoderPos5_old = getPositionSPI(ENC_5, RES14);
encoderPos6_old = getPositionSPI(ENC_6, RES14);
}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2){
  analogWrite(pwm,pwmVal);
  if(dir == 1){
    digitalWrite(in1,HIGH);
    digitalWrite(in2,LOW);
  }
  else if(dir == -1){
    digitalWrite(in1,LOW);
    digitalWrite(in2,HIGH);
  }
  else{
    digitalWrite(in1,LOW);
    digitalWrite(in2,LOW);
  }  
}

void position_control(int tg1_input, int tg2_input, int tg3_input){

  // set target position
  int target1 = tg1_input;
  int target2 = tg2_input;
  int target3 = tg3_input;

  // PID constants
  float kp = 1.0;
  float kd = 0;
  float ki = 0;

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT))/( 1.0e6 );
  prevT = currT;

  int pos1 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = encoderPosition1;
  }
  
  // error
  int e1 = pos1 - target1;
  // derivative
  // float dedt1 = (e1-eprev1)/(deltaT);
  float dedt1 = -1 * encoderAngVel1;
  // integral
  eintegral1 = eintegral1 + e1*deltaT;
  // control signal
  float u1 = (kp*e1) + (kd*dedt1) + (ki*eintegral1);
  // motor power
  float pwr1 = fabs(u1);
  if( pwr1 > 255 ){
    pwr1 = 255;
  }
  // motor direction
  int dir1 = 1;
  if(u1<0){
    dir1 = -1;
  }


  // signal the motor
  if(encoderPosition1<=5248 || encoderPosition1>=9500){
    pwr1 = 0;
  }
  setMotor(dir1,pwr1,pwm1,in1_1,in1_2);

  // store previous error
  eprev1 = e1;
  int pos2 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos2 = encoderPosition3;
  }
  
  // error
  int e2 = pos2 - target2;
  // derivative
  float dedt2 = (e2-eprev2)/(deltaT);
  // integral
  eintegral2 = eintegral2 + e2*deltaT;
  // control signal
  float u2 = kp*e2 + kd*dedt2 + ki*eintegral2;

  // motor power
  float pwr2 = fabs(u2);
  if( pwr2 > 255 ){
    pwr2 = 255;
  }
  // motor direction
  int dir2 = 1;
  if(u2<0){
    dir2 = -1;
  }
  // signal the motor
  if(encoderPosition3<=10340 || encoderPosition3>=14000)
    {pwr2 = 0;}
  setMotor(dir2,pwr2,pwm2,in2_1,in2_2);

  // store previous error
  eprev2 = e2;
  int pos3 = 0; 
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos3 = encoderPosition5;
  }
  
  // error
  int e3 = pos3 - target3;
  // derivative
  float dedt3 = (e3-eprev3)/(deltaT);
  // integral
  eintegral3 = eintegral3 + e3*deltaT;
  // control signal
  float u3 = kp*e3 + kd*dedt3 + ki*eintegral3;
  // motor power
  float pwr3 = fabs(u3);
  if( pwr3 > 255 ){
    pwr3 = 255;
  }
  // motor direction
  int dir3 = 1;
  if(u3<0){
    dir3 = -1;
  }
  // signal the motor
  if(encoderPosition5<=5150 || encoderPosition5>=9000)
    {pwr3 = 0;}
  setMotor(dir3,pwr3,pwm3,in3_1,in3_2);
  // store previous error
  eprev3 = e3;
}

uint16_t encoderPositionIN1 = 4284;
uint16_t encoderPositionIN2 = 3552;
uint16_t encoderPositionIN3 = 9288;
uint16_t encoderPositionIN4 = 6707;
uint16_t encoderPositionIN5 = 4023;
uint16_t encoderPositionIN6 = 3586;

void inverse_kin(float deg1_input, float deg2_input, float deg3_input, float* pos_x, float* pos_y, float* pos_z){
  float f = 31;
  float re = 36;
  float rf = 20;
  float e = 9.7;
  float pi = 3.140;
  float theta1 = (((deg1_input+10) * 71) / 4068.00);
  float theta2 = (((deg2_input+10) * 71) / 4068.00);
  float theta3 = (((deg3_input+10) * 71) / 4068.00);
  
  float t = (f-e)*tan30/2;
  float dtr = pi/(float)180.0;

  float y1 = -(t + rf*cos(theta1));
  float z1 = -rf*sin(theta1);

  float y2 = (t + rf*cos(theta2))*sin30;
  float x2 = y2*tan60;
  float z2 = -rf*sin(theta2);

  float y3 = (t + rf*cos(theta3))*sin30;
  float x3 = -y3*tan60;
  float z3 = -rf*sin(theta3);

  float dnm = (y2-y1)*x3-(y3-y1)*x2;

  float w1 = y1*y1 + z1*z1;
  float w2 = x2*x2 + y2*y2 + z2*z2;
  float w3 = x3*x3 + y3*y3 + z3*z3;
  
  // x = (a1*z + b1)/dnm
  float a1 = (z2-z1)*(y3-y1)-(z3-z1)*(y2-y1);
  float b1 = -((w2-w1)*(y3-y1)-(w3-w1)*(y2-y1))/2.0;

  // y = (a2*z + b2)/dnm;
  float a2 = -(z2-z1)*x3+(z3-z1)*x2;
  float b2 = ((w2-w1)*x3 - (w3-w1)*x2)/2.0;

  // a*z^2 + b*z + c = 0
  float a = a1*a1 + a2*a2 + dnm*dnm;
  float b = 2*(a1*b1 + a2*(b2-y1*dnm) - z1*dnm*dnm);
  float c = (b2-y1*dnm)*(b2-y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re);

  // discriminant
  float d = b*b - (float)4.0*a*c;
  if (d < 0) return -1; // non-existing point

  float z0 = -(float)0.5*(b+sqrt(d))/a;
  float x0 = (a1*z0 + b1)/dnm;
  float y0 = (a2*z0 + b2)/dnm;

  *pos_z = z0;
  *pos_x = x0;
  *pos_y = y0;

  return;
}

void inverse_kin_old(float deg1_input, float deg2_input, float deg3_input, float* pos_x, float* pos_y, float* pos_z){
  float f = 31;
  float Re = 41.5;
  // float Re1 = 42.5;
  // float Re2 = 41.5;
  // float Re3 = 41.5;
  float Rf = 20;
  float e = 9.7;
  float pi = 3.140;
  float rad1 = (((deg1_input+10) * 71) / 4068.00);
  float rad2 = (((deg2_input+10) * 71) / 4068.00);
  float rad3 = (((deg3_input+10) * 71) / 4068.00);
  t = 0.500*(f-e)*tan(pi/6);
  y1 = -1*(t+Rf*cos(rad1));
  z1 = -1*Rf*(sin(rad1));
  y2 = (t+Rf*cos(rad2))*(sin(pi/6.00));
  z2 = -1*Rf*(sin(rad2));
  y3 = (t+Rf*cos(rad3))*(sin(pi/6));
  x2 = 1*y2*tan(pi/3);
  x3 = -1*y3*tan(pi/3);
  z3 = -1*Rf*sin(rad3);
  D = ((y2-y1)*x3)-((y3-y1)*x2);
  w1 = sq(y1) + sq(z1);
  w2 = sq(x2) + sq(y2) + sq(z2);
  w3 = sq(x3) + sq(y3) + sq(z3);
  a1 = ((z2-z1)*(y3-y1))-((z3-z1)*(y2-y1));
  b1 = -0.5*(((w2-w1)*(y3-y1))-((w3-w1)*(y2-y1)));
  a2 = (-1*(z2-z1)*(x3))+((z3-z1)*x2);
  b2 = 0.5*(((w2-w1)*(x3))-((w3-w1)*(x2)));
  A = sq(a1) + sq(a2) + sq(D);
  B = 2*((a1*b1)+(a2*(b2-(y1*D)))-(z1*(sq(D))));
  C = sq(b2-(y1*D))+sq(b1)+(sq(D)*(sq(z1)-sq(Re)));
  Dfinal = (sq(B))-(4*A*C);
  *pos_z = ((B+ sqrt(Dfinal))/(2*A));
  *pos_y = -1 * ((a1*Z)+b1)/D ;
  *pos_x = -1 * ((a2*Z)+b2)/D ;

  return;
}

volatile uint16_t counter = 0;

ISR(TIMER4_COMPA_vect) {
    
  encoderPos1_new = encoderPosition1 = getPositionSPI(ENC_1, RES14); 
  encoderPos2_new = encoderPosition2 = getPositionSPI(ENC_2, RES14);
  encoderPos3_new = encoderPosition3 = getPositionSPI(ENC_3, RES14); 
  encoderPos4_new = encoderPosition4 = getPositionSPI(ENC_4, RES14);
  encoderPos5_new = encoderPosition5 = getPositionSPI(ENC_5, RES14);
  encoderPos6_new = encoderPosition6 = getPositionSPI(ENC_6, RES14);

  encoderAngVel1 = (double)((2.0 * PI * ((int)(encoderPos1_new-encoderPos1_old)*1000)) / 16384);
  encoderAngVel2 = (double)((2.0 * PI * ((int)(encoderPos2_new-encoderPos2_old)*1000)) / 16384);
  encoderAngVel3 = (double)((2.0 * PI * ((int)(encoderPos3_new-encoderPos3_old)*1000)) / 16384);
  encoderAngVel4 = (double)((2.0 * PI * ((int)(encoderPos4_new-encoderPos4_old)*1000)) / 16384);
  encoderAngVel5 = (double)((2.0 * PI * ((int)(encoderPos5_new-encoderPos5_old)*1000)) / 16384);
  encoderAngVel6 = (double)((2.0 * PI * ((int)(encoderPos6_new-encoderPos6_old)*1000)) / 16384);

  encoderPos1_old = encoderPos1_new;
  encoderPos2_old = encoderPos2_new;
  encoderPos3_old = encoderPos3_new;
  encoderPos4_old = encoderPos4_new;
  encoderPos5_old = encoderPos5_new;
  encoderPos6_old = encoderPos6_new;
  
}

int cubic_trajectoryPlanner(int trajectory_target, float current_deg_in, int min_limit, int max_limit){
  int trajectory_duration = 3000;
  int trajectory_difference = trajectory_target - current_deg_in;
  int trajectory_increment = trajectory_difference/(trajectory_duration/10); 
  for(int i=0; i<trajectory_duration; i+=10){
    int  trajectory_angle_out = current_deg_in + (trajectory_increment*1/10);
    constrain(trajectory_angle_out, min_limit, max_limit);
  
  }
}

void program1(){
  //Serial.println("Run1");
//set-up
  if(os1 == 0 || os2 == 0 || os3 == 0 ){
    //Serial.println("Run2");
      delay(3000);
      for (int i = 0; i<15;i++){
        dif1 = encoderPosition1-encoderPosition2;
        dif2 = encoderPosition3-encoderPosition4;
        dif3 = encoderPosition5-encoderPosition6;
        difflist1[i] = dif1;
        difflist2[i] = dif2;
        difflist3[i] = dif3;
      }
      os1 = difflist1[10]/*+difflist[1]+difflist[2]+difflist[3]+difflist[4]+difflist[5]+difflist[6]+difflist[7]+difflist[8]+difflist[9])/10*/;
      os2 = difflist2[10]/*+difflist[1]+difflist[2]+difflist[3]+difflist[4]+difflist[5]+difflist[6]+difflist[7]+difflist[8]+difflist[9])/10*/;
      os3 = difflist3[10]/*+difflist[1]+difflist[2]+difflist[3]+difflist[4]+difflist[5]+difflist[6]+difflist[7]+difflist[8]+difflist[9])/10*/;
      //Serial.print("os1:"); Serial.print(os1); Serial.print(", ");
      //Serial.print("os2:"); Serial.print(os2); Serial.print(", ");
      //Serial.print("os3:"); Serial.print(os3); Serial.print(", ");
      tg1 = 6248;
      tg2 = 11340;
      tg3 = 6150;
      //Serial.print("tg1:"); Serial.print(tg1); Serial.print(", ");
      //Serial.print("tg2:"); Serial.print(tg2); Serial.print(", ");
      //Serial.print("tg3:"); Serial.print(tg3); Serial.print(", ");
      startMillis = millis();
      //Serial.println("Run2");
      }
    else{
      //Serial.println("Run3");
      if(encoderPosition1>=encoderPositionIN1){
        deg1 = (encoderPosition1-encoderPositionIN1)*0.0219726563;
      }
      else if(encoderPosition1<encoderPositionIN1){
        deg1 = 0;
      }

      if(encoderPosition2>=encoderPositionIN2){
        deg2 = (encoderPosition2-encoderPositionIN2)*0.0219726563;
      }
      else if(encoderPosition2<encoderPositionIN2){
        deg2 = 0;
      }

      if(encoderPosition3>=encoderPositionIN3){
        deg3 = (encoderPosition3-encoderPositionIN3)*0.0219726563;
      }
      else if(encoderPosition3<encoderPositionIN3){
        deg3 = 0;
      }

      if(encoderPosition4>=encoderPositionIN4){
        deg4 = (encoderPosition4-encoderPositionIN4)*0.0219726563;
      }
      else if(encoderPosition4<encoderPositionIN4){
        deg4 = 0;
      }

      if(encoderPosition5>=encoderPositionIN5){
        deg5 = (encoderPosition5-encoderPositionIN5)*0.0219726563;
      }
      else if(encoderPosition5<encoderPositionIN5){
        deg5 = 0;
      }

      if(encoderPosition6>=encoderPositionIN6){
        deg6 = (encoderPosition6-encoderPositionIN6)*0.0219726563;
      }
      else if(encoderPosition6<encoderPositionIN6){
        deg6 = 0;
      }

    if((millis() - startMillis) < 5000){ 
      //Serial.println("Run4");
      position_control(tg1, tg2, tg3);
      //inverse_kin(deg1,deg3,deg5,&motor_x,&motor_y,&motor_z); //Motor
      inverse_kin(deg2,deg4,deg6,&actual_x,&actual_y,&actual_z); //Actual
      // Serial.print("Actual ");
      // Serial.print("X: ");
      // Serial.print(actual_x); //print the position X
      // Serial.print("\t");
      // Serial.print("Y: ");
      // Serial.print(actual_y); //print the position Y
      // Serial.print("\t");
      // Serial.print("Z: ");
      // Serial.print(actual_z); //print the position Z
      // Serial.print("Motor ");
      // Serial.print("X: ");
      // Serial.print(motor_x); //print the position X
      // Serial.print("\t");
      // Serial.print("Y: ");
      // Serial.print(motor_y); //print the position Y
      // Serial.print("\t");
      // Serial.print("Z: ");
      // Serial.print(motor_z); //print the position Z
      //Serial.write(NEWLINE);
      static_x = actual_x;
      static_y = actual_y;
    }
    else{
      position_control(tg1, tg2, tg3);
      //inverse_kin(deg1,deg3,deg5,&motor_x,&motor_y,&motor_z); //Motor
      inverse_kin(deg2,deg4,deg6,&actual_x,&actual_y,&actual_z); //Actual
      vx = actual_x - static_x;
      vy = actual_y - static_y;
      // Serial.print("Velocity   ");
      // Serial.print("Vx: ");
      // Serial.print(vx); 
      // Serial.print("\t");
      // Serial.print("Vy: ");
      // Serial.print(vy);
      // Serial.write(NEWLINE);
    }
      
   }
}

void program2(){
  // Serial.println("Run1");
//set-up
  if(os1 == 0 || os2 == 0 || os3 == 0 ){
    // Serial.println("Run2");
      delay(3000);
      for (int i = 0; i<15;i++){
        dif1 = encoderPosition1-encoderPosition2;
        dif2 = encoderPosition3-encoderPosition4;
        dif3 = encoderPosition5-encoderPosition6;
        difflist1[i] = dif1;
        difflist2[i] = dif2;
        difflist3[i] = dif3;
      }
      os1 = difflist1[10]/*+difflist[1]+difflist[2]+difflist[3]+difflist[4]+difflist[5]+difflist[6]+difflist[7]+difflist[8]+difflist[9])/10*/;
      os2 = difflist2[10]/*+difflist[1]+difflist[2]+difflist[3]+difflist[4]+difflist[5]+difflist[6]+difflist[7]+difflist[8]+difflist[9])/10*/;
      os3 = difflist3[10]/*+difflist[1]+difflist[2]+difflist[3]+difflist[4]+difflist[5]+difflist[6]+difflist[7]+difflist[8]+difflist[9])/10*/;
      // Serial.print("os1:"); Serial.print(os1); Serial.print(", ");
      // Serial.print("os2:"); Serial.print(os2); Serial.print(", ");
      // Serial.print("os3:"); Serial.print(os3); Serial.print(", ");
      //OLD
      // tg1 = 6248;
      // tg2 = 11340;
      // tg3 = 6150;
      //NEW
      tg1 = 6446;
      tg2 = 11700;
      tg3 = 6109;      
      // Serial.print("tg1:"); Serial.print(tg1); Serial.print(", ");
      // Serial.print("tg2:"); Serial.print(tg2); Serial.print(", ");
      // Serial.print("tg3:"); Serial.print(tg3); Serial.print(", ");
      startMillis = millis();
      // Serial.println("Run2");
      }
    else{
      // Serial.println("Run3");
      if(encoderPosition1>=encoderPositionIN1){
        deg1 = (encoderPosition1-encoderPositionIN1)*0.0219726563;
      }
      else if(encoderPosition1<encoderPositionIN1){
        deg1 = 0;
      }

      if(encoderPosition2>=encoderPositionIN2){
        deg2 = (encoderPosition2-encoderPositionIN2)*0.0219726563;
      }
      else if(encoderPosition2<encoderPositionIN2){
        deg2 = 0;
      }

      if(encoderPosition3>=encoderPositionIN3){
        deg3 = (encoderPosition3-encoderPositionIN3)*0.0219726563;
      }
      else if(encoderPosition3<encoderPositionIN3){
        deg3 = 0;
      }

      if(encoderPosition4>=encoderPositionIN4){
        deg4 = (encoderPosition4-encoderPositionIN4)*0.0219726563;
      }
      else if(encoderPosition4<encoderPositionIN4){
        deg4 = 0;
      }

      if(encoderPosition5>=encoderPositionIN5){
        deg5 = (encoderPosition5-encoderPositionIN5)*0.0219726563;
      }
      else if(encoderPosition5<encoderPositionIN5){
        deg5 = 0;
      }

      if(encoderPosition6>=encoderPositionIN6){
        deg6 = (encoderPosition6-encoderPositionIN6)*0.0219726563;
      }
      else if(encoderPosition6<encoderPositionIN6){
        deg6 = 0;
      }

    if((millis() - startMillis) < 5000){ 
      //Serial.println("Run4");
      position_control(tg1-300, tg2-300, tg3-300);
      //inverse_kin(deg1,deg3,deg5,&motor_x,&motor_y,&motor_z); //Motor
      inverse_kin(deg2,deg4,deg6,&actual_x,&actual_y,&actual_z); //Actual
      // Serial.print("Actual ");
      // Serial.print("X: ");
      // Serial.print(actual_x); //print the position X
      // Serial.print("\t");
      // Serial.print("Y: ");
      // Serial.print(actual_y); //print the position Y
      // Serial.print("\t");
      // Serial.print("Z: ");
      // Serial.print(actual_z); //print the position Z
      // Serial.print("Motor ");
      // Serial.print("X: ");
      // Serial.print(motor_x); //print the position X
      // Serial.print("\t");
      // Serial.print("Y: ");
      // Serial.print(motor_y); //print the position Y
      // Serial.print("\t");
      // Serial.print("Z: ");
      // Serial.print(motor_z); //print the position Z
      // Serial.write(NEWLINE);
      static_x = actual_x;
      static_y = actual_y;
    }
    else{
      position_control(tg1-300, tg2-300, tg3-300);
      //inverse_kin(deg1,deg3,deg5,&motor_x,&motor_y,&motor_z); //Motor
      inverse_kin_old(deg2,deg4,deg6,&actual_x,&actual_y,&actual_z); //Actual
      vx = actual_x - static_x;
      vy = actual_y - static_y;
      
      // Serial.print("X: ");
      // Serial.print(actual_x); //print the position X
      // Serial.print("\t");
      // Serial.print("Y: ");
      // Serial.print(actual_y); //print the position Y
      // Serial.print("\t");
      // Serial.print("Z: ");
      // Serial.print(actual_z); //print the position Z
      // Serial.print("\t");
      // Serial.print("\n");

      // Serial.print("Motor ");
      // Serial.print("X: ");
      // Serial.print(motor_x); //print the position X
      // Serial.print("\t");
      // Serial.print("Y: ");
      // Serial.print(motor_y); //print the position Y
      // Serial.print("\t");
      // Serial.print("Z: ");
      // Serial.print(motor_z); //print the position Z
      // Serial.write(NEWLINE);

      // Serial.print(actual_x);
      // Serial.print(",");
      // Serial.println(actual_y);


      // Serial.print(deg1);
      // Serial.print(",");
      // Serial.println(deg2);

      // Serial.print("Velocity   ");
      // Serial.print("Vx: ");
      // Serial.print(vx); 
      // Serial.print("\t");
      // Serial.print("Vy: ");
      // Serial.print(vy);
      // Serial.write(NEWLINE);
    }
    
   }
   velocity_msg.linear.x = vx ;
    velocity_msg.linear.y = vy ;
    chatter.publish( &velocity_msg );
    nh.spinOnce(); 
}

void test_timer_interupt(){
  //Serial.print(encoderPosition1);
  //Serial.print("\t");
  //Serial.print(encoderAngVel1);
  //Serial.print("\n");
}

void loop() {
  program2();
  nh.spinOnce(); 
  }
