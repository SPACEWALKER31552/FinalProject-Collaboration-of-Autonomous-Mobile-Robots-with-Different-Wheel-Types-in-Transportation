#define PWM_motor1 2
#define PWM_motor2 4
#define PWM_motor3 12
#define PWM_motor4 13

#define Direction_motor1 3 
#define Direction_motor2 5
#define Direction_motor3 30
#define Direction_motor4 32

void setup() 
{
  pinMode(PWM_motor1,OUTPUT);
  pinMode(PWM_motor2,OUTPUT);
  pinMode(PWM_motor3,OUTPUT);
  pinMode(PWM_motor4,OUTPUT);

  pinMode(Direction_motor1,OUTPUT);
  pinMode(Direction_motor2,OUTPUT);
  pinMode(Direction_motor3,OUTPUT);
  pinMode(Direction_motor4,OUTPUT);
}

void loop() 
{
  digitalWrite(Direction_motor1,HIGH);
  digitalWrite(Direction_motor2,HIGH);
  digitalWrite(Direction_motor3,HIGH);
  digitalWrite(Direction_motor4,HIGH);

  analogWrite(PWM_motor1,255);
  analogWrite(PWM_motor2,255);
  analogWrite(PWM_motor3,255);
  analogWrite(PWM_motor4,255);

  delay(1000);

  digitalWrite(Direction_motor1,LOW);
  digitalWrite(Direction_motor2,LOW);
  digitalWrite(Direction_motor3,LOW);
  digitalWrite(Direction_motor4,LOW);

  analogWrite(PWM_motor1,255);
  analogWrite(PWM_motor2,255);
  analogWrite(PWM_motor3,255);
  analogWrite(PWM_motor4,255);

  delay(1000);
}
