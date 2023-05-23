int pinA = 12;
int pinB = 11;
int pinPWM = 10 ;
//int pinA = 6;
///int pinB = 7 ;
//int pinPWM = 5 ;

void setup() 
{
  pinMode(pinA,OUTPUT);
  pinMode(pinB,OUTPUT);
  pinMode(pinPWM,OUTPUT);

}

void loop() 
{
  digitalWrite(pinA,HIGH);
  digitalWrite(pinB,LOW);
  analogWrite(pinPWM,255);
  }
