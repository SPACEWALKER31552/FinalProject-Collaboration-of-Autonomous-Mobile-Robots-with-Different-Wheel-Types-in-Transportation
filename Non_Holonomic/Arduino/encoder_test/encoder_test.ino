// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define encoder_encoder1A 21
#define encoder_encoder2A 20
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

// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
volatile int counter1 = 0;
volatile int counter2 = 0;
volatile int counter3 = 0;
volatile int counter4 = 0;


// One-second interval for measurements
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

void setup() 
{
  // Open the serial port at 115200 bps
  Serial.begin(115200); 
  
  // Set pin states of the encoder
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

void loop() 
{

  // Record the time  
  currentMillis = millis();

  // If one second has passed, print the number of ticks
  if (currentMillis - previousMillis > interval) 
  {
    previousMillis = currentMillis;

    Serial.println("Number of Ticks1: ");
    Serial.println(counter1);
    Serial.println(digitalRead(encoder_encoder1A));
    Serial.println(digitalRead(encoder_encoder1B));
    Serial.println();

    Serial.println("Number of Ticks2: ");
    Serial.println(counter2);
    Serial.println(digitalRead(encoder_encoder2A));
    Serial.println(digitalRead(encoder_encoder2B));
    Serial.println();

    Serial.println("Number of Ticks3: ");
    Serial.println(counter3);
    Serial.println(digitalRead(encoder_encoder3A));
    Serial.println(digitalRead(encoder_encoder3B));
    Serial.println();

    Serial.println("Number of Ticks4: ");
    Serial.println(counter4);
    Serial.println(digitalRead(encoder_encoder4A));
    Serial.println(digitalRead(encoder_encoder4B));
    Serial.println();
   }
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
