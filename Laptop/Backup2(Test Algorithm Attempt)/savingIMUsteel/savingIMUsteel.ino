
#include <Wire.h>
//#include <PID_v1.h>
#include <MPU6050_light.h>
#include <SPI.h>
#include <SD.h>

double imux = 0;
double imuy = 0;
double imuz = 0;
unsigned long lastmillis = 0;

File myFile; 
File myFile2; 
const int chipSelect = 4;


MPU6050 mpu(Wire);
unsigned long timer = 0;

void setup()
{

Serial.begin(9600);

Serial.print("Initializing SD card...");
pinMode(SS, OUTPUT);

if (!SD.begin(chipSelect)) 
{
Serial.println("initialization failed!");
return;
}
Serial.println("initialization done.");

myFile = SD.open("IMUvalue.txt", FILE_WRITE);
myFile2 = SD.open("Time.txt", FILE_WRITE);

Wire.begin();
byte status = mpu.begin();
mpu.calcOffsets();

lastmillis = millis();
}

void loop()
{
  unsigned long allSeconds=millis()/1000;
  mpu.update();
  delay(10);
  imuz = mpu.getAngleZ();
  /*if((millis()-lastmillis) >= 100)
  {
    imuz = mpu.getAngleZ();
    Serial.print("imuz = ");
    Serial.println(imuz);
    lastmillis = millis();
  }*/
  if((millis()-lastmillis) <= 5000)
  {
    if ((millis() - timer) > 100) 
    {
      myFile.println(imuz);
      myFile2.println(allSeconds); 
      Serial.print("imuz = ");
      Serial.println(imuz);
      timer = millis();
    }
  }
  else
  {
    myFile.close(); 
    myFile2.close(); 
    Serial.print("done : ");
    Serial.println(imuz);
  }
}
