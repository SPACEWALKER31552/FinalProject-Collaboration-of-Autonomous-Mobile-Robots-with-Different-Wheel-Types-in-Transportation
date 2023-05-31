

#include <SPI.h>
#include <SD.h>
unsigned long lastmillis = 0;

File myFile; 
const int chipSelect = 4;

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

myFile = SD.open("test.txt", FILE_WRITE);


if (myFile) 
{
Serial.print("Writing to test.txt...");
myFile.println("Hello motherfakkar"); 
myFile.close(); 
Serial.println("done.");
} else {

Serial.println("error opening test.txt");
}


/*myFile = SD.open("test.txt"); 
if (myFile) {
Serial.println("test.txt:");

while (myFile.available()) {
Serial.write(myFile.read());
}
myFile.close(); 
} 
else 
{
  Serial.println("error opening test.txt");
} */

lastmillis = millis();
}

void loop()
{
  if((millis()-lastmillis) <= 5000)
  {
    myFile.println("Hello motherfakkar"); 
  }
  myFile.close(); 
  Serial.println("done.");
}
