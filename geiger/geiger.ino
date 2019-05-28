#include <SD.h>
#include <SPI.h>
#include <Bounce2.h>

int digitalPin = 9;
const int chipSelect = BUILTIN_SDCARD;

unsigned long prevMillis=  0;
unsigned long prevMillis1 = 0;
const long interval1 = 40000;
const long interval2 = 500;
Bounce bouncer = Bounce();

void setup()
{
  pinMode(digitalPin, INPUT);
  digitalWrite(digitalPin, HIGH);
  bouncer.attach(digitalPin);
  bouncer .interval(5);
 // Open serial communications and wait for port to open:
  Serial.begin(9600);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.print("Initializing SD card...");
  
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop()
{
  // make a string for assembling the data to log:
  String dataString = "";
    
  if(bouncer.update()){
     if(bouncer.read() == 0){
        dataString += String(1);
        dataString += ","; 
     }
  }



  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("gieger.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.print(bouncer.read());
    Serial.print(", ");
    Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  } 
}
