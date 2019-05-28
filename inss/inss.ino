#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>
#include <SPI.h>
const int chipSelect = BUILTIN_SDCARD;
int rx = 7; //blue - rxd
int tx = 8; //yellow - txd
int baudrate = 4800; //4800 bps from https://www.parallax.com/sites/default/files/downloads/PMB-648-v0.1.pdf
int scale = 3; //scale of ADXL337
float rawX; //teensy3.6 port 23
float rawY; //teensy3.6 port 22
float rawZ; //teensy3.6 port 21
float scaledX, scaledY, scaledZ; // Scaled values for each axis, based of adxl raw value scale
int rawScale = 1023; //based off the supply value of the Teensy3.6
int led = 13;
int c=0;
TinyGPS gps;
SoftwareSerial ss(rx, tx);

void setup()
{
  Serial.begin(115200);

  pinMode(led, OUTPUT);
   while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  ss.begin(baudrate);
}

void loop()
{
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  c++;
  
  File dataFile = SD.open("inss5.csv", FILE_WRITE);
  
  // make a string for assembling the data to log:
  String dataString = "";
  
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // Collect each second
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    //********** Accelerometer ****************//
    //read accelerometer data from ports A9, A8, A7 on Teensy 3.6, which are connected to the X,Y,Z output ports of ADXL337 respectively
    rawX = analogRead(A9); //23
    rawY = analogRead(A8); //22
    rawZ = analogRead(A7); //21

    //use the map function to create the scaled values as integers
    scaledX = map(rawX, 0, rawScale, -scale, scale);
    scaledY = map(rawY, 0, rawScale, -scale, scale);
    scaledZ = map(rawZ, 0, rawScale, -scale, scale);
    
    while (ss.available())
    {
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }
  digitalWrite(led, LOW);
    Serial.print("raw X: "); Serial.print(rawX);
    Serial.print("   raw Y: "); Serial.print(rawY);
    Serial.print("   raw Z: "); Serial.print(rawZ);
    Serial.print("      scaled X: "); Serial.print(scaledX); Serial.print("g");
    Serial.print("   scaled Y: "); Serial.print(scaledY); Serial.print("g");
    Serial.print("   scaled Z: "); Serial.print(scaledZ); Serial.print("g");
    Serial.println("");

    dataString += String(rawX);
    dataString += ","; 
    dataString += String(rawY);
    dataString += ","; 
    dataString += String(rawZ);
    dataString += ","; 
    dataString += String(scaledX);
    dataString += ","; 
    dataString += String(scaledY);
    dataString += ","; 
    dataString += String(scaledZ);

   
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(", ");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(", ");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(", ");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
  
    gps.stats(&chars, &sentences, &failed);
    Serial.print(" CHARS=");
    Serial.print(", ");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(", ");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.print(", ");
    Serial.println(failed);
    
    dataString += ","; 
    dataString += String(flat);
    dataString += ","; 
    dataString += String(flon);
    dataString += ","; 
    
//    dataFile.print(dataString);
//    dataFile.print(flat);
//    dataFile.print(",");
//    dataFile.print(flon);
//    dataString = "";
 
    dataString += String(age);
    dataString += ","; 
    dataString += String(chars);
    dataString += ","; 
    dataString += String(sentences);
    dataString += ","; 
    dataString += String(failed);

    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
    }  
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening file");
    }
}
