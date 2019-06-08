#include <Wire.h>
#include <Adafruit_INA219.h>
#include <SD.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

Adafruit_INA219 ina219;

const int chipSelect = BUILTIN_SDCARD;

int led = 13;
int sweep_v1 = 0;
int sweep_v2 = 0;
int sweep_pin1 = 29; //0-3V
int sweep_pin2 = 30; //0-2V
int sweep_pin3 = 35; //1V

int sweep_v1_thresh = 232; //3.0V
int sweep_v2_thresh = 232; //3.0V
int sweep_v3 = 90; // >1.0V

int step_pin1 =  38; // port 38
int step_pin2 =  37; // port 37
int step_pin3 =  36; // port 36

int step_v1 = 0;
int step_v2 = 0;
int step_v3 = 0;

int t = 900;

int rx = 7; //blue - rxd
int tx = 8; //yellow - txd
int baudrate = 4800; //4800 bps from https://www.parallax.com/sites/default/files/downloads/PMB-648-v0.1.pdf
int scale = 3; //scale of ADXL337
float rawX; //teensy3.6 port 23
float rawY; //teensy3.6 port 22
float rawZ; //teensy3.6 port 21
float scaledX, scaledY, scaledZ; // Scaled values for each axis, based of adxl raw value scale
int rawScale = 1023; //based off the supply value of the Teensy3.6

TinyGPS gps;
SoftwareSerial ss(rx, tx);

void setup() {
  Serial.begin(115200);
  pinMode(led, OUTPUT);   
  while (!Serial) {
      // will pause Zero, Leonardo, etc until serial console opens
      delay(1);
  }

  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  ss.begin(baudrate);
  uint32_t currentFrequency;

  ina219.begin();
  
  pinMode(step_pin1, OUTPUT);
  pinMode(step_pin2, OUTPUT);
  pinMode(step_pin3, OUTPUT);
  pinMode(sweep_pin1, OUTPUT);
  pinMode(sweep_pin2, OUTPUT);
  pinMode(sweep_pin3, OUTPUT);
  analogWriteResolution(8);
  Serial.begin(9600);
}

void loop() {
    step_v1 = 77;
    analogWrite(step_pin1, step_v1);
    sweep();
    //delay(t);
    step_v1 = 155;
    analogWrite(step_pin1, step_v1);
    sweep();
    //delay(t);
    step_v1 = 232;
    analogWrite(step_pin1, step_v1);
    sweep();
    //delay(t);

    step_v2 = 77;
    analogWrite(step_pin2, step_v2);
    sweep();
    //delay(t);
    step_v2 = 155;
    analogWrite(step_pin2, step_v2);
    sweep();
    //delay(t);
    step_v2 = 232;
    analogWrite(step_pin1, step_v2);
    sweep();
    //delay(t);

    step_v3 = 77;
    analogWrite(step_pin3, step_v3);
    sweep();
    //delay(t);
    step_v3 = 155;
    analogWrite(step_pin3, step_v3);
    sweep();
    //delay(t);
    step_v3 = 232;
    analogWrite(step_pin3, step_v3);
    sweep();
    //delay(t);
    
    step_v1 = 0;
    step_v2 = 0;
    step_v3 = 0;
    analogWrite(step_pin1, step_v1);
    analogWrite(step_pin2, step_v2);
    analogWrite(step_pin3, step_v3);
    sweep();
    //delay(t);
}

void sweep(){
  digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;
  
  File dataFile = SD.open("tt2.csv", FILE_WRITE);
  String dataString = "";
  analogWriteFrequency(sweep_pin1, 100);
  analogWriteFrequency(sweep_pin2, 100);
  
  if(sweep_v1 < sweep_v1_thresh){
    sweep_v1++;
  }
  else if(sweep_v1 == sweep_v1_thresh && sweep_v2 < sweep_v2_thresh){
    sweep_v2++;
  }
  else{
    sweep_v1=0;
    sweep_v2=0;
  }
  analogWrite(sweep_pin1, sweep_v1);
  analogWrite(sweep_pin2, sweep_v2);
  analogWrite(sweep_pin3, sweep_v3);
  delay(20);
  float current_mA = 0;
  current_mA = ina219.getCurrent_mA();
  Serial.print("Current:       "); Serial.print(current_mA); Serial.println(" mA");
  Serial.println("");
  
  Serial.print(sweep_v1);
  Serial.print(" ");
  Serial.print(sweep_v2);
  Serial.print(" ");
  Serial.print(step_v1);
  Serial.print(" ");
  Serial.print(step_v2);
  Serial.print(" ");
  Serial.println(step_v3);

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

    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    gps.stats(&chars, &sentences, &failed);

  // if the file is available, write to it:
  if (dataFile) {
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
    dataString += ","; 
    dataString += String(flat);
    dataString += ","; 
    dataString += String(flon);
    dataString += ","; 
    dataString += String(age);
    dataString += ","; 
    dataString += String(chars);
    dataString += ","; 
    dataString += String(sentences);
    dataString += ","; 
    dataString += String(failed);
    dataString += ",";
    dataString += String(current_mA);
    dataString += ",";
    dataString += String((sweep_v3-(sweep_v1+sweep_v2)));
    dataString += ",";
    dataString += String((step_v1+step_v2+step_v3));
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }  
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening file");
  }
  digitalWrite(led, LOW);   // turn the LED on (HIGH is the voltage level)
}
