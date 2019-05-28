#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_INA219.h>

//Current measurement
Adafruit_INA219 ina219;
float current_mA = 0;

//Voltage-Sweep variables
IntervalTimer vsweep_Timer;
int sweep_v1 = 0;
int sweep_v2 = 0;
int sweep_pin1 = 29; //0-3V
int sweep_pin2 = 30; //0-3V
int sweep_pin3 = 35; //1V
int sweep_v1_thresh = 232; //3.0V
int sweep_v2_thresh = 232; //3.0V
int sweep_v3 = 90; //1.0V

//Voltage-Step varaibles
IntervalTimer vstep_Timer;
const int v1 =  38; // port 38
const int v2 =  37; // port 37
const int v3 =  36; // port 36
int v1_val = 0;
int v2_val = 0;
int v3_val = 0;

//SD Card variables
String dataString = "";
String dataCopy;
const int chipSelect = BUILTIN_SDCARD;

//INSS varaibles
unsigned long chars;
unsigned short sentences, failed;
float flat, flon;
unsigned long age;
int scale = 3; //scale of ADXL337
float rawX; //teensy3.6 port 23
float rawY; //teensy3.6 port 22
float rawZ; //teensy3.6 port 21
float scaledX, scaledY, scaledZ; // Scaled values for each axis, based of adxl raw value scale
int rawScale = 1023; //based off the supply value of the Teensy3.6
int count = 0;
int rx = 7; //yellow - rxd
int tx = 8; //blue - txd
int baudrate = 4800; //4800 bps from https://www.parallax.com/sites/default/files/downloads/PMB-648-v0.1.pdf
IntervalTimer inssTimer;
TinyGPS gps;
SoftwareSerial ss(rx, tx);

//LED varaibles
int ledState = LOW;
const int led = LED_BUILTIN;  // the pin with a LED

// The setup() method runs once, when the sketch starts

void setup()   {
  Serial.begin(9600); //9600 115200
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  uint32_t currentFrequency;
  ina219.begin();
  Serial.println("Initializing Card...");

  
  // initialize the digitals pin as an outputs
  pinMode(v1, OUTPUT);
  pinMode(v2, OUTPUT);
  pinMode(v3, OUTPUT);
  analogWriteResolution(8);
  
  pinMode(led, OUTPUT);
  vstep_Timer.begin(voltage_step, 8000000-5); //1 second
  ss.begin(baudrate);
  inssTimer.begin(inss_isr, 1000);  //1 ms
  vsweep_Timer.begin(voltage_sweep, 16000); //0.16 seconds

}

void voltage_sweep(void){
    
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    digitalWrite(led, ledState);

 
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
  current_mA = ina219.getCurrent_mA();
  
  analogWrite(sweep_pin1, sweep_v1);
  analogWrite(sweep_pin2, sweep_v2);
  analogWrite(sweep_pin3, sweep_v3);
}


void inss_isr(void){

  bool newData = false;

    //********** Accelerometer ****************//
  //read accelerometer data from ports A9, A8, A7 on Teensy 3.6, which are connected to the X,Y,Z output ports of ADXL337 respectively
  rawX = analogRead(A9); //23
  rawY = analogRead(A8); //22
  rawZ = analogRead(A7); //21

  //use the map function to create the scaled values as integers
  scaledX = map(rawX, 0, rawScale, -scale, scale);
  scaledY = map(rawY, 0, rawScale, -scale, scale);
  scaledZ = map(rawZ, 0, rawScale, -scale, scale);

  // For one second we parse GPS data and report some key values
  //for (unsigned long start = millis(); millis() - start < 1000;)
    while (ss.available())
    {
      char c = ss.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }

  if (count==1000) //if collected data for 1 second
  {
    count = 0;
    gps.f_get_position(&flat, &flon, &age);  
    gps.stats(&chars, &sentences, &failed);
    if (chars == 0)
      Serial.println("** No characters received from GPS: check wiring **");  
    }
    count++;

    
    dataString = "";
    dataString += String(current_mA);
    dataString += ",";
    dataString += String((v1_val+v2_val+v3_val)/77);
    dataString += ",";
    dataString += String((sweep_v3 - (sweep_v2+sweep_v1))/77);
    dataString += ",";
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

    //Serial.println(dataString);

    File dataFile = SD.open("integrated.txt", FILE_WRITE);
    Serial.print("sizeof(dataString)=");
    Serial.print(sizeof(dataString));
    Serial.println("");
    if (dataFile) {
      Serial.println("HEllo");
       dataFile.println(dataString);
       dataFile.close();
    }  
     // if the file isn't open, pop up an error:
    else {
       //Serial.println("error opening file");
    }
}
void voltage_step(void)
{
  if(v1_val < 231){
    v1_val += 77; //int 77 = 1V
  }
  else if(v1_val >=231 && v2_val < 231){
    v2_val += 77;
  }
  else if(v1_val >=231 && v2_val>=231 && v3_val < 231){
    v3_val += 77;
  }
  else if(v1_val >=231 && v2_val>=231 && v3_val >= 231){
    v1_val = 0;
    v2_val = 0;
    v3_val = 0;
  }

  analogWrite(v1,v1_val);
  analogWrite(v2,v2_val);
  analogWrite(v3,v3_val);
}


void loop()                     
{
  
}
