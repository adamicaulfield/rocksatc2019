#include <SoftwareSerial.h>
#include <TinyGPS.h>

/* Based on sample simple_test by Mikal Hart 
 *  https://github.com/mikalhart/TinyGPS/releases/tag/v13
*/

//changed port values for teensy 3.6
int rx = 7; //yellow - rxd
int tx = 8; //blue - txd
int baudrate = 4800; //4800 bps from https://www.parallax.com/sites/default/files/downloads/PMB-648-v0.1.pdf
const int ledPin = LED_BUILTIN;  // the pin with a LED
int ledState = LOW;
int count = 0;
IntervalTimer myTimer;
TinyGPS gps;
SoftwareSerial ss(rx, tx);

void setup()
{
  Serial.begin(115200);
  ss.begin(baudrate);
  myTimer.begin(blinkLED, 1000);
   pinMode(ledPin, OUTPUT);
}

void blinkLED() {

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  //for (unsigned long start = millis(); millis() - start < 1000;)
  //{
    while (ss.available())
    {
      char c = ss.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  //}

  if (count==1000)
  {
      if (ledState == LOW) {
        ledState = HIGH;
      } else {
        ledState = LOW;
      }
  digitalWrite(ledPin, ledState);
    count = 0;
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
  if (chars == 0)
    Serial.println("** No characters received from GPS: check wiring **");
  }
  count++;
}

void loop()
{
//  bool newData = false;
//  unsigned long chars;
//  unsigned short sentences, failed;
//
//  // For one second we parse GPS data and report some key values
//  for (unsigned long start = millis(); millis() - start < 1000;)
//  {
//    while (ss.available())
//    {
//      char c = ss.read();
//      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
//      if (gps.encode(c)) // Did a new valid sentence come in?
//        newData = true;
//    }
//  }
//
// // if (newData)
// // {
//    float flat, flon;
//    unsigned long age;
//    gps.f_get_position(&flat, &flon, &age);
//    Serial.print("LAT=");
//    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
//    Serial.print(" LON=");
//    Serial.print(", ");
//    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
//    Serial.print(" SAT=");
//    Serial.print(", ");
//    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
//    Serial.print(" PREC=");
//    Serial.print(", ");
//    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
//  //}
//  
//  gps.stats(&chars, &sentences, &failed);
//  Serial.print(" CHARS=");
//    Serial.print(", ");
//  Serial.print(chars);
//  Serial.print(" SENTENCES=");
//    Serial.print(", ");
//  Serial.print(sentences);
//  Serial.print(" CSUM ERR=");
//    Serial.print(", ");
//  Serial.println(failed);
//  if (chars == 0)
//    Serial.println("** No characters received from GPS: check wiring **");
}
