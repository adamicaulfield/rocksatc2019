#include <SoftwareSerial.h>
#include <TinyGPS.h>

/* Based on sample simple_test by Mikal Hart 
 *  https://github.com/mikalhart/TinyGPS/releases/tag/v13
*/
//changed port values for teensy 3.6
int rx = 7; //yellow - rxd
int tx = 8; //blue - txd
int baudrate = 4800; //4800 bps from https://www.parallax.com/sites/default/files/downloads/PMB-648-v0.1.pdf
TinyGPS gps;
SoftwareSerial ss(rx, tx);

// Create an IntervalTimer object 
IntervalTimer myTimer;

const int ledPin = LED_BUILTIN;  // the pin with a LED

void setup() {
  pinMode(ledPin, OUTPUT);
  Serial.begin(115200);
  ss.begin(baudrate);
  myTimer.begin(gps, 10000);  // blinkLED to run every 0.15 seconds
}

// The interrupt will blink the LED, and keep
// track of how many times it has blinked.
int ledState = LOW;
volatile unsigned long blinkCount = 0; // use volatile for shared variables

// functions called by IntervalTimer should be short, run as quickly as
// possible, and should avoid calling other functions if possible.
void gps() {
  if (ledState == LOW) {
    ledState = HIGH;
    blinkCount = blinkCount + 1;  // increase when LED turns on
  } else {
    ledState = LOW;
  }
  digitalWrite(ledPin, ledState);

   bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (ss.available())
    {
      char c = ss.read();
      //Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

 // if (newData)
 // {
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
  //}
  
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

// The main program will print the blink count
// to the Arduino Serial Monitor
void loop() {
  unsigned long blinkCopy;  // holds a copy of the blinkCount

  // to read a variable which the interrupt code writes, we
  // must temporarily disable interrupts, to be sure it will
  // not change while we are reading.  To minimize the time
  // with interrupts off, just quickly make a copy, and then
  // use the copy while allowing the interrupt to keep working.
  noInterrupts();
  blinkCopy = blinkCount;
  interrupts();

  Serial.print("blinkCount = ");
  Serial.println(blinkCopy);
  delay(100);
}
