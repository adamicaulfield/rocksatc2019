int redPin =  21;
int greenPin =  22;
int bluePin =  23;

// The setup() method runs once, when the sketch starts

void setup()   {                
  // initialize the digitals pin as an outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
}

// the loop() method runs over and over again,

void loop()                     
{
  digitalWrite(redPin, HIGH);
  delay(500);
  digitalWrite(greenPin, HIGH);
  delay(500);
  digitalWrite(bluePin, HIGH);
  delay(500);
  digitalWrite(redPin, LOW);
  delay(500);
  digitalWrite(greenPin, LOW);
  delay(500);
  digitalWrite(bluePin, LOW);
  delay(500);
}
