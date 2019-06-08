int v1 =  23;
int v2 =  22;
int v3 =  21;

// The setup() method runs once, when the sketch starts

void setup()   {                
  // initialize the digitals pin as an outputs
  pinMode(v1, OUTPUT);
  pinMode(v2, OUTPUT);
  pinMode(v3, OUTPUT);
}

// the loop() method runs over and over again,

void loop()                     
{
//  analogWrite(v1, 85);
//  delay(1000);
//  analogWrite(v1, 170);
//  delay(1000);
  analogWrite(v1, 255);
//  delay(1000);
  
//  analogWrite(v2, 85);
//  delay(1000);
//  analogWrite(v2, 170);
//  delay(1000);
  analogWrite(v2, 255);
//  delay(1000);

//  analogWrite(v3, 85);
//  delay(1000);
//  analogWrite(v3, 170);
//  delay(1000);
  analogWrite(v3, 255);
  delay(1000);

  analogWrite(v1, 0);
  analogWrite(v2, 0);
  analogWrite(v3, 0);
  delay(1000);
 Serial.println(HIGH);
}
