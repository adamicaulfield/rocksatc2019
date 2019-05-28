int v1 =  38; // port 38
int v2 =  37; // port 37
int v3 =  36; // port 36
int t = 900;

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
  analogWrite(v1, 77);
  delay(t);
  analogWrite(v1, 155);
  delay(t);
  analogWrite(v1, 232);
  delay(t);
  
  analogWrite(v2, 77);
  delay(t);
  analogWrite(v2, 155);
  delay(t);
  analogWrite(v2, 232);
  delay(t);

  analogWrite(v3, 77);
  delay(t);
  analogWrite(v3, 155);
  delay(t);
  analogWrite(v3, 232);
  delay(t);

  analogWrite(v1, 0);
  analogWrite(v2, 0);
  analogWrite(v3, 0);
  delay(t);
}
