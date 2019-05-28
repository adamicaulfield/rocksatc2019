
IntervalTimer myTimer;

const int v1 =  38; // port 38
const int v2 =  37; // port 37
const int v3 =  36; // port 36
int t = 900;

int v1_val = 0;
int v2_val = 0;
int v3_val = 0;
int ledState = LOW;

const int led = LED_BUILTIN;  // the pin with a LED

// The setup() method runs once, when the sketch starts

void setup()   {                
  // initialize the digitals pin as an outputs
  pinMode(v1, OUTPUT);
  pinMode(v2, OUTPUT);
  pinMode(v3, OUTPUT);

  pinMode(led, OUTPUT);
  myTimer.begin(voltage_step, 1000000); //1 second
  Serial.begin(9600);
}

// the loop() method runs over and over again,
void voltage_step(void)
{
  if (ledState == LOW) {
    ledState = HIGH;  
  } else {
    ledState = LOW;
  }
  digitalWrite(led, ledState);

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
  Serial.println("step");
}


void loop()                     
{
  noInterrupts();
  interrupts();
  delay(100);
}
