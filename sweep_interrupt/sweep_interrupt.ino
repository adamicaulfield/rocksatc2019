IntervalTimer vsweep_Timer;

int sweep_v1 = 0;
int sweep_v2 = 0;
int sweep_pin1 = 29; //0-3V
int sweep_pin2 = 30; //0-2V
int sweep_pin3 = 35; //1V

int sweep_v1_thresh = 232; //3.0V
int sweep_v2_thresh = 232; //2.0V
int sweep_v3 = 90; //1.0V

int ledState = LOW;
const int led = LED_BUILTIN;  // the pin with a LED

void setup() {
  pinMode(sweep_pin1, OUTPUT);
  pinMode(sweep_pin2, OUTPUT);
  pinMode(sweep_pin3, OUTPUT);
  analogWriteResolution(8);

  pinMode(led, OUTPUT);
  vsweep_Timer.begin(voltage_sweep, 16000); //0.16 seconds
  Serial.begin(9600);
}

void voltage_sweep(void){
  if (ledState == LOW) {
    ledState = HIGH;
  } else {
    ledState = LOW;
  }
  digitalWrite(led, ledState);
//
//  analogWriteFrequency(sweep_pin1, 100);
//  analogWriteFrequency(sweep_pin2, 100);
  
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
}

void loop() {
//  // put your main code here, to run repeatedly:
//  analogWriteFrequency(sweep_pin1, 100);
//  analogWriteFrequency(sweep_pin2, 100);
//  
//  if(sweep_v1 < sweep_v1_thresh){
//    sweep_v1++;
//  }
//  else if(sweep_v1 == sweep_v1_thresh && sweep_v2 < sweep_v2_thresh){
//    sweep_v2++;
//  }
//  else{
//    sweep_v1=0;
//    sweep_v2=0;
//  }
//  analogWrite(sweep_pin1, sweep_v1);
//  analogWrite(sweep_pin2, sweep_v2);
//  analogWrite(sweep_pin3, sweep_v3);
//  delay(20);
  noInterrupts();
  interrupts();
  delay(100);
  Serial.print(sweep_v1);
  Serial.print(" ");
  Serial.println(sweep_v2);
}
