void setup() {
  Serial.begin(115200);
}

void loop() {
  float raw_v1 = analogRead(A20);
  float raw_v2 = analogRead(A14);

  float v1 = 3.3*raw_v1/1023;
  float v2 = 3.3*raw_v2/1023;

  float r = 62; //r=62 ohms

  float i = (v1-v2)/r;

  Serial.print("raw_v1: "); Serial.print(raw_v1);
  Serial.print("   raw_v2: "); Serial.print(raw_v2);
  Serial.print("   v1: "); Serial.print(v1);
  Serial.print("   v2: "); Serial.print(v2);
  Serial.print("   r: "); Serial.print(r);
  Serial.print("   i: "); Serial.print(i);
  Serial.println(" ");
  delay(1000);

}
