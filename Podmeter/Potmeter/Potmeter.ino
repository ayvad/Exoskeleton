int PotVal, PotAngle;
void setup() {
  Serial.begin(38400);
}

void loop() {
  PotVal = analogRead(0);
  PotAngle = map(PotVal, 0, 1024, 0, 180);
  Serial.println(PotAngle);
}
