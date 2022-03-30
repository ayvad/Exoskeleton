int PotVal, PotAngle;
void setup() {
  Serial.begin(9600);
}

void loop() {
  PotVal = analogRead(A0);
  PotAngle = map(PotVal, 0, 1023, 180, 0);
  Serial.println(PotAngle);
}
