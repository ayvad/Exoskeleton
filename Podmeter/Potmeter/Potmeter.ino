int potAngle, potAngleNonScale;

//Potmeter config
#define maxAngleReal 0        // the maximum angle of the knee in relation with the potmeter
#define minAngleReal 90       // the minimum angle of the knee in relation with the potmeter
#define maxAnglePotmeter 557  // the maximum angle of the potmeter in relation with the knee
#define minAnglePotmeter 211  // the minimum angle of the potmeter in relation with the knee
#define potOffset 0           // the offset of the potmeter

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println(analogRead(A0));
  potAngle = map(analogRead(A0), minAnglePotmeter, maxAnglePotmeter, minAngleReal, maxAngleReal);
  Serial.println(potAngle);
  delay(500);
}
