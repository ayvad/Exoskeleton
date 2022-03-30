#include <Arduino_LSM6DS3.h>
#include "BTS7960.h" //https://github.com/luisllamasbinaburo/Arduino-BTS7960


using namespace std;

#define HOEK1 20
#define HOEK2 0
#define HOEK3 58
#define HOEK4 0
#define REMMARGE 5
#define FOUTMARGE 15
#define RESETTIME 4000000
#define potOffset 106

//pins
const uint8_t R_EN = 7;
const uint8_t L_EN = 8;
const uint8_t L_PWM = 5;
const uint8_t R_PWM = 6;

BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);

int potAngleNonScale;
int potAngle;
int counter = 0;
int angleZ = 0;
float x, y, z;
int anglePos = 0;
int angleNeg = 0;
int beweging = 0;
int lastAngle;

void setup() {
  Serial.begin(9600);
  IMU.begin();
  motorController.Enable();
}

void loop() {
  //potmeter
  
  potAngleNonScale = map(analogRead(A0), 0, 1023, 180, 0) - potOffset;
  potAngle = map(potAngleNonScale, 0, 63, 0, 90);
  counter += 1;
  if (counter == RESETTIME) {
    lastAngle = potAngle ;
    counter = 0;
  }
  //Gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    angleZ += z / 104;
    anglePos = angleZ - FOUTMARGE;
    angleNeg = angleZ + FOUTMARGE;

    if (angleNeg < 0) {
      beweging = -1;
      Serial.println("beweging: -1");
    }
    else if (anglePos > 0) {
      beweging = 1;
      Serial.println("beweging: 1");
    }
    else {
      beweging = 0;
      Serial.println("beweging: 0");
    }
  }
  // loopbeweging
  if (beweging == 1) {
    if (potAngle > lastAngle) {
      if ((HOEK1 - potAngle) < REMMARGE) {
        // Eerste boog omhoog snel
        motorController.TurnRight(100);
        Serial.println("Eerste boog omhoog snel");
      }
      else {
        //Eerste boog omhoog langzaam
        motorController.TurnRight(20);
        Serial.println("Eerste boog omhoog langzaam");
      }
      if (potAngle < lastAngle) {
        if ((potAngle - HOEK2) < REMMARGE) {
          // Eerste boog omlaag snel
          motorController.TurnLeft(100);
          Serial.println("Eerste boog omlaag snel");

        }
        else {
          // Eerste boog omlaag langzaam
          motorController.TurnLeft(20);
          Serial.println("Eerste boog omlaag snel");
        }
      }
    }
    if (beweging == -1) {
      if (potAngle > lastAngle) {
        if ((HOEK3 - potAngle) < REMMARGE) {
          //Tweede boog omhoog snel
          motorController.TurnRight(120);
          Serial.println("Tweede boog omhoog snel");
        }
        else {
          //Tweede boog omhoog langzaam
          motorController.TurnRight(20);
          Serial.println("Tweede boog omhoog langzaam");
        }
      }
      if (potAngle < lastAngle) {
        if ((potAngle - HOEK4) < REMMARGE) {
          //Tweede boog omlaag snel
          motorController.TurnLeft(100);
          Serial.println("Tweede boog omlaag snel");
        }
        else {
          //Tweede boog omlaag langzaam
          motorController.TurnLeft(20);
          Serial.println("Tweede boog omlaag langzaam");
        }
      }
    }
  }
}
