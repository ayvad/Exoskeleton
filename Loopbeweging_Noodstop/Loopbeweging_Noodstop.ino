/*  Project: Exoskeleton
 *  Projectgroup: MeH2.A1
*/
#include <Arduino_LSM6DS3.h>
#include "BTS7960.h" //https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include <math.h>
using namespace std;

//Potmeter config
#define maxAngleReal 0        // the maximum angle of the knee in relation with the potmeter
#define minAngleReal 180      // the minimum angle of the knee in relation with the potmeter
#define maxAnglePotmeter 1023 // the maximum angle of the potmeter in relation with the knee
#define minAnglePotmeter 0    // the minimum angle of the potmeter in relation with the knee
#define potOffset 105         // the offset of the potmeter

#define maxPotAngle 62        // the maximum angle of the scaled potmeter
#define minPotAngle 0         // the minimum angle of the scaled potmeter
#define maxRealAngle 90       // the real maximum angle
#define minRealAngle 0        // the real minimum angle

//Walking angles
#define ANGLE1 20  // the angle of the knee at the first peak
#define ANGLE2 0   // the angle of the knee between the peaks
#define ANGLE3 58  // the angle of the knee at the second peak
#define ANGLE4 0   // the angle of the knee at the end
//Resetvalues
#define REMMARGE 5      //
#define FOUTMARGE 15    //
#define RESETTIME 32767 // 
//Motorspeeds
#define SLOW_SPEED 255   // max 255   
#define FAST_SPEED 255  // max 255
#define ULTRA_SPEED 255 // max 255

//pins
#define potmeter A0
const uint8_t R_EN = 7;           //
const uint8_t L_EN = 8;           //
const uint8_t L_PWM = 5;          //
const uint8_t R_PWM = 6;          //
const int pinEmergencyBrake = 2;  //
const int endSwitchUp = 3;        //
const int endSwitchDown = 4;      //

//Variables
BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);
int potAngleNonScale;
int potAngle;
int counter = 0;
int angleZ = 0;
float x, y, z;
int anglePos = 0;
int angleNeg = 0;
int movement;
enum movementDirection {up = 1, down = -1, still = 0};
int lastAngle;

void emergency(){                           // If the emergency brake is pushed
  motorController.Stop();
  motorController.Disable();
  Serial.println("Emergency brake!");
  while(digitalRead(pinEmergencyBrake)){};
}

void setup() {
  Serial.begin(9600);
  IMU.begin();
  motorController.Enable();
  pinMode(pinEmergencyBrake, INPUT);
  attachInterrupt(digitalPinToInterrupt(pinEmergencyBrake), emergency, CHANGE);
  Serial.println("Initialisation done");
}

void loop() {
  //potmeter
  potAngleNonScale = map(analogRead(potmeter), minAnglePotmeter, maxAnglePotmeter, minAngleReal, maxAngleReal) - potOffset;
  potAngle = map(potAngleNonScale, minPotAngle, maxPotAngle, minRealAngle, maxRealAngle);
  counter += 1;
  if (counter == RESETTIME) {
    lastAngle = potAngle ;
    counter = 0;
  }
  //Gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    angleZ += z / IMU.gyroscopeSampleRate();
    anglePos = angleZ - FOUTMARGE;
    angleNeg = angleZ + FOUTMARGE;

    if (angleNeg < 0) {
      movement = down;
      Serial.println("beweging: -1");
    }
    else if (anglePos > 0) {
      movement = up;
      Serial.println("beweging: 1");
    }
    else {
      movement = still;
      Serial.println("beweging: 0");
    }
  }
  // loopbeweging
  if (movement == up) {
    if (potAngle > lastAngle) {
      if ((ANGLE1 - potAngle) < REMMARGE) {
        // Eerste boog omhoog snel
        motorController.TurnRight(FAST_SPEED);
        Serial.println("Eerste boog omhoog snel");
      }
      else {
        //Eerste boog omhoog langzaam
        motorController.TurnRight(SLOW_SPEED);
        Serial.println("Eerste boog omhoog langzaam");
      }
      if (potAngle < lastAngle) {
        if ((potAngle - ANGLE2) < REMMARGE) {
          // Eerste boog omlaag snel
          motorController.TurnLeft(FAST_SPEED);
          Serial.println("Eerste boog omlaag snel");

        }
        else {
          // Eerste boog omlaag langzaam
          motorController.TurnLeft(SLOW_SPEED);
          Serial.println("Eerste boog omlaag snel");
        }
      }
    }
    if (movement == down) {
      if (potAngle > lastAngle) {
        if ((ANGLE3 - potAngle) < REMMARGE) {
          //Tweede boog omhoog snel
          motorController.TurnRight(ULTRA_SPEED);
          Serial.println("Tweede boog omhoog snel");
        }
        else {
          //Tweede boog omhoog langzaam
          motorController.TurnRight(SLOW_SPEED);
          Serial.println("Tweede boog omhoog langzaam");
        }
      }
      if (potAngle < lastAngle) {
        if ((potAngle - ANGLE4) < REMMARGE) {
          //Tweede boog omlaag snel
          motorController.TurnLeft(ULTRA_SPEED);
          Serial.println("Tweede boog omlaag snel");
        }
        else {
          //Tweede boog omlaag langzaam
          motorController.TurnLeft(SLOW_SPEED);
          Serial.println("Tweede boog omlaag langzaam");
        }
      }
    }
  }
}
