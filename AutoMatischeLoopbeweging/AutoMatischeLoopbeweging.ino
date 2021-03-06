/*  Project: Exoskeleton
    Projectgroup: MeH2.A1
*/
#include <Arduino_LSM6DS3.h>
#include "BTS7960.h" //https://github.com/luisllamasbinaburo/Arduino-BTS7960
#include <math.h>
using namespace std;

#define totaleTijd 1440  //2000 miliseconden

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

#define returnAngle 5
//Resetvalues
#define REMMARGE 5        //
#define FOUTMARGE 0       //
#define RESETTIME 32767   //
#define GYROSAMPLERATE 52 //
//Motorspeeds
#define SLOW_SPEED 100  // max 255   
#define FAST_SPEED 200  // max 255
#define MAX_SPEED 255   // max 255

//pins
#define potmeter A0
const uint8_t R_EN = 7;           //
const uint8_t L_EN = 8;           //
const uint8_t L_PWM = 5;          //
const uint8_t R_PWM = 6;          //
const int pinEmergencyBrake = 2;  //
const int pinStartButton = 9;     //
const int endSwitchUp = 3;        //
const int endSwitchDown = 4;      //

const int looptijd = totaleTijd / 8;

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
enum states {startup = 0, idle = 1, active = 2};
int lastAngle;
int walkingAnglePosition;
int state;

//void emergency() {                          // If the emergency brake is pushed
//  motorController.Stop();
//  motorController.Disable();
//  Serial.println("Emergency brake!");
//  while (digitalRead(pinEmergencyBrake)) {
//    delay(100);
//  };
//}

void changeState() {
  if (state == idle) {
    state = active;
  } else {
    state = idle;
  }; //Toggle state
  while (digitalRead(pinStartButton)) {
    delay(1000);
  };
}

void setup() {
  state = startup;
  Serial.begin(9600);
  IMU.begin();
  motorController.Enable();
  pinMode(pinEmergencyBrake, INPUT);
  pinMode(pinStartButton, INPUT);
  //attachInterrupt(digitalPinToInterrupt(pinEmergencyBrake), emergency, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(pinStartButton), changeState, RISING);
  Serial.println("Initialisation done");
  walkingAnglePosition = 1;
  state = idle;
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
    angleZ += z / GYROSAMPLERATE;
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
  Serial.println(digitalRead(pinStartButton));
  if (!digitalRead(pinStartButton)) {
    if (state == idle) {
      state = active;
    } else {
      state = idle;
    }; //Toggle state
  }

  //emergency();
  if (state == active) {
    if (movement == up) {
      Serial.println("Walk movement");
      motorController.TurnRight(FAST_SPEED);
      delay(looptijd);
      motorController.TurnLeft(MAX_SPEED);
      delay(looptijd);
      motorController.Stop();
    }
  }
}
