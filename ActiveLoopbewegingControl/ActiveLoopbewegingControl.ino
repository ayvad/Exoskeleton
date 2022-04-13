/*  Project: Exoskeleton
    Projectgroup: MeH2.A1
*/
#include <Arduino_LSM6DS3.h>
#include "BTS7960.h" //https://github.com/luisllamasbinaburo/Arduino-BTS7960
//#include <math.h>
using namespace std;

#define totaleTijd 1440  // miliseconden

//Potmeter config
#define maxAngleReal 0        // the maximum angle of the knee in relation with the potmeter
#define minAngleReal 90       // the minimum angle of the knee in relation with the potmeter
#define maxAnglePotmeter 557  // the maximum angle of the potmeter in relation with the knee
#define minAnglePotmeter 211  // the minimum angle of the potmeter in relation with the knee
#define potOffset 0           // the offset of the potmeter

//Walking angles
#define ANGLE1 8    // the angle of the knee at the first peak
#define ANGLE2 0    // the angle of the knee between the peaks
#define ANGLE3 35   // the angle of the knee at the second peak
#define ANGLE4 0    // the angle of the knee at the end

#define returnAngle 5
//Resetvalues
#define REMMARGE 5        //
#define FOUTMARGE 5       //
#define RESETTIME 32767   //
#define GYROSAMPLERATE 52 //

//Motorspeeds
#define SLOW_SPEED 150  // max 255   
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
BTS7960 MC(L_EN, R_EN, L_PWM, R_PWM);
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

void calcPotAngle() {
  potAngle = map(analogRead(potmeter), minAnglePotmeter, maxAnglePotmeter, minAngleReal, maxAngleReal) - potOffset;
  //return potAngle;
}

void emergency() {                          // If the emergency brake is pushed
  MC.Stop();
  MC.Disable();
  Serial.println("EMERGENCY BRAKE!");
  delay(1000);
}

void changeState() {
  if (state == idle) {
    state = active;
    Serial.println("State: ACTIVE");
  } else {
    state = idle;
    Serial.println("State: IDLE");
    MC.Stop();
  }; //Toggle state
  delay(500);
}

void setup() {
  state = startup;
  Serial.begin(9600);
  Serial.println("State: STARTUP");
  IMU.begin();
  MC.Enable();
  pinMode(endSwitchDown, INPUT);
  pinMode(endSwitchUp, INPUT);
  pinMode(pinEmergencyBrake, INPUT);
  pinMode(pinStartButton, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinEmergencyBrake), emergency, LOW);
  walkingAnglePosition = 1;
  state = active;
  Serial.println("Initialisation done");
  Serial.println("State: IDLE");
}

void loop() {
  //Gyroscope
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    angleZ += z / GYROSAMPLERATE;
    anglePos = angleZ - FOUTMARGE;
    angleNeg = angleZ + FOUTMARGE;

    if (angleNeg < 0) {
      movement = down;
      //Serial.println("movement: DOWN");
    }
    else if (anglePos > 0) {
      movement = up;
      //Serial.println("movement: UP");
    }
    else {
      movement = still;
      counter = 1;
    }
  }

  if (digitalRead(pinStartButton)) {
    changeState();
  }

  //  if (!digitalRead(endSwitchDown)){
  //    Serial.println("Endswitch Down = TRUE");
  //    }
  //  if (!digitalRead(endSwitchUp)){
  //    Serial.println("Endswitch Up = TRUE");
  //    }

  if ((movement == up) and (counter == 1) and (state == active)) {
    MC.TurnLeft(SLOW_SPEED);
    calcPotAngle();
    if (potAngle > ANGLE3) {
      while ((potAngle > ANGLE1)) {
        MC.TurnRight(MAX_SPEED);
        calcPotAngle();
        counter = 0;
      }
    }
  }
}
