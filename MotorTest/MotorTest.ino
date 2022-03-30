#include "BTS7960.h" //https://github.com/luisllamasbinaburo/Arduino-BTS7960

//pins
const uint8_t R_EN = 7;
const uint8_t L_EN = 8;
const uint8_t R_PWM = 6;
const uint8_t L_PWM = 5;

BTS7960 motorController(L_EN, R_EN, L_PWM, R_PWM);
void setup() {
  motorController.Enable();
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);
}

void loop() {
  motorController.Enable();
  //  digitalWrite(R_EN, HIGH);
  //  digitalWrite(L_EN, HIGH);



  while (1) {
    //    analogWrite(L_PWM, 0);
    //    analogWrite(R_PWM, 100);
    motorController.TurnRight(100);
    delay(500);

    //    analogWrite(R_PWM, 0);
    //    analogWrite(L_PWM, 0);
    motorController.Stop();
    delay(1000);

    //    analogWrite(R_PWM, 0);
    //    analogWrite(L_PWM, 100);
    motorController.TurnLeft(100);
    delay(500);

    //    analogWrite(R_PWM, 0);
    //    analogWrite(L_PWM, 0);
    motorController.Stop();
    delay(1000);
  }
}
