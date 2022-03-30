#include <Arduino_LSM6DS3.h>
void setup() {
  // put your setup code here, to run once:
  IMU.begin();
  Serial.begin(9600);
}
#define foutmarge 10;
int hoekz = 0;
float x,y,z;
int hoekpositief = 0;
int hoeknegatief = 0;
int beweging = 0;



void loop() {    

if (IMU.gyroscopeAvailable()) {
  IMU.readGyroscope(x, y, z);    
  hoekz += z/104;
  hoekpositief = hoekz - foutmarge;
  hoeknegatief = hoekz + foutmarge;
  if (hoeknegatief < 0){
    beweging = 1;
  }
  else if(hoekpositief > 0){
    beweging = 2;
  }
  else {
    beweging = 0;
  }
  Serial.println(beweging);
}

    
 /*   if (IMU.gyroscopeAvailable()) {
      teller += 1;
      if (teller >= 1){
        IMU.readGyroscope(x, y, z);
        //Serial.println(z);
        hoekx += x/104;
        hoeky += y/104;        
        hoekz += z/104;
        Serial.print(hoekx);
        Serial.println('\t');
        Serial.print(hoeky);
        Serial.println('\t');
        Serial.println(hoekz);
        teller = 0;
      }    
   } 
   */ 
}
