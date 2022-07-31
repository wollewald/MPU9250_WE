/***************************************************************************
* Example sketch for the MPU9250_WE library
*
* This sketch checks which device you have.
* 
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
* 
***************************************************************************/

#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

void setup() {
  byte whoAmICode = 0x00;
  Serial.begin(115200);
  Wire.begin();
  myMPU9250.init();
  
  whoAmICode = myMPU9250.whoAmI();
  Serial.print("WhoAmI Register: 0x");
  Serial.println(whoAmICode, HEX);
  switch(whoAmICode){
    case(0x70):
      Serial.println("Your device is an MPU6500.");
      Serial.println("The MPU6500 does not have a magnetometer."); 
      break;
    case(0x71):
      Serial.println("Your device is an MPU9250");
      break;
    case(0x73):
      Serial.println("Your device is an MPU9255");
      Serial.println("Not sure if it works with this library, just try");
      break;
    case(0x75):
      Serial.println("Your device is probably an MPU6515"); 
      Serial.println("Not sure if it works with this library, just try");
      break;
    case(0x00):
      Serial.println("Can't connect to your device. Check all connections.");
      break;
    default:
      Serial.println("Unknown device - it may work with this library or not, just try"); 
  }  
}

void loop() {
}
