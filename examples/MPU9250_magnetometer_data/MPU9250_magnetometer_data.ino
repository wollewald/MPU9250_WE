/***************************************************************************
* Example sketch for the MPU9250_WE library
*
* This sketch shows how to read magnetometer values from the MPU9250, or 
* - to be more exact - from the AK8983. The AK8963 is part of the MPU9250 only
* to a certain extent. It has an own I2C address and needs to be initiated 
* separately. However the MPU9250 needs to be initiated before. 
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

/* There are several ways to create your MPU9250 object:
 * MPU9250_WE myMPU9250 = MPU9250_WE()              -> uses Wire / I2C Address = 0x68
 * MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR)  -> uses Wire / MPU9250_ADDR
 * MPU9250_WE myMPU9250 = MPU9250_WE(&wire2)        -> uses the TwoWire object wire2 / MPU9250_ADDR
 * MPU9250_WE myMPU9250 = MPU9250_WE(&wire2, MPU9250_ADDR) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  if(!myMPU9250.initMagnetometer()){
    Serial.println("Magnetometer does not respond");
  }
  else{
    Serial.println("Magnetometer is connected");
  }

  /* You can choose the following operational modes
   * AK8963_PWR_DOWN            power down (default)
   * AK8963_CONT_MODE_8HZ       continuous at 8Hz sample rate
   * AK8963_CONT_MODE_100HZ     continuous at 100Hz sample rate 
   * 
   * In trigger mode the AK8963 goes into power down after the measurement
   */
  myMPU9250.setMagOpMode(AK8963_CONT_MODE_100HZ);
  
  /* In continuous mode you need to wait for the first data to be available. If you 
   * comment the line below you will probably obtain zero. 
   */
  delay(200);
}

void loop() {
  xyzFloat magValue = myMPU9250.getMagValues(); // returns magnetic flux density [µT] 

  Serial.println("Magnetometer Data in µTesla: ");
  Serial.print(magValue.x);
  Serial.print("   ");
  Serial.print(magValue.y);
  Serial.print("   ");
  Serial.println(magValue.z);

  delay(1000);
}
