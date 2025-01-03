/***************************************************************************
* Example sketch for the MPU9250_WE library (MPU6050)
*
* This sketch shows how to get acceleration, gyroscocope and temperature 
* data from the MPU6050. In essence, the MPU6050 is an older version of the 
* MPU6500. Due to the poor register documentation, not all functions do work
* on the MPU6050 like they do on the MPU6500 or MPU9250. 
* 
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
* 
***************************************************************************/
#include <MPU6050_WE.h>
#include <Wire.h>
#define MPU6050_ADDR 0x68

/* There are several ways to create your MPU6050 object:
 * MPU6050_WE myMPU6050 = MPU6050_WE()              -> uses Wire / I2C Address = 0x68
 * MPU6050_WE myMPU6050 = MPU6050_WE(MPU6050_ADDR)  -> uses Wire / MPU6050_ADDR
 * MPU6050_WE myMPU6050 = MPU6050_WE(&wire2)        -> uses the TwoWire object wire2 / MPU6050_ADDR
 * MPU6050_WE myMPU6050 = MPU6050_WE(&wire2, MPU6050_ADDR) -> all together
 * Successfully tested with two I2C busses on an ESP32
 */
MPU6050_WE myMPU6050 = MPU6050_WE(MPU6050_ADDR);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU6050.init()){
    Serial.println("MPU6050 does not respond");
  }
  else{
    Serial.println("MPU6050 is connected");
  }
  
  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU6050 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU6050 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
  Serial.println("Position you MPU6050 flat and don't move it - calibrating...");
  delay(1000);
  myMPU6050.autoOffsets();
  Serial.println("Done!");
  
  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  Use either autoOffset or setAccOffsets, not both.
   */
  //myMPU6050.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  The gyroscope data is not zero, even if you don't move the MPU6050. 
   *  To start at zero, you can apply offset values. These are the gyroscope raw values you obtain
   *  using the +/- 250 degrees/s range. 
   *  Use either autoOffset or setGyrOffsets, not both.
   */
  //myMPU6050.setGyrOffsets(45.0, 145.0, -105.0);

  /*  MPU6050_GYRO_RANGE_250       250 degrees per second (default)
   *  MPU6050_GYRO_RANGE_500       500 degrees per second
   *  MPU6050_GYRO_RANGE_1000     1000 degrees per second
   *  MPU6050_GYRO_RANGE_2000     2000 degrees per second
   */
  myMPU6050.setGyrRange(MPU6050_GYRO_RANGE_250);

  /*  MPU6050_ACC_RANGE_2G      2 g   (default)
   *  MPU6050_ACC_RANGE_4G      4 g
   *  MPU6050_ACC_RANGE_8G      8 g   
   *  MPU6050_ACC_RANGE_16G    16 g
   */
  myMPU6050.setAccRange(MPU6050_ACC_RANGE_2G);
  
  delay(200);
}

void loop() {
  xyzFloat gValue = myMPU6050.getGValues();
  xyzFloat gyr = myMPU6050.getGyrValues();
  float temp = myMPU6050.getTemperature();
  float resultantG = myMPU6050.getResultantG(gValue);

  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  Serial.print("Resultant g: ");
  Serial.println(resultantG);

  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.print("Temperature in °C: ");
  Serial.println(temp);

  Serial.println("********************************************");

  delay(1000);
}
