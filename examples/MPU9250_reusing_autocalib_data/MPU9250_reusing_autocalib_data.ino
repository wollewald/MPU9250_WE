/***************************************************************************
* Example sketch for the MPU9250_WE library
*
* This sketch shows how to reuse autocalibration data by saving it to the 
* EEPROM or by writing it to the sketch. Use the defines REUSE_OFFSETS and 
* USE_EEPROM to switch between modes and options. 
*
* The sketch has been written for an ESP32 board. If you use a different 
* board, you might have to adjust the EEPROM functions.
* 
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
* 
***************************************************************************/

//#define REUSE_OFFSETS // comment out for calibration, uncomment for reuse
#define USE_EEPROM  // comment out if you don't want to use the EEPROM 
#ifdef USE_EEPROM
#include <EEPROM.h>
#endif
#include <MPU9250_WE.h>
#include <Wire.h>
#define MPU9250_ADDR 0x68

MPU9250_WE myMPU9250 = MPU9250_WE(MPU9250_ADDR);
xyzFloat aOffs; // acceleration offsets
xyzFloat gOffs; // gyroscope offsets

void setup() {
  delay(2000); // optional, might be needed to display everything on serial monitor
#ifdef USE_EEPROM
  EEPROM.begin(sizeof(xyzFloat)*2); // space for to xyzFloats
#endif
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  /* Apply the auto-calibration to determine the offsets for reuse.
   * Uncomment "#define reuse_offsets" at the top if you want to reuse previous offsets.
   */
#ifndef REUSE_OFFSETS 
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  Serial.println();
  
  /* Get the offsets from calibration and display them. 
  *  Uncomment line 20 if you want to reuse previous offsets.
  */
  aOffs = myMPU9250.getAccOffsets();  // get acceleration offsets
  gOffs = myMPU9250.getGyrOffsets();  // get gyroscope offsets 
  
  char buffer[35];
  sprintf(buffer, "{%d.0, %d.0, %d.0}", (int)aOffs.x, (int)aOffs.y, (int)aOffs.z);  
  Serial.print("Acceleration offsets, rounded: ");
  Serial.println(buffer); 
  sprintf(buffer, "{%d.0, %d.0, %d.0}", (int)gOffs.x, (int)gOffs.y, (int)gOffs.z);  
  Serial.print("Gyroscope offsets, rounded   : ");
  Serial.println(buffer); 

#ifdef USE_EEPROM
  EEPROM.put(0, aOffs);
  EEPROM.put(sizeof(xyzFloat), gOffs);
  EEPROM.commit();
#endif // USE_EEPROM
#endif // not REUSE_OFFSETS
  
#ifdef REUSE_OFFSETS
#ifndef USE_EEPROM 
  /* Now you can copy the offsets for reuse. Uncomment line 20. */
  xyzFloat aOffs = { /* copy your offsets here for reuse */ }; 
  xyzFloat gOffs = { /* copy your offsets here for reuse */ };
#endif // not USE_EEPROM
  
  /* Alternatively read your offsets back from EEPROM, if you have saved them there */
#ifdef USE_EEPROM
  xyzFloat aOffs = EEPROM.get(0, aOffs);
  xyzFloat gOffs = EEPROM.get(sizeof(xyzFloat), gOffs);
#endif // USE_EEPROM
  
  myMPU9250.setAccOffsets(aOffs);
  myMPU9250.setGyrOffsets(gOffs);
#endif // REUSE_OFFSETS
  
  /* Settings */
  myMPU9250.enableGyrDLPF();
  myMPU9250.setGyrDLPF(MPU9250_DLPF_6);
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setGyrRange(MPU9250_GYRO_RANGE_250);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  delay(200);
}

void loop() {
/* _Now you can test your calibration data */
#ifdef REUSE_OFFSETS  
  xyzFloat gValue = myMPU9250.getGValues();
  xyzFloat gyr = myMPU9250.getGyrValues();
   
  Serial.println("Acceleration in g (x,y,z):");
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
  
  Serial.println("Gyroscope data in degrees/s: ");
  Serial.print(gyr.x);
  Serial.print("   ");
  Serial.print(gyr.y);
  Serial.print("   ");
  Serial.println(gyr.z);

  Serial.println("********************************************");

  delay(2000);
#endif // REUSE_OFFSETS
}
