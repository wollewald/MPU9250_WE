/***************************************************************************
* Example sketch for the MPU9250_WE library
*
* This sketch shows how use the data ready interrupt and the cycle function. 
* In cycle function the MPU9250 awakes periodically from power down mode, takes 
* a sample and goes back into sleep mode again. The data ready interrupt informs 
* new data is available. You see there is no delay in the main loop. The output 
* rate is cycle controlled.
* 
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
* 
***************************************************************************/

#include <MPU9250_WE.h>
#include <Wire.h>

const int intPin = 2;
volatile bool dataReady = false;

MPU9250_WE myMPU9250 = MPU9250_WE(0x68);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }
  
  /* The slope of the curve of acceleration vs measured values fits quite well to the theoretical 
   * values, e.g. 16384 units/g in the +/- 2g range. But the starting point, if you position the 
   * MPU9250 flat, is not necessarily 0g/0g/1g for x/y/z. The autoOffset function measures offset 
   * values. It assumes your MPU9250 is positioned flat with its x,y-plane. The more you deviate 
   * from this, the less accurate will be your results.
   * The function also measures the offset of the gyroscope data. The gyroscope offset does not   
   * depend on the positioning.
   * This function needs to be called at the beginning since it can overwrite your settings!
   */
  Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  
  /*  This is a more accurate method for calibration. You have to determine the minimum and maximum 
   *  raw acceleration values of the axes determined in the range +/- 2 g. 
   *  You call the function as follows: setAccOffsets(xMin,xMax,yMin,yMax,zMin,zMax);
   *  Use either autoOffset or setAccOffsets, not both.
   */
  //myMPU9250.setAccOffsets(-14240.0, 18220.0, -17280.0, 15590.0, -20930.0, 12080.0);

  /*  Sample rate divider divides the output rate of the gyroscope and accelerometer.
   *  Sample rate = Internal sample rate / (1 + divider) 
   *  It can only be applied if the corresponding DLPF is enabled and 0<DLPF<7!
   *  Divider is a number 0...255
   */
  myMPU9250.setSampleRateDivider(5);
  
  /*  MPU9250_ACC_RANGE_2G      2 g   (default)
   *  MPU9250_ACC_RANGE_4G      4 g
   *  MPU9250_ACC_RANGE_8G      8 g   
   *  MPU9250_ACC_RANGE_16G    16 g
   */
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);

  /*  Enable/disable the digital low pass filter for the accelerometer 
   *  If disabled the the bandwidth is 1.13 kHz, delay is 0.75 ms, output rate is 4 kHz
   */
  myMPU9250.enableAccDLPF(false);

  /*  Set accelerometer output data rate in low power mode (cycle enabled)
   *   MPU9250_LP_ACC_ODR_0_24          0.24 Hz
   *   MPU9250_LP_ACC_ODR_0_49          0.49 Hz
   *   MPU9250_LP_ACC_ODR_0_98          0.98 Hz
   *   MPU9250_LP_ACC_ODR_1_95          1.95 Hz
   *   MPU9250_LP_ACC_ODR_3_91          3.91 Hz
   *   MPU9250_LP_ACC_ODR_7_81          7.81 Hz
   *   MPU9250_LP_ACC_ODR_15_63        15.63 Hz
   *   MPU9250_LP_ACC_ODR_31_25        31.25 Hz
   *   MPU9250_LP_ACC_ODR_62_5         62.5 Hz
   *   MPU9250_LP_ACC_ODR_125         125 Hz
   *   MPU9250_LP_ACC_ODR_250         250 Hz
   *   MPU9250_LP_ACC_ODR_500         500 Hz
   */
  //myMPU9250.setLowPowerAccDataRate(MPU9250_LP_ACC_ODR_500);

  /*  Set the interrupt pin:
   *  MPU9250_ACT_LOW  = active-low
   *  MPU9250_ACT_HIGH = active-high (default) 
   */
  myMPU9250.setIntPinPolarity(MPU9250_ACT_HIGH); 

  /*  If latch is enabled the Interrupt Pin Level is held until the Interrupt Status 
   *  is cleared. If latch is disabled the Interrupt Puls is ~50µs (default).
   */
  myMPU9250.enableIntLatch(true);

  /*  The Interrupt can be cleared by any read. Otherwise the Interrupt will only be
   *  cleared if the Interrupt Status register is read (default).
   */
  myMPU9250.enableClearIntByAnyRead(false); 

  /*  Enable/disable interrupts:
   *  MPU9250_DATA_READY 
   *  MPU9250_FIFO_OVF   
   *  MPU9250_WOM_INT    
   *  
   *  You can enable all interrupts.
   */
  myMPU9250.enableInterrupt(MPU9250_DATA_READY); 
  //myMPU9250.disableInterrupt(MPU9250_FIFO_OVF);

  /* If cycle is set, and standby or sleep are not set, the module will cycle between
   *  sleep and taking a sample at a rate determined by setLowPowerAccDataRate().
   */
  myMPU9250.enableCycle(true);

  /* You can enable or disable the axes for gyroscope and/or accelerometer measurements.
   * By default all axes are enabled. Parameters are:  
   * MPU9250_ENABLE_XYZ  //all axes are enabled (default)
   * MPU9250_ENABLE_XY0  // X, Y enabled, Z disabled
   * MPU9250_ENABLE_X0Z   
   * MPU9250_ENABLE_X00
   * MPU9250_ENABLE_0YZ
   * MPU9250_ENABLE_0Y0
   * MPU9250_ENABLE_00Z
   * MPU9250_ENABLE_000  // all axes disabled
   */
  //myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
  attachInterrupt(digitalPinToInterrupt(intPin), dataReadyISR, RISING);
  
}

void loop() {
  if(dataReady){
    byte source = myMPU9250.readAndClearInterrupts();
    Serial.println("Interrupt!");
    if(myMPU9250.checkInterrupt(source, MPU9250_DATA_READY)){
      Serial.println("Interrupt Type: Data Ready");
      printData();
    }
    dataReady = false;
    myMPU9250.readAndClearInterrupts(); // if additional interrupts have occured in the meantime
  }
}

void printData(){
  xyzFloat gValue;
  gValue = myMPU9250.getGValues();
    
  Serial.print(gValue.x);
  Serial.print("   ");
  Serial.print(gValue.y);
  Serial.print("   ");
  Serial.println(gValue.z);
}

void dataReadyISR() {
  dataReady = true;
}
