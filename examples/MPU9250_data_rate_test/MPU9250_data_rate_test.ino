/***************************************************************************
* Example sketch for the MPU9250_WE library
*
* This sketch shows how to obtain (the maximum) data rates for the acceleration 
* sensor and the gyroscope. I have tested it on an AVR-based Arduino. 
* 
* If you process the data, the bottleneck might not be the MPU9250. In particular,
* if you print the measured values on the serial monitor, you will measure the 
* speed of your serial connection and not the data rate.  
*
* To achieve the highest data rates, you need SPI. I2C is too slow.
* 
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
* 
***************************************************************************/

#include <MPU9250_WE.h>
const unsigned long int numOfAccMeasurements = 4000;
const unsigned long int numOfGyrMeasurements = 32000;
const int csPin = 10;  // Chip Select Pin
bool useSPI = true;    // SPI use flag
const int intPin = 2;
volatile bool dataReady = false;

/* You need to use SPI to achieve a data rate of 4/32 kHz (acc/gyr) */
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, csPin, useSPI);

void setup() {
  Serial.begin(115200);
  myMPU9250.init();
  
  /* Settings for acceleration data rate test */
  myMPU9250.enableGyrAxes(MPU9250_ENABLE_000); // Disable gyroscope
  myMPU9250.setSampleRateDivider(0);
  myMPU9250.enableAccDLPF(false); // => should give a data rate of 4 kHz
  myMPU9250.setIntPinPolarity(MPU9250_ACT_HIGH); 
  myMPU9250.enableIntLatch(true);
  myMPU9250.enableClearIntByAnyRead(false); 
  myMPU9250.enableInterrupt(MPU9250_DATA_READY); 
  
  attachInterrupt(digitalPinToInterrupt(intPin), dataReadyISR, RISING);
  
  /* Perform acceleration data rate test and output */
  unsigned long duration = testDataRate(numOfAccMeasurements);
  Serial.println("Acceleration Data Rate Test:");
  char buf[50];
  snprintf(buf, sizeof(buf), "%lu measurements took %lu milliseconds", numOfAccMeasurements, duration);
  Serial.println(buf);
  Serial.println();
  
  /* Settings for gyroscope data rate test */
  myMPU9250.enableAccAxes(MPU9250_ENABLE_000); // disable acceleration
  myMPU9250.enableGyrAxes(MPU9250_ENABLE_XYZ); // enable gyroscope
  myMPU9250.disableGyrDLPF(MPU9250_BW_WO_DLPF_3600); // => should give a data rate of 32 kHz

  /* Perform gyroscope data rate test and output */
  duration = testDataRate(numOfGyrMeasurements);
  Serial.println("Gyroscope Data Rate Test:");
  snprintf(buf, sizeof(buf), "%lu measurements took %lu milliseconds", numOfGyrMeasurements, duration);
  Serial.println(buf);
  Serial.println();
}

void loop() {}

unsigned long int testDataRate(unsigned int numOfMeasurements){
  unsigned long int counter = 0;
  unsigned long int start = millis();
  while(counter < numOfMeasurements){
    while(!dataReady){}
    counter++;
    dataReady = false;
    myMPU9250.readAndClearInterrupts();
  }
  unsigned long int elapsedTime = millis() - start;
  return(elapsedTime);
}

void dataReadyISR() {
  dataReady = true;
}
