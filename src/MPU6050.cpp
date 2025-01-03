/********************************************************************
* This is a library for the 9-axis gyroscope, accelerometer and magnetometer MPU9250.
*
* You'll find an example which should enable you to use the library.
*
* You are free to use it, change it or build on it. In case you like
* it, it would be cool if you give it a star.
*
* If you find bugs, please inform me!
*
* Written by Wolfgang (Wolle) Ewald
*
* For further information visit my blog:
*
* https://wolles-elektronikkiste.de/mpu9250-9-achsen-sensormodul-teil-1  (German)
* https://wolles-elektronikkiste.de/en/mpu9250-9-axis-sensor-module-part-1  (English)
*
*********************************************************************/

#include "MPU6050_WE.h"

/* Others */
uint8_t constexpr MPU6050_WE::WHO_AM_I_CODE;

/************  Constructors ************/

MPU6050_WE::MPU6050_WE(uint8_t addr)
    : MPU6500_WE(addr)
{
    // intentionally empty
}

MPU6050_WE::MPU6050_WE()
    : MPU6500_WE()
{
    // intentionally empty
}

MPU6050_WE::MPU6050_WE(TwoWire *w, uint8_t addr)
    : MPU6500_WE(w, addr)
{
    // intentionally empty
}

MPU6050_WE::MPU6050_WE(TwoWire *w)
    : MPU6500_WE(w)
{
    // intentionally empty
}

MPU6050_WE::MPU6050_WE(SPIClass *s, int cs, bool spi)
    : MPU6500_WE(s, cs, spi)
{
    // intentionally empty
}

/************ Init  ************/

bool MPU6050_WE::init(){
    return MPU6500_WE::init(WHO_AM_I_CODE);
}

/********* Temperature *********/
/* replaces the method of MPU6500 / 9250 */

float MPU6050_WE::getTemperature(){
    int16_t regVal16 = readMPU9250Register16(REGISTER_TEMP_OUT);
    float tmp = (regVal16/340.0 + 36.53);
    return tmp;
}
