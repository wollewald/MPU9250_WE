/********************************************************************
* 
* This is a library for the 6-axis gyroscope and accelerometer MPU6500.
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

#include "MPU6500_WE.h"

/* Registers MPU6500 */
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_X_GYRO     ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Y_GYRO     ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Z_GYRO     ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_X_ACCEL    ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Y_ACCEL    ;
uint8_t constexpr MPU6500_WE::REGISTER_SELF_TEST_Z_ACCEL    ;
uint8_t constexpr MPU6500_WE::REGISTER_XG_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_XG_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_YG_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_YG_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZG_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZG_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_SMPLRT_DIV           ;
uint8_t constexpr MPU6500_WE::REGISTER_CONFIG               ;
uint8_t constexpr MPU6500_WE::REGISTER_GYRO_CONFIG          ;
uint8_t constexpr MPU6500_WE::REGISTER_ACCEL_CONFIG         ;
uint8_t constexpr MPU6500_WE::REGISTER_ACCEL_CONFIG_2       ;
uint8_t constexpr MPU6500_WE::REGISTER_LP_ACCEL_ODR         ;
uint8_t constexpr MPU6500_WE::REGISTER_WOM_THR              ;
uint8_t constexpr MPU6500_WE::REGISTER_FIFO_EN              ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_MST_CTRL         ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_ADDR        ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_REG         ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_CTRL        ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_MST_STATUS       ;
uint8_t constexpr MPU6500_WE::REGISTER_INT_PIN_CFG          ;
uint8_t constexpr MPU6500_WE::REGISTER_INT_ENABLE           ;
uint8_t constexpr MPU6500_WE::REGISTER_INT_STATUS           ;
uint8_t constexpr MPU6500_WE::REGISTER_ACCEL_OUT            ;
uint8_t constexpr MPU6500_WE::REGISTER_TEMP_OUT             ;
uint8_t constexpr MPU6500_WE::REGISTER_GYRO_OUT             ;
uint8_t constexpr MPU6500_WE::REGISTER_EXT_SLV_SENS_DATA_00 ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_SLV0_DO          ;
uint8_t constexpr MPU6500_WE::REGISTER_I2C_MST_DELAY_CTRL   ;
uint8_t constexpr MPU6500_WE::REGISTER_SIGNAL_PATH_RESET    ;
uint8_t constexpr MPU6500_WE::REGISTER_MOT_DET_CTRL         ;
uint8_t constexpr MPU6500_WE::REGISTER_USER_CTRL            ;
uint8_t constexpr MPU6500_WE::REGISTER_PWR_MGMT_1           ;
uint8_t constexpr MPU6500_WE::REGISTER_PWR_MGMT_2           ;
uint8_t constexpr MPU6500_WE::REGISTER_FIFO_COUNT           ;
uint8_t constexpr MPU6500_WE::REGISTER_FIFO_R_W             ;
uint8_t constexpr MPU6500_WE::REGISTER_WHO_AM_I             ;
uint8_t constexpr MPU6500_WE::REGISTER_XA_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_XA_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_YA_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_YA_OFFSET_L          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZA_OFFSET_H          ;
uint8_t constexpr MPU6500_WE::REGISTER_ZA_OFFSET_L          ;

/* Register Values */
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_RESET          ;
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_BYPASS_EN      ;
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_I2C_MST_EN     ;
uint8_t constexpr MPU6500_WE::REGISTER_VALUE_CLK_SEL_PLL    ;

/* Others */
float constexpr MPU6500_WE::ROOM_TEMPERATURE_OFFSET         ;
float constexpr MPU6500_WE::TEMPERATURE_SENSITIVITY         ;
float constexpr MPU6500_WE::WHO_AM_I_CODE                   ;

/************  Constructors ************/

MPU6500_WE::MPU6500_WE(uint8_t addr)
    : MPU6500_WE(&Wire, addr)
{
    // intentionally empty
}

MPU6500_WE::MPU6500_WE(TwoWire *w, uint8_t addr)
    : _wire(w)
    , i2cAddress(addr)
{
    // intentionally empty
}

MPU6500_WE::MPU6500_WE(SPIClass *s, int cs, bool spi)
    : _spi(s)
    , csPin(cs)
    , useSPI(spi)
{
    // intentionally empty
}


/************ Basic Settings ************/

bool MPU6500_WE::init(uint8_t const expectedValue){
    if(useSPI){
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        _spi->begin();
        mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);      
    }   
    reset_MPU9250();
    delay(10);
    writeMPU9250Register(REGISTER_INT_PIN_CFG, REGISTER_VALUE_BYPASS_EN);  // Bypass Enable
    delay(10);
    if(whoAmI() != expectedValue){
        return false;
    }

    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    accRangeFactor = 1;
    gyrOffsetVal.x = 0.0;
    gyrOffsetVal.y = 0.0;
    gyrOffsetVal.z = 0.0;
    gyrRangeFactor = 1;
    fifoType = MPU9250_FIFO_ACC;
    sleep(false);

    return true;
}


bool MPU6500_WE::init(){
    return init(WHO_AM_I_CODE);
}

uint8_t MPU6500_WE::whoAmI(){
    return readMPU9250Register8(REGISTER_WHO_AM_I);
}

void MPU6500_WE::autoOffsets(){
    enableGyrDLPF();
    setGyrDLPF(MPU9250_DLPF_6);  // lowest noise
    setGyrRange(MPU9250_GYRO_RANGE_250); // highest resolution
    setAccRange(MPU9250_ACC_RANGE_2G);
    enableAccDLPF(true);
    setAccDLPF(MPU9250_DLPF_6);
    delay(100);

    xyzFloat accelerationOffsetAccumulator{0.f, 0.f, 0.f};
    xyzFloat gyroOffsetAccumulator{0.f, 0.f, 0.f};
    for(int i=0; i<50; i++){
        // acceleration
        accelerationOffsetAccumulator += getAccRawValues();
        // gyro
        gyroOffsetAccumulator += getGyrRawValues();
        delay(1);
    }

    // acceleration
    accelerationOffsetAccumulator /= 50.f;
    accelerationOffsetAccumulator.z -= 16384.0f;
    accOffsetVal = accelerationOffsetAccumulator;
    // gyro
    gyrOffsetVal = gyroOffsetAccumulator / 50.f;

}

void MPU6500_WE::setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    accOffsetVal.x = (xMax + xMin) * 0.5;
    accOffsetVal.y = (yMax + yMin) * 0.5;
    accOffsetVal.z = (zMax + zMin) * 0.5;
}

void MPU6500_WE::setGyrOffsets(float xOffset, float yOffset, float zOffset){
    gyrOffsetVal.x = xOffset;
    gyrOffsetVal.y = yOffset;
    gyrOffsetVal.z = zOffset;
}

void MPU6500_WE::setGyrDLPF(MPU9250_dlpf dlpf){
    uint8_t regVal = readMPU9250Register8(REGISTER_CONFIG);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(REGISTER_CONFIG, regVal);
}

void MPU6500_WE::setSampleRateDivider(uint8_t splRateDiv){
    writeMPU9250Register(REGISTER_SMPLRT_DIV, splRateDiv);
}

void MPU6500_WE::setGyrRange(MPU9250_gyroRange gyroRange){
    uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
    regVal &= 0xE7;
    regVal |= (gyroRange<<3);
    writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
    gyrRangeFactor = (1<<gyroRange);
}

void MPU6500_WE::enableGyrDLPF(){
    uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
    regVal &= 0xFC;
    writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
}

void MPU6500_WE::disableGyrDLPF(MPU9250_bw_wo_dlpf bw){
    uint8_t regVal = readMPU9250Register8(REGISTER_GYRO_CONFIG);
    regVal &= 0xFC;
    regVal |= bw;
    writeMPU9250Register(REGISTER_GYRO_CONFIG, regVal);
}

void MPU6500_WE::setAccRange(MPU9250_accRange accRange){
    uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG);
    regVal &= 0xE7;
    regVal |= (accRange<<3);
    writeMPU9250Register(REGISTER_ACCEL_CONFIG, regVal);
    accRangeFactor = 1<<accRange;
}

void MPU6500_WE::enableAccDLPF(bool enable){
    uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
    if(enable){
        regVal &= ~8;
    }
    else{
        regVal |= 8;
    }
    writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
}

void MPU6500_WE::setAccDLPF(MPU9250_dlpf dlpf){
    uint8_t regVal = readMPU9250Register8(REGISTER_ACCEL_CONFIG_2);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(REGISTER_ACCEL_CONFIG_2, regVal);
}

void MPU6500_WE::setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr){
    writeMPU9250Register(REGISTER_LP_ACCEL_ODR, lpaodr);
}

void MPU6500_WE::enableAccAxes(MPU9250_xyzEn enable){
    uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
    regVal &= ~(0x38);
    regVal |= (enable<<3);
    writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
}

void MPU6500_WE::enableGyrAxes(MPU9250_xyzEn enable){
    uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_2);
    regVal &= ~(0x07);
    regVal |= enable;
    writeMPU9250Register(REGISTER_PWR_MGMT_2, regVal);
}

void MPU6500_WE::setSPIClockSpeed(unsigned long clock){
    mySPISettings = SPISettings(clock, MSBFIRST, SPI_MODE0);
}

/************* x,y,z results *************/

xyzFloat MPU6500_WE::getAccRawValues(){
    uint8_t rawData[6]; 
    readMPU9250Register3x16(REGISTER_ACCEL_OUT, rawData);
    int16_t const xRaw = static_cast<int16_t>((rawData[0] << 8) | rawData[1]);
    int16_t const yRaw = static_cast<int16_t>((rawData[2] << 8) | rawData[3]);
    int16_t const zRaw = static_cast<int16_t>((rawData[4] << 8) | rawData[5]);
    return xyzFloat{static_cast<float>(xRaw), static_cast<float>(yRaw), static_cast<float>(zRaw)};
}

xyzFloat MPU6500_WE::getCorrectedAccRawValues(){
    xyzFloat rawValue = getAccRawValues();
    correctAccRawValues(rawValue);
    return rawValue;
}

xyzFloat MPU6500_WE::getGValues(){
    xyzFloat const acceleration = getCorrectedAccRawValues();
    return acceleration * (static_cast<float>(accRangeFactor) / 16384.0f);
}

xyzFloat MPU6500_WE::getAccRawValuesFromFifo(){
    xyzFloat accRawVal = readMPU9250xyzValFromFifo();
    return accRawVal;
}

xyzFloat MPU6500_WE::getCorrectedAccRawValuesFromFifo(){
    xyzFloat accRawVal = getAccRawValuesFromFifo();
    correctAccRawValues(accRawVal);
    return accRawVal;
}

xyzFloat MPU6500_WE::getGValuesFromFifo(){
    xyzFloat accRawVal = getCorrectedAccRawValuesFromFifo();
    return accRawVal * (static_cast<float>(accRangeFactor) / 16384.0f);
}

float MPU6500_WE::getResultantG(xyzFloat gVal){
    float resultant = 0.0;
    resultant = sqrt(sq(gVal.x) + sq(gVal.y) + sq(gVal.z));

    return resultant;
}

float MPU6500_WE::getTemperature(){
    int16_t regVal16 = readMPU9250Register16(REGISTER_TEMP_OUT);
    float tmp = (regVal16*1.0 - ROOM_TEMPERATURE_OFFSET)/TEMPERATURE_SENSITIVITY + 21.0;
    return tmp;
}

xyzFloat MPU6500_WE::getGyrRawValues(){
    uint8_t rawData[6]; 
    readMPU9250Register3x16(REGISTER_GYRO_OUT, rawData);
    int16_t const xRaw = static_cast<int16_t>((rawData[0] << 8) | rawData[1]);
    int16_t const yRaw = static_cast<int16_t>((rawData[2] << 8) | rawData[3]);
    int16_t const zRaw = static_cast<int16_t>((rawData[4] << 8) | rawData[5]);
    return xyzFloat{static_cast<float>(xRaw), static_cast<float>(yRaw), static_cast<float>(zRaw)};
}

xyzFloat MPU6500_WE::getCorrectedGyrRawValues(){
    xyzFloat gyrRawVal = getGyrRawValues();
    correctGyrRawValues(gyrRawVal);
    return gyrRawVal;
}

xyzFloat MPU6500_WE::getGyrValues(){
    xyzFloat const gyroValues = getCorrectedGyrRawValues();
    return gyroValues * (static_cast<float>(gyrRangeFactor) * 250.f / 32768.0f);
}

xyzFloat MPU6500_WE::getGyrValuesFromFifo(){
    xyzFloat gyroValues = readMPU9250xyzValFromFifo();
    correctGyrRawValues(gyroValues);
    return gyroValues * (static_cast<float>(gyrRangeFactor) * 250.f / 32768.0f);
}

/********* Power, Sleep, Standby *********/

void MPU6500_WE::sleep(bool sleep){
    uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
    if(sleep){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
}

void MPU6500_WE::enableCycle(bool cycle){
    uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
    if(cycle){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
}

void MPU6500_WE::enableGyrStandby(bool gyroStandby){
    uint8_t regVal = readMPU9250Register8(REGISTER_PWR_MGMT_1);
    if(gyroStandby){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(REGISTER_PWR_MGMT_1, regVal);
}


/******** Angles and Orientation *********/

xyzFloat MPU6500_WE::getAngles(){
    xyzFloat angleVal;
    xyzFloat gVal = getGValues();
    if(gVal.x > 1.0){
        gVal.x = 1.0;
    }
    else if(gVal.x < -1.0){
        gVal.x = -1.0;
    }
    angleVal.x = (asin(gVal.x)) * 57.296;

    if(gVal.y > 1.0){
        gVal.y = 1.0;
    }
    else if(gVal.y < -1.0){
        gVal.y = -1.0;
    }
    angleVal.y = (asin(gVal.y)) * 57.296;

    if(gVal.z > 1.0){
        gVal.z = 1.0;
    }
    else if(gVal.z < -1.0){
        gVal.z = -1.0;
    }
    angleVal.z = (asin(gVal.z)) * 57.296;

    return angleVal;
}

MPU9250_orientation MPU6500_WE::getOrientation(){
    xyzFloat angleVal = getAngles();
    MPU9250_orientation orientation = MPU9250_FLAT;
    if(abs(angleVal.x) < 45){      // |x| < 45
        if(abs(angleVal.y) < 45){      // |y| < 45
            if(angleVal.z > 0){          //  z  > 0
                orientation = MPU9250_FLAT;
            }
            else{                        //  z  < 0
                orientation = MPU9250_FLAT_1;
            }
        }
        else{                         // |y| > 45
            if(angleVal.y > 0){         //  y  > 0
                orientation = MPU9250_XY;
            }
            else{                       //  y  < 0
                orientation = MPU9250_XY_1;
            }
        }
    }
    else{                           // |x| >= 45
        if(angleVal.x > 0){           //  x  >  0
            orientation = MPU9250_YX;
        }
        else{                       //  x  <  0
            orientation = MPU9250_YX_1;
        }
    }
    return orientation;
}

String MPU6500_WE::getOrientationAsString(){
    MPU9250_orientation orientation = getOrientation();
    String orientationAsString = "";
    switch(orientation){
        case MPU9250_FLAT:      orientationAsString = "z up";   break;
        case MPU9250_FLAT_1:    orientationAsString = "z down"; break;
        case MPU9250_XY:        orientationAsString = "y up";   break;
        case MPU9250_XY_1:      orientationAsString = "y down"; break;
        case MPU9250_YX:        orientationAsString = "x up";   break;
        case MPU9250_YX_1:      orientationAsString = "x down"; break;
    }
    return orientationAsString;
}

float MPU6500_WE::getPitch(){
    xyzFloat angleVal = getAngles();
    float pitch = (atan2(-angleVal.x, sqrt(abs((angleVal.y*angleVal.y + angleVal.z*angleVal.z))))*180.0)/M_PI;
    return pitch;
}


float MPU6500_WE::getRoll(){
    xyzFloat angleVal = getAngles();
    float roll = (atan2(angleVal.y, angleVal.z)*180.0)/M_PI;
    return roll;
}


/************** Interrupts ***************/

void MPU6500_WE::setIntPinPolarity(MPU9250_intPinPol pol){
    uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
    if(pol){
        regVal |= 0x80;
    }
    else{
        regVal &= ~(0x80);
    }
    writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
}

void MPU6500_WE::enableIntLatch(bool latch){
    uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
    if(latch){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
}

void MPU6500_WE::enableClearIntByAnyRead(bool clearByAnyRead){
    uint8_t regVal = readMPU9250Register8(REGISTER_INT_PIN_CFG);
    if(clearByAnyRead){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(REGISTER_INT_PIN_CFG, regVal);
}

void MPU6500_WE::enableInterrupt(MPU9250_intType intType){
    uint8_t regVal = readMPU9250Register8(REGISTER_INT_ENABLE);
    regVal |= intType;
    writeMPU9250Register(REGISTER_INT_ENABLE, regVal);
}

void MPU6500_WE::disableInterrupt(MPU9250_intType intType){
    uint8_t regVal = readMPU9250Register8(REGISTER_INT_ENABLE);
    regVal &= ~intType;
    writeMPU9250Register(REGISTER_INT_ENABLE, regVal);
}

bool MPU6500_WE::checkInterrupt(uint8_t source, MPU9250_intType type){
    source &= type;
    return source;
}

uint8_t MPU6500_WE::readAndClearInterrupts(){
    uint8_t regVal = readMPU9250Register8(REGISTER_INT_STATUS);
    return regVal;
}

void MPU6500_WE::setWakeOnMotionThreshold(uint8_t womthresh){
    writeMPU9250Register(REGISTER_WOM_THR, womthresh);
}

void MPU6500_WE::enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn){
    uint8_t regVal = 0;
    if(womEn){
        regVal |= 0x80;
    }
    if(womCompEn){
        regVal |= 0x40;
    }
    writeMPU9250Register(REGISTER_MOT_DET_CTRL, regVal);
}

/***************** FIFO ******************/

/* fifo is a byte which defines the data stored in the FIFO
 * It is structured as:
 * Bit 7 = TEMP,              Bit 6 = GYRO_X,  Bit 5 = GYRO_Y   Bit 4 = GYRO_Z,
 * Bit 3 = ACCEL (all axes), Bit 2 = SLAVE_2, Bit 1 = SLAVE_1, Bit 0 = SLAVE_0;
 * e.g. 0b11001001 => TEMP, GYRO_X, ACCEL, SLAVE0 are enabled
 */
void MPU6500_WE::startFifo(MPU9250_fifo_type fifo){
    fifoType = fifo;
    writeMPU9250Register(REGISTER_FIFO_EN, fifoType);
}

void MPU6500_WE::stopFifo(){
    writeMPU9250Register(REGISTER_FIFO_EN, 0);
}

void MPU6500_WE::enableFifo(bool fifo){
    uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
    if(fifo){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(REGISTER_USER_CTRL, regVal);
}

void MPU6500_WE::resetFifo(){
    uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
    regVal |= 0x04;
    writeMPU9250Register(REGISTER_USER_CTRL, regVal);
}

int16_t MPU6500_WE::getFifoCount(){
    uint16_t regVal16 = (uint16_t) readMPU9250Register16(REGISTER_FIFO_COUNT);
    return regVal16;
}

void MPU6500_WE::setFifoMode(MPU9250_fifoMode mode){
    uint8_t regVal = readMPU9250Register8(REGISTER_CONFIG);
    if(mode){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(REGISTER_CONFIG, regVal);

}

int16_t MPU6500_WE::getNumberOfFifoDataSets(){
    int16_t numberOfSets = getFifoCount();

    if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
        numberOfSets /= 6;
    }
    else if(fifoType==MPU9250_FIFO_ACC_GYR){
        numberOfSets /= 12;
    }

    return numberOfSets;
}

void MPU6500_WE::findFifoBegin(){
    int16_t count = getFifoCount();

    if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
        if(count > 510){
            for(int i=0; i<2; i++){
                readMPU9250Register8(REGISTER_FIFO_R_W);
            }
        }
    }
    else if(fifoType==MPU9250_FIFO_ACC_GYR){
        if(count > 504){
            for(int i=0; i<8; i++){
                readMPU9250Register8(REGISTER_FIFO_R_W);
            }
        }
    }
}

/************************************************
     Private Functions
*************************************************/

void MPU6500_WE::correctAccRawValues(xyzFloat & rawValues){
    rawValues.x -= (accOffsetVal.x / accRangeFactor);
    rawValues.y -= (accOffsetVal.y / accRangeFactor);
    rawValues.z -= (accOffsetVal.z / accRangeFactor);
}

void MPU6500_WE::correctGyrRawValues(xyzFloat & rawValues){
    rawValues.x -= (gyrOffsetVal.x / gyrRangeFactor);
    rawValues.y -= (gyrOffsetVal.y / gyrRangeFactor);
    rawValues.z -= (gyrOffsetVal.z / gyrRangeFactor);
}

void MPU6500_WE::reset_MPU9250(){
    writeMPU9250Register(REGISTER_PWR_MGMT_1, REGISTER_VALUE_RESET);
    delay(10);  // wait for registers to reset
}

void MPU6500_WE::enableI2CMaster(){
    uint8_t regVal = readMPU9250Register8(REGISTER_USER_CTRL);
    regVal |= REGISTER_VALUE_I2C_MST_EN;
    writeMPU9250Register(REGISTER_USER_CTRL, regVal); //enable I2C master
    writeMPU9250Register(REGISTER_I2C_MST_CTRL, 0x00); // set I2C clock to 400 kHz
    delay(10);
}

void MPU6500_WE::writeMPU9250Register(uint8_t reg, uint8_t val){
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->write(val);
        _wire->endTransmission();
    }
    else{
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        _spi->transfer(val);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
}

uint8_t MPU6500_WE::readMPU9250Register8(uint8_t reg){
    uint8_t regValue = 0;
    
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress,(uint8_t)1);
        if(_wire->available()){
            regValue = _wire->read();
        }
    }
    else{
        reg |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        regValue = _spi->transfer(0x00);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
    return regValue;
}

int16_t MPU6500_WE::readMPU9250Register16(uint8_t reg){
    uint8_t MSByte = 0, LSByte = 0;
    int16_t regValue = 0;
    
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress,(uint8_t)2);
        if(_wire->available()){
            MSByte = _wire->read();
            LSByte = _wire->read();
        }
    }
    else{
        reg |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg); 
        MSByte = _spi->transfer(0x00);
        LSByte = _spi->transfer(0x00);
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
    regValue = (MSByte<<8) + LSByte;
    return regValue;
}

void MPU6500_WE::readMPU9250Register3x16(uint8_t reg, uint8_t *buf){
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(reg);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress,(uint8_t)6);
        if(_wire->available()){
            for(int i=0; i<6; i++){
                buf[i] = _wire->read();
            }
        }
    }
    else{
        reg |= 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg);
        for(int i=0; i<6; i++){
                buf[i] = _spi->transfer(0x00);
        }
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }
}

xyzFloat MPU6500_WE::readMPU9250xyzValFromFifo(){
    uint8_t fifoTriple[6];
    
    if(!useSPI){
        _wire->beginTransmission(i2cAddress);
        _wire->write(REGISTER_FIFO_R_W);
        _wire->endTransmission(false);
        _wire->requestFrom(i2cAddress,(uint8_t)6);
        if(_wire->available()){
            for(int i=0; i<6; i++){
                fifoTriple[i] = _wire->read();
            }
        }
    }
    else{
        uint8_t reg = REGISTER_FIFO_R_W | 0x80;
        _spi->beginTransaction(mySPISettings);
        digitalWrite(csPin, LOW);
        _spi->transfer(reg);
        for(int i=0; i<6; i++){
                fifoTriple[i] = _spi->transfer(0x00);
        }
        digitalWrite(csPin, HIGH);
        _spi->endTransaction();
    }

    xyzFloat xyzResult = {0.0, 0.0, 0.0};
    xyzResult.x = static_cast<float>((int16_t)((fifoTriple[0]<<8) + fifoTriple[1]));
    xyzResult.y = static_cast<float>((int16_t)((fifoTriple[2]<<8) + fifoTriple[3]));
    xyzResult.z = static_cast<float>((int16_t)((fifoTriple[4]<<8) + fifoTriple[5]));

    return xyzResult;
}

/************ end ************/
