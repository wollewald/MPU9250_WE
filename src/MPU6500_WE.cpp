/********************************************************************
* This is a library for the 6-axis gyroscope and accelerometer MPU6500.
*********************************************************************/

#include "MPU6500_WE.h"
/********************************************************************
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

/************  Constructors ************/

MPU6500_WE::MPU6500_WE(int addr)
    : MPU6500_WE(&Wire, addr)
{
    // intentionally empty
}

MPU6500_WE::MPU6500_WE()
    : MPU6500_WE(&Wire, 0x68)
{
    // intentionally empty
}

MPU6500_WE::MPU6500_WE(TwoWire *w, int addr)
    : _wire(w)
    , i2cAddress(addr)
{
    // intentionally empty
}

MPU6500_WE::MPU6500_WE(TwoWire *w)
    : MPU6500_WE(w, 0x68)
{
    // intentionally empty
}


/************ Basic Settings ************/

bool MPU6500_WE::init(uint8_t const expectedValue){
    reset_MPU9250();
    delay(10);
    writeMPU9250Register(MPU6500_INT_PIN_CFG, MPU6500_BYPASS_EN);  // Bypass Enable
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
    return init(MPU6500_WHO_AM_I_CODE);
}

uint8_t MPU6500_WE::whoAmI(){
    return readMPU9250Register8(MPU6500_WHO_AM_I);
}

void MPU6500_WE::autoOffsets(){
    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;

    gyrOffsetVal.x = 0.0;
    gyrOffsetVal.y = 0.0;
    gyrOffsetVal.z = 0.0;

    enableGyrDLPF();
    setGyrDLPF(MPU9250_DLPF_6);  // lowest noise
    setGyrRange(MPU9250_GYRO_RANGE_250); // highest resolution
    setAccRange(MPU9250_ACC_RANGE_2G);
    enableAccDLPF(true);
    setAccDLPF(MPU9250_DLPF_6);
    delay(100);

    for(int i=0; i<50; i++){
        // acceleration
        getAccRawValues();
        accOffsetVal.x += accRawVal.x;
        accOffsetVal.y += accRawVal.y;
        accOffsetVal.z += accRawVal.z;
        // gyro
        getGyrRawValues();
        gyrOffsetVal.x += gyrRawVal.x;
        gyrOffsetVal.y += gyrRawVal.y;
        gyrOffsetVal.z += gyrRawVal.z;
        delay(1);
    }

    // acceleration
    accOffsetVal.x /= 50;
    accOffsetVal.y /= 50;
    accOffsetVal.z /= 50;
    accOffsetVal.z -= 16384.0;
    // gyro
    gyrOffsetVal.x /= 50;
    gyrOffsetVal.y /= 50;
    gyrOffsetVal.z /= 50;

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
    regVal = readMPU9250Register8(MPU6500_CONFIG);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(MPU6500_CONFIG, regVal);
}

void MPU6500_WE::setSampleRateDivider(uint8_t splRateDiv){
    writeMPU9250Register(MPU6500_SMPLRT_DIV, splRateDiv);
}

void MPU6500_WE::setGyrRange(MPU9250_gyroRange gyroRange){
    regVal = readMPU9250Register8(MPU6500_GYRO_CONFIG);
    regVal &= 0xE7;
    regVal |= (gyroRange<<3);
    writeMPU9250Register(MPU6500_GYRO_CONFIG, regVal);
    gyrRangeFactor = (1<<gyroRange);
}

void MPU6500_WE::enableGyrDLPF(){
    regVal = readMPU9250Register8(MPU6500_GYRO_CONFIG);
    regVal &= 0xFC;
    writeMPU9250Register(MPU6500_GYRO_CONFIG, regVal);
}

void MPU6500_WE::disableGyrDLPF(MPU9250_bw_wo_dlpf bw){
    regVal = readMPU9250Register8(MPU6500_GYRO_CONFIG);
    regVal &= 0xFC;
    regVal |= bw;
    writeMPU9250Register(MPU6500_GYRO_CONFIG, regVal);
}

void MPU6500_WE::setAccRange(MPU9250_accRange accRange){
    regVal = readMPU9250Register8(MPU6500_ACCEL_CONFIG);
    regVal &= 0xE7;
    regVal |= (accRange<<3);
    writeMPU9250Register(MPU6500_ACCEL_CONFIG, regVal);
    accRangeFactor = 1<<accRange;
}

void MPU6500_WE::enableAccDLPF(bool enable){
    regVal = readMPU9250Register8(MPU6500_ACCEL_CONFIG_2);
    if(enable){
        regVal &= ~8;
    }
    else{
        regVal |= 8;
    }
    writeMPU9250Register(MPU6500_ACCEL_CONFIG_2, regVal);
}

void MPU6500_WE::setAccDLPF(MPU9250_dlpf dlpf){
    regVal = readMPU9250Register8(MPU6500_ACCEL_CONFIG_2);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(MPU6500_ACCEL_CONFIG_2, regVal);
}

void MPU6500_WE::setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr){
    writeMPU9250Register(MPU6500_LP_ACCEL_ODR, lpaodr);
}

void MPU6500_WE::enableAccAxes(MPU9250_xyzEn enable){
    regVal = readMPU9250Register8(MPU6500_PWR_MGMT_2);
    regVal &= ~(0x38);
    regVal |= (enable<<3);
    writeMPU9250Register(MPU6500_PWR_MGMT_2, regVal);
}

void MPU6500_WE::enableGyrAxes(MPU9250_xyzEn enable){
    regVal = readMPU9250Register8(MPU6500_PWR_MGMT_2);
    regVal &= ~(0x07);
    regVal |= enable;
    writeMPU9250Register(MPU6500_PWR_MGMT_2, regVal);
}

/************* x,y,z results *************/

xyzFloat MPU6500_WE::getAccRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU6500_ACCEL_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);

    accRawVal.x = xRaw * 1.0;
    accRawVal.y = yRaw * 1.0;
    accRawVal.z = zRaw * 1.0;

    return accRawVal;
}

xyzFloat MPU6500_WE::getCorrectedAccRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU6500_ACCEL_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);

    accRawVal.x = xRaw * 1.0;
    accRawVal.y = yRaw * 1.0;
    accRawVal.z = zRaw * 1.0;

    correctAccRawValues();

    return accRawVal;
}

xyzFloat MPU6500_WE::getGValues(){
    xyzFloat gVal;
    getCorrectedAccRawValues();

    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

xyzFloat MPU6500_WE::getAccRawValuesFromFifo(){
    xyzFloat accRawVal = readMPU9250xyzValFromFifo();
    return accRawVal;
}

xyzFloat MPU6500_WE::getCorrectedAccRawValuesFromFifo(){
    accRawVal = getAccRawValuesFromFifo();

    correctAccRawValues();

    return accRawVal;
}

xyzFloat MPU6500_WE::getGValuesFromFifo(){
    xyzFloat gVal;
    getCorrectedAccRawValuesFromFifo();

    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}



float MPU6500_WE::getResultantG(xyzFloat gVal){
    float resultant = 0.0;
    resultant = sqrt(sq(gVal.x) + sq(gVal.y) + sq(gVal.z));

    return resultant;
}

float MPU6500_WE::getTemperature(){
    int16_t regVal16 = readMPU9250Register16(MPU6500_TEMP_OUT);
    float tmp = (regVal16*1.0 - MPU6500_ROOM_TEMP_OFFSET)/MPU6500_T_SENSITIVITY + 21.0;
    return tmp;
}

xyzFloat MPU6500_WE::getGyrRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU6500_GYRO_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);

    gyrRawVal.x = xRaw * 1.0;
    gyrRawVal.y = yRaw * 1.0;
    gyrRawVal.z = zRaw * 1.0;

    return gyrRawVal;
}

xyzFloat MPU6500_WE::getCorrectedGyrRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU6500_GYRO_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);

    gyrRawVal.x = xRaw * 1.0;
    gyrRawVal.y = yRaw * 1.0;
    gyrRawVal.z = zRaw * 1.0;

    correctGyrRawValues();

    return gyrRawVal;
}

xyzFloat MPU6500_WE::getGyrValues(){
    xyzFloat gyrVal;
    getCorrectedGyrRawValues();

    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;

    return gyrVal;
}

xyzFloat MPU6500_WE::getGyrValuesFromFifo(){
    xyzFloat gyrVal;
    gyrRawVal = readMPU9250xyzValFromFifo();

    correctGyrRawValues();
    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;

    return gyrVal;
}

/********* Power, Sleep, Standby *********/

void MPU6500_WE::sleep(bool sleep){
    regVal = readMPU9250Register8(MPU6500_PWR_MGMT_1);
    if(sleep){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU6500_PWR_MGMT_1, regVal);
}

void MPU6500_WE::enableCycle(bool cycle){
    regVal = readMPU9250Register8(MPU6500_PWR_MGMT_1);
    if(cycle){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(MPU6500_PWR_MGMT_1, regVal);
}

void MPU6500_WE::enableGyrStandby(bool gyroStandby){
    regVal = readMPU9250Register8(MPU6500_PWR_MGMT_1);
    if(gyroStandby){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(MPU6500_PWR_MGMT_1, regVal);
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
    float pitch = (atan2(angleVal.x, sqrt(abs((angleVal.x*angleVal.y + angleVal.z*angleVal.z))))*180.0)/M_PI;
    return pitch;
}

float MPU6500_WE::getRoll(){
    xyzFloat angleVal = getAngles();
    float roll = (atan2(angleVal.y, angleVal.z)*180.0)/M_PI;
    return roll;
}


/************** Interrupts ***************/

void MPU6500_WE::setIntPinPolarity(MPU9250_intPinPol pol){
    regVal = readMPU9250Register8(MPU6500_INT_PIN_CFG);
    if(pol){
        regVal |= 0x80;
    }
    else{
        regVal &= ~(0x80);
    }
    writeMPU9250Register(MPU6500_INT_PIN_CFG, regVal);
}

void MPU6500_WE::enableIntLatch(bool latch){
    regVal = readMPU9250Register8(MPU6500_INT_PIN_CFG);
    if(latch){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(MPU6500_INT_PIN_CFG, regVal);
}

void MPU6500_WE::enableClearIntByAnyRead(bool clearByAnyRead){
    regVal = readMPU9250Register8(MPU6500_INT_PIN_CFG);
    if(clearByAnyRead){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(MPU6500_INT_PIN_CFG, regVal);
}

void MPU6500_WE::enableInterrupt(MPU9250_intType intType){
    regVal = readMPU9250Register8(MPU6500_INT_ENABLE);
    regVal |= intType;
    writeMPU9250Register(MPU6500_INT_ENABLE, regVal);
}

void MPU6500_WE::disableInterrupt(MPU9250_intType intType){
    regVal = readMPU9250Register8(MPU6500_INT_ENABLE);
    regVal &= ~intType;
    writeMPU9250Register(MPU6500_INT_ENABLE, regVal);
}

bool MPU6500_WE::checkInterrupt(uint8_t source, MPU9250_intType type){
    source &= type;
    return source;
}

uint8_t MPU6500_WE::readAndClearInterrupts(){
    regVal = readMPU9250Register8(MPU6500_INT_STATUS);
    return regVal;
}

void MPU6500_WE::setWakeOnMotionThreshold(uint8_t womthresh){
    writeMPU9250Register(MPU6500_WOM_THR, womthresh);
}

void MPU6500_WE::enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn){
    regVal = 0;
    if(womEn){
        regVal |= 0x80;
    }
    if(womCompEn){
        regVal |= 0x40;
    }
    writeMPU9250Register(MPU6500_MOT_DET_CTRL, regVal);
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
    writeMPU9250Register(MPU6500_FIFO_EN, fifoType);
}

void MPU6500_WE::stopFifo(){
    writeMPU9250Register(MPU6500_FIFO_EN, 0);
}

void MPU6500_WE::enableFifo(bool fifo){
    regVal = readMPU9250Register8(MPU6500_USER_CTRL);
    if(fifo){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU6500_USER_CTRL, regVal);
}

void MPU6500_WE::resetFifo(){
    regVal = readMPU9250Register8(MPU6500_USER_CTRL);
    regVal |= 0x04;
    writeMPU9250Register(MPU6500_USER_CTRL, regVal);
}

int16_t MPU6500_WE::getFifoCount(){
    uint16_t regVal16 = (uint16_t) readMPU9250Register16(MPU6500_FIFO_COUNT);
    return regVal16;
}

void MPU6500_WE::setFifoMode(MPU9250_fifoMode mode){
    regVal = readMPU9250Register8(MPU6500_CONFIG);
    if(mode){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU6500_CONFIG, regVal);

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
                readMPU9250Register8(MPU6500_FIFO_R_W);
            }
        }
    }
    else if(fifoType==MPU9250_FIFO_ACC_GYR){
        if(count > 504){
            for(int i=0; i<8; i++){
                readMPU9250Register8(MPU6500_FIFO_R_W);
            }
        }
    }
}

/************************************************
     Private Functions
*************************************************/

void MPU6500_WE::correctAccRawValues(){
    accRawVal.x -= (accOffsetVal.x / accRangeFactor);
    accRawVal.y -= (accOffsetVal.y / accRangeFactor);
    accRawVal.z -= (accOffsetVal.z / accRangeFactor);
}

void MPU6500_WE::correctGyrRawValues(){
    gyrRawVal.x -= (gyrOffsetVal.x / gyrRangeFactor);
    gyrRawVal.y -= (gyrOffsetVal.y / gyrRangeFactor);
    gyrRawVal.z -= (gyrOffsetVal.z / gyrRangeFactor);
}

void MPU6500_WE::reset_MPU9250(){
    writeMPU9250Register(MPU6500_PWR_MGMT_1, MPU6500_RESET);
    delay(10);  // wait for registers to reset
}

void MPU6500_WE::enableI2CMaster(){
    regVal = readMPU9250Register8(MPU6500_USER_CTRL);
    regVal |= MPU6500_I2C_MST_EN;
    writeMPU9250Register(MPU6500_USER_CTRL, regVal); //enable I2C master
    writeMPU9250Register(MPU6500_I2C_MST_CTRL, 0x00); // set I2C clock to 400 kHz
    delay(10);
}

void MPU6500_WE::writeMPU9250Register(uint8_t reg, uint8_t val){
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

uint8_t MPU6500_WE::readMPU9250Register8(uint8_t reg){
    uint8_t regValue = 0;
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,1);
    if(_wire->available()){
        regValue = _wire->read();
    }
    return regValue;
}

int16_t MPU6500_WE::readMPU9250Register16(uint8_t reg){
    uint8_t MSByte = 0, LSByte = 0;
    int16_t regValue = 0;
    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,2);
    if(_wire->available()){
        MSByte = _wire->read();
        LSByte = _wire->read();
    }
    regValue = (MSByte<<8) + LSByte;
    return regValue;
}

uint64_t MPU6500_WE::readMPU9250Register3x16(uint8_t reg){
    uint8_t mpu9250Triple[6];
    uint64_t regValue = 0;

    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,6);
    if(_wire->available()){
        for(int i=0; i<6; i++){
            mpu9250Triple[i] = _wire->read();
        }
    }

    regValue = ((uint64_t) mpu9250Triple[0]<<40) + ((uint64_t) mpu9250Triple[1]<<32) +((uint64_t) mpu9250Triple[2]<<24) +
           + ((uint64_t) mpu9250Triple[3]<<16) + ((uint64_t) mpu9250Triple[4]<<8) +  (uint64_t) mpu9250Triple[5];
    return regValue;
}

xyzFloat MPU6500_WE::readMPU9250xyzValFromFifo(){
    uint8_t fifoTriple[6];
    xyzFloat xyzResult = {0.0, 0.0, 0.0};

    _wire->beginTransmission(i2cAddress);
    _wire->write(MPU6500_FIFO_R_W);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress,6);
    if(_wire->available()){
        for(int i=0; i<6; i++){
            fifoTriple[i] = _wire->read();
        }
    }

    xyzResult.x = ((int16_t)((fifoTriple[0]<<8) + fifoTriple[1])) * 1.0;
    xyzResult.y = ((int16_t)((fifoTriple[2]<<8) + fifoTriple[3])) * 1.0;
    xyzResult.z = ((int16_t)((fifoTriple[4]<<8) + fifoTriple[5])) * 1.0;

    return xyzResult;
}

/************ end ************/
