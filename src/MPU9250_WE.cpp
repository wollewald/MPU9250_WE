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

#include "MPU9250_WE.h"

/************  Constructors ************/

MPU9250_WE::MPU9250_WE(int addr){
    i2cAddress = addr;
    
}

MPU9250_WE::MPU9250_WE(){
    i2cAddress = 0x68;
}

/************ Basic Settings ************/
    

bool MPU9250_WE::init(){ 
    if(!reset_MPU9250()){
        return false;
    }
    delay(100);
    writeMPU9250Register(MPU9250_INT_PIN_CFG, MPU9250_BYPASS_EN);  // Bypass Enable
    delay(100);
    
    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    accRangeFactor = 1;
    gyrOffsetVal.x = 0.0;
    gyrOffsetVal.y = 0.0;
    gyrOffsetVal.z = 0.0;
    gyrRangeFactor = 1;
    fifoType = MPU9250_FIFO_ACC;
    
    return true;
}

void MPU9250_WE::autoOffsets(){
    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    
    enableGyrDLPF();
    setGyrDLPF(MPU9250_DLPF_6);  // lowest noise
    setGyrRange(MPU9250_GYRO_RANGE_250); // highest resolution
    setAccRange(MPU9250_ACC_RANGE_2G);
    enableAccDLPF(true);
    setAccDLPF(MPU9250_DLPF_6);
    delay(100);
    
    for(int i=0; i<50; i++){
        getAccRawValues();
        accOffsetVal.x += accRawVal.x;
        accOffsetVal.y += accRawVal.y;
        accOffsetVal.z += accRawVal.z;
        delay(1);
    }
    
    accOffsetVal.x /= 50;
    accOffsetVal.y /= 50;
    accOffsetVal.z /= 50;
    accOffsetVal.z -= 16384.0;
    
    for(int i=0; i<50; i++){
        getGyrRawValues();
        gyrOffsetVal.x += gyrRawVal.x;
        gyrOffsetVal.y += gyrRawVal.y;
        gyrOffsetVal.z += gyrRawVal.z;
        delay(1);
    }
    
    gyrOffsetVal.x /= 50;
    gyrOffsetVal.y /= 50;
    gyrOffsetVal.z /= 50;
    
}

void MPU9250_WE::setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax){
    accOffsetVal.x = (xMax + xMin) * 0.5;
    accOffsetVal.y = (yMax + yMin) * 0.5;
    accOffsetVal.z = (zMax + zMin) * 0.5;
}

void MPU9250_WE::setGyrOffsets(float xOffset, float yOffset, float zOffset){
    gyrOffsetVal.x = xOffset;
    gyrOffsetVal.y = yOffset;
    gyrOffsetVal.z = zOffset;
}

void MPU9250_WE::setGyrDLPF(MPU9250_dlpf dlpf){
    regVal = readMPU9250Register8(MPU9250_CONFIG);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(MPU9250_CONFIG, regVal);
}

void MPU9250_WE::setSampleRateDivider(uint8_t splRateDiv){
    writeMPU9250Register(MPU9250_SMPLRT_DIV, splRateDiv);
}

void MPU9250_WE::setGyrRange(MPU9250_gyroRange gyroRange){
    regVal = readMPU9250Register8(MPU9250_GYRO_CONFIG);
    regVal &= 0xE7;
    regVal |= (gyroRange<<3);
    writeMPU9250Register(MPU9250_GYRO_CONFIG, regVal);
    gyrRangeFactor = (1<<gyroRange);
}

void MPU9250_WE::enableGyrDLPF(){
    regVal = readMPU9250Register8(MPU9250_GYRO_CONFIG);
    regVal &= 0xFC;
    writeMPU9250Register(MPU9250_GYRO_CONFIG, regVal);
}

void MPU9250_WE::disableGyrDLPF(MPU9250_bw_wo_dlpf bw){
    regVal = readMPU9250Register8(MPU9250_GYRO_CONFIG);
    regVal &= 0xFC;
    regVal |= bw;
    writeMPU9250Register(MPU9250_GYRO_CONFIG, regVal);
}

void MPU9250_WE::setAccRange(MPU9250_accRange accRange){
    regVal = readMPU9250Register8(MPU9250_ACCEL_CONFIG);
    regVal &= 0xE7;
    regVal |= (accRange<<3);
    writeMPU9250Register(MPU9250_ACCEL_CONFIG, regVal);
    accRangeFactor = 1<<accRange;
}

void MPU9250_WE::enableAccDLPF(bool enable){
    regVal = readMPU9250Register8(MPU9250_ACCEL_CONFIG_2);
    if(enable){
        regVal &= ~8;
    }
    else{
        regVal |= 8;
    }
    writeMPU9250Register(MPU9250_ACCEL_CONFIG_2, regVal);
}

void MPU9250_WE::setAccDLPF(MPU9250_dlpf dlpf){
    regVal = readMPU9250Register8(MPU9250_ACCEL_CONFIG_2);
    regVal &= 0xF8;
    regVal |= dlpf;
    writeMPU9250Register(MPU9250_ACCEL_CONFIG_2, regVal);
}

void MPU9250_WE::setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr){
    writeMPU9250Register(MPU9250_LP_ACCEL_ODR, lpaodr); 
}

void MPU9250_WE::enableAccAxes(MPU9250_xyzEn enable){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_2);
    regVal &= ~(0x38);
    regVal |= (enable<<3);
    writeMPU9250Register(MPU9250_PWR_MGMT_2, regVal);
}

void MPU9250_WE::enableGyrAxes(MPU9250_xyzEn enable){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_2);
    regVal &= ~(0x07);
    regVal |= enable;
    writeMPU9250Register(MPU9250_PWR_MGMT_2, regVal);
}

/************* x,y,z results *************/
        
xyzFloat MPU9250_WE::getAccRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_ACCEL_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    accRawVal.x = xRaw * 1.0;
    accRawVal.y = yRaw * 1.0;
    accRawVal.z = zRaw * 1.0;
     
    return accRawVal;
}

xyzFloat MPU9250_WE::getCorrectedAccRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_ACCEL_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    accRawVal.x = xRaw * 1.0;
    accRawVal.y = yRaw * 1.0;
    accRawVal.z = zRaw * 1.0;
    
    correctAccRawValues();
    
    return accRawVal;
}

xyzFloat MPU9250_WE::getGValues(){
    xyzFloat gVal;
    getCorrectedAccRawValues();
    
    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

xyzFloat MPU9250_WE::getAccRawValuesFromFifo(){
    xyzFloat accRawVal = readMPU9250xyzValFromFifo();
    return accRawVal;   
}

xyzFloat MPU9250_WE::getCorrectedAccRawValuesFromFifo(){
    accRawVal = getAccRawValuesFromFifo();
    
    correctAccRawValues();
    
    return accRawVal;
}

xyzFloat MPU9250_WE::getGValuesFromFifo(){
    xyzFloat gVal;
    getCorrectedAccRawValuesFromFifo();
    
    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}



float MPU9250_WE::getResultantG(xyzFloat gVal){
    float resultant = 0.0;
    resultant = sqrt(sq(gVal.x) + sq(gVal.y) + sq(gVal.z));
    
    return resultant;
}

float MPU9250_WE::getTemperature(){
    int16_t regVal16 = readMPU9250Register16(MPU9250_TEMP_OUT);
    float tmp = (regVal16*1.0 - MPU9250_ROOM_TEMP_OFFSET)/MPU9250_T_SENSITIVITY + 21.0;
    return tmp;
}

xyzFloat MPU9250_WE::getGyrRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_GYRO_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    gyrRawVal.x = xRaw * 1.0;
    gyrRawVal.y = yRaw * 1.0;
    gyrRawVal.z = zRaw * 1.0;
     
    return gyrRawVal;
}

xyzFloat MPU9250_WE::getCorrectedGyrRawValues(){
    uint64_t xyzDataReg = readMPU9250Register3x16(MPU9250_GYRO_OUT);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    gyrRawVal.x = xRaw * 1.0;
    gyrRawVal.y = yRaw * 1.0;
    gyrRawVal.z = zRaw * 1.0;
     
    correctGyrRawValues();
    
    return gyrRawVal;
}
    
xyzFloat MPU9250_WE::getGyrValues(){
    xyzFloat gyrVal;
    getCorrectedGyrRawValues();
    
    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;
     
    return gyrVal;
}

xyzFloat MPU9250_WE::getGyrValuesFromFifo(){
    xyzFloat gyrVal;
    xyzFloat gyrRawVal = readMPU9250xyzValFromFifo();
    
    correctGyrRawValues();
    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;
    
    return gyrVal;  
}

xyzFloat MPU9250_WE::getMagValues(){
    xyzFloat magVal;
    
    uint64_t xyzDataReg = readAK8963Register3x16(AK8963_HXL);
    int16_t xRaw = (int16_t)((xyzDataReg >> 32) & 0xFFFF);
    int16_t yRaw = (int16_t)((xyzDataReg >> 16) & 0xFFFF);
    int16_t zRaw = (int16_t)(xyzDataReg & 0xFFFF);
    
    magVal.x = xRaw * 4912.0 / 32760.0 * magCorrFactor.x;
    magVal.y = yRaw * 4912.0 / 32760.0 * magCorrFactor.y;
    magVal.z = zRaw * 4912.0 / 32760.0 * magCorrFactor.z;
    
    getStatus2Register(); // completes a read
    
    return magVal;
}

/********* Power, Sleep, Standby *********/ 

void MPU9250_WE::sleep(bool sleep){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_1);
    if(sleep){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU9250_PWR_MGMT_1, regVal);
}

void MPU9250_WE::enableCycle(bool cycle){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_1);
    if(cycle){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(MPU9250_PWR_MGMT_1, regVal);
}

void MPU9250_WE::enableGyrStandby(bool gyroStandby){
    regVal = readMPU9250Register8(MPU9250_PWR_MGMT_1);
    if(gyroStandby){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(MPU9250_PWR_MGMT_1, regVal);
}

        
/******** Angles and Orientation *********/ 
    
xyzFloat MPU9250_WE::getAngles(){
    xyzFloat angleVal;
    xyzFloat gVal = getGValues();
    if(gVal.x > 1){
        gVal.x = 1;
    }
    else if(gVal.x < -1){
        gVal.x = -1;
    }
    angleVal.x = (asin(gVal.x)) * 57.296;
    
    if(gVal.y > 1){
        gVal.y = 1;
    }
    else if(gVal.y < -1){
        gVal.y = -1;
    }
    angleVal.y = (asin(gVal.y)) * 57.296;
    
    if(gVal.z > 1){
        gVal.z = 1;
    }
    else if(gVal.z < -1){
        gVal.z = -1;
    }
    angleVal.z = (asin(gVal.z)) * 57.296;
    
    return angleVal;
}

MPU9250_orientation MPU9250_WE::getOrientation(){
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

String MPU9250_WE::getOrientationAsString(){
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

float MPU9250_WE::getPitch(){
    xyzFloat angleVal = getAngles();
    float pitch = (atan2(angleVal.x, sqrt(angleVal.x*angleVal.y + angleVal.z*angleVal.z))*180.0)/M_PI;
    return pitch;
}
    
float MPU9250_WE::getRoll(){
    xyzFloat angleVal = getAngles();
    float roll = (atan2(angleVal.y, angleVal.z)*180.0)/M_PI;
    return roll;
}
    

/************** Interrupts ***************/

void MPU9250_WE::setIntPinPolarity(MPU9250_intPinPol pol){
    
    if(pol){
        regVal |= 0x80;
    }
    else{
        regVal &= ~(0x80);
    }
    writeMPU9250Register(MPU9250_INT_PIN_CFG, regVal);
}

void MPU9250_WE::enableIntLatch(bool latch){
    regVal = readMPU9250Register8(MPU9250_INT_PIN_CFG);
    if(latch){
        regVal |= 0x20;
    }
    else{
        regVal &= ~(0x20);
    }
    writeMPU9250Register(MPU9250_INT_PIN_CFG, regVal);
}

void MPU9250_WE::enableClearIntByAnyRead(bool clearByAnyRead){
    regVal = readMPU9250Register8(MPU9250_INT_PIN_CFG);
    if(clearByAnyRead){
        regVal |= 0x10;
    }
    else{
        regVal &= ~(0x10);
    }
    writeMPU9250Register(MPU9250_INT_PIN_CFG, regVal);
}

void MPU9250_WE::enableInterrupt(MPU9250_intType intType){
    regVal = readMPU9250Register8(MPU9250_INT_ENABLE);
    regVal |= intType;
    writeMPU9250Register(MPU9250_INT_ENABLE, regVal);   
}

void MPU9250_WE::disableInterrupt(MPU9250_intType intType){
    regVal = readMPU9250Register8(MPU9250_INT_ENABLE);
    regVal &= ~intType;
    writeMPU9250Register(MPU9250_INT_ENABLE, regVal);
}

bool MPU9250_WE::checkInterrupt(uint8_t source, MPU9250_intType type){
    source &= type;
    return source;
}

uint8_t MPU9250_WE::readAndClearInterrupts(){
    regVal = readMPU9250Register8(MPU9250_INT_STATUS);
    return regVal;
}

void MPU9250_WE::setWakeOnMotionThreshold(uint8_t womthresh){
    writeMPU9250Register(MPU9250_WOM_THR, womthresh);
}

void MPU9250_WE::enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn){
    regVal = 0;
    if(womEn){
        regVal |= 0x80;
    }
    if(womCompEn){
        regVal |= 0x40;
    }
    writeMPU9250Register(MPU9250_MOT_DET_CTRL, regVal);
}

/***************** FIFO ******************/

/* fifo is a byte which defines the data stored in the FIFO
 * It is structured as: 
 * Bit 7 = TEMP,              Bit 6 = GYRO_X,  Bit 5 = GYRO_Y   Bit 4 = GYRO_Z,
 * Bit 3 = ACCEL (all axes), Bit 2 = SLAVE_2, Bit 1 = SLAVE_1, Bit 0 = SLAVE_0;
 * e.g. 0b11001001 => TEMP, GYRO_X, ACCEL, SLAVE0 are enabled
 */
void MPU9250_WE::startFifo(MPU9250_fifo_type fifo){
    fifoType = fifo;
    writeMPU9250Register(MPU9250_FIFO_EN, fifoType);
}

void MPU9250_WE::stopFifo(){
    writeMPU9250Register(MPU9250_FIFO_EN, 0);
}

void MPU9250_WE::enableFifo(bool fifo){
    regVal = readMPU9250Register8(MPU9250_USER_CTRL);
    if(fifo){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU9250_USER_CTRL, regVal);
}

void MPU9250_WE::resetFifo(){
    regVal = readMPU9250Register8(MPU9250_USER_CTRL);
    regVal |= 0x04;
    writeMPU9250Register(MPU9250_USER_CTRL, regVal);
}

int16_t MPU9250_WE::getFifoCount(){
    uint16_t regVal16 = (uint16_t) readMPU9250Register16(MPU9250_FIFO_COUNT);
    return regVal16;
}

void MPU9250_WE::setFifoMode(MPU9250_fifoMode mode){
    regVal = readMPU9250Register8(MPU9250_CONFIG);
    if(mode){
        regVal |= 0x40;
    }
    else{
        regVal &= ~(0x40);
    }
    writeMPU9250Register(MPU9250_CONFIG, regVal);
    
}

int16_t MPU9250_WE::getNumberOfFifoDataSets(){
    int16_t numberOfSets = getFifoCount();
        
    if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
        numberOfSets /= 6;
    }
    else if(fifoType==MPU9250_FIFO_ACC_GYR){
        numberOfSets /= 12;
    }
    
    return numberOfSets;
}

void MPU9250_WE::findFifoBegin(){
    int16_t count = getFifoCount();
        
    if((fifoType == MPU9250_FIFO_ACC) || (fifoType == MPU9250_FIFO_GYR)){
        if(count > 510){
            for(int i=0; i<2; i++){
                readMPU9250Register8(MPU9250_FIFO_R_W);
            }
        }
    }
    else if(fifoType==MPU9250_FIFO_ACC_GYR){
        if(count > 504){
            for(int i=0; i<8; i++){
                readMPU9250Register8(MPU9250_FIFO_R_W);
            }
        }
    }
}

/************** Magnetometer **************/

bool MPU9250_WE::initMagnetometer(){
    
    if(!readAK8963Register8(AK8963_WIA)){
        return false;
    }
    resetMagnetometer();
    setMagOpMode(AK8963_FUSE_ROM_ACC_MODE);
    getAsaVals();
    setMagOpMode(AK8963_PWR_DOWN);
    setMagnetometer16Bit();
    setMagOpMode(AK8963_CONT_MODE_8HZ); 
    
    return true;
}

void MPU9250_WE::setMagOpMode(AK8963_opMode opMode){
    regVal = readAK8963Register8(AK8963_CNTL_1);
    regVal &= 0xF0;
    regVal |= opMode;
    writeAK8963Register(AK8963_CNTL_1, regVal);
}

bool MPU9250_WE::isMagOverflow(){
    regVal = readAK8963Register8(AK8963_STATUS_2);
    if(regVal & AK8963_OVF){
        return true;
    }
    else{
        return false;
    }
}

void MPU9250_WE::startMagMeasurement(){
    setMagOpMode(AK8963_TRIGGER_MODE);
    while(!isMagDataReady()){}
}

bool MPU9250_WE::isMagDataReady(){
    regVal = readAK8963Register8(AK8963_STATUS_1);
    return (regVal & 1);
}

/************************************************ 
     Private Functions
*************************************************/

void MPU9250_WE::correctAccRawValues(){
    accRawVal.x -= (accOffsetVal.x / accRangeFactor);
    accRawVal.y -= (accOffsetVal.y / accRangeFactor);
    accRawVal.z -= (accOffsetVal.z / accRangeFactor);
}

void MPU9250_WE::correctGyrRawValues(){
    gyrRawVal.x -= (gyrOffsetVal.x / gyrRangeFactor);
    gyrRawVal.y -= (gyrOffsetVal.y / gyrRangeFactor);
    gyrRawVal.z -= (gyrOffsetVal.z / gyrRangeFactor);
}

void MPU9250_WE::resetMagnetometer(){
    writeAK8963Register(AK8963_CNTL_2, 1);
}

uint8_t MPU9250_WE::reset_MPU9250(){
    uint8_t ack = writeMPU9250Register(MPU9250_PWR_MGMT_1, MPU9250_RESET);
    delay(100);  // wait for registers to reset
    return (ack == 0);
}

void MPU9250_WE::getAsaVals(){
    byte rawCorr = 0;
    rawCorr = readAK8963Register8(AK8963_ASAX);
    magCorrFactor.x = (0.5 * (rawCorr-128)/128.0) + 1.0;
    rawCorr = readAK8963Register8(AK8963_ASAY);
    magCorrFactor.y = (0.5 * (rawCorr-128)/128.0) + 1.0;
    rawCorr = readAK8963Register8(AK8963_ASAZ);
    magCorrFactor.z = (0.5 * (rawCorr-128)/128.0) + 1.0;
}

uint8_t MPU9250_WE::writeMPU9250Register(uint8_t reg, uint8_t val){
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.write(val);
    
    return Wire.endTransmission();
}
  
uint8_t MPU9250_WE::writeMPU9250Register16(uint8_t reg, int16_t val){
    int8_t MSByte = (val>>7) & 0xFF;
    uint8_t LSByte = (val<<1) & 0xFE;
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.write(MSByte);
    Wire.write(LSByte);
    
    return Wire.endTransmission();  
}

uint8_t MPU9250_WE::writeAK8963Register(uint8_t reg, uint8_t val){
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(reg);
    Wire.write(val);
     
    return Wire.endTransmission();
}
  
uint8_t MPU9250_WE::readMPU9250Register8(uint8_t reg){
    uint8_t regValue = 0;
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress,1);
    if(Wire.available()){
        regValue = Wire.read();
    }
    return regValue;
}

uint8_t MPU9250_WE::readAK8963Register8(uint8_t reg){
    uint8_t regValue = 0;
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(AK8963_ADDRESS,1);
    if(Wire.available()){
        regValue = Wire.read();
    }
    return regValue;
}

uint64_t MPU9250_WE::readAK8963Register3x16(uint8_t reg){    
    uint8_t byte0 = 0, byte1 = 0, byte2 = 0, byte3 = 0, byte4 = 0, byte5 = 0;
    uint64_t regValue = 0;
    Wire.beginTransmission(AK8963_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(AK8963_ADDRESS,6);
    if(Wire.available()){
        byte0 = Wire.read();
        byte1 = Wire.read();
        byte2 = Wire.read();
        byte3 = Wire.read();
        byte4 = Wire.read();
        byte5 = Wire.read();
    }
    regValue = ((uint64_t) byte1<<40) + ((uint64_t) byte0<<32) +((uint64_t) byte3<<24) + 
           + ((uint64_t) byte2<<16) + ((uint64_t) byte5<<8) +  (uint64_t)byte4;
    return regValue;
}

int16_t MPU9250_WE::readMPU9250Register16(uint8_t reg){
    uint8_t MSByte = 0, LSByte = 0;
    int16_t regValue = 0;
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress,2);
    if(Wire.available()){
        MSByte = Wire.read();
        LSByte = Wire.read();
    }
    regValue = (MSByte<<8) + LSByte;
    return regValue;
}

uint64_t MPU9250_WE::readMPU9250Register3x16(uint8_t reg){    
    uint8_t byte0 = 0, byte1 = 0, byte2 = 0, byte3 = 0, byte4 = 0, byte5 = 0;
    uint64_t regValue = 0;
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(i2cAddress,6);
    if(Wire.available()){
        byte0 = Wire.read();
        byte1 = Wire.read();
        byte2 = Wire.read();
        byte3 = Wire.read();
        byte4 = Wire.read();
        byte5 = Wire.read();
    }
    regValue = ((uint64_t) byte0<<40) + ((uint64_t) byte1<<32) +((uint64_t) byte2<<24) + 
           + ((uint64_t) byte3<<16) + ((uint64_t) byte4<<8) +  (uint64_t)byte5;
    return regValue;
}

xyzFloat MPU9250_WE::readMPU9250xyzValFromFifo(){
    uint8_t MSByte = 0, LSByte = 0;
    xyzFloat xyzResult = {0.0, 0.0, 0.0};
    MSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    LSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    xyzResult.x = (MSByte<<8) + LSByte;
    MSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    LSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    xyzResult.y = (MSByte<<8) + LSByte;
    MSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    LSByte = readMPU9250Register8(MPU9250_FIFO_R_W);
    xyzResult.z = (MSByte<<8) + LSByte;
    return xyzResult; 
}

void MPU9250_WE::setMagnetometer16Bit(){
    regVal = readAK8963Register8(AK8963_CNTL_1);
    regVal |= AK8963_16_BIT;
    writeAK8963Register(AK8963_CNTL_1, regVal);
}

uint8_t MPU9250_WE::getStatus2Register(){
    regVal = readAK8963Register8(AK8963_STATUS_2);
    return regVal;
}


