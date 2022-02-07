/******************************************************************************
 *
 * This is a library for the 9-axis gyroscope, accelerometer and magnetometer MPU9250.
 *
 * You'll find several example sketches which should enable you to use the library. 
 *
 * You are free to use it, change it or build on it. In case you like it, it would
 * be cool if you give it a star.
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
 * 
 ******************************************************************************/

#ifndef MPU9250_WE_H_
#define MPU9250_WE_H_

#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include <Wire.h>

#define AK8963_ADDRESS 0x0C

/* Registers MPU9250*/
#define MPU9250_SELF_TEST_X_GYRO    0x00
#define MPU9250_SELF_TEST_Y_GYRO    0x01
#define MPU9250_SELF_TEST_Z_GYRO    0x02
#define MPU9250_SELF_TEST_X_ACCEL   0x0D
#define MPU9250_SELF_TEST_Y_ACCEL   0x0E
#define MPU9250_SELF_TEST_Z_ACCEL   0x0F
#define MPU9250_XG_OFFSET_H         0x13
#define MPU9250_XG_OFFSET_L         0x14
#define MPU9250_YG_OFFSET_H         0x15
#define MPU9250_YG_OFFSET_L         0x16
#define MPU9250_ZG_OFFSET_H         0x17
#define MPU9250_ZG_OFFSET_L         0x18
#define MPU9250_SMPLRT_DIV          0x19
#define MPU9250_CONFIG              0x1A
#define MPU9250_GYRO_CONFIG         0x1B
#define MPU9250_ACCEL_CONFIG        0x1C
#define MPU9250_ACCEL_CONFIG_2      0x1D
#define MPU9250_LP_ACCEL_ODR        0x1E
#define MPU9250_WOM_THR             0x1F
#define MPU9250_FIFO_EN             0x23
#define MPU9250_I2C_MST_CTRL        0x24
#define MPU9250_I2C_SLV0_ADDR       0x25
#define MPU9250_I2C_SLV0_REG        0x26
#define MPU9250_I2C_SLV0_CTRL       0x27
#define MPU9250_I2C_MST_STATUS      0x36
#define MPU9250_INT_PIN_CFG         0x37
#define MPU9250_INT_ENABLE          0x38
#define MPU9250_INT_STATUS          0x3A
#define MPU9250_ACCEL_OUT           0x3B // accel data registers begin
#define MPU9250_TEMP_OUT            0x41
#define MPU9250_GYRO_OUT            0x43 // gyro data registers begin
#define MPU9250_EXT_SLV_SENS_DATA_00    0x49
#define MPU9250_I2C_SLV0_DO         0x63
#define MPU9250_I2C_MST_DELAY_CTRL  0x67
#define MPU9250_SIGNAL_PATH_RESET   0x68
#define MPU9250_MOT_DET_CTRL        0x69
#define MPU9250_USER_CTRL           0x6A
#define MPU9250_PWR_MGMT_1          0x6B
#define MPU9250_PWR_MGMT_2          0x6C
#define MPU9250_FIFO_COUNT          0x72 // 0x72 is COUNT_H
#define MPU9250_FIFO_R_W            0x74
#define MPU9250_WHO_AM_I            0x75
#define MPU9250_XA_OFFSET_H         0x77
#define MPU9250_XA_OFFSET_L         0x78
#define MPU9250_YA_OFFSET_H         0x7A
#define MPU9250_YA_OFFSET_L         0x7B 
#define MPU9250_ZA_OFFSET_H         0x7D
#define MPU9250_ZA_OFFSET_L         0x7E

/* Registers AK8963 */
#define AK8963_WIA      0x00 // Who am I
#define AK8963_INFO     0x01
#define AK8963_STATUS_1 0x02 
#define AK8963_HXL      0x03
#define AK8963_HYL      0x05
#define AK8963_HZL      0x07
#define AK8963_STATUS_2 0x09
#define AK8963_CNTL_1   0x0A
#define AK8963_CNTL_2   0x0B
#define AK8963_ASTC     0x0C // Self Test
#define AK8963_I2CDIS   0x0F
#define AK8963_ASAX     0x10
#define AK8963_ASAY     0x11
#define AK8963_ASAZ     0x12

/* Register Values */
#define MPU9250_RESET       0x80
#define MPU9250_BYPASS_EN   0x02
#define MPU9250_I2C_MST_EN  0x20
#define MPU9250_CLK_SEL_PLL 0x01
#define AK8963_16_BIT       0x10
#define AK8963_OVF          0x08
#define AK8963_READ         0x80

/* Others */
#define MPU9250_ROOM_TEMP_OFFSET    0.0f
#define MPU9250_T_SENSITIVITY       333.87f
#define MPU9250_WHO_AM_I_CODE       0x71
#define AK8963_WHO_AM_I_CODE        0x48


/* Enums */


typedef enum MPU9250_BW_WO_DLPF {
    MPU9250_BW_WO_DLPF_3600 = 0x02, 
    MPU9250_BW_WO_DLPF_8800 = 0x01
} MPU9250_bw_wo_dlpf;

typedef enum MPU9250_DLPF {
    MPU9250_DLPF_0, MPU9250_DLPF_1, MPU9250_DLPF_2, MPU9250_DLPF_3, MPU9250_DLPF_4, MPU9250_DLPF_5, 
    MPU9250_DLPF_6, MPU9250_DLPF_7
} MPU9250_dlpf;

typedef enum MPU9250_GYRO_RANGE {
    MPU9250_GYRO_RANGE_250, MPU9250_GYRO_RANGE_500, MPU9250_GYRO_RANGE_1000, MPU9250_GYRO_RANGE_2000
} MPU9250_gyroRange;

typedef enum MPU9250_ACC_RANGE {
    MPU9250_ACC_RANGE_2G, MPU9250_ACC_RANGE_4G, MPU9250_ACC_RANGE_8G, MPU9250_ACC_RANGE_16G
} MPU9250_accRange;

typedef enum MPU9250_LOW_PWR_ACC_ODR {
    MPU9250_LP_ACC_ODR_0_24, MPU9250_LP_ACC_ODR_0_49, MPU9250_LP_ACC_ODR_0_98, MPU9250_LP_ACC_ODR_1_95,
    MPU9250_LP_ACC_ODR_3_91, MPU9250_LP_ACC_ODR_7_81, MPU9250_LP_ACC_ODR_15_63, MPU9250_LP_ACC_ODR_31_25,
    MPU9250_LP_ACC_ODR_62_5, MPU9250_LP_ACC_ODR_125, MPU9250_LP_ACC_ODR_250, MPU9250_LP_ACC_ODR_500
} MPU9250_lpAccODR;

typedef enum MPU9250_INT_PIN_POL {
    MPU9250_ACT_HIGH, MPU9250_ACT_LOW
} MPU9250_intPinPol;

typedef enum MPU9250_INT_TYPE {
    MPU9250_DATA_READY = 0x01,
    MPU9250_FIFO_OVF   = 0x10,
    MPU9250_WOM_INT    = 0x40
} MPU9250_intType;

typedef enum MPU9250_WOM_EN {
    MPU9250_WOM_DISABLE, MPU9250_WOM_ENABLE
} MPU9250_womEn;

typedef enum MPU9250_WOM_COMP {
    MPU9250_WOM_COMP_DISABLE, MPU9250_WOM_COMP_ENABLE
} MPU9250_womCompEn;

typedef enum MPU9250_XYZ_ENABLE {
    MPU9250_ENABLE_XYZ,  //all axes are enabled (default)
    MPU9250_ENABLE_XY0,  // x, y enabled, z disabled
    MPU9250_ENABLE_X0Z,   
    MPU9250_ENABLE_X00,
    MPU9250_ENABLE_0YZ,
    MPU9250_ENABLE_0Y0,
    MPU9250_ENABLE_00Z,
    MPU9250_ENABLE_000,  // all axes disabled
} MPU9250_xyzEn;

typedef enum MPU9250_ORIENTATION {
  MPU9250_FLAT, MPU9250_FLAT_1, MPU9250_XY, MPU9250_XY_1, MPU9250_YX, MPU9250_YX_1
} MPU9250_orientation;

typedef enum MPU9250_FIFO_MODE {
    MPU9250_CONTINUOUS, MPU9250_STOP_WHEN_FULL
} MPU9250_fifoMode;

typedef enum MPU9250_FIFO_TYPE {
    MPU9250_FIFO_ACC        = 0x08,
    MPU9250_FIFO_GYR        = 0x70,
    MPU9250_FIFO_ACC_GYR    = 0x78
} MPU9250_fifo_type;

typedef enum AK8963_OP_MODE {
    AK8963_PWR_DOWN           = 0x00,
    AK8963_TRIGGER_MODE       = 0x01,
    AK8963_CONT_MODE_8HZ      = 0x02,
    AK8963_CONT_MODE_100HZ    = 0x06,
    AK8963_FUSE_ROM_ACC_MODE  = 0x0F
} AK8963_opMode;

struct xyzFloat {
    float x;
    float y;
    float z;
};


class MPU9250_WE
{
public: 
    /* Constructors */
    
    MPU9250_WE(int addr);
    MPU9250_WE();
    MPU9250_WE(TwoWire *w, int addr);
    MPU9250_WE(TwoWire *w);
   
    /* Basic settings */
    
    bool init();
    uint8_t whoAmI();
    void autoOffsets();
    void setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax);
    void setGyrOffsets(float xOffset, float yOffset, float zOffset);
    void setGyrDLPF(MPU9250_dlpf dlpf);
    void setSampleRateDivider(uint8_t splRateDiv);
    void setGyrRange(MPU9250_gyroRange gyroRange);
    void enableGyrDLPF();
    void disableGyrDLPF(MPU9250_bw_wo_dlpf bw);
    void setAccRange(MPU9250_accRange accRange);
    void enableAccDLPF(bool enable);
    void setAccDLPF(MPU9250_dlpf dlpf);
    void setLowPowerAccDataRate(MPU9250_lpAccODR lpaodr);
    void enableAccAxes(MPU9250_xyzEn enable);
    void enableGyrAxes(MPU9250_xyzEn enable);
        
    /* x,y,z results */
     
    xyzFloat getAccRawValues();
    xyzFloat getCorrectedAccRawValues();
    xyzFloat getGValues();
    xyzFloat getAccRawValuesFromFifo();
    xyzFloat getCorrectedAccRawValuesFromFifo();
    xyzFloat getGValuesFromFifo();
    float getResultantG(xyzFloat gVal); 
    float getTemperature();
    xyzFloat getGyrRawValues();
    xyzFloat getCorrectedGyrRawValues();
    xyzFloat getGyrValues(); 
    xyzFloat getGyrValuesFromFifo();
    xyzFloat getMagValues();
    
        
    /* Angles and Orientation */ 
    
    xyzFloat getAngles();
    MPU9250_orientation getOrientation();
    String getOrientationAsString();
    float getPitch();
    float getRoll();
    
    /* Power, Sleep, Standby */ 
    
    void sleep(bool sleep);
    void enableCycle(bool cycle);
    void enableGyrStandby(bool gyroStandby);
   
    /* Interrupts */
   
    void setIntPinPolarity(MPU9250_intPinPol pol);
    void enableIntLatch(bool latch);
    void enableClearIntByAnyRead(bool clearByAnyRead);
    void enableInterrupt(MPU9250_intType intType);
    void disableInterrupt(MPU9250_intType intType);
    bool checkInterrupt(uint8_t source, MPU9250_intType type);
    uint8_t readAndClearInterrupts();
    void setWakeOnMotionThreshold(uint8_t womthresh);
    void enableWakeOnMotion(MPU9250_womEn womEn, MPU9250_womCompEn womCompEn);
       
    /* FIFO */
    
    void startFifo(MPU9250_fifo_type fifo);
    void stopFifo();
    void enableFifo(bool fifo);
    void resetFifo();
    int16_t getFifoCount();
    void setFifoMode(MPU9250_fifoMode mode);
    int16_t getNumberOfFifoDataSets();
    void findFifoBegin();
    
    /* Magnetometer */
    
    bool initMagnetometer();
    uint8_t whoAmIMag();
    void setMagOpMode(AK8963_opMode opMode);
    void startMagMeasurement();
       
private:
    TwoWire *_wire;
    int i2cAddress;
    xyzFloat accRawVal;
    xyzFloat gyrRawVal;
    xyzFloat accOffsetVal;
    xyzFloat gyrOffsetVal;
    xyzFloat magCorrFactor;
    uint8_t accRangeFactor;
    uint8_t gyrRangeFactor;
    uint8_t regVal;   // intermediate storage of register values
    MPU9250_fifo_type fifoType;
    
    void correctAccRawValues();
    void correctGyrRawValues();
    void getAsaVals();
    void reset_MPU9250();
    void enableI2CMaster();
    void enableMagDataRead(uint8_t reg, uint8_t bytes);
    void resetMagnetometer();
    void writeMPU9250Register(uint8_t reg, uint8_t val);
    void writeAK8963Register(uint8_t reg, uint8_t val);
    uint8_t readMPU9250Register8(uint8_t reg);
    uint8_t readAK8963Register8(uint8_t reg);
    uint64_t readAK8963Data();
    int16_t readMPU9250Register16(uint8_t reg);
    uint64_t readMPU9250Register3x16(uint8_t reg);
    xyzFloat readMPU9250xyzValFromFifo();
    void setMagnetometer16Bit();
    uint8_t getStatus2Register();
};

#endif
