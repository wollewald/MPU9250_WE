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

#include "MPU6500_WE.h"

#define AK8963_ADDRESS MPU9250_WE::MAGNETOMETER_I2C_ADDRESS

/*
 * Please note that the following '#define's are only kept for backward-compatibility.
 * They do only reference correctly typed and static [i.e. uniquely instantiated]
 * values defined in the respective class.
 * If you do want to change a value, do not change any of these '#define's but
 * instead the actual referenced value.
 *
 */

/* Registers MPU9250 */
#define MPU9250_SELF_TEST_X_GYRO        MPU6500_WE::REGISTER_SELF_TEST_X_GYRO
#define MPU9250_SELF_TEST_Y_GYRO        MPU6500_WE::REGISTER_SELF_TEST_Y_GYRO
#define MPU9250_SELF_TEST_Z_GYRO        MPU6500_WE::REGISTER_SELF_TEST_Z_GYRO
#define MPU9250_SELF_TEST_X_ACCEL       MPU6500_WE::REGISTER_SELF_TEST_X_ACCEL
#define MPU9250_SELF_TEST_Y_ACCEL       MPU6500_WE::REGISTER_SELF_TEST_Y_ACCEL
#define MPU9250_SELF_TEST_Z_ACCEL       MPU6500_WE::REGISTER_SELF_TEST_Z_ACCEL
#define MPU9250_XG_OFFSET_H             MPU6500_WE::REGISTER_XG_OFFSET_H
#define MPU9250_XG_OFFSET_L             MPU6500_WE::REGISTER_XG_OFFSET_L
#define MPU9250_YG_OFFSET_H             MPU6500_WE::REGISTER_YG_OFFSET_H
#define MPU9250_YG_OFFSET_L             MPU6500_WE::REGISTER_YG_OFFSET_L
#define MPU9250_ZG_OFFSET_H             MPU6500_WE::REGISTER_ZG_OFFSET_H
#define MPU9250_ZG_OFFSET_L             MPU6500_WE::REGISTER_ZG_OFFSET_L
#define MPU9250_SMPLRT_DIV              MPU6500_WE::REGISTER_SMPLRT_DIV
#define MPU9250_CONFIG                  MPU6500_WE::REGISTER_CONFIG
#define MPU9250_GYRO_CONFIG             MPU6500_WE::REGISTER_GYRO_CONFIG
#define MPU9250_ACCEL_CONFIG            MPU6500_WE::REGISTER_ACCEL_CONFIG
#define MPU9250_ACCEL_CONFIG_2          MPU6500_WE::REGISTER_ACCEL_CONFIG_2
#define MPU9250_LP_ACCEL_ODR            MPU6500_WE::REGISTER_LP_ACCEL_ODR
#define MPU9250_WOM_THR                 MPU6500_WE::REGISTER_WOM_THR
#define MPU9250_FIFO_EN                 MPU6500_WE::REGISTER_FIFO_EN
#define MPU9250_I2C_MST_CTRL            MPU6500_WE::REGISTER_I2C_MST_CTRL
#define MPU9250_I2C_SLV0_ADDR           MPU6500_WE::REGISTER_I2C_SLV0_ADDR
#define MPU9250_I2C_SLV0_REG            MPU6500_WE::REGISTER_I2C_SLV0_REG
#define MPU9250_I2C_SLV0_CTRL           MPU6500_WE::REGISTER_I2C_SLV0_CTRL
#define MPU9250_I2C_MST_STATUS          MPU6500_WE::REGISTER_I2C_MST_STATUS
#define MPU9250_INT_PIN_CFG             MPU6500_WE::REGISTER_INT_PIN_CFG
#define MPU9250_INT_ENABLE              MPU6500_WE::REGISTER_INT_ENABLE
#define MPU9250_INT_STATUS              MPU6500_WE::REGISTER_INT_STATUS
#define MPU9250_ACCEL_OUT               MPU6500_WE::REGISTER_ACCEL_OUT
#define MPU9250_TEMP_OUT                MPU6500_WE::REGISTER_TEMP_OUT
#define MPU9250_GYRO_OUT                MPU6500_WE::REGISTER_GYRO_OUT
#define MPU9250_EXT_SLV_SENS_DATA_00    MPU6500_WE::REGISTER_EXT_SLV_SENS_DATA_00
#define MPU9250_I2C_SLV0_DO             MPU6500_WE::REGISTER_I2C_SLV0_DO
#define MPU9250_I2C_MST_DELAY_CTRL      MPU6500_WE::REGISTER_I2C_MST_DELAY_CTRL
#define MPU9250_SIGNAL_PATH_RESET       MPU6500_WE::REGISTER_SIGNAL_PATH_RESET
#define MPU9250_MOT_DET_CTRL            MPU6500_WE::REGISTER_MOT_DET_CTRL
#define MPU9250_USER_CTRL               MPU6500_WE::REGISTER_USER_CTRL
#define MPU9250_PWR_MGMT_1              MPU6500_WE::REGISTER_PWR_MGMT_1
#define MPU9250_PWR_MGMT_2              MPU6500_WE::REGISTER_PWR_MGMT_2
#define MPU9250_FIFO_COUNT              MPU6500_WE::REGISTER_FIFO_COUNT
#define MPU9250_FIFO_R_W                MPU6500_WE::REGISTER_FIFO_R_W
#define MPU9250_WHO_AM_I                MPU6500_WE::REGISTER_WHO_AM_I
#define MPU9250_XA_OFFSET_H             MPU6500_WE::REGISTER_XA_OFFSET_H
#define MPU9250_XA_OFFSET_L             MPU6500_WE::REGISTER_XA_OFFSET_L
#define MPU9250_YA_OFFSET_H             MPU6500_WE::REGISTER_YA_OFFSET_H
#define MPU9250_YA_OFFSET_L             MPU6500_WE::REGISTER_YA_OFFSET_L
#define MPU9250_ZA_OFFSET_H             MPU6500_WE::REGISTER_ZA_OFFSET_H
#define MPU9250_ZA_OFFSET_L             MPU6500_WE::REGISTER_ZA_OFFSET_L

/* Registers AK8963 */
#define AK8963_WIA                      MPU9250_WE::REGISTER_AK8963_WIA      // Who am I
#define AK8963_INFO                     MPU9250_WE::REGISTER_AK8963_INFO
#define AK8963_STATUS_1                 MPU9250_WE::REGISTER_AK8963_STATUS_1
#define AK8963_HXL                      MPU9250_WE::REGISTER_AK8963_HXL
#define AK8963_HYL                      MPU9250_WE::REGISTER_AK8963_HYL
#define AK8963_HZL                      MPU9250_WE::REGISTER_AK8963_HZL
#define AK8963_STATUS_2                 MPU9250_WE::REGISTER_AK8963_STATUS_2
#define AK8963_CNTL_1                   MPU9250_WE::REGISTER_AK8963_CNTL_1
#define AK8963_CNTL_2                   MPU9250_WE::REGISTER_AK8963_CNTL_2
#define AK8963_ASTC                     MPU9250_WE::REGISTER_AK8963_ASTC     // Self Test
#define AK8963_I2CDIS                   MPU9250_WE::REGISTER_AK8963_I2CDIS
#define AK8963_ASAX                     MPU9250_WE::REGISTER_AK8963_ASAX
#define AK8963_ASAY                     MPU9250_WE::REGISTER_AK8963_ASAY
#define AK8963_ASAZ                     MPU9250_WE::REGISTER_AK8963_ASAZ

/* Register Values */
#define MPU9250_RESET                   MPU6500_WE::REGISTER_VALUE_RESET
#define MPU9250_BYPASS_EN               MPU6500_WE::REGISTER_VALUE_BYPASS_EN
#define MPU9250_I2C_MST_EN              MPU6500_WE::REGISTER_VALUE_I2C_MST_EN
#define MPU9250_CLK_SEL_PLL             MPU6500_WE::REGISTER_VALUE_CLK_SEL_PLL
#define AK8963_16_BIT                   MPU9250_WE::REGISTER_VALUE_AK8963_16_BIT
#define AK8963_OVF                      MPU9250_WE::REGISTER_VALUE_AK8963_OVF
#define AK8963_READ                     MPU9250_WE::REGISTER_VALUE_AK8963_READ

/* Others */
#define MPU9250_ROOM_TEMP_OFFSET        MPU6500_WE::ROOM_TEMPERATURE_OFFSET
#define MPU9250_T_SENSITIVITY           MPU6500_WE::TEMPERATURE_SENSITIVITY
#define MPU9250_WHO_AM_I_CODE           MPU9250_WE::WHO_AM_I_CODE
#define AK8963_WHO_AM_I_CODE            MPU9250_WE::MAGNETOMETER_WHO_AM_I_CODE


/* Enums */
typedef enum AK8963_OP_MODE {
    AK8963_PWR_DOWN           = 0x00,
    AK8963_TRIGGER_MODE       = 0x01,
    AK8963_CONT_MODE_8HZ      = 0x02,
    AK8963_CONT_MODE_100HZ    = 0x06,
    AK8963_FUSE_ROM_ACC_MODE  = 0x0F
} AK8963_opMode;


class MPU9250_WE : public MPU6500_WE
{
public:


    /* Registers AK8963 */
    static uint8_t constexpr REGISTER_AK8963_WIA            = 0x00; // Who am I
    static uint8_t constexpr REGISTER_AK8963_INFO           = 0x01;
    static uint8_t constexpr REGISTER_AK8963_STATUS_1       = 0x02;
    static uint8_t constexpr REGISTER_AK8963_HXL            = 0x03;
    static uint8_t constexpr REGISTER_AK8963_HYL            = 0x05;
    static uint8_t constexpr REGISTER_AK8963_HZL            = 0x07;
    static uint8_t constexpr REGISTER_AK8963_STATUS_2       = 0x09;
    static uint8_t constexpr REGISTER_AK8963_CNTL_1         = 0x0A;
    static uint8_t constexpr REGISTER_AK8963_CNTL_2         = 0x0B;
    static uint8_t constexpr REGISTER_AK8963_ASTC           = 0x0C; // Self Test
    static uint8_t constexpr REGISTER_AK8963_I2CDIS         = 0x0F;
    static uint8_t constexpr REGISTER_AK8963_ASAX           = 0x10;
    static uint8_t constexpr REGISTER_AK8963_ASAY           = 0x11;
    static uint8_t constexpr REGISTER_AK8963_ASAZ           = 0x12;

    /* Register Values */
    static uint8_t constexpr REGISTER_VALUE_AK8963_16_BIT   = 0x10;
    static uint8_t constexpr REGISTER_VALUE_AK8963_OVF      = 0x08;
    static uint8_t constexpr REGISTER_VALUE_AK8963_READ     = 0x80;

    /* Others */
    static uint8_t constexpr WHO_AM_I_CODE                  = 0x71;
    static uint8_t constexpr MAGNETOMETER_I2C_ADDRESS       = 0x0C;
    static uint8_t constexpr MAGNETOMETER_WHO_AM_I_CODE     = 0x48;

    /* Constructors */

    MPU9250_WE(uint8_t addr);
    MPU9250_WE();
    MPU9250_WE(TwoWire *w, uint8_t addr);
    MPU9250_WE(TwoWire *w);
    MPU9250_WE(SPIClass *s, int cs, bool spi);

    /* Basic settings */

    bool init();

    /* x,y,z results */

    xyzFloat getMagValues();

    /* Magnetometer */

    bool initMagnetometer();
    uint8_t whoAmIMag();
    void setMagOpMode(AK8963_opMode opMode);
    void startMagMeasurement();

protected:
    void getAsaVals();
    void enableMagDataRead(uint8_t reg, uint8_t bytes);
    void resetMagnetometer();
    void writeAK8963Register(uint8_t reg, uint8_t val);
    uint8_t readAK8963Register8(uint8_t reg);
    void readAK8963Data(uint8_t *buf);
    void setMagnetometer16Bit();
    uint8_t getStatus2Register();

private:
    xyzFloat magCorrFactor;

};

#endif
