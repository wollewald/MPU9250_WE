/******************************************************************************
 *
 * This is a library for the 6-axis gyroscope, accelerometer and magnetometer MPU6050.
 *
 * Please note: the library is primarily written for the MPU6500 and the MPU9250. The
 * support for the MPU6050 is limited.
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

#ifndef MPU6050_WE_H_
#define MPU6050_WE_H_

#include "MPU6500_WE.h"

/*
 * Please note that the following '#define's are only kept for backward-compatibility.
 * They do only reference correctly typed and static [i.e. uniquely instantiated]
 * values defined in the respective class.
 * If you do want to change a value, do not change any of these '#define's but
 * instead the actual referenced value.
 *
 */

/* Registers MPU6050 */
#define MPU6050_SELF_TEST_X_GYRO        MPU6500_WE::REGISTER_SELF_TEST_X_GYRO
#define MPU6050_SELF_TEST_Y_GYRO        MPU6500_WE::REGISTER_SELF_TEST_Y_GYRO
#define MPU6050_SELF_TEST_Z_GYRO        MPU6500_WE::REGISTER_SELF_TEST_Z_GYRO
#define MPU6050_SELF_TEST_X_ACCEL       MPU6500_WE::REGISTER_SELF_TEST_X_ACCEL
#define MPU6050_SELF_TEST_Y_ACCEL       MPU6500_WE::REGISTER_SELF_TEST_Y_ACCEL
#define MPU6050_SELF_TEST_Z_ACCEL       MPU6500_WE::REGISTER_SELF_TEST_Z_ACCEL
#define MPU6050_XG_OFFSET_H             MPU6500_WE::REGISTER_XG_OFFSET_H
#define MPU6050_XG_OFFSET_L             MPU6500_WE::REGISTER_XG_OFFSET_L
#define MPU6050_YG_OFFSET_H             MPU6500_WE::REGISTER_YG_OFFSET_H
#define MPU6050_YG_OFFSET_L             MPU6500_WE::REGISTER_YG_OFFSET_L
#define MPU6050_ZG_OFFSET_H             MPU6500_WE::REGISTER_ZG_OFFSET_H
#define MPU6050_ZG_OFFSET_L             MPU6500_WE::REGISTER_ZG_OFFSET_L
#define MPU6050_SMPLRT_DIV              MPU6500_WE::REGISTER_SMPLRT_DIV
#define MPU6050_CONFIG                  MPU6500_WE::REGISTER_CONFIG
#define MPU6050_GYRO_CONFIG             MPU6500_WE::REGISTER_GYRO_CONFIG
#define MPU6050_ACCEL_CONFIG            MPU6500_WE::REGISTER_ACCEL_CONFIG
#define MPU6050_ACCEL_CONFIG_2          MPU6500_WE::REGISTER_ACCEL_CONFIG_2
#define MPU6050_LP_ACCEL_ODR            MPU6500_WE::REGISTER_LP_ACCEL_ODR
#define MPU6050_WOM_THR                 MPU6500_WE::REGISTER_WOM_THR
#define MPU6050_FIFO_EN                 MPU6500_WE::REGISTER_FIFO_EN
#define MPU6050_I2C_MST_CTRL            MPU6500_WE::REGISTER_I2C_MST_CTRL
#define MPU6050_I2C_SLV0_ADDR           MPU6500_WE::REGISTER_I2C_SLV0_ADDR
#define MPU6050_I2C_SLV0_REG            MPU6500_WE::REGISTER_I2C_SLV0_REG
#define MPU6050_I2C_SLV0_CTRL           MPU6500_WE::REGISTER_I2C_SLV0_CTRL
#define MPU6050_I2C_MST_STATUS          MPU6500_WE::REGISTER_I2C_MST_STATUS
#define MPU6050_INT_PIN_CFG             MPU6500_WE::REGISTER_INT_PIN_CFG
#define MPU6050_INT_ENABLE              MPU6500_WE::REGISTER_INT_ENABLE
#define MPU6050_INT_STATUS              MPU6500_WE::REGISTER_INT_STATUS
#define MPU6050_ACCEL_OUT               MPU6500_WE::REGISTER_ACCEL_OUT
#define MPU6050_TEMP_OUT                MPU6500_WE::REGISTER_TEMP_OUT
#define MPU6050_GYRO_OUT                MPU6500_WE::REGISTER_GYRO_OUT
#define MPU6050_EXT_SLV_SENS_DATA_00    MPU6500_WE::REGISTER_EXT_SLV_SENS_DATA_00
#define MPU6050_I2C_SLV0_DO             MPU6500_WE::REGISTER_I2C_SLV0_DO
#define MPU6050_I2C_MST_DELAY_CTRL      MPU6500_WE::REGISTER_I2C_MST_DELAY_CTRL
#define MPU6050_SIGNAL_PATH_RESET       MPU6500_WE::REGISTER_SIGNAL_PATH_RESET
#define MPU6050_MOT_DET_CTRL            MPU6500_WE::REGISTER_MOT_DET_CTRL
#define MPU6050_USER_CTRL               MPU6500_WE::REGISTER_USER_CTRL
#define MPU6050_PWR_MGMT_1              MPU6500_WE::REGISTER_PWR_MGMT_1
#define MPU6050_PWR_MGMT_2              MPU6500_WE::REGISTER_PWR_MGMT_2
#define MPU6050_FIFO_COUNT              MPU6500_WE::REGISTER_FIFO_COUNT
#define MPU6050_FIFO_R_W                MPU6500_WE::REGISTER_FIFO_R_W
#define MPU6050_WHO_AM_I                MPU6500_WE::REGISTER_WHO_AM_I
#define MPU6050_XA_OFFSET_H             MPU6500_WE::REGISTER_XA_OFFSET_H
#define MPU6050_XA_OFFSET_L             MPU6500_WE::REGISTER_XA_OFFSET_L
#define MPU6050_YA_OFFSET_H             MPU6500_WE::REGISTER_YA_OFFSET_H
#define MPU6050_YA_OFFSET_L             MPU6500_WE::REGISTER_YA_OFFSET_L
#define MPU6050_ZA_OFFSET_H             MPU6500_WE::REGISTER_ZA_OFFSET_H
#define MPU6050_ZA_OFFSET_L             MPU6500_WE::REGISTER_ZA_OFFSET_L

/* Register Values */
#define MPU6050_RESET                   MPU6500_WE::REGISTER_VALUE_RESET
#define MPU6050_BYPASS_EN               MPU6500_WE::REGISTER_VALUE_BYPASS_EN
#define MPU6050_I2C_MST_EN              MPU6500_WE::REGISTER_VALUE_I2C_MST_EN
#define MPU6050_CLK_SEL_PLL             MPU6500_WE::REGISTER_VALUE_CLK_SEL_PLL

/* Others */
#define MPU6050_ROOM_TEMP_OFFSET        MPU6500_WE::ROOM_TEMPERATURE_OFFSET
#define MPU6050_T_SENSITIVITY           MPU6500_WE::TEMPERATURE_SENSITIVITY
#define MPU6050_WHO_AM_I_CODE           MPU9250_WE::WHO_AM_I_CODE

class MPU6050_WE : public MPU6500_WE
{
public:

    /* Who Am I */
    static uint8_t constexpr WHO_AM_I_CODE                  = 0x68;
    static uint8_t constexpr REGISTER_MOT_DUR               = 0x20;

    /* Constructors */

    MPU6050_WE(uint8_t addr);
    MPU6050_WE();
    MPU6050_WE(TwoWire *w, uint8_t addr);
    MPU6050_WE(TwoWire *w);
    MPU6050_WE(SPIClass *s, int cs, bool spi, bool pc = false);
    MPU6050_WE(SPIClass *s, int cs, int mosi, int miso, int sck, bool spi, bool pc = true);

    /* Basic settings */

    bool init();
    
    float getTemperature();

protected:
    /* none */
private:
    /* none */

};

#endif
