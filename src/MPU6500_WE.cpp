/********************************************************************
* This is a library for the 6-axis gyroscope and accelerometer MPU6500.
*********************************************************************/

#include "MPU6500_WE.h"

static uint8_t constexpr MPU6500_WHO_AM_I_CODE = 0x70;

/************  Constructors ************/

MPU6500_WE::MPU6500_WE(int const addr)
    : MPU9250_WE(addr)
{
}

MPU6500_WE::MPU6500_WE()
    : MPU9250_WE()
{
}

MPU6500_WE::MPU6500_WE(TwoWire * const w, int const addr)
    : MPU9250_WE(w, addr)
{
}

MPU6500_WE::MPU6500_WE(TwoWire * const w)
    : MPU9250_WE(w)
{
}


/************ Basic Settings ************/


bool MPU6500_WE::init(){
    return MPU9250_WE::init(MPU6500_WHO_AM_I_CODE);
}

/************ end ************/
