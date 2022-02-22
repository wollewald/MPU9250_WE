/******************************************************************************
 *
 * This is a library for the 6-axis gyroscope and accelerometer MPU6500.
 *
 ******************************************************************************/

#ifndef MPU6500_WE_H_
#define MPU6500_WE_H_

#include "MPU9250_WE.h"


class MPU6500_WE : protected MPU9250_WE
{
public:
    /* Constructors */

    MPU6500_WE(int const addr);
    MPU6500_WE();
    MPU6500_WE(TwoWire * const w, int const addr);
    MPU6500_WE(TwoWire * const w);

    /* Basic settings */

    bool init();

    /* reuse MPU9250_WE */

    using MPU9250_WE::whoAmI;
    using MPU9250_WE::autoOffsets;
    using MPU9250_WE::setAccOffsets;
    using MPU9250_WE::setGyrOffsets;
    using MPU9250_WE::setGyrDLPF;
    using MPU9250_WE::setSampleRateDivider;
    using MPU9250_WE::setGyrRange;
    using MPU9250_WE::enableGyrDLPF;
    using MPU9250_WE::disableGyrDLPF;
    using MPU9250_WE::setAccRange;
    using MPU9250_WE::enableAccDLPF;
    using MPU9250_WE::setAccDLPF;
    using MPU9250_WE::setLowPowerAccDataRate;
    using MPU9250_WE::enableAccAxes;
    using MPU9250_WE::enableGyrAxes;

    /* x,y,z results */

    using MPU9250_WE::getAccRawValues;
    using MPU9250_WE::getCorrectedAccRawValues;
    using MPU9250_WE::getGValues;
    using MPU9250_WE::getAccRawValuesFromFifo;
    using MPU9250_WE::getCorrectedAccRawValuesFromFifo;
    using MPU9250_WE::getGValuesFromFifo;
    using MPU9250_WE::getResultantG;
    using MPU9250_WE::getTemperature;
    using MPU9250_WE::getGyrRawValues;
    using MPU9250_WE::getCorrectedGyrRawValues;
    using MPU9250_WE::getGyrValues;
    using MPU9250_WE::getGyrValuesFromFifo;


    /* Angles and Orientation */

    using MPU9250_WE::getAngles;
    using MPU9250_WE::getOrientation;
    using MPU9250_WE::getOrientationAsString;
    using MPU9250_WE::getPitch;
    using MPU9250_WE::getRoll;

    /* Power, Sleep, Standby */

    using MPU9250_WE::sleep;
    using MPU9250_WE::enableCycle;
    using MPU9250_WE::enableGyrStandby;

    /* Interrupts */

    using MPU9250_WE::setIntPinPolarity;
    using MPU9250_WE::enableIntLatch;
    using MPU9250_WE::enableClearIntByAnyRead;
    using MPU9250_WE::enableInterrupt;
    using MPU9250_WE::disableInterrupt;
    using MPU9250_WE::checkInterrupt;
    using MPU9250_WE::readAndClearInterrupts;
    using MPU9250_WE::setWakeOnMotionThreshold;
    using MPU9250_WE::enableWakeOnMotion;

    /* FIFO */

    using MPU9250_WE::startFifo;
    using MPU9250_WE::stopFifo;
    using MPU9250_WE::enableFifo;
    using MPU9250_WE::resetFifo;
    using MPU9250_WE::getFifoCount;
    using MPU9250_WE::setFifoMode;
    using MPU9250_WE::getNumberOfFifoDataSets;
    using MPU9250_WE::findFifoBegin;

};

#endif // MPU6500_WE_H_
