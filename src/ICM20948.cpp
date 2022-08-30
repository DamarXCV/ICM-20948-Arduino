/********************************************************************
 * This is a library for the 9-axis gyroscope, accelerometer and magnetometer ICM20948.
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
 * Edited by DamarXCV
 *
 * Further information can be found on:
 *
 * https://wolles-elektronikkiste.de/icm-20948-9-achsensensor-teil-i (German)
 * https://wolles-elektronikkiste.de/en/icm-20948-9-axis-sensor-part-i (English)
 *
 *********************************************************************/

#include "ICM20948.h"

///////////////////////////////////////////////
// Constructors
///////////////////////////////////////////////

ICM20948::ICM20948(int addr)
{
    _wire = &Wire;
    i2cAddress = addr;
}

ICM20948::ICM20948()
{
    _wire = &Wire;
    i2cAddress = 0x69;
}

ICM20948::ICM20948(TwoWire* w, int addr)
{
    _wire = w;
    i2cAddress = addr;
}

ICM20948::ICM20948(TwoWire* w)
{
    _wire = w;
    i2cAddress = 0x69;
}

///////////////////////////////////////////////
// Basic Settings
///////////////////////////////////////////////

bool ICM20948::init()
{
    currentBank = 0;

    resetICM20948();
    if (whoAmI() != ICM20948_WHO_AM_I_CONTENT) {
        return false;
    }

    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;
    accCorrFactor.x = 1.0;
    accCorrFactor.y = 1.0;
    accCorrFactor.z = 1.0;
    accRangeFactor = 1.0;
    gyrOffsetVal.x = 0.0;
    gyrOffsetVal.y = 0.0;
    gyrOffsetVal.z = 0.0;
    gyrRangeFactor = 1.0;
    fifoType = ICM20948_FIFO_ACC;

    wakeup();
    writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR

    return true;
}

void ICM20948::autoOffsets(uint8_t runs)
{
    xyzFloat accRawVal, gyrRawVal;
    accOffsetVal.x = 0.0;
    accOffsetVal.y = 0.0;
    accOffsetVal.z = 0.0;

    setGyrDLPF(ICM20948_DLPF_6); // lowest noise
    setGyrRange(ICM20948_GYRO_RANGE_250); // highest resolution
    setAccRange(ICM20948_ACC_RANGE_2G);
    setAccDLPF(ICM20948_DLPF_6);
    delay(100);

    for (int i = 0; i < runs; i++) {
        readSensor();

        accRawVal = getAccRawValues();
        accOffsetVal.x += accRawVal.x;
        accOffsetVal.y += accRawVal.y;
        accOffsetVal.z += accRawVal.z;

        gyrRawVal = getGyrRawValues();
        gyrOffsetVal.x += gyrRawVal.x;
        gyrOffsetVal.y += gyrRawVal.y;
        gyrOffsetVal.z += gyrRawVal.z;

        delay(10);
    }

    accOffsetVal.x /= runs;
    accOffsetVal.y /= runs;
    accOffsetVal.z /= runs;
    accOffsetVal.z -= 16384.0;

    gyrOffsetVal.x /= runs;
    gyrOffsetVal.y /= runs;
    gyrOffsetVal.z /= runs;
}

void ICM20948::setAccOffsets(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax)
{
    accOffsetVal.x = (xMax + xMin) * 0.5;
    accOffsetVal.y = (yMax + yMin) * 0.5;
    accOffsetVal.z = (zMax + zMin) * 0.5;
    accCorrFactor.x = (xMax + abs(xMin)) / 32768.0;
    accCorrFactor.y = (yMax + abs(yMin)) / 32768.0;
    accCorrFactor.z = (zMax + abs(zMin)) / 32768.0;
}

void ICM20948::setGyrOffsets(float xOffset, float yOffset, float zOffset)
{
    gyrOffsetVal.x = xOffset;
    gyrOffsetVal.y = yOffset;
    gyrOffsetVal.z = zOffset;
}

uint8_t ICM20948::whoAmI()
{
    return readRegister8(0, ICM20948_WHO_AM_I);
}

void ICM20948::enableAcc()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
    regVal &= ~ICM20948_ACC_EN;
    writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
}

void ICM20948::disableAcc()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
    regVal |= ICM20948_ACC_EN;
    writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
}

void ICM20948::setAccRange(ICM20948_accRange accRange)
{
    regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);
    regVal &= ~(0x06);
    regVal |= (accRange << 1);
    writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
    accRangeFactor = 1 << accRange;
}

void ICM20948::setAccDLPF(ICM20948_dlpf dlpf)
{
    regVal = readRegister8(2, ICM20948_ACCEL_CONFIG);
    if (dlpf == ICM20948_DLPF_OFF) {
        regVal &= 0xFE;
        writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
        return;
    } else {
        regVal |= 0x01;
        regVal &= 0xC7;
        regVal |= (dlpf << 3);
    }
    writeRegister8(2, ICM20948_ACCEL_CONFIG, regVal);
}

void ICM20948::setAccSampleRateDivider(uint16_t accSplRateDiv)
{
    writeRegister16(2, ICM20948_ACCEL_SMPLRT_DIV_1, accSplRateDiv);
}

void ICM20948::enableGyr()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
    regVal &= ~ICM20948_GYR_EN;
    writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
}

void ICM20948::disableGyr()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_2);
    regVal |= ICM20948_GYR_EN;
    writeRegister8(0, ICM20948_PWR_MGMT_2, regVal);
}

void ICM20948::setGyrRange(ICM20948_gyroRange gyroRange)
{
    regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);
    regVal &= ~(0x06);
    regVal |= (gyroRange << 1);
    writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
    gyrRangeFactor = (1 << gyroRange);
}

void ICM20948::setGyrDLPF(ICM20948_dlpf dlpf)
{
    regVal = readRegister8(2, ICM20948_GYRO_CONFIG_1);
    if (dlpf == ICM20948_DLPF_OFF) {
        regVal &= 0xFE;
        writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
        return;
    } else {
        regVal |= 0x01;
        regVal &= 0xC7;
        regVal |= (dlpf << 3);
    }
    writeRegister8(2, ICM20948_GYRO_CONFIG_1, regVal);
}

void ICM20948::setGyrSampleRateDivider(uint8_t gyrSplRateDiv)
{
    writeRegister8(2, ICM20948_GYRO_SMPLRT_DIV, gyrSplRateDiv);
}

void ICM20948::setTempDLPF(ICM20948_dlpf dlpf)
{
    writeRegister8(2, ICM20948_TEMP_CONFIG, dlpf);
}

void ICM20948::setI2CMstSampleRate(uint8_t rateExp)
{
    if (rateExp < 16) {
        writeRegister8(3, ICM20948_I2C_MST_ODR_CFG, rateExp);
    }
}

///////////////////////////////////////////////
// x,y,z results
///////////////////////////////////////////////

void ICM20948::readSensor()
{
    readAllData(buffer);
}

xyzFloat ICM20948::getAccRawValues()
{
    xyzFloat accRawVal;
    accRawVal.x = (int16_t)(((buffer[0]) << 8) | buffer[1]) * 1.0;
    accRawVal.y = (int16_t)(((buffer[2]) << 8) | buffer[3]) * 1.0;
    accRawVal.z = (int16_t)(((buffer[4]) << 8) | buffer[5]) * 1.0;
    return accRawVal;
}

xyzFloat ICM20948::getCorrectedAccRawValues()
{
    xyzFloat accRawVal = getAccRawValues();
    accRawVal = correctAccRawValues(accRawVal);

    return accRawVal;
}

xyzFloat ICM20948::getGValues()
{
    xyzFloat gVal, accRawVal;
    accRawVal = getCorrectedAccRawValues();

    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

xyzFloat ICM20948::getAccRawValuesFromFifo()
{
    xyzFloat accRawVal = readICM20948xyzValFromFifo();
    return accRawVal;
}

xyzFloat ICM20948::getCorrectedAccRawValuesFromFifo()
{
    xyzFloat accRawVal = getAccRawValuesFromFifo();
    accRawVal = correctAccRawValues(accRawVal);

    return accRawVal;
}

xyzFloat ICM20948::getGValuesFromFifo()
{
    xyzFloat gVal, accRawVal;
    accRawVal = getCorrectedAccRawValuesFromFifo();

    gVal.x = accRawVal.x * accRangeFactor / 16384.0;
    gVal.y = accRawVal.y * accRangeFactor / 16384.0;
    gVal.z = accRawVal.z * accRangeFactor / 16384.0;
    return gVal;
}

float ICM20948::getResultantG(xyzFloat gVal)
{
    float resultant = 0.0;
    resultant = sqrt(sq(gVal.x) + sq(gVal.y) + sq(gVal.z));

    return resultant;
}

float ICM20948::getTemperature()
{
    int16_t rawTemp = (int16_t)(((buffer[12]) << 8) | buffer[13]);
    float tmp = (rawTemp * 1.0 - ICM20948_ROOM_TEMP_OFFSET) / ICM20948_T_SENSITIVITY + 21.0;
    return tmp;
}

xyzFloat ICM20948::getGyrRawValues()
{
    xyzFloat gyrRawVal;

    gyrRawVal.x = (int16_t)(((buffer[6]) << 8) | buffer[7]) * 1.0;
    gyrRawVal.y = (int16_t)(((buffer[8]) << 8) | buffer[9]) * 1.0;
    gyrRawVal.z = (int16_t)(((buffer[10]) << 8) | buffer[11]) * 1.0;

    return gyrRawVal;
}

xyzFloat ICM20948::getCorrectedGyrRawValues()
{
    xyzFloat gyrRawVal = getGyrRawValues();
    gyrRawVal = correctGyrRawValues(gyrRawVal);
    return gyrRawVal;
}

xyzFloat ICM20948::getGyrValues()
{
    xyzFloat gyrVal = getCorrectedGyrRawValues();

    gyrVal.x = gyrVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrVal.z * gyrRangeFactor * 250.0 / 32768.0;

    return gyrVal;
}

xyzFloat ICM20948::getGyrValuesFromFifo()
{
    xyzFloat gyrVal;
    xyzFloat gyrRawVal = readICM20948xyzValFromFifo();

    gyrRawVal = correctGyrRawValues(gyrRawVal);
    gyrVal.x = gyrRawVal.x * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.y = gyrRawVal.y * gyrRangeFactor * 250.0 / 32768.0;
    gyrVal.z = gyrRawVal.z * gyrRangeFactor * 250.0 / 32768.0;

    return gyrVal;
}

xyzFloat ICM20948::getMagValues()
{
    int16_t x, y, z;
    xyzFloat mag;

    x = (int16_t)((buffer[15]) << 8) | buffer[14];
    y = (int16_t)((buffer[17]) << 8) | buffer[16];
    z = (int16_t)((buffer[19]) << 8) | buffer[18];

    mag.x = x * AK09916_MAG_LSB;
    mag.y = y * AK09916_MAG_LSB;
    mag.z = z * AK09916_MAG_LSB;

    return mag;
}

///////////////////////////////////////////////
// Power, Sleep, Standby
///////////////////////////////////////////////

void ICM20948::enableCycle(ICM20948_cycle cycle)
{
    regVal = readRegister8(0, ICM20948_LP_CONFIG);
    regVal &= 0x0F;
    regVal |= cycle;

    writeRegister8(0, ICM20948_LP_CONFIG, regVal);
}

void ICM20948::enableLowPower()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    regVal |= ICM20948_LP_EN;
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

void ICM20948::disableLowPower()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    regVal &= ~ICM20948_LP_EN;
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

void ICM20948::setGyrAverageInCycleMode(ICM20948_gyroAvgLowPower avg)
{
    writeRegister8(2, ICM20948_GYRO_CONFIG_2, avg);
}

void ICM20948::setAccAverageInCycleMode(ICM20948_accAvgLowPower avg)
{
    writeRegister8(2, ICM20948_ACCEL_CONFIG_2, avg);
}

void ICM20948::sleep()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    regVal |= ICM20948_SLEEP;
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

void ICM20948::wakeup()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    regVal &= ~ICM20948_SLEEP;
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
}

///////////////////////////////////////////////
// Interrupts
///////////////////////////////////////////////

void ICM20948::setIntPinPolarity(ICM20948_intPinPol pol)
{
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    if (pol) {
        regVal |= ICM20948_INT1_ACTL;
    } else {
        regVal &= ~ICM20948_INT1_ACTL;
    }
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::enableIntLatch()
{
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    regVal |= ICM20948_INT_1_LATCH_EN;
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::disableIntLatch()
{
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    regVal &= ~ICM20948_INT_1_LATCH_EN;
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::enableClearIntByAnyRead()
{
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    regVal |= ICM20948_INT_ANYRD_2CLEAR;
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::disableClearIntByAnyRead()
{
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    regVal &= ~ICM20948_INT_ANYRD_2CLEAR;
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::setFSyncIntPolarity(ICM20948_intPinPol pol)
{
    regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
    if (pol) {
        regVal |= ICM20948_ACTL_FSYNC;
    } else {
        regVal &= ~ICM20948_ACTL_FSYNC;
    }
    writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
}

void ICM20948::enableInterrupt(ICM20948_intType intType)
{
    switch (intType) {
    case ICM20948_FSYNC_INT:
        regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
        regVal |= ICM20948_FSYNC_INT_MODE_EN;
        writeRegister8(0, ICM20948_INT_PIN_CFG, regVal); // enable FSYNC as interrupt pin
        regVal = readRegister8(0, ICM20948_INT_ENABLE);
        regVal |= 0x80;
        writeRegister8(0, ICM20948_INT_ENABLE, regVal); // enable wake on FSYNC interrupt
        break;

    case ICM20948_WOM_INT:
        regVal = readRegister8(0, ICM20948_INT_ENABLE);
        regVal |= 0x08;
        writeRegister8(0, ICM20948_INT_ENABLE, regVal);
        regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
        regVal |= 0x02;
        writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
        break;

    case ICM20948_DMP_INT:
        regVal = readRegister8(0, ICM20948_INT_ENABLE);
        regVal |= 0x02;
        writeRegister8(0, ICM20948_INT_ENABLE, regVal);
        break;

    case ICM20948_DATA_READY_INT:
        writeRegister8(0, ICM20948_INT_ENABLE_1, 0x01);
        break;

    case ICM20948_FIFO_OVF_INT:
        writeRegister8(0, ICM20948_INT_ENABLE_2, 0x01);
        break;

    case ICM20948_FIFO_WM_INT:
        writeRegister8(0, ICM20948_INT_ENABLE_3, 0x01);
        break;
    }
}

void ICM20948::disableInterrupt(ICM20948_intType intType)
{
    switch (intType) {
    case ICM20948_FSYNC_INT:
        regVal = readRegister8(0, ICM20948_INT_PIN_CFG);
        regVal &= ~ICM20948_FSYNC_INT_MODE_EN;
        writeRegister8(0, ICM20948_INT_PIN_CFG, regVal);
        regVal = readRegister8(0, ICM20948_INT_ENABLE);
        regVal &= ~(0x80);
        writeRegister8(0, ICM20948_INT_ENABLE, regVal);
        break;

    case ICM20948_WOM_INT:
        regVal = readRegister8(0, ICM20948_INT_ENABLE);
        regVal &= ~(0x08);
        writeRegister8(0, ICM20948_INT_ENABLE, regVal);
        regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
        regVal &= ~(0x02);
        writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
        break;

    case ICM20948_DMP_INT:
        regVal = readRegister8(0, ICM20948_INT_ENABLE);
        regVal &= ~(0x02);
        writeRegister8(0, ICM20948_INT_ENABLE, regVal);
        break;

    case ICM20948_DATA_READY_INT:
        writeRegister8(0, ICM20948_INT_ENABLE_1, 0x00);
        break;

    case ICM20948_FIFO_OVF_INT:
        writeRegister8(0, ICM20948_INT_ENABLE_2, 0x00);
        break;

    case ICM20948_FIFO_WM_INT:
        writeRegister8(0, ICM20948_INT_ENABLE_3, 0x00);
        break;
    }
}

uint8_t ICM20948::readAndClearInterrupts()
{
    uint8_t intSource = 0;
    regVal = readRegister8(0, ICM20948_I2C_MST_STATUS);
    if (regVal & 0x80) {
        intSource |= 0x01;
    }
    regVal = readRegister8(0, ICM20948_INT_STATUS);
    if (regVal & 0x08) {
        intSource |= 0x02;
    }
    if (regVal & 0x02) {
        intSource |= 0x04;
    }
    regVal = readRegister8(0, ICM20948_INT_STATUS_1);
    if (regVal & 0x01) {
        intSource |= 0x08;
    }
    regVal = readRegister8(0, ICM20948_INT_STATUS_2);
    if (regVal & 0x01) {
        intSource |= 0x10;
    }
    regVal = readRegister8(0, ICM20948_INT_STATUS_3);
    if (regVal & 0x01) {
        intSource |= 0x20;
    }
    return intSource;
}

bool ICM20948::checkInterrupt(uint8_t source, ICM20948_intType type)
{
    source &= type;
    return source;
}
void ICM20948::setWakeOnMotionThreshold(uint8_t womThresh, ICM20948_womCompEn womCompEn)
{
    regVal = readRegister8(2, ICM20948_ACCEL_INTEL_CTRL);
    if (womCompEn) {
        regVal |= 0x01;
    } else {
        regVal &= ~(0x01);
    }
    writeRegister8(2, ICM20948_ACCEL_INTEL_CTRL, regVal);
    writeRegister8(2, ICM20948_ACCEL_WOM_THR, womThresh);
}

///////////////////////////////////////////////
// FIFO
///////////////////////////////////////////////

void ICM20948::enableFifo()
{
    regVal = readRegister8(0, ICM20948_USER_CTRL);
    regVal |= ICM20948_FIFO_EN;
    writeRegister8(0, ICM20948_USER_CTRL, regVal);
}

void ICM20948::disableFifo()
{
    regVal = readRegister8(0, ICM20948_USER_CTRL);
    regVal &= ~ICM20948_FIFO_EN;
    writeRegister8(0, ICM20948_USER_CTRL, regVal);
}

void ICM20948::setFifoMode(ICM20948_fifoMode mode)
{
    if (mode) {
        regVal = 0x01;
    } else {
        regVal = 0x00;
    }
    writeRegister8(0, ICM20948_FIFO_MODE, regVal);
}

void ICM20948::startFifo(ICM20948_fifoType fifo)
{
    fifoType = fifo;
    writeRegister8(0, ICM20948_FIFO_EN_2, fifoType);
}

void ICM20948::stopFifo()
{
    writeRegister8(0, ICM20948_FIFO_EN_2, 0);
}

void ICM20948::resetFifo()
{
    writeRegister8(0, ICM20948_FIFO_RST, 0x01);
    writeRegister8(0, ICM20948_FIFO_RST, 0x00);
}

int16_t ICM20948::getFifoCount()
{
    int16_t regVal16 = (int16_t)readRegister16(0, ICM20948_FIFO_COUNT);
    return regVal16;
}

int16_t ICM20948::getNumberOfFifoDataSets()
{
    int16_t numberOfSets = getFifoCount();

    if ((fifoType == ICM20948_FIFO_ACC) || (fifoType == ICM20948_FIFO_GYR)) {
        numberOfSets /= 6;
    } else if (fifoType == ICM20948_FIFO_ACC_GYR) {
        numberOfSets /= 12;
    }

    return numberOfSets;
}

void ICM20948::findFifoBegin()
{
    uint16_t count = getFifoCount();
    int16_t start = 0;

    if ((fifoType == ICM20948_FIFO_ACC) || (fifoType == ICM20948_FIFO_GYR)) {
        start = count % 6;
        for (int i = 0; i < start; i++) {
            readRegister8(0, ICM20948_FIFO_R_W);
        }
    } else if (fifoType == ICM20948_FIFO_ACC_GYR) {
        start = count % 12;
        for (int i = 0; i < start; i++) {
            readRegister8(0, ICM20948_FIFO_R_W);
        }
    }
}

///////////////////////////////////////////////
// Magnetometer
///////////////////////////////////////////////

bool ICM20948::initMagnetometer()
{
    enableI2CMaster();
    resetMag();
    resetICM20948();
    wakeup();
    writeRegister8(2, ICM20948_ODR_ALIGN_EN, 1); // aligns ODR
    delay(10);
    enableI2CMaster();
    delay(10);

    int16_t whoAmI = whoAmIMag();
    if (!((whoAmI == AK09916_WHO_AM_I_1) || (whoAmI == AK09916_WHO_AM_I_2))) {
        return false;
    }
    setMagOpMode(AK09916_CONT_MODE_100HZ);

    return true;
}

int16_t ICM20948::whoAmIMag()
{
    return readAK09916Register16(AK09916_WIA_1);
}

void ICM20948::setMagOpMode(AK09916_opMode opMode)
{
    writeAK09916Register8(AK09916_CNTL_2, opMode);
    delay(10);
    if (opMode != AK09916_PWR_DOWN) {
        enableMagDataRead(AK09916_HXL, 0x08);
    }
}

void ICM20948::resetMag()
{
    writeAK09916Register8(AK09916_CNTL_3, 0x01);
    delay(100);
}

///////////////////////////////////////////////
// Private Functions
///////////////////////////////////////////////

void ICM20948::setClockToAutoSelect()
{
    regVal = readRegister8(0, ICM20948_PWR_MGMT_1);
    regVal |= 0x01;
    writeRegister8(0, ICM20948_PWR_MGMT_1, regVal);
    delay(10);
}

xyzFloat ICM20948::correctAccRawValues(xyzFloat accRawVal)
{
    accRawVal.x = (accRawVal.x - (accOffsetVal.x / accRangeFactor)) / accCorrFactor.x;
    accRawVal.y = (accRawVal.y - (accOffsetVal.y / accRangeFactor)) / accCorrFactor.y;
    accRawVal.z = (accRawVal.z - (accOffsetVal.z / accRangeFactor)) / accCorrFactor.z;

    return accRawVal;
}

xyzFloat ICM20948::correctGyrRawValues(xyzFloat gyrRawVal)
{
    gyrRawVal.x -= (gyrOffsetVal.x / gyrRangeFactor);
    gyrRawVal.y -= (gyrOffsetVal.y / gyrRangeFactor);
    gyrRawVal.z -= (gyrOffsetVal.z / gyrRangeFactor);

    return gyrRawVal;
}

void ICM20948::switchBank(uint8_t newBank)
{
    if (newBank != currentBank) {
        currentBank = newBank;
        _wire->beginTransmission(i2cAddress);
        _wire->write(ICM20948_REG_BANK_SEL);
        _wire->write(currentBank << 4);
        _wire->endTransmission();
    }
}

void ICM20948::writeRegister8(uint8_t bank, uint8_t reg, uint8_t val)
{
    switchBank(bank);

    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->write(val);
    _wire->endTransmission();
}

void ICM20948::writeRegister16(uint8_t bank, uint8_t reg, int16_t val)
{
    switchBank(bank);
    int8_t MSByte = (int8_t)((val >> 8) & 0xFF);
    uint8_t LSByte = val & 0xFF;

    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->write(MSByte);
    _wire->write(LSByte);
    _wire->endTransmission();
}

uint8_t ICM20948::readRegister8(uint8_t bank, uint8_t reg)
{
    switchBank(bank);
    uint8_t regValue = 0;

    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress, 1);
    if (_wire->available()) {
        regValue = _wire->read();
    }

    return regValue;
}

int16_t ICM20948::readRegister16(uint8_t bank, uint8_t reg)
{
    switchBank(bank);
    uint8_t MSByte = 0, LSByte = 0;
    int16_t reg16Val = 0;

    _wire->beginTransmission(i2cAddress);
    _wire->write(reg);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress, 2);
    if (_wire->available()) {
        MSByte = _wire->read();
        LSByte = _wire->read();
    }

    reg16Val = (MSByte << 8) + LSByte;
    return reg16Val;
}

void ICM20948::readAllData(uint8_t* data)
{
    switchBank(0);

    _wire->beginTransmission(i2cAddress);
    _wire->write(ICM20948_ACCEL_OUT);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress, 20);
    if (_wire->available()) {
        for (int i = 0; i < 20; i++) {
            data[i] = _wire->read();
        }
    }
}

xyzFloat ICM20948::readICM20948xyzValFromFifo()
{
    uint8_t fifoTriple[6];
    xyzFloat xyzResult = { 0.0, 0.0, 0.0 };
    switchBank(0);

    _wire->beginTransmission(i2cAddress);
    _wire->write(ICM20948_FIFO_R_W);
    _wire->endTransmission(false);
    _wire->requestFrom(i2cAddress, 6);
    if (_wire->available()) {
        for (int i = 0; i < 6; i++) {
            fifoTriple[i] = _wire->read();
        }
    }

    xyzResult.x = ((int16_t)((fifoTriple[0] << 8) + fifoTriple[1])) * 1.0;
    xyzResult.y = ((int16_t)((fifoTriple[2] << 8) + fifoTriple[3])) * 1.0;
    xyzResult.z = ((int16_t)((fifoTriple[4] << 8) + fifoTriple[5])) * 1.0;

    return xyzResult;
}

void ICM20948::writeAK09916Register8(uint8_t reg, uint8_t val)
{
    writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS); // write AK09916
    writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be written to
    writeRegister8(3, ICM20948_I2C_SLV0_DO, val);
}

uint8_t ICM20948::readAK09916Register8(uint8_t reg)
{
    enableMagDataRead(reg, 0x01);
    enableMagDataRead(AK09916_HXL, 0x08);
    regVal = readRegister8(0, ICM20948_EXT_SLV_SENS_DATA_00);
    return regVal;
}

int16_t ICM20948::readAK09916Register16(uint8_t reg)
{
    int16_t regValue = 0;
    enableMagDataRead(reg, 0x02);
    regValue = readRegister16(0, ICM20948_EXT_SLV_SENS_DATA_00);
    enableMagDataRead(AK09916_HXL, 0x08);
    return regValue;
}

void ICM20948::resetICM20948()
{
    writeRegister8(0, ICM20948_PWR_MGMT_1, ICM20948_RESET);
    delay(10); // wait for registers to reset
}

void ICM20948::enableI2CMaster()
{
    writeRegister8(0, ICM20948_USER_CTRL, ICM20948_I2C_MST_EN); // enable I2C master
    writeRegister8(3, ICM20948_I2C_MST_CTRL, 0x07); // set I2C clock to 345.60 kHz
    delay(10);
}

void ICM20948::enableMagDataRead(uint8_t reg, uint8_t bytes)
{
    writeRegister8(3, ICM20948_I2C_SLV0_ADDR, AK09916_ADDRESS | AK09916_READ); // read AK09916
    writeRegister8(3, ICM20948_I2C_SLV0_REG, reg); // define AK09916 register to be read
    writeRegister8(3, ICM20948_I2C_SLV0_CTRL, 0x80 | bytes); // enable read | number of byte
    delay(10);
}
