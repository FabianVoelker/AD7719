/**************************************************************************/
/*!
    @file     ADS7719.cpp
    This is a library for the Low Voltage, Low Power, Factory-Calibrated 
    16-/24-Bit Dual AD7719 Analog to Digital Converter from Analog Devices.
    Written by Fabian Voelker for promesstec GmbH.
    MIT license, all text here must be included in any redistribution.
*/
/**************************************************************************/


#include "AD7719.h"
#include <SPI.h>

AD7719::~AD7719() 
{
  if (spi_dev){delete spi_dev;}
}

bool AD7719::begin(uint8_t cs, SPIClass *thisSPI)
{
    _cs = cs;
    if (spi_dev){delete spi_dev;}
    spi_dev = new Adafruit_SPIDevice(cs, AD7719_SPI_FREQ, AD7719_SPI_ORDER, AD7719_SPI_MODE, thisSPI);
    return spi_dev->begin();
}

bool AD7719::begin(uint8_t sclk, uint8_t mosi, uint8_t miso, uint8_t cs)
{
    _cs = cs;
    spi_dev = new Adafruit_SPIDevice(cs, sclk, miso, mosi, AD7719_SPI_FREQ, AD7719_SPI_ORDER, AD7719_SPI_MODE);
    return spi_dev->begin();
}


uint8_t AD7719::getStatus()
{
    uint8_t cmd, status;
    cmd = AD7719_READ_STATUS_REG;
    spi_dev->write_then_read(&cmd,1,&status,1);
    return status;
}


uint8_t AD7719::getMode()
{
    uint8_t cmd, mode;
    cmd = AD7719_READ_MODE_REG;
    spi_dev->write_then_read(&cmd,1,&mode,1);
    return mode;
}

void AD7719::setMode(uint8_t mode)
{
    _mode = mode;
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
}

bool AD7719::getBuffer()
{
    uint8_t cmd, mode;
    cmd = AD7719_READ_MODE_REG;
    spi_dev->write_then_read(&cmd,1,&mode,1);
    if(mode & AD7719_MODE_REGBIT_BUF)
    {
        _isbuffered = false;
        return _isbuffered;
    }
    else
    {
        _isbuffered = true;
        return _isbuffered;
    }
}

void AD7719::setBuffer(bool isbuffered)
{
    _mode = getMode();
    _isbuffered = isbuffered;

    if(_isbuffered)
    {
        ((_mode) |= (1UL << (AD7719_MODE_REGBIT_BUF)));
    }
    else
    {
        ((_mode) &= ~(1UL << (AD7719_MODE_REGBIT_BUF)));
    }
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
}

