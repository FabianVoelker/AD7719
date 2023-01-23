/**************************************************************************/
/*!
    @file     ADS7719.h
    This is a library for the Low Voltage, Low Power, Factory-Calibrated 
    16-/24-Bit Dual AD7719 Analog to Digital Converter from Analog Devices.
    Written by Fabian Voelker for promesstec GmbH.
    MIT license, all text here must be included in any redistribution.
*/
/**************************************************************************/


#ifndef __AD7719_H__
#define __AD7719_H__

#include <SPI.h>
#include <Adafruit_SPIDevice.h>


#define AD7719_COM_REG 0x00 


class AD7719 {
public:
  ~AD7719();
  bool begin(uint8_t cs = SS, SPIClass *theSPI = &SPI);
  bool begin(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs);
  int readADC(uint8_t channel);
  int readADCDifference(uint8_t differential);

private:
  Adafruit_SPIDevice *spi_dev = NULL; ///< Pointer to SPI bus interface
  uint8_t _cs;
  uint8_t buffer[3];
  int SPIxADC(uint8_t channel, bool differential);
};



















#endif