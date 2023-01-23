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


#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <Adafruit_SPIDevice.h>
#include <SPI.h>


#define AD7719_SPI_ORDER  SPI_BITORDER_MSBFIRST
#define AD7719_SPI_MODE   SPI_MODE0
#define AD7719_SPI_FREQ   1000000



/*================*/
/* Register table */
/*================*/

// Write Register
#define AD7719_WRITE_MODE_REG           0x01    // 0000 0001  Write MODE register     [8-Bit]
#define AD7719_WRITE_AD0CON_REG         0x02    // 0000 0010  Write AD0CON register   [8-Bit]
#define AD7719_WRITE_AD1CON_REG         0x03    // 0000 0011  Write AD1CON register   [8-Bit]
#define AD7719_WRITE_FILT_REG           0x04    // 0000 0011  Write FILTER register   [8-Bit]
#define AD7719_WRITE_IOCON_REG          0x07    // 0000 0111  Write IOCON register    [16-Bit]

// Calibration register -> Done bei Analog Devices
#define AD7719_WRITE_AD0OFFS_REG        0x08    // 0000 1000  Write AD0OFFS register  [24-Bit] 
#define AD7719_WRITE_AD1OFFS_REG        0x09    // 0000 1001  Write AD1OFFS register  [16-Bit]
#define AD7719_WRITE_AD0GAIN_REG        0x0A    // 0000 1010  Write AD0GAIN register  [24-Bit]
#define AD7719_WRITE_AD1GAIN_REG        0x0B    // 0000 1011  Write AD1GAIN register  [16-Bit]


// Read Register
#define AD7719_READ_MODE_REG            0x41    // 0100 0001  read MODE register      [8-Bit]
#define AD7719_READ_AD0CON_REG          0x42    // 0100 0010  read AD0CON register    [8-Bit]
#define AD7719_READ_AD1CON_REG          0x43    // 0100 0011  read AD1CON register    [8-Bit]
#define AD7719_READ_FILT_REG            0x44    // 0100 0100  read FILTER register    [8-Bit]
#define AD7719_READ_IOCON_REG           0x47    // 0100 0111  read IOCON register     [16-Bit]
#define AD7719_READ_AD0OFFS_REG         0x48    // 0100 1000  read AD0OFFS register   [24-Bit]
#define AD7719_READ_AD1OFFS_REG         0x49    // 0100 1001  read AD1OFFS register   [16-Bit]
#define AD7719_READ_AD0GAIN_REG         0x4A    // 0100 1010  read AD0GAIN register   [24-Bit]
#define AD7719_READ_AD1GAIN_REG         0x4B    // 0100 1011  read AD1GAIN register   [16-Bit]

// Read only Register
#define AD7719_READ_STATUS_REG          0x40    // 0100 0000  read STATUS register  [    8 Bit]
#define AD7719_READ_AD0DATA_REG         0x45    // 0100 0101  read AD0DATA register [16/24 Bit]
#define AD7719_READ_AD1DATA_REG         0x46    // 0100 0110  read AD1DATA register [16    Bit]
#define AD7719_READ_ID_REG              0x4F    // 0100 1111  read ID register      [    8 Bit]



/*============================*/
/* Default values of register */
/*============================*/
#define AD7719_MODE_DEFAULT             0x00        // 0000 0000  Default Value MODE register             [8-Bit]
#define AD7719_AD0CON_DEFAULT           0x07        // 0000 0111  Default Value AD0CON register           [8-Bit]
#define AD7719_AD1CON_DEFAULT           0x01        // 0000 0001  Default Value AD1CON register           [8-Bit]
#define AD7719_FILT_DEFAULT             0x45        // 0100 0101  Default Value FILTER register           [8-Bit]
#define AD7719_IOCON_DEFAULT            0x0000      // 0000 0000 0000 0000  Default Value IOCON register  [16-Bit]



/*--------------------*/
/* Mode Register Bits */
/*--------------------*/
#define AD7719_MODE_REGBIT_BUF          0x40        // Erklärung
#define AD7719_MODE_REGBIT_CHCON        0x10        // Erklärung



class AD7719
{
public:
  ~AD7719();
  bool begin(uint8_t cs = SS, SPIClass *theSPI = &SPI);
  bool begin(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs);

  uint8_t getStatus();

  uint8_t getMode();
  void setMode(uint8_t mode);
  bool getBuffer();
  void setBuffer(bool isbuffered);

  int readADC(uint8_t channel);
  int readADCDifference(uint8_t differential);

private:
  Adafruit_SPIDevice *spi_dev = NULL;
  uint8_t _cs;
  uint8_t _mode;
  bool _isbuffered;
};



















#endif