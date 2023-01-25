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
#define AD7719_WRITE_FILT_REG           0x04    // 0000 0100  Write FILTER register   [8-Bit]
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
#define AD7719_MODE_REGBIT_OSCPD        0x08        // Erklärung
#define AD7719_MODE_REGBITS             0x07        // Erklärung

#define AD7719_MAIN_PSEUDODIFF          1 
#define AD7719_MAIN_FULLYDIFF           0

#define AD7719_OSCI_STANDBY             1
#define AD7719_OSCI_RUN                 0

#define AD7719_POWERDOWNMODE            0
#define AD7719_IDLEMODE                 1
#define AD7719_SINGLECONVMODE           2
#define AD7719_CONTINUOUSMODE           3
#define AD7719_INTZEROSCCAL             4
#define AD7719_INTFULLSCCAL             5
#define AD7719_SYSTEMZEROSCCAL          6
#define AD7719_SYSTEMFULLSCCAL          7



/*------------------------*/
/*  AD0CON Register Bits  */
/*------------------------*/
#define AD7719_CONTROL_REGBIT_AD0EN       0x80        // Erklärung
#define AD7719_CONTROL_REGBIT_WL          0x40        // Erklärung
#define AD7719_CONTROL_REGBIT_CH1         0x20        // Erklärung
#define AD7719_CONTROL_REGBIT_CH0         0x10        // Erklärung
#define AD7719_CONTROL_REGBIT_CHSEL       0x30        // Erklärung
#define AD7719_CONTROL_REGBIT_UB          0x08        // Erklärung
#define AD7719_CONTROL_REGBITS_RANGE      0x07        // Erklärung

#define AD7719_MAINADC_RESOLUTION_24BIT   0
#define AD7719_MAINADC_RESOLUTION_16BIT   1

#define AD7719_MAINADC_CHSEL_0_AIN1_AIN2  0
#define AD7719_MAINADC_CHSEL_0_AIN3_AIN4  1
#define AD7719_MAINADC_CHSEL_0_AIN2_AIN2  2
#define AD7719_MAINADC_CHSEL_0_AIN3_AIN2  3
#define AD7719_MAINADC_CHSEL_1_AIN1_AIN4  4
#define AD7719_MAINADC_CHSEL_1_AIN3_AIN4  5
#define AD7719_MAINADC_CHSEL_1_AIN4_AIN4  6
#define AD7719_MAINADC_CHSEL_1_AIN2_AIN4  7

#define AD7719_MAINADC_POL_UNIPOLAR       1
#define AD7719_MAINADC_POL_BIPOLAR        0

#define AD7719_MAINADC_RANGE_20mV         0
#define AD7719_MAINADC_RANGE_40mV         1
#define AD7719_MAINADC_RANGE_80mV         2
#define AD7719_MAINADC_RANGE_160mV        3
#define AD7719_MAINADC_RANGE_320mV        4
#define AD7719_MAINADC_RANGE_640mV        5
#define AD7719_MAINADC_RANGE_1V28         6
#define AD7719_MAINADC_RANGE_2V56         7



/*------------------------*/
/*  AD1CON Register Bits  */
/*------------------------*/
#define AD7719_AUXCONTROL_REGBIT_AD1EN      0x80        // Erklärung
#define AD7719_AUXCONTROL_REGBIT_CH2        0x40        // Erklärung
#define AD7719_AUXCONTROL_REGBIT_CH1        0x20        // Erklärung
#define AD7719_AUXCONTROL_REGBIT_CH0        0x10        // Erklärung
#define AD7719_AUXCONTROL_REGBIT_CHSEL      0x70        // Erklärung
#define AD7719_AUXCONTROL_REGBIT_UB         0x08        // Erklärung
#define AD7719_AUXCONTROL_REGBIT_RANGE      0x01        // Erklärung

#define AD7719_AUXADC_CHSEL_0_AIN3_AGND     0
#define AD7719_AUXADC_CHSEL_0_AIN4_AGND     1
#define AD7719_AUXADC_CHSEL_0_AIN5_AIN6     2
#define AD7719_AUXADC_CHSEL_0_TEMP          3
#define AD7719_AUXADC_CHSEL_0_AGND_AGND     4
#define AD7719_AUXADC_CHSEL_1_AIN5_AGND     8
#define AD7719_AUXADC_CHSEL_1_AIN6_AGND     9
#define AD7719_AUXADC_CHSEL_1_AIN5_AIN6     10
#define AD7719_AUXADC_CHSEL_1_TEMP          11
#define AD7719_AUXADC_CHSEL_1_AGND_AGND     12

#define AD7719_AUXADC_POL_UNIPOLAR          1
#define AD7719_AUXADC_POL_BIPOLAR           0

#define AD7719_AUXADC_RANGE_REFIN2          1
#define AD7719_AUXADC_RANGE_REFIN05         0



/*------------------------*/
/*  Filter Register Bits  */
/*------------------------*/
#define AD7719_FILT_MIN         0x0D    // Erklärung
#define AD7719_FILT_MAX         0xFF    // Erklärung 



/*---------------------*/
/*  I/O Register Bits  */
/*---------------------*/
#define AD7719_IO_REGBIT_PSW2       0x8000        // Erklärung
#define AD7719_IO_REGBIT_PSW1       0x4000        // Erklärung
#define AD7719_IO_REGBIT_BO         0x1000        // Erklärung
#define AD7719_IO_REGBIT_I2PIN      0x0800        // Erklärung
#define AD7719_IO_REGBIT_I1PIN      0x0400        // Erklärung
#define AD7719_IO_REGBIT_I2EN       0x0200        // Erklärung
#define AD7719_IO_REGBIT_I1EN       0x0100        // Erklärung
#define AD7719_IO_REGBIT_P4DIR      0x0080        // Erklärung
#define AD7719_IO_REGBIT_P3DIR      0x0040        // Erklärung
#define AD7719_IO_REGBIT_P2EN       0x0020        // Erklärung
#define AD7719_IO_REGBIT_P1EN       0x0010        // Erklärung
#define AD7719_IO_REGBIT_P4DAT      0x0008        // Erklärung
#define AD7719_IO_REGBIT_P3DAT      0x0004        // Erklärung
#define AD7719_IO_REGBIT_P4DAT      0x0002        // Erklärung
#define AD7719_IO_REGBIT_P1DAT      0x0001        // Erklärung

#define AD7719_PSW_PWRGND           1
#define AD7719_PSW_STDIO            0



class AD7719
{
public:
  ~AD7719();
  bool begin(uint8_t cs = SS, SPIClass *theSPI = &SPI);
  bool begin(uint8_t sck, uint8_t mosi, uint8_t miso, uint8_t cs);

  uint8_t getStatus(void);

  uint8_t getMode(void);
  void setMode(uint8_t mode);

  bool getBuffered(void);
  void setBuffered(bool isbuffered);

  bool getChannelConfiguration(void);
  void setChannelConfiguration(bool channelconfig);

  bool getOscillatorPowerDown(void);
  void setOscillatorPowerDown(bool opd);

  uint8_t getADCMode(void);
  void setADCMode(uint8_t adcmode);


  uint8_t getMainADCControl(void);
  void setMainADCControl(uint8_t maincontrol);
  
  bool isMainADCEnable(void);
  void enableMainADC(void);
  void disableMainADC(void);

  uint8_t getMainADCResolution(void);
  void setMainADCResolution(uint8_t resolution);

  uint8_t getMainADCChannelSelection(void);
  void setMainADCChannelSelection(uint8_t channelselection);

  bool getMainADCPolarity(void);
  void setMainADCPolarity(bool polarity);

  uint8_t getMainADCInputRange(void);
  void setMainADCInputRange(uint8_t inputrange);

  
  uint8_t getAuxADCControl(void);
  void setAuxADCControl(uint8_t auxcontrol);
  
  bool isAuxADCEnable(void);
  void enableAuxADC(void);
  void disableAuxADC(void);

 
  uint8_t getAuxADCChannelSelection(void);
  void setAuxADCChannelSelection(uint8_t channelselection);

  bool getAuxADCPolarity(void);
  void setAuxADCPolarity(bool polarity);

  uint8_t getAuxADCInputRange(void);
  void setAuxADCInputRange(uint8_t inputrange);

  
  uint8_t getADCFilter(void);
  void setADCFilter(uint8_t filter);

  
  uint16_t getIOCON(void);
  void setIOCON(uint16_t iocon);

  bool getPowerSwitch2Control(void);
  void setPowerSwitch2Control(bool psw2);

  bool getPowerSwitch1Control(void);
  void setPowerSwitch1Control(bool psw1);

  bool isBurnoutCurrentEnabled(void);
  void enableBurnoutCurrent(bool burnout);

  bool getIEXE2Direction(void);
  void setIEXE2Direction(bool iexe2);

  bool getIEXE1Direction(void);
  void setIEXE1Direction(bool iexe1);

  bool isIEXC2Enabled(void);
  void enableIEXC2(bool iexc2);

  bool isIEXC1Enabled(void);
  void enableIEXC1(bool iexc1);

  bool getP4PinMode(void);
  void setP4PinMode(bool p4dir);
  bool getP4State(void);

  bool getP3PinMode(void);
  void setP3PinMode(bool p3dir);
  bool getP3State(void);

  bool getP2OutputFunction(void);
  void setP2OutputFunction(bool p2out);

  bool getP1OutputFunction(void);
  void setP1OutputFunction(bool p1out);


  uint32_t readMainADC(void);
  uint16_t readAuxADC(void);


  uint8_t getRev(void);


private:
  Adafruit_SPIDevice *spi_dev = NULL;
  uint8_t _cs;
  uint8_t _mode;
  bool _isbuffered;
  bool _channelconfig;
  bool _opd;
  uint8_t _adcmode;
  uint8_t _maincontrol;
  bool _ismainenabled;
  uint8_t _mainresolution;
  uint8_t _channelselection;
  bool _polarity;
  uint8_t _inputrange;
  uint8_t _auxcontrol;
  bool _isauxnabled;
  uint8_t _filter;
  uint16_t _iocon;
  bool _psw2;
  bool _psw1;
  bool _burnout;
  bool _iexe2;
  bool _iexe1;
  bool _iexc2;
  bool _iexc1;
  bool _p4dir;
  bool _p3dir;
  bool _p2en;
  bool _p1en;
  bool _p4dat;
  bool _p3dat;
  bool _p2dat;
  bool _p1dat;
  uint8_t _id;
};


#endif