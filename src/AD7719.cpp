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


bool AD7719::getBuffered()
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

void AD7719::setBuffered(bool isbuffered)
{
    _mode = getMode();
    _isbuffered = isbuffered;
    if(_isbuffered)
    {
        _mode |= AD7719_MODE_REGBIT_BUF;
    }
    else
    {
        _mode &= ~AD7719_MODE_REGBIT_BUF;
    }
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
}


bool AD7719::getChannelConfiguration()
{
    uint8_t cmd, mode;
    cmd = AD7719_READ_MODE_REG;
    spi_dev->write_then_read(&cmd,1,&mode,1);
    if(mode & AD7719_MODE_REGBIT_CHCON)
    {
        _channelconfig = AD7719_MAIN_PSEUDODIFF;
        return _channelconfig;
    }
    else
    {
        _channelconfig = AD7719_MAIN_FULLYDIFF;
        return _channelconfig;
    }
}

void AD7719::setChannelConfiguration(bool channelconfig)
{
    _mode = getMode();
    _channelconfig = channelconfig;
    if(_channelconfig)
    {
        _mode |= AD7719_MODE_REGBIT_CHCON;
    }
    else
    {
        _mode &= ~AD7719_MODE_REGBIT_CHCON;
    }
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
}


bool AD7719::getOscillatorPowerDown()
{
    uint8_t cmd, mode;
    cmd = AD7719_READ_MODE_REG;
    spi_dev->write_then_read(&cmd,1,&mode,1);
    if(mode & AD7719_MODE_REGBIT_OSCPD)
    {
        _opd = AD7719_OSCI_STANDBY;
        return _opd;
    }
    else
    {
        _opd = AD7719_OSCI_RUN;
        return _opd;
    }
}

void AD7719::setOscillatorPowerDown(bool opd)
{
    _mode = getMode();
    _opd = opd;
    if(_opd)
    {
        _mode |= AD7719_MODE_REGBIT_OSCPD;
    }
    else
    {
        _mode &= ~AD7719_MODE_REGBIT_OSCPD;
    }
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
}

u_int8_t AD7719::getADCMode()
{
    uint8_t cmd, mode;
    cmd = AD7719_READ_MODE_REG;
    spi_dev->write_then_read(&cmd,1,&mode,1);
    _adcmode = (mode & AD7719_MODE_REGBITS);
    return _adcmode;
}

void AD7719::setADCMode(uint8_t adcmode)
{
    _mode = getMode();
    switch (adcmode)
    {
    case AD7719_POWERDOWNMODE:
      _adcmode = AD7719_POWERDOWNMODE;
      break;
    case AD7719_IDLEMODE:
      _adcmode = AD7719_IDLEMODE;
      break; 
    case AD7719_SINGLECONVMODE:    
      _adcmode = AD7719_SINGLECONVMODE; 
      break;
    case AD7719_CONTINUOUSMODE:    
      _adcmode = AD7719_CONTINUOUSMODE;  
      break;
    case AD7719_INTZEROSCCAL:    
      _adcmode = AD7719_INTZEROSCCAL;
      break;
    case AD7719_INTFULLSCCAL:    
      _adcmode = AD7719_INTFULLSCCAL;
      break;
    case AD7719_SYSTEMZEROSCCAL:    
      _adcmode = AD7719_SYSTEMZEROSCCAL;
      break;
    case AD7719_SYSTEMFULLSCCAL:    
      _adcmode = AD7719_SYSTEMFULLSCCAL;
      break;
    default:
      _adcmode = AD7719_IDLEMODE;
      return;
      break;
    }
    _mode&=0xF8;
    _mode+=_adcmode;
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getMainADCControl(void)
{
    uint8_t cmd, maincontrol;
    cmd = AD7719_READ_AD0CON_REG;
    spi_dev->write_then_read(&cmd,1,&maincontrol,1);
    return maincontrol;
}

void AD7719::setMainADCControl(uint8_t maincontrol)
{
    _maincontrol = maincontrol;
    uint8_t cmd[2] = {AD7719_WRITE_AD0CON_REG,_maincontrol};
    spi_dev->write(cmd,2);
}
  

bool AD7719::isMainADCEnable(void)
{
    uint8_t cmd, maincontrol;
    cmd = AD7719_READ_AD0CON_REG;
    spi_dev->write_then_read(&cmd,1,&maincontrol,1);
    if(maincontrol & AD7719_CONTROL_REGBIT_AD0EN)
    {
        _ismainenabled = true;
        return _ismainenabled;
    }
    else
    {
        _ismainenabled = false;
        return _ismainenabled;
    }
}

void AD7719::enableMainADC(void)
{
    _maincontrol = getMainADCControl();
    _maincontrol |= AD7719_CONTROL_REGBIT_AD0EN;
    uint8_t cmd[2] = {AD7719_WRITE_AD0CON_REG,_maincontrol};
    spi_dev->write(cmd,2);
}

void AD7719::disableMainADC(void)
{
    _maincontrol = getMainADCControl();
    _maincontrol &= ~AD7719_CONTROL_REGBIT_AD0EN;
    uint8_t cmd[2] = {AD7719_WRITE_AD0CON_REG,_maincontrol};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getMainADCResolution(void)
{
    uint8_t cmd, mainresulotion;
    cmd = AD7719_READ_AD0CON_REG;
    spi_dev->write_then_read(&cmd,1,&mainresulotion,1);
    if(mainresulotion & AD7719_CONTROL_REGBIT_WL)
    {
        _mainresolution = 16;
        return _mainresolution;
    }
    else
    {
        _mainresolution = 24;
        return _mainresolution;
    }
}

void AD7719::setMainADCResolution(uint8_t resolution)
{
    _maincontrol = getMainADCControl();
    switch (resolution)
    {
    case AD7719_MAINADC_RESOLUTION_24BIT:
      _maincontrol &= ~AD7719_CONTROL_REGBIT_WL;
      break;
    case AD7719_MAINADC_RESOLUTION_16BIT:
      _maincontrol |= AD7719_CONTROL_REGBIT_WL;
      break; 
    case 24:    
      _maincontrol &= ~AD7719_CONTROL_REGBIT_WL;
      break;
    case 16:    
      _maincontrol |= AD7719_CONTROL_REGBIT_WL; 
      break;
    default:
      _maincontrol &= ~AD7719_CONTROL_REGBIT_WL;
      break;
    }
    uint8_t cmd[2] = {AD7719_WRITE_AD0CON_REG,_maincontrol};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getMainADCChannelSelection(void)
{
    uint8_t modeReg = getMode();
    uint8_t cmd, channelselection;
    cmd = AD7719_READ_AD0CON_REG;
    spi_dev->write_then_read(&cmd,1,&channelselection,1);

    channelselection &= AD7719_CONTROL_REGBIT_CHSEL;
    modeReg &= AD7719_MODE_REGBIT_CHCON;
    modeReg = modeReg << 2;
    channelselection |= modeReg;
    channelselection = channelselection >> 4;

    return channelselection;
}

void AD7719::setMainADCChannelSelection(uint8_t channelselection)
{
    _maincontrol = getMainADCControl();
    _mode = getMode();

    switch (channelselection)
    {
    case AD7719_MAINADC_CHSEL_0_AIN1_AIN2:
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH1;
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH0;
      break;
    case AD7719_MAINADC_CHSEL_0_AIN3_AIN4:
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH1;
      _maincontrol |= AD7719_CONTROL_REGBIT_CH0;
      break; 
    case AD7719_MAINADC_CHSEL_0_AIN2_AIN2:    
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _maincontrol |= AD7719_CONTROL_REGBIT_CH1;
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH0;
      break;
    case AD7719_MAINADC_CHSEL_0_AIN3_AIN2:    
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _maincontrol |= AD7719_CONTROL_REGBIT_CH1;
      _maincontrol |= AD7719_CONTROL_REGBIT_CH0;
      break;
    case AD7719_MAINADC_CHSEL_1_AIN1_AIN4:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH1;
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH0;
      break;
    case AD7719_MAINADC_CHSEL_1_AIN3_AIN4:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH1;
      _maincontrol |= AD7719_CONTROL_REGBIT_CH0;
      break;
    case AD7719_MAINADC_CHSEL_1_AIN4_AIN4:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _maincontrol |= AD7719_CONTROL_REGBIT_CH1;
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH0;
      break;
    case AD7719_MAINADC_CHSEL_1_AIN2_AIN4:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _maincontrol |= AD7719_CONTROL_REGBIT_CH1;
      _maincontrol |= AD7719_CONTROL_REGBIT_CH0;
      break;
    default:
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH1;
      _maincontrol &= ~AD7719_CONTROL_REGBIT_CH0;
      break;
    }
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
    uint8_t cmd[2] = {AD7719_WRITE_AD0CON_REG,_maincontrol};
    spi_dev->write(cmd,2);
}


bool AD7719::getMainADCPolarity(void)
{
    uint8_t cmd, maincontrol;
    cmd = AD7719_READ_AD0CON_REG;
    spi_dev->write_then_read(&cmd,1,&maincontrol,1);
    if(maincontrol & AD7719_CONTROL_REGBIT_UB)
    {
        _polarity = true;
        return AD7719_MAINADC_POL_UNIPOLAR
    }
    else
    {
        _polarity = false;
        return AD7719_MAINADC_POL_BIPOLAR;
    }
}

void AD7719::setMainADCPolarity(bool polarity)
{
    _maincontrol = getMainADCControl();
    if(polarity)
    {
        _maincontrol |= AD7719_CONTROL_REGBIT_UB;
    }
    else
    {
        _maincontrol &= ~AD7719_CONTROL_REGBIT_UB;
    }
    uint8_t cmd[2] = {AD7719_WRITE_AD0CON_REG,_maincontrol};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getMainADCInputRange(void)
{
    uint8_t cmd, maincontrol;
    cmd = AD7719_READ_AD0CON_REG;
    spi_dev->write_then_read(&cmd,1,&maincontrol,1);
    _adcmode = (maincontrol & AD7719_CONTROL_REGBITS_RANGE);
    return _adcmode;
}

void AD7719::setMainADCInputRange(uint8_t inputrange)
{
    _maincontrol = getMainADCControl();

    switch (inputrange)
    {
    case AD7719_MAINADC_RANGE_20mV:
      _inputrange = AD7719_MAINADC_RANGE_20mV;
      break;
    case AD7719_MAINADC_RANGE_40mV:
      _inputrange = AD7719_MAINADC_RANGE_40mV;
      break; 
    case AD7719_MAINADC_RANGE_80mV:    
      _inputrange = AD7719_MAINADC_RANGE_80mV; 
      break;
    case AD7719_MAINADC_RANGE_160mV:    
      _inputrange = AD7719_MAINADC_RANGE_160mV;  
      break;
    case AD7719_MAINADC_RANGE_320mV:    
      _inputrange = AD7719_MAINADC_RANGE_320mV;
      break;
    case AD7719_MAINADC_RANGE_640mV:    
      _inputrange = AD7719_MAINADC_RANGE_640mV;
      break;
    case AD7719_MAINADC_RANGE_1V28:    
      _inputrange = AD7719_MAINADC_RANGE_1V28;
      break;
    case AD7719_MAINADC_RANGE_2V56:    
      _inputrange = AD7719_MAINADC_RANGE_2V56;
      break;
    default:
      _inputrange = AD7719_MAINADC_RANGE_2V56;
      return;
      break;
    }
    _maincontrol&=0xF8;
    _maincontrol+=_inputrange;
    uint8_t cmd[2] = {AD7719_WRITE_AD0CON_REG,_maincontrol};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getAuxADCControl(void)
{
    uint8_t cmd, auxcontrol;
    cmd = AD7719_READ_AD1CON_REG;
    spi_dev->write_then_read(&cmd,1,&auxcontrol,1);
    return auxcontrol;
}

void AD7719::setAuxADCControl(uint8_t auxcontrol)
{
    _auxcontrol = auxcontrol;
    uint8_t cmd[2] = {AD7719_WRITE_AD1CON_REG,_auxcontrol};
    spi_dev->write(cmd,2);
}

  
bool AD7719::isAuxADCEnable(void)
{
    uint8_t cmd, auxcontrol;
    cmd = AD7719_READ_AD1CON_REG;
    spi_dev->write_then_read(&cmd,1,&auxcontrol,1);
    if(auxcontrol & AD7719_AUXCONTROL_REGBIT_AD1EN)
    {
        _isauxnabled = true;
        return _isauxnabled;
    }
    else
    {
        _isauxnabled = false;
        return _isauxnabled;
    }
}

void AD7719::enableAuxADC(void)
{
    _auxcontrol = getAuxADCControl();
    _auxcontrol |= AD7719_AUXCONTROL_REGBIT_AD1EN;
    uint8_t cmd[2] = {AD7719_WRITE_AD1CON_REG,_auxcontrol};
    spi_dev->write(cmd,2);
}

void AD7719::disableAuxADC(void)
{
    _auxcontrol = getAuxADCControl();
    _auxcontrol |= AD7719_AUXCONTROL_REGBIT_AD1EN;
    uint8_t cmd[2] = {AD7719_WRITE_AD1CON_REG,_auxcontrol};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getAuxADCChannelSelection(void)
{
    uint8_t modeReg = getMode();
    uint8_t cmd, channelselection;
    cmd = AD7719_READ_AD1CON_REG;
    spi_dev->write_then_read(&cmd,1,&channelselection,1);

    channelselection &= AD7719_AUXCONTROL_REGBIT_CHSEL;
    modeReg &= AD7719_MODE_REGBIT_CHCON;
    modeReg = modeReg << 3;
    channelselection |= modeReg;
    channelselection = channelselection >> 4;

    return channelselection;
}

void AD7719::setAuxADCChannelSelection(uint8_t channelselection)
{
    _auxcontrol = getAuxADCControl();
    _mode = getMode();

    switch (channelselection)
    {
    case AD7719_AUXADC_CHSEL_0_AIN3_AGND:
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_0_AIN4_AGND:
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH0;
      break; 
    case AD7719_AUXADC_CHSEL_0_AIN5_AIN6:    
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_0_TEMP:    
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_0_AGND_AGND:    
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_1_AIN5_AGND:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_1_AIN6_AGND:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_1_AIN5_AIN6:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_1_TEMP:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    case AD7719_AUXADC_CHSEL_1_AGND_AGND:    
      _mode |= AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol |= AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    default:
      _mode &= ~AD7719_MODE_REGBIT_CHCON;    
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH2;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH1;
      _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_CH0;
      break;
    }
    uint8_t cmd[2] = {AD7719_WRITE_MODE_REG,_mode};
    spi_dev->write(cmd,2);
    uint8_t cmd[2] = {AD7719_WRITE_AD1CON_REG,_auxcontrol};
    spi_dev->write(cmd,2);
}


bool AD7719::getMainADCPolarity(void)
{
    uint8_t cmd, auxcontrol;
    cmd = AD7719_READ_AD1CON_REG;
    spi_dev->write_then_read(&cmd,1,&auxcontrol,1);
    if(auxcontrol & AD7719_AUXCONTROL_REGBIT_UB)
    {
        _polarity = true;
        return AD7719_AUXADC_POL_UNIPOLAR;
    }
    else
    {
        _polarity = false;
        return AD7719_AUXADC_POL_BIPOLAR;
    }
}

void AD7719::setMainADCPolarity(bool polarity)
{
    _auxcontrol = getAuxADCControl();
    if(polarity)
    {
        _auxcontrol |= AD7719_AUXCONTROL_REGBIT_UB;
    }
    else
    {
        _auxcontrol &= ~AD7719_AUXCONTROL_REGBIT_UB;
    }
    uint8_t cmd[2] = {AD7719_WRITE_AD1CON_REG,_auxcontrol};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getAuxADCInputRange(void)
{
    uint8_t cmd, auxcontrol;
    cmd = AD7719_READ_AD1CON_REG;
    spi_dev->write_then_read(&cmd,1,&auxcontrol,1);
    _adcmode = (auxcontrol & AD7719_AUXCONTROL_REGBIT_RANGE);
    return _adcmode;
}

void AD7719::setAuxADCInputRange(uint8_t inputrange)
{
    _auxcontrol = getAuxADCControl();

    switch (inputrange)
    {
    case AD7719_AUXADC_RANGE_REFIN05:
      _inputrange = AD7719_AUXADC_RANGE_REFIN05;
      break;
    case AD7719_AUXADC_RANGE_REFIN2:
      _inputrange = AD7719_AUXADC_RANGE_REFIN2;
      break; 
    default:
      _inputrange = AD7719_AUXADC_RANGE_REFIN2;
      return;
      break;
    }
    _auxcontrol&=0xFE;
    _auxcontrol+=_inputrange;
    uint8_t cmd[2] = {AD7719_WRITE_AD1CON_REG,_auxcontrol};
    spi_dev->write(cmd,2);
}


uint8_t AD7719::getADCFilter(void)
{
    uint8_t cmd, filter;
    cmd = AD7719_READ_FILT_REG;
    spi_dev->write_then_read(&cmd,1,&filter,1);
    return filter;
}

void AD7719::setADCFilter(uint8_t filter)
{
    if(filter<AD7719_FILT_MIN)filter=AD7719_FILT_MIN;
    if(filter>AD7719_FILT_MAX)filter=AD7719_FILT_MAX;
    _filter = filter;
    uint8_t cmd[2] = {AD7719_WRITE_FILT_REG,_filter};
    spi_dev->write(cmd,2);
}
