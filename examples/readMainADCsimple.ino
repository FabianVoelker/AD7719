#include <Arduino.h>
#include <AD7719.h>

AD7719 myADC;

#define CS 10

void setup()
{
    Serial.begin(115200);

    // ADC Initialisieren
    myADC.begin(CS);

    /* 1. IOCON-Register */

    // 200uA Current Source IEXE2 to IOUT1
    myADC.setIEXE2Direction(AD7719_IEXE2_TO_IOUT1);

    // Enable Current Source IEXC2
    myADC.enableIEXC2(true);

    /* 2. FILTER-Register */

    // Set filter to ~20Hz [50ms]
    myADC.setADCFilter(0x45);

    /* 3. FILTER-Register */

    // Set Main ADC to 24-Bit Resolution
    myADC.setMainADCResolution(AD7719_MAINADC_RESOLUTION_24BIT);

    // Set Main ADC Input to AIN1 and AIN2
    myADC.setMainADCChannelSelection(AD7719_MAINADC_CHSEL_0_AIN1_AIN2);

    // Set Main ADC to Unipolar 0x00 0000 to 0xFF FFFF
    myADC.setMainADCPolarity(AD7719_MAINADC_POL_UNIPOLAR);

    // Set Main ADC input range to 80mV
    myADC.setMainADCInputRange(AD7719_MAINADC_RANGE_80mV);

    // Enable Main ADC
    myADC.enableMainADC();

    /* 4. MODE-Register */

    // Set ADC in buffered mode
    myADC.setBuffered(true);

    // Set ADC in Fully differential Input mode
    myADC.setChannelConfiguration(AD7719_MAIN_FULLYDIFF);

    // Set ADC in Continous Measurement mode
    myADC.setADCMode(AD7719_SYSTEMFULLSCCAL);

    while(myADC.getADCMode()!=1)
    {
        delay(100);
    }
    myADC.setADCMode(AD7719_SYSTEMFULLSCCAL);

    while(myADC.getADCMode()!=1)
    {
        delay(100);
    }

    myADC.setADCMode(AD7719_CONTINUOUSMODE);
    delay(100);
    myADC.setADCMode(AD7719_CONTINUOUSMODE);
}

void loop()
{
    while(!(myADC.getStatus() && 0b10000000))    
    {
        delay(100);
    }

    uint32_t value = myADC.readMainADC();
    delay(1000);
}