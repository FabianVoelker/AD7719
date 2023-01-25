#include <Arduino.h>
#include <AD7719.h>

AD7719 myADC;

#define CS 10

void setup()
{
    // ADC Initialisieren
    myADC.begin(CS);

    // 200uA Current Source IEXE2 to IOUT1
    myADC.setIEXE2Direction(AD7719_IEXE2_TO_IOUT1);

    // Enable Current Source IEXC2
    myADC.enableIEXC2(true);
}

void loop()
{

}