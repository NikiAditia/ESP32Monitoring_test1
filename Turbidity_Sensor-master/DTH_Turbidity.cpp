/*
 *  Turbidity sensor library.
*/
#include "DTH_Turbidity.h"  // Make sure to update the include path in the Arduino IDE

DTH_Turbidity::DTH_Turbidity()
{
    _sample = TURBIDITY_SAMPLE;
}

DTH_Turbidity::DTH_Turbidity(int sensorPin)
{
    _sample = TURBIDITY_SAMPLE;
    begin(sensorPin, -1);
}

DTH_Turbidity::DTH_Turbidity(int sensorPin, int powerPin)
{
    _sample = TURBIDITY_SAMPLE;
    begin(sensorPin, powerPin);
}

DTH_Turbidity::~DTH_Turbidity()
{
}

void DTH_Turbidity::selectionSort(float *buff, int len)
{
    int i, z, pos;
    float tmp;

    for (i = 0; i < (len - 1); i++)
    {
        pos = i;
        for (z = i + 1; z < len; z++)
        {
            if (buff[pos] > buff[z])
                pos = z;
        }
        if (pos != i)
        {
            tmp = buff[i];
            buff[i] = buff[pos];
            buff[pos] = tmp;
        }
    }
}

float DTH_Turbidity::getVoltageFilterAvg(int sample, int sensorPin)
{
    int i;
    float val_buff[sample];
    float val = 0.0;

    for (i = 0; i < sample; i++)
    {
        val_buff[i] = analogRead(_sensorPin);
        delayMicroseconds(10);
    }
    /* Sort buffer */
    selectionSort(val_buff, sample);
    /* get averages in the middle of the buffer */
    for (i = sample / 4; i < (3 * (sample / 4)); i++)
        val += val_buff[i];
    val = val / (sample / 2);
    val = (val / 4095.0) * 5.0; // Adjusted for 12-bit ADC
    return val;
}

/*
 *  Convert voltage to NTU
 *  NTU:     0 - 1000 NTU 
 *  Voltage: 0V - 5V
 */
float DTH_Turbidity::convertToNTU(float volt)
{
    float ntu = 0;

    // Ensure voltage is within the expected range
    if (volt < 0) volt = 0;
    if (volt > 5) volt = 5;

    // Map voltage to NTU (0V - 5V maps to 0 NTU - 1000 NTU)
    ntu = (volt / 5.0) * 1000.0;

    return ntu;
}

void DTH_Turbidity::setSampe(int sample)
{
    _sample = sample;
}

void DTH_Turbidity::begin(int sensorPin, int powerPin)
{
    _sensorPin = sensorPin;
    _powerPin = powerPin;
    pinMode(_sensorPin, INPUT_PULLUP);
    if (_powerPin != -1)
    {
        pinMode(_powerPin, OUTPUT);
        digitalWrite(_powerPin, LOW); /* Power off */
    }
}

float DTH_Turbidity::getVoltage(void)
{
    float volt;

    if (_powerPin != -1)
    {
        digitalWrite(_powerPin, HIGH); /* Power on */
        delay(100);
    }
    /* Read data */
    volt = getVoltageFilterAvg(_sample, _sensorPin);
    if (_powerPin != -1)
        digitalWrite(_powerPin, LOW); /* Power off */
    return volt;
}

float DTH_Turbidity::readTurbidity(void)
{
    float volt;

    volt = getVoltage();
    return convertToNTU(volt);
}
