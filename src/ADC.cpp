#include <ADC.hpp>

ADC::ADC(int leftAddress, int rightAddress)
{
    Wire.begin(leftAddress);
    Wire.begin(rightAddress);

    this->leftAddress = leftAddress;
    this->rightAddress = rightAddress;
}

int ADC::getLeftData()
{
    Wire.beginTransmission(leftAddress);
    Wire.write(0b00010000);
    Wire.endTransmission();

    Wire.requestFrom(leftAddress, 1);
    return Wire.available();
}