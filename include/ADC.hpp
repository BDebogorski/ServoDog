#include <Wire.h>

class ADC
{
    private:

    int leftAddress;
    int rightAddress;

    public:

    ADC(int leftAddress, int rightAddress);

    int getLeftData();

};