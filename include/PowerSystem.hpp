#pragma once

class PowerSystem
{
    private:

    int enablePin;
    int voltagePin;

    float minBatteryVoltage;
    float maxBatteryVoltage;
    float divisorRate;

    float batteryVoltage;
    float batteryLevel;

    public:

    PowerSystem(int enablePin, int voltagePin, float minBatteryVoltage, float maxBatteryVoltage, float divisorRate);

    void on();
    void off();
    float getBatteryVoltage();
    int getBatteryLevel();
    bool isEmpty();
};