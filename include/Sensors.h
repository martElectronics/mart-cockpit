#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <SPI.h>
#include <Mcp320x.h>
#include "Config.h"

struct SensorData {
    int valAnalog;
    int valScaled;
    int valAnalogUP;
    int valAnalogDOWN;
    int valScaledUP;
    int valScaledDOWN;
    int range;
};

class Sensors {
public:
    Sensors();
    void init();
    void readAll();
    
    // Getters
    int getAPPS1Scaled();
    int getAPPS2Scaled();
    int getBrakeAnalog();
    bool isAPPSImplausible();
    int getVbatRaw();

private:
    MCP3208 adc;
    SensorData apps1;
    SensorData apps2;
    int stsBrake;
    int stsBrake2;
    int stsVbatRAW;

    void configureAPPS();
    int checkAPPSImplausibility(int difMAX, int valDesc);
};

#endif // SENSORS_H