#include "Sensors.h"

Sensors::Sensors() : adc(ADC_VREF, PIN_SPI_CS) {}

void Sensors::init() {
    pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH);

    SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
    SPI.begin();
    SPI.beginTransaction(settings);

    configureAPPS();
}

void Sensors::configureAPPS() {
    apps1.valAnalogUP = 2270;   
    apps1.valAnalogDOWN = 2030; 
    apps2.valAnalogUP = 924;    
    apps2.valAnalogDOWN = 1920; 
    
    apps1.valScaledUP = apps2.valScaledUP = 0;
    apps1.valScaledDOWN = apps2.valScaledDOWN = 1000;
}

void Sensors::readAll() {
    stsBrake = adc.read(MCP3208::Channel::SINGLE_4);
    stsBrake2 = adc.read(MCP3208::Channel::SINGLE_5);
    apps1.valAnalog = adc.read(MCP3208::Channel::SINGLE_2);
    apps2.valAnalog = adc.read(MCP3208::Channel::SINGLE_3);
    stsVbatRAW = adc.read(MCP3208::Channel::SINGLE_6);

    apps1.valScaled = map(apps1.valAnalog, apps1.valAnalogUP, apps1.valAnalogDOWN, apps1.valScaledUP, apps1.valScaledDOWN);
    apps2.valScaled = map(apps2.valAnalog, apps2.valAnalogUP, apps2.valAnalogDOWN, apps2.valScaledUP, apps2.valScaledDOWN);
}

int Sensors::getAPPS1Scaled() { return apps1.valScaled; }
int Sensors::getAPPS2Scaled() { return apps2.valScaled; }
int Sensors::getBrakeAnalog() { return stsBrake2; }
int Sensors::getVbatRaw() { return stsVbatRAW; }

bool Sensors::isAPPSImplausible() {
    return (checkAPPSImplausibility(CFG_APPS_DIFF, 0) != 0);
}

int Sensors::checkAPPSImplausibility(int difMAX, int valDesc) {
    int val = abs(apps1.valScaled - apps2.valScaled);
    if ((apps1.valScaled <= valDesc) || (apps2.valScaled <= valDesc)) {
        return -1;
    } else if (val >= difMAX) {
        return 1;
    }
    return 0;
}