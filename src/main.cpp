#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "R2D_Logic.h"
#include "CAN_Manager.h"

Sensors carSensors;
R2D_Logic r2dLogic;
CAN_Manager canManager;

int cfgCurrentACMAX = 190;
int cfgCurrentDCMAX = 60;
int currentTarget = 0;

void setup() {
    Serial.begin(115200);
    
    carSensors.init();
    r2dLogic.init();
    canManager.init();
    
    Serial.println("Cockpit VCU (STM32) Iniciado");
}

void loop() {
    // 1. Refrescar estado CAN (Procesa mensajes entrantes)
    canManager.update();

    // 2. Leer Sensores
    carSensors.readAll();
    bool btnStart = !digitalRead(PIN_START);
    bool isBrakePressed = (carSensors.getBrakeAnalog() >= CFG_BRAKE_TH);
    bool tsonState = canManager.getTSONState(); // Obtenido del BMS por CAN
    bool appsFault = carSensors.isAPPSImplausible();

    // 3. Máquina de Estados (R2D)
    bool isR2D = r2dLogic.update(tsonState, btnStart, isBrakePressed);

    // 4. Calcular consignas
    if (!isR2D || appsFault) {
        currentTarget = 0;
    } else {
        currentTarget = carSensors.getAPPS2Scaled();
        if (currentTarget < 100) currentTarget = 0;
        if (currentTarget > 1000) currentTarget = 1000;
    }

    // 5. Enviar comandos de Inversor vía CAN
    canManager.sendInverterCmd(isR2D, currentTarget, cfgCurrentACMAX, cfgCurrentDCMAX);

    // 6. Enviar Telemetría (APPS, Freno, VBat) vía CAN
    canManager.sendTelemetry(
        carSensors.getAPPS1Scaled(), carSensors.getAPPS2Scaled(),
        carSensors.getAPPS1Analog(), carSensors.getAPPS2Analog(), // <- Nota: El Getter getAPPS1Analog debes crearlo en Sensors.h
        carSensors.getBrakeAnalog(), carSensors.getBrakeAnalog(), 
        carSensors.getVbatRaw()
    );

    // Pequeño retardo para no saturar el bus CAN (Ajustar según necesidad o usar millis())
    delay(10); 
}