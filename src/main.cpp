#include <Arduino.h>
#include "Config.h"
#include "Sensors.h"
#include "R2D_Logic.h"
// #include "CAN_Manager.h" // Próximo módulo a crear

Sensors carSensors;
R2D_Logic r2dLogic;

// Variables globales de control
int RPMtarget = 0;
int currentTarget = 0;

void setup() {
    Serial.begin(115200);
    
    carSensors.init();
    r2dLogic.init();
    // canManager.init(); // Futuro
    
    Serial.println("Cockpit VCU Inicializado (STM32 Port)");
}

void loop() {
    // 1. Leer todas las entradas (Hardware)
    carSensors.readAll();
    bool btnStart = !digitalRead(PIN_START);
    bool isBrakePressed = (carSensors.getBrakeAnalog() >= CFG_BRAKE_TH);
    bool tsonState = true; // Aquí iría la lectura por CAN o Pin del TSON
    
    // 2. Procesar Lógica (Software)
    bool isR2D = r2dLogic.update(tsonState, btnStart, isBrakePressed);
    bool appsFault = carSensors.isAPPSImplausible();

    // 3. Cálculo de Consignas para el Inversor
    if (!isR2D || appsFault) {
        RPMtarget = 0;
        currentTarget = 0;
    } else {
        RPMtarget = map(carSensors.getAPPS1Scaled(), 1000, 0, 0, CFG_RPMAX * 10);
        currentTarget = carSensors.getAPPS2Scaled();
        if (currentTarget < 100) currentTarget = 0;
        if (currentTarget > 1000) currentTarget = 1000;
    }

    // 4. Enviar datos por CAN
    // canManager.sendInverterCmd(isR2D, RPMtarget, currentTarget);
    // canManager.sendTelemetry(...);
    
    delay(10); // O mejor usar temporizadores no bloqueantes
}