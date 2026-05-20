#ifndef CAN_MANAGER_H
#define CAN_MANAGER_H

#include <Arduino.h>
#include "MART_CAN.h"
#include "Config.h"

class CAN_Manager {
public:
    CAN_Manager();
    void init();
    void update(); // Llama a receive y send en cada ciclo

    // --- Lecturas (Getters) ---
    bool getTSONState();

    // --- Envíos (Setters) ---
    void sendInverterCmd(bool driveEnable, int16_t targetCurrentPCTG, int16_t maxAC, int16_t maxDC);
    void sendTelemetry(int apps1Scaled, int apps2Scaled, int apps1Analog, int apps2Analog, int brakeAnalog, int brakeAnalog2, int vbatRaw);

private:
    CAN_BUS canBus;

    // Buffers de recepción
    byte canDataBMSStatus[8];
    bool tsonState;

    // Procesamiento interno
    void processReceivedPackets();
};

#endif // CAN_MANAGER_H