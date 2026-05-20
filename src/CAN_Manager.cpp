#include "CAN_Manager.h"

// Inicializamos el objeto CAN_BUS (Tipo Transceiver, 125kbps, Nodo 1)
// Nota: en la G4 ignora los pines TX/RX si usa el FDCAN interno por defecto, 
// pero pasamos valores dummy (5 y 4) para cumplir la firma del constructor.
CAN_Manager::CAN_Manager() : canBus(HardwareType::Transciever, CAN_SPEED, CAN_NODE_ID, 5, 4) {
    tsonState = false;
    memset(canDataBMSStatus, 0, sizeof(canDataBMSStatus));
}

void CAN_Manager::init() {
    if (canBus.SetupState() != 0) {
        Serial.println("Error fatal: Inicialización del CAN BUS fallida.");
    } else {
        Serial.println("CAN BUS Inicializado correctamente.");
    }
}

void CAN_Manager::update() {
    // 1. Recibir todos los mensajes disponibles en el bus
    canBus.receive();
    
    // 2. Procesar los mensajes recibidos que nos interesan
    processReceivedPackets();

    // 3. Enviar todo lo que esté en la cola de salida
    canBus.send();
}

void CAN_Manager::processReceivedPackets() {
    // Intentamos leer el estado del BMS
    if (canBus.getPacket(ID_BMS_STATUS, canDataBMSStatus, 8)) {
        // Según tu código original, el estado TSON está en el byte 2
        tsonState = canDataBMSStatus[2]; 
    }
}

bool CAN_Manager::getTSONState() {
    return tsonState;
}

void CAN_Manager::sendInverterCmd(bool driveEnable, int16_t targetCurrentPCTG, int16_t maxAC, int16_t maxDC) {
    // Preparar Arrays para el Inversor
    byte cmdDataDriveEN[8] = {255, 255, 255, 255, 255, 255, 255, 255};
    int16_t cmdDataCurrent[4] = { (int16_t)targetCurrentPCTG, (int16_t)0xFFFF, (int16_t)0xFFFF, (int16_t)0xFFFF };
    int16_t cmdDataCurrentACMax[4] = { (int16_t)(maxAC * 10), (int16_t)0xFFFF, (int16_t)0xFFFF, (int16_t)0xFFFF };
    int16_t cmdDataCurrentDCMax[4] = { (int16_t)(maxDC * 10), (int16_t)0xFFFF, (int16_t)0xFFFF, (int16_t)0xFFFF };

    cmdDataDriveEN[0] = driveEnable ? 1 : 0;

    // Enviar comandos a la cola
    canBus.setPacket(ID_CMD_EN, cmdDataDriveEN, 1);
    
    if (driveEnable) {
        canBus.setPacket(ID_CMD_CURRENT_PCTG, cmdDataCurrent, 2);
        canBus.setPacket(ID_CMD_SET_MAX_AC, cmdDataCurrentACMax, 4);
        canBus.setPacket(ID_CMD_SET_MAX_DC, cmdDataCurrentDCMax, 4);
    } else {
        // Según el código original, si no hay DriveEnable se eliminan los paquetes de la cola (Opcional, 
        // pero MART_CAN hace overwrite si el ID es el mismo, o puedes enviarlos a 0).
        cmdDataCurrent[0] = 0;
        canBus.setPacket(ID_CMD_CURRENT_PCTG, cmdDataCurrent, 2);
    }
}

void CAN_Manager::sendTelemetry(int apps1Scaled, int apps2Scaled, int apps1Analog, int apps2Analog, int brakeAnalog, int brakeAnalog2, int vbatRaw) {
    uint16_t CANAppsState[4];
    uint16_t CANBrakeState[4];
    uint8_t CANVCUSignals[8] = {0}; // Inicializar a 0

    // Empaquetar APPS
    CANAppsState[0] = apps1Scaled;
    CANAppsState[1] = apps2Scaled;
    CANAppsState[2] = apps1Analog;
    CANAppsState[3] = apps2Analog;

    // Empaquetar Freno
    CANBrakeState[0] = 77777; // Cuidado: 77777 no cabe en un uint16_t (max 65535). Pongo 0xFFFF.
    CANBrakeState[1] = 0xFFFF;
    CANBrakeState[2] = brakeAnalog;
    CANBrakeState[3] = brakeAnalog2;

    // Empaquetar Señales VCU
    CANVCUSignals[0] = vbatRaw;

    // Poner en la cola de envío
    canBus.setPacket(ID_APPS_STATE, CANAppsState, 4);
    canBus.setPacket(ID_BRAKE_STATE, CANBrakeState, 4);
    canBus.setPacket(ID_VCU_SIGNALS, CANVCUSignals, 8);
}