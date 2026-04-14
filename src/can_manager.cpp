#include "can_manager.h"
#include "shared_data.h"
#include <Arduino.h>
#include <MART_CAN.h> // Tu librería original de Arduino

// --- DEFINICIÓN DE IDs (Igual que en tu código original) ---
const uint32_t idNode = 1;
const unsigned long int idBMSStatus = 10;
const unsigned long int idAPPSState = 1163;
const unsigned long int idBrakeState = 1164;
const unsigned long int idVCUSignals = 1166;
const int canSendingPeriod = 100;

// Función original para combinar IDs
uint32_t combineInts(uint32_t int1, uint32_t int2) {
    return (int1 << 5) | int2; // Recreación matemática (ej: 12 y 1 = 385)
}

uint32_t idCmdRPM = combineInts(3, idNode);
uint32_t idCmdEN = combineInts(12, idNode);
uint32_t idCmdCurrent = combineInts(1, idNode);
uint32_t idCmdCurrentPCTG = combineInts(5, idNode);
uint32_t idCmdSetMaxACCurrent = combineInts(8, idNode);
uint32_t idCmdSetMaxDCCurrent = combineInts(10, idNode);

// --- INICIALIZACIÓN DE LA LIBRERÍA ---
CAN_BUS CAN(HardwareType::Transciever, 125, 1);

void can_task(void *pvParameters) {
    Serial.println("CAN_Task: Iniciando en el Núcleo 0...");

    // Configuración de los timers de envío de tu código original
    CAN.setPacketTimer(idCmdEN, canSendingPeriod);
    CAN.setPacketTimer(idCmdCurrentPCTG, canSendingPeriod);
    CAN.setPacketTimer(idCmdCurrent, canSendingPeriod);
    CAN.setPacketTimer(idCmdSetMaxACCurrent, canSendingPeriod);
    CAN.setPacketTimer(idCmdSetMaxDCCurrent, canSendingPeriod);

    while (1) {
        // 1. Recibir datos del bus CAN
        CAN.receive();
        
        // Ejemplo: Leer estado del BMS (id 10)
        byte canDataBMSStatus[8];
        CAN.getPacket(idBMSStatus, canDataBMSStatus, 8);

        // 2. Leer las órdenes que el Núcleo 1 (Vehicle Control) ha calculado
        InverterCmd localCmd;
        VCU_Telemetry localVCU;
        
        if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            localCmd = cmdData;
            localVCU = vcuData;
            xSemaphoreGive(dataMutex);
        }

        // 3. Lógica de control del Inversor usando MART_CAN
        if (localCmd.driveEnable) {
            byte cmdDataDriveEN[8] = { (byte)1, 255, 255, 255, 255, 255, 255, 255 };
            int32_t cmdDataRPM[2] = { localCmd.RPMtarget, (int32_t)0xFFFFFFFF };
            int16_t cmdDataCurrent[4] = { (int16_t)localCmd.currentTarget, -1, -1, -1 };
            int16_t cmdDataCurrentACMax[4] = { (int16_t)(localCmd.cfgCurrentACMAX * 10), -1, -1, -1 };
            int16_t cmdDataCurrentDCMax[4] = { (int16_t)(localCmd.cfgCurrentDCMAX * 10), -1, -1, -1 };

            if (localCmd.ctrlByPctg) {
                CAN.setPacket(idCmdCurrentPCTG, cmdDataCurrent, 2);
                CAN.DataOUT.removePacket(idCmdCurrent);
            } else {
                CAN.setPacket(idCmdCurrent, cmdDataCurrent, 2);
                CAN.DataOUT.removePacket(idCmdCurrentPCTG);
            }
            
            CAN.setPacket(idCmdSetMaxACCurrent, cmdDataCurrentACMax, 4);
            CAN.setPacket(idCmdSetMaxDCCurrent, cmdDataCurrentDCMax, 4);
            CAN.setPacket(idCmdEN, cmdDataDriveEN, 1);
        } 
        else {
            // Si el R2D no está activo, detenemos los paquetes de tracción
            CAN.DataOUT.removePacket(idCmdEN);
            CAN.DataOUT.removePacket(idCmdCurrentPCTG);
            CAN.DataOUT.removePacket(idCmdCurrent);
            CAN.DataOUT.removePacket(idCmdSetMaxACCurrent);
            CAN.DataOUT.removePacket(idCmdSetMaxDCCurrent);
        }

        // 4. Enviar datos de estado general (Pedales, etc)
        uint16_t CANAppsState[4] = { localVCU.apps1_scaled, localVCU.apps2_scaled, localVCU.apps1_analog, localVCU.apps2_analog };
        uint16_t CANBrakeState[4] = { 777, 777, (uint16_t)localVCU.brake_analog, (uint16_t)localVCU.brake2_analog };
        uint8_t CANVCUSignals[8] = { (uint8_t)localVCU.vbat_raw, 0 }; // Rellenar con ceros el resto

        CAN.setPacket(idAPPSState, CANAppsState, 4);
        CAN.setPacket(idBrakeState, CANBrakeState, 4);
        CAN.setPacket(idVCUSignals, CANVCUSignals, 8);

        // 5. Enviar al bus físico y ceder tiempo a RTOS
        CAN.send();
        vTaskDelay(pdMS_TO_TICKS(10)); // Bucle a 100Hz
    }
}