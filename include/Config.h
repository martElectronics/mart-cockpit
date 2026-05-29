#pragma once
//==============================================================================
// Config.h — Configuración centralizada de la VCU.
// Toda la parametrización (pines, IDs CAN, límites, tiempos, ADC) vive aquí,
// como constantes constexpr en el namespace cfg.
//==============================================================================
#include <Arduino.h>
#include <Mcp320x.h>
#include "global.h"   // combineInts() (constexpr)

namespace cfg {

// --- CAN ---
constexpr unsigned CAN_SPEED_KBPS = 125;   // Velocidad del bus CAN (kbps).
constexpr int      NODE_ID        = 1;     // ID/perfil de nodo de la VCU.

// --- Pines (ESP32-S3) ---
constexpr uint8_t PIN_START       = 15;    // Pulsador Start (INPUT_PULLUP, activo a 0).
constexpr uint8_t PIN_BUZZER      = 8;     // Buzzer R2D.
constexpr uint8_t PIN_R2D_DIGITAL = 2;     // DriveEnable digital hacia el inversor.
constexpr uint8_t PIN_LED         = 48;    // LED NeoPixel de estado.
constexpr uint8_t LED_COUNT       = 1;
constexpr uint8_t PIN_SPI_CS      = 10;    // Chip-select del ADC MCP3208.

// --- ADC MCP3208 ---
constexpr uint16_t ADC_VREF = 3300;        // Vref del ADC (mV).
constexpr uint32_t ADC_CLK  = 160000;      // Reloj SPI del ADC (Hz).
constexpr MCP3208::Channel CH_APPS1  = MCP3208::Channel::SINGLE_2;
constexpr MCP3208::Channel CH_APPS2  = MCP3208::Channel::SINGLE_3;
constexpr MCP3208::Channel CH_BRAKE1 = MCP3208::Channel::SINGLE_4;
constexpr MCP3208::Channel CH_BRAKE2 = MCP3208::Channel::SINGLE_5;
constexpr MCP3208::Channel CH_VBAT   = MCP3208::Channel::SINGLE_6;

// --- Tiempos (ms salvo indicación) ---
constexpr uint32_t DEBUG_PERIOD_MS   = 500;
constexpr uint32_t INVERTER_WD_MS    = 1000;
constexpr uint32_t BUZZER_ON_MS      = 2000;
constexpr uint32_t BMS_WD_MS         = 2500;   // Sin trama del BMS -> SDC no presente (fail-safe).
constexpr uint32_t TASK_WDT_TIMEOUT_S = 2;     // Watchdog del micro (s).

// --- BMS por CAN (bms_master_26, rama testing) ---
constexpr uint32_t ID_BMS_STATUS = 10;   // ID 10 (0x0A): estado general del BMS (DLC 1, ~800 ms).
constexpr uint8_t  BMS_SDC_BIT   = 2;    // BMS_SDC = byte 0, bit 2 (SDC presente).

// --- Límites de control ---
constexpr int BRAKE_TH       = 550;   // Umbral de freno (cuentas ADC) para R2D.
constexpr int CURRENT_AC_MAX = 190;   // Corriente AC máx (Apk).
constexpr int CURRENT_DC_MAX = 60;    // Corriente DC máx (Adc).
constexpr int RPM_MAX        = 1500;  // ERPM target máximo.

// --- IDs CAN de comandos al inversor (combineInts(PID, NODE_ID)) ---
constexpr uint32_t ID_CMD_RPM            = combineInts(0x1C, NODE_ID);
constexpr uint32_t ID_CMD_EN             = combineInts(0x24, NODE_ID);
constexpr uint32_t ID_CMD_CURRENT_PCTG   = combineInts(0x1E, NODE_ID);
constexpr uint32_t ID_CMD_SET_MAX_AC     = combineInts(0x20, NODE_ID);
constexpr uint32_t ID_CMD_SET_MAX_DC     = combineInts(0x22, NODE_ID);

// --- IDs CAN de estado del inversor ---
constexpr uint32_t ID_STS_INV_2 = combineInts(0x02, NODE_ID);
constexpr uint32_t ID_STS_INV_4 = combineInts(0x04, NODE_ID);

// --- IDs CAN de telemetría publicada por la VCU ---
constexpr unsigned long ID_APPS_STATE  = 1163;
constexpr unsigned long ID_BRAKE_STATE = 1164;
constexpr unsigned long ID_VCU_SIGNALS = 1166;

} // namespace cfg
