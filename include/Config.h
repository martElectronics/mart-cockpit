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

// --- Pines (STM32G474RE / NUCLEO-G474RE) — pinout real del VCU ---
// Se usan los MACROS de pin Arduino del variant (PB7, PC8...) = números de pin, para
// que valgan en pinMode/digitalWrite, en la librería MCP3208 (CS uint8_t) y en SPIClass.
// (Los PinName 'PX_n' solo valen en pinMode/digitalWrite por sobrecarga, NO en librerías.)
// CAN va fijo en PA11/PA12 (RX/TX), lo configura MART_CAN; no se declara aquí.
constexpr uint8_t PIN_START       = PB7;   // Pulsador Start (INPUT_PULLUP, activo a 0).
constexpr uint8_t PIN_BUZZER      = PC8;   // Buzzer R2D.
constexpr uint8_t PIN_R2D_DIGITAL = PC6;   // DriveEnable digital hacia el inversor.
// Velocidad de rueda (SNDH-H3L-G01, pulsos Hall por interrupción). ⚠ AJUSTAR a la PCB.
constexpr uint8_t PIN_WHEEL_L     = PA8;   // Rueda izquierda (entrada de pulsos).
constexpr uint8_t PIN_WHEEL_R     = PA9;   // Rueda derecha (entrada de pulsos).
// ADC MCP3208 por SPI2 (CS PB12, CLK PB13, DOUT->MISO PB14, DIN->MOSI PB15).
// (Nombres PIN_ADC_* para no chocar con los macros PIN_SPI_* del framework STM32duino.)
constexpr uint8_t PIN_ADC_CS   = PB12;
constexpr uint8_t PIN_ADC_SCK  = PB13;
constexpr uint8_t PIN_ADC_MISO = PB14;  // DOUT del ADC -> MISO del MCU.
constexpr uint8_t PIN_ADC_MOSI = PB15;  // DIN del ADC  -> MOSI del MCU.

// --- ADC MCP3208 ---
constexpr uint16_t ADC_VREF = 3300;        // Vref del ADC (mV).
constexpr uint32_t ADC_CLK  = 160000;      // Reloj SPI del ADC (Hz).
constexpr MCP3208::Channel CH_APPS1  = MCP3208::Channel::SINGLE_2;
constexpr MCP3208::Channel CH_APPS2  = MCP3208::Channel::SINGLE_3;
constexpr MCP3208::Channel CH_BRAKE1 = MCP3208::Channel::SINGLE_4;
constexpr MCP3208::Channel CH_BRAKE2 = MCP3208::Channel::SINGLE_5;
constexpr MCP3208::Channel CH_VBAT   = MCP3208::Channel::SINGLE_6;
constexpr MCP3208::Channel CH_STEER  = MCP3208::Channel::SINGLE_0;  // PSC-360 (string pot dirección)

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
constexpr int CURRENT_AC_MAX     = 190;   // Corriente AC máx en Arms (referencia del limitador; 100% = 190 Arms).
constexpr int CURRENT_AC_MAX_APK = 269;   // Set Max AC del DTI en Apk (= 190 Arms × √2 = Motor Current Max).
constexpr int CURRENT_DC_MAX     = 125;   // Set Max DC del DTI (Adc) = fusible de batería (125 A).
constexpr int RPM_MAX        = 1500;  // ERPM target máximo.

// --- Limitador dinámico de potencia (VCU → DTI), ver EMRAX188HV_DTI_HV500_v3 §5 ---
// i_ac_max = (V_dc·I_fuse·η)/(K_T·ω_mec), clampeado a P_MAX y a CURRENT_AC_MAX.
// Solo muerde por encima de ~base; sin field weakening (el rango queda por debajo).
constexpr float    KT_EFF       = 0.4865f;  // Nm/Arms (1.5·p·λ_pm)
constexpr float    I_FUSE_MAX_A = 125.0f;   // A DC — límite del fusible
constexpr float    ETA_INV      = 0.95f;    // eficiencia inversor
constexpr float    P_MAX_W      = 65000.0f; // W — Maximum Wattage del DTI
constexpr uint8_t  POLE_PAIRS   = 10;       // EMRAX 188 (eRPM = RPM·10)
constexpr float    V_PACK_MIN_OP = 350.0f;  // V — mínimo operativo (⚠ 292 para 10 módulos)

// --- Sensores nuevos (⚠ CALIBRAR con datos reales) ---
// Dirección (PSC-360 string pot): cuentas ADC en tope izquierda / centro / derecha.
constexpr int STEER_ADC_LEFT   = 0;      // tope izquierda  → -100 %
constexpr int STEER_ADC_CENTER = 2048;   // centro          →    0 %
constexpr int STEER_ADC_RIGHT  = 4095;   // tope derecha    → +100 %
// Velocidad de rueda: nº de dientes de la rueda fónica (pulsos por vuelta de rueda).
constexpr uint16_t WHEEL_TEETH       = 1;     // ⚠ contar los dientes de la corona
constexpr uint32_t WHEEL_WINDOW_MS   = 100;   // ventana de medida de frecuencia

// --- IDs CAN de comandos al inversor (combineInts(PID, NODE_ID)) ---
// Numeracion de packets del manual DTI V2.5 (commands 0x01-0x0C). El esquema
// VIEJO (V2.3: 0x1A-0x24) estaba mal y colisionaba con la telemetria.
constexpr uint32_t ID_CMD_RPM            = combineInts(0x03, NODE_ID);   // 0x061 Set ERPM
constexpr uint32_t ID_CMD_EN             = combineInts(0x0C, NODE_ID);   // 0x181 Drive enable
constexpr uint32_t ID_CMD_CURRENT_PCTG   = combineInts(0x05, NODE_ID);   // 0x0A1 Set relative current
constexpr uint32_t ID_CMD_SET_MAX_AC     = combineInts(0x08, NODE_ID);   // 0x101 Set max AC current
constexpr uint32_t ID_CMD_SET_MAX_DC     = combineInts(0x0A, NODE_ID);   // 0x141 Set max DC current

// --- IDs CAN de estado del inversor (transmit packets 0x20-0x24 -> 0x401-0x481,
//     Standard ID, node 1). Temps/Fault = 0x22 (0x441); Throttle/Brake/DriveEnable
//     = 0x24 (0x481). Sin colision con los comandos (0x21-0x181). ---
constexpr uint32_t ID_STS_INV_0 = combineInts(0x20, NODE_ID);   // 0x401 (eRPM b0-3 + Vin b6-7)
constexpr uint32_t ID_STS_INV_2 = combineInts(0x22, NODE_ID);   // 0x441 (temps + fault code)
constexpr uint32_t ID_STS_INV_4 = combineInts(0x24, NODE_ID);   // 0x481 (throttle/brake/drive enable)

// --- IDs CAN de telemetría publicada por la VCU ---
constexpr unsigned long ID_VCU_DIAG    = 1160;  // 0x488: diagnóstico/post-mortem (hueco "FAIL CODES" libre).
constexpr unsigned long ID_APPS_STATE    = 1163;
constexpr unsigned long ID_BRAKE_STATE   = 1164;
constexpr unsigned long ID_STEER_WHEELS  = 1165;  // 0x48D: dirección + velocidad ruedas
constexpr unsigned long ID_VCU_SIGNALS   = 1166;

} // namespace cfg
