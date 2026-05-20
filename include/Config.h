#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// --- PINES ---
#define PIN_LED_RGB     48
#define PIN_SPI_CS      10
#define PIN_TSON        21
#define PIN_START       15
#define PIN_BUZZ        8
#define PIN_TSON_EXT    9
#define PIN_SDC         16
#define PIN_STS_USB     2
#define PIN_CMD_USB     14

// --- CONFIGURACIÓN MCP3208 ---
#define ADC_VREF        3300
#define ADC_CLK         1600000

// --- CONSTANTES DEL VEHÍCULO ---
#define CFG_RPMAX           1500
#define CFG_BRAKE_TH        550
#define CFG_TIME_SOUND_R2D  2000
#define CFG_APPS_DIFF       100
#define CFG_APPS_DESC       5000

// --- CAN BUS CONFIG ---
#define CAN_SPEED       125     // 125 kbps (según el main original)
#define CAN_NODE_ID     1       // ID de este nodo

// --- CAN IDs ---
#define ID_BMS_STATUS           10
#define ID_APPS_STATE           1163
#define ID_BRAKE_STATE          1164
#define ID_VCU_SIGNALS          1166

// Inverter IDs (Asumiendo Node ID = 1)
#define ID_CMD_EN               385  // 0x181
#define ID_CMD_CURRENT_PCTG     5    // Asignar el valor real de tu combineInts
#define ID_CMD_SET_MAX_AC       8    // Asignar el valor real
#define ID_CMD_SET_MAX_DC       10   // Asignar el valor real

#endif // CONFIG_H