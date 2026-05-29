#pragma once
//==============================================================================
// Vcu.h — Estado compartido y prototipos de los módulos de la VCU.
// Las definiciones de los objetos/variables están en main.cpp; aquí se declaran
// como extern para que los módulos (R2d, Simulation, InverterControl, Diagnostics)
// las compartan. La parametrización vive en Config.h (namespace cfg); aquí se
// exponen alias locales con los nombres usados en el código.
//==============================================================================
#include <Arduino.h>
#include <MART_CAN.h>
#include <Mcp320x.h>
#include <Adafruit_NeoPixel.h>
#include "esp_task_wdt.h"
#include "global.h"
#include "PairedAnalogSensor.h"
#include "Config.h"

// --------- Centinelas "no usado" para los paquetes del inversor ---------
#define UNUSED_BYTE  0xFF
#define UNUSED_SHORT ((int16_t)0x7FFF)
#define UNUSED_INT32 0xFFFFFFFF

// --------- Alias de Config.h (constexpr -> enlace interno por TU) ---------
// CAN / nodo / tiempos
constexpr unsigned  CAN_SPEED_KBPS     = cfg::CAN_SPEED_KBPS;
constexpr int       NODE_ID            = cfg::NODE_ID;
constexpr uint32_t  DEBUG_PERIOD_MS    = cfg::DEBUG_PERIOD_MS;
constexpr uint32_t  INVERTER_WD_MS     = cfg::INVERTER_WD_MS;
constexpr uint32_t  BUZZER_ON_MS       = cfg::BUZZER_ON_MS;
constexpr uint32_t  TASK_WDT_TIMEOUT_S = cfg::TASK_WDT_TIMEOUT_S;
// BMS por CAN
constexpr uint32_t  ID_BMS_STATUS      = cfg::ID_BMS_STATUS;
constexpr uint8_t   BMS_SDC_BIT        = cfg::BMS_SDC_BIT;
constexpr uint32_t  BMS_WD_MS          = cfg::BMS_WD_MS;
// Hardware (ADC / LED / SPI)
constexpr uint16_t  ADC_VREF           = cfg::ADC_VREF;
constexpr uint32_t  ADC_CLK            = cfg::ADC_CLK;
constexpr uint8_t   SPI_CS             = cfg::PIN_SPI_CS;
constexpr uint8_t   LED_PIN            = cfg::PIN_LED;
constexpr uint8_t   LED_COUNT          = cfg::LED_COUNT;
// Pines de E/S
constexpr uint8_t   pinStart           = cfg::PIN_START;
constexpr uint8_t   pinBUZZ            = cfg::PIN_BUZZER;
constexpr uint8_t   pinR2D_Digital     = cfg::PIN_R2D_DIGITAL;
// IDs de comandos al inversor
constexpr uint32_t  idCmdRPM             = cfg::ID_CMD_RPM;
constexpr uint32_t  idCmdEN              = cfg::ID_CMD_EN;
constexpr uint32_t  idCmdCurrentPCTG     = cfg::ID_CMD_CURRENT_PCTG;
constexpr uint32_t  idCmdSetMaxACCurrent = cfg::ID_CMD_SET_MAX_AC;
constexpr uint32_t  idCmdSetMaxDCCurrent = cfg::ID_CMD_SET_MAX_DC;
// IDs de estado del inversor
constexpr uint32_t  id2StsInverter       = cfg::ID_STS_INV_2;
constexpr uint32_t  id4StsInverter       = cfg::ID_STS_INV_4;
// IDs de telemetría publicada por la VCU
constexpr unsigned long idAPPSState  = cfg::ID_APPS_STATE;
constexpr unsigned long idBrakeState = cfg::ID_BRAKE_STATE;
constexpr unsigned long idVCUSignals = cfg::ID_VCU_SIGNALS;
// Límites de control
constexpr int  cfgBrakeTH      = cfg::BRAKE_TH;
constexpr int  cfgCurrentACMAX = cfg::CURRENT_AC_MAX;
constexpr int  cfgCurrentDCMAX = cfg::CURRENT_DC_MAX;
constexpr int  cfgRPMax        = cfg::RPM_MAX;

// --------- Modo de control ---------
enum ControlMode { MODE_CAN, MODE_DIRECT };

// --------- Perfiles de simulación ---------
enum SimProfile {
  SIM_OFF = 0,
  SIM_APPS_STEP = 1,
  SIM_FAULT_OVERVOLTAGE = 2,
  SIM_FAULT_UNDERVOLTAGE = 3,
  SIM_FAULT_CTRL_OVERTEMP = 4,
  SIM_FAULT_MOTOR_OVERTEMP = 5,
  SIM_RANDOM = 6
};

// --------- Objetos hardware (definidos en main.cpp) ---------
extern CAN_BUS CAN;
extern MCP3208 adc;
extern Adafruit_NeoPixel pixels;

// --------- APPS ---------
extern PairedAnalogSensorConfig appsCfg;
extern PairedAnalogSensor appsSensor;
extern uint16_t rawApps1, rawApps2;
extern SensorState appsState;

// --------- Estado de la VCU ---------
extern bool stsStart, stsR2D, stsSDC;
extern int  stsBrake, stsBrake2, stsVbatRAW;
extern byte canBMSStatus[1];
extern uint32_t lastInverterMsg, lastBMSMsg, lastDebug;
extern bool debugEnabled;

// --------- Buffers de telemetría VCU ---------
extern uint16_t CANAppsState[4];
extern uint16_t CANBrakeState[4];
extern uint8_t  CANVCUSignals[8];

// --------- Buffers de comandos al inversor ---------
extern int32_t cmdDataRPM[2];
extern int16_t cmdDataCurrent[4];
extern int16_t cmdDataCurrentACMax[4];
extern int16_t cmdDataCurrentDCMax[4];
extern byte    cmdDataDriveEN[8];

// --------- Modo de control / simulación ---------
extern ControlMode controlMode;
extern SimProfile simProfile;
extern uint32_t simStepT;
extern int simAppsProgress;

// --------- Prototipos de los módulos ---------
PairedAnalogSensorConfig buildAppsConfig();             // InverterControl.cpp
bool R2D(bool sdc, bool start, bool brake);             // R2d.cpp
void runSimulation();                                   // Simulation.cpp
void controlInverter();                                 // InverterControl.cpp
void watchdogCAN();                                     // InverterControl.cpp
void readInverterStatus();                              // InverterControl.cpp
void debug();                                           // Diagnostics.cpp
void serialMenu();                                      // Diagnostics.cpp
void processMenu();                                     // Diagnostics.cpp
