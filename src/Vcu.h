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
#include <IWatchdog.h>
#include "global.h"
#include "PairedAnalogSensor.h"
#include "Config.h"
#include "InverterController.h"
#include "PowerLimiter.h"
#include "R2DStateMachine.h"
#include "SteeringSensor.h"
#include "WheelSpeed.h"
#include "SafetyLogic.h"   // funciones puras (watchdog BMS, rampa subtensión) — testeables en native

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
// Hardware (ADC / SPI2). Números de pin Arduino del variant.
constexpr uint16_t  ADC_VREF       = cfg::ADC_VREF;
constexpr uint32_t  ADC_CLK        = cfg::ADC_CLK;
constexpr uint8_t   SPI_CS         = cfg::PIN_ADC_CS;
constexpr uint8_t   adcSck         = cfg::PIN_ADC_SCK;
constexpr uint8_t   adcMiso        = cfg::PIN_ADC_MISO;
constexpr uint8_t   adcMosi        = cfg::PIN_ADC_MOSI;
// Pines de E/S
constexpr uint8_t   pinStart       = cfg::PIN_START;
constexpr uint8_t   pinBUZZ        = cfg::PIN_BUZZER;
constexpr uint8_t   pinR2D_Digital = cfg::PIN_R2D_DIGITAL;
// IDs de comandos al inversor (combineInts(packet, NODE_ID=1) → Standard ID).
// Numeración de packets del manual DTI CAN V2.5 §4.
constexpr uint32_t  idCmdRPM             = cfg::ID_CMD_RPM;            // 0x061 Set ERPM (packet 0x03) — NO se usa: pasaría el DTI a control por velocidad
constexpr uint32_t  idCmdEN              = cfg::ID_CMD_EN;             // 0x181 Drive enable (packet 0x0C)
constexpr uint32_t  idCmdCurrentPCTG     = cfg::ID_CMD_CURRENT_PCTG;  // 0x0A1 Set Relative current (% ×10, 0..1000; packet 0x05)
constexpr uint32_t  idCmdSetMaxACCurrent = cfg::ID_CMD_SET_MAX_AC;    // 0x101 Set max AC current (Apk ×10; packet 0x08)
constexpr uint32_t  idCmdSetMaxDCCurrent = cfg::ID_CMD_SET_MAX_DC;    // 0x141 Set max DC current (Adc ×10; packet 0x0A)
// IDs de estado que EMITE el inversor (transmit packets del DTI → Standard ID, node 1).
constexpr uint32_t  id0StsInverter       = cfg::ID_STS_INV_0;  // 0x401 (packet 0x20): eRPM b0-3 (int32), Duty b4-5, Vin b6-7 (int16, V)
constexpr uint32_t  id2StsInverter       = cfg::ID_STS_INV_2;  // 0x441 (packet 0x22): temps + fault code (byte 4)
constexpr uint32_t  id4StsInverter       = cfg::ID_STS_INV_4;  // 0x481 (packet 0x24): throttle/brake/drive enable (byte 3)
// IDs de telemetría que PUBLICA la VCU.
constexpr unsigned long idVCUDiag     = cfg::ID_VCU_DIAG;      // 0x488 diagnóstico/post-mortem (faultCause, heartbeat, resetCause)
constexpr unsigned long idAPPSState   = cfg::ID_APPS_STATE;    // 0x48B APPS (escalado 0..1000 + raw de ambos sensores)
constexpr unsigned long idBrakeState  = cfg::ID_BRAKE_STATE;   // 0x48C freno (raw de ambos sensores)
constexpr unsigned long idSteerWheels = cfg::ID_STEER_WHEELS;  // 0x48D dirección (% + raw) + velocidad de ruedas (RPM)
constexpr unsigned long idVCUSignals  = cfg::ID_VCU_SIGNALS;   // 0x48E señales (Vbat, SDC, Start, R2D, estado APPS)
// Sensores nuevos (dirección + ruedas). Valores reales en Config.h.
constexpr MCP3208::Channel chSteer    = cfg::CH_STEER;         // canal ADC del string pot PSC-360 (dirección)
constexpr uint8_t  pinWheelL          = cfg::PIN_WHEEL_L;      // pin de pulsos rueda IZQ (entrada por interrupción)
constexpr uint8_t  pinWheelR          = cfg::PIN_WHEEL_R;      // pin de pulsos rueda DER (entrada por interrupción)
constexpr int      steerAdcLeft       = cfg::STEER_ADC_LEFT;   // cuenta ADC en tope izquierda  → −100 %
constexpr int      steerAdcCenter     = cfg::STEER_ADC_CENTER; // cuenta ADC en centro          →    0 %
constexpr int      steerAdcRight      = cfg::STEER_ADC_RIGHT;  // cuenta ADC en tope derecha    → +100 %
constexpr uint16_t wheelTeeth         = cfg::WHEEL_TEETH;      // dientes de la corona = pulsos por vuelta de rueda
constexpr uint32_t wheelWindowMs      = cfg::WHEEL_WINDOW_MS;  // ventana de medida de RPM (ms)
// Límites de control
constexpr int  cfgBrakeTH      = cfg::BRAKE_TH;               // cuentas ADC de freno para armar R2D
constexpr int  cfgCurrentACMAX    = cfg::CURRENT_AC_MAX;       // Arms — referencia del limitador (100% = este valor)
constexpr int  cfgCurrentACMAXApk = cfg::CURRENT_AC_MAX_APK;   // Apk — valor del comando Set Max AC al DTI
constexpr int  cfgCurrentDCMAX    = cfg::CURRENT_DC_MAX;       // Adc — comando Set Max DC al DTI (= fusible)
constexpr int  cfgRPMax        = cfg::RPM_MAX;                // ERPM target máximo (no usado: control por corriente)
// Limitador dinámico de potencia (ver PowerLimiter.h)
constexpr float cfgKtEff       = cfg::KT_EFF;        // Nm/Arms — constante de par efectiva (1.5·p·λ_pm)
constexpr float cfgIFuseMax    = cfg::I_FUSE_MAX_A;  // A DC — límite del fusible de batería
constexpr float cfgEtaInv      = cfg::ETA_INV;       // eficiencia del inversor (0..1)
constexpr float cfgPMaxW       = cfg::P_MAX_W;       // W — Maximum Wattage configurado en el DTI
constexpr uint8_t cfgPolePairs = cfg::POLE_PAIRS;    // pares de polos (eRPM = RPM_mec × pares)
constexpr float cfgVPackMinOp  = cfg::V_PACK_MIN_OP; // V — mínimo operativo; por debajo → corte de par
constexpr float cfgVPackRampV  = cfg::V_PACK_RAMP_V; // V — ancho de rampa de subtensión (corte suave)

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

// --------- APPS ---------
extern PairedAnalogSensorConfig appsCfg;
extern PairedAnalogSensor appsSensor;
extern uint16_t rawApps1, rawApps2;
extern SensorState appsState;

// --------- Estado de la VCU ---------
extern bool stsStart, stsR2D, stsSDC;
extern int  stsBrake, stsBrake2, stsVbatRAW;
extern byte canBMSStatus[1];
extern uint32_t lastBMSMsg, lastDebug;
extern bool debugEnabled;
extern uint8_t  resetCause;    // Causa del último reset del micro (se lee al arrancar).
extern uint16_t heartbeat;     // Contador de loop (se congela si el firmware se cuelga).

// --------- Inversor (E/S CAN encapsulada) ---------
extern InverterController inverter;

// --------- Limitador dinámico de potencia ---------
extern PowerLimiter powerLimiter;

// --------- Ready-to-Drive (máquina de estados) ---------
extern R2DStateMachine r2dSM;

// --------- Buffers de telemetría VCU ---------
extern uint16_t CANAppsState[4];
extern uint16_t CANBrakeState[4];
extern uint8_t  CANVCUSignals[8];
extern uint8_t  CANVCUDiag[8];

// --------- Modo de control / simulación ---------
extern ControlMode controlMode;
extern SimProfile simProfile;
extern uint32_t simStepT;
extern int simAppsProgress;

// --------- Prototipos de los módulos ---------
PairedAnalogSensorConfig buildAppsConfig();             // InverterControl.cpp
void runSimulation();                                   // Simulation.cpp
void controlInverter();                                 // InverterControl.cpp
void watchdogCAN();                                     // InverterControl.cpp
void readInverterStatus();                              // InverterControl.cpp
void debug();                                           // Diagnostics.cpp
void serialMenu();                                      // Diagnostics.cpp
void processMenu();                                     // Diagnostics.cpp
