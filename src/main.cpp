//==============================================================================
// main.cpp — VCU MART. Punto de entrada: define el estado compartido (declarado en
// Vcu.h) y orquesta setup()/loop(). La lógica vive en los módulos:
//   R2d.cpp · Simulation.cpp · InverterControl.cpp · Diagnostics.cpp
// La parametrización está en include/Config.h (namespace cfg).
//==============================================================================
#include <SPI.h>
#include "Vcu.h"

// ===================== HARDWARE =====================
CAN_BUS CAN(HardwareType::Transciever, CAN_SPEED_KBPS, NODE_ID); // CAN por FDCAN1 (PA11/PA12).
SPIClass SPI_2(adcMosi, adcMiso, adcSck);                        // Bus SPI2 del ADC (DIN PB15 / DOUT PB14 / CLK PB13).
MCP3208 adc(ADC_VREF, SPI_CS, &SPI_2);                           // ADC MCP3208 sobre SPI2.

// ===================== APPS =====================
PairedAnalogSensorConfig appsCfg = buildAppsConfig();
PairedAnalogSensor appsSensor(appsCfg);
uint16_t rawApps1 = 0, rawApps2 = 0;
SensorState appsState = SensorState::NORMAL;

// ===================== ESTADO VCU =====================
bool stsStart = false, stsR2D = false, stsSDC = false;
int  stsBrake = 0, stsBrake2 = 0, stsVbatRAW = 0;
byte canBMSStatus[1];
uint32_t lastInverterMsg = 0, lastBMSMsg = 0, lastDebug = 0;
bool debugEnabled = false;
uint8_t  resetCause = 0;   // Causa del último reset (se lee al arrancar de los flags RCC).
uint16_t heartbeat  = 0;   // Contador de loop para la telemetría de diagnóstico.

// ===================== BUFFERS DE TELEMETRÍA =====================
uint16_t CANAppsState[4];
uint16_t CANBrakeState[4];
uint8_t  CANVCUSignals[8];
uint8_t  CANVCUDiag[8];

// ===================== BUFFERS DE COMANDOS AL INVERSOR =====================
int32_t cmdDataRPM[2]          = {0, (int32_t)UNUSED_INT32};
int16_t cmdDataCurrent[4]      = {0, UNUSED_SHORT, UNUSED_SHORT, UNUSED_SHORT};
int16_t cmdDataCurrentACMax[4] = {0, UNUSED_SHORT, UNUSED_SHORT, UNUSED_SHORT};
int16_t cmdDataCurrentDCMax[4] = {0, UNUSED_SHORT, UNUSED_SHORT, UNUSED_SHORT};
byte    cmdDataDriveEN[8]      = {0, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE};

// ===================== MODO DE CONTROL / SIMULACIÓN =====================
ControlMode controlMode = MODE_DIRECT;   // Modo por defecto: DIRECTO.
SimProfile  simProfile  = SIM_OFF;
uint32_t    simStepT    = 0;
int         simAppsProgress = 0;

// Lee la causa del último reset de los flags RCC del STM32 (para diagnóstico:
// detectar si reseteó el watchdog IWDG por loop colgado). Códigos: 1=power/BOR,
// 2=pin NRST, 3=software, 4=IWDG, 5=WWDG, 6=low-power, 0=desconocido.
static uint8_t readResetCause() {
  uint8_t rc = 0;
  if      (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST)) rc = 4;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_WWDGRST)) rc = 5;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_LPWRRST)) rc = 6;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST))  rc = 3;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_BORRST))  rc = 1;
  else if (__HAL_RCC_GET_FLAG(RCC_FLAG_PINRST))  rc = 2;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  return rc;
}

void setup() {
  Serial.begin(115200);                 // Inicializa puerto serie para debug.
  resetCause = readResetCause();        // Causa del último reset (antes de tocar nada).
  lastDebug = millis();                 // Marca de tiempo inicial para debug.
  lastInverterMsg = millis();           // Marca de tiempo inicial para watchdog CAN.
  lastBMSMsg = millis();                // Marca de tiempo inicial para watchdog del BMS (SDC).

  // Configuración de pines.
  pinMode(pinStart, INPUT_PULLUP);
  pinMode(pinBUZZ, OUTPUT);
  pinMode(pinR2D_Digital, OUTPUT);
  digitalWrite(pinR2D_Digital, LOW);   // Estado seguro: DriveEnable LOW al arrancar.
  // (Recomendado: pull-down hardware en pinR2D_Digital para que esté LOW durante el boot/reset.)

  // Watchdog del micro (IWDG del STM32): si el loop se cuelga > TASK_WDT_TIMEOUT_S,
  // resetea -> DriveEnable a LOW. IWatchdog.begin() recibe el timeout en microsegundos.
  IWatchdog.begin(TASK_WDT_TIMEOUT_S * 1000000UL);

  // Inicializa SPI para el ADC MCP3208.
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI_2.begin();
  SPI_2.beginTransaction(settings);

  // La auto-calibración del APPS la hace AnalogSensor internamente por tabla de voltaje
  // (ver buildAppsConfig). Con la tabla vacía se usan los límites estáticos.

  CAN.config.simulating = false;        // Desactiva simulación por defecto.
  simProfile = SIM_OFF;

  Serial.println("VCU inicializada.");  // Mensaje de inicio.
  serialMenu();                         // Muestra menú interactivo por serie.
}

void loop() {
  if (CAN.config.simulating) {
    runSimulation();
  } else {
    CAN.receive();

    // SDC desde el BMS por CAN (ID 10, byte 0, bit 2). Independiente del modo de control.
    if (CAN.getPacket(ID_BMS_STATUS, canBMSStatus, 1)) {
      lastBMSMsg = millis();
      stsSDC = (canBMSStatus[0] >> BMS_SDC_BIT) & 0x01;
    }
    if ((millis() - lastBMSMsg) > BMS_WD_MS) {
      stsSDC = false;   // fail-safe: sin tramas del BMS, el SDC no se considera presente
    }

    if (controlMode == MODE_CAN) {
      if (CAN.getPacket(id2StsInverter, stsInverterCAN_22_FULL, 8) ||
          CAN.getPacket(id4StsInverter, stsInverterCAN_24_FULL, 8)) {
        lastInverterMsg = millis();
      }
    }
    rawApps1   = adc.read(MCP3208::Channel::SINGLE_2);
    rawApps2   = adc.read(MCP3208::Channel::SINGLE_3);
    stsBrake   = adc.read(MCP3208::Channel::SINGLE_4);
    stsBrake2  = adc.read(MCP3208::Channel::SINGLE_5);
    stsVbatRAW = adc.read(MCP3208::Channel::SINGLE_6);
  }

  controlInverter();

  if (controlMode == MODE_CAN) {
    watchdogCAN();
    readInverterStatus();
  }

  if (debugEnabled) {   // ← solo imprime si está activado
    debug();
  }

  processMenu();

  heartbeat++;          // Para la telemetría de diagnóstico (se congela si el loop se cuelga).
  IWatchdog.reload();   // Alimenta el watchdog del micro (si el loop se cuelga, reset -> DriveEnable LOW).
}
