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
uint32_t lastBMSMsg = 0, lastDebug = 0;
bool debugEnabled = false;
uint8_t  resetCause = 0;   // Causa del último reset (se lee al arrancar de los flags RCC).
uint16_t heartbeat  = 0;   // Contador de loop para la telemetría de diagnóstico.

// ===================== INVERSOR (E/S CAN encapsulada) =====================
InverterController inverter(CAN, id0StsInverter, id2StsInverter, id4StsInverter,
                            idCmdEN, idCmdCurrentPCTG,
                            idCmdSetMaxACCurrent, idCmdSetMaxDCCurrent);

// Limitador dinámico de potencia (capa el throttle según V_dc/eRPM del inversor).
PowerLimiter powerLimiter(cfgKtEff, cfgIFuseMax, (float)cfgCurrentACMAX,
                          cfgEtaInv, cfgPMaxW, cfgPolePairs);

// ===================== READY-TO-DRIVE =====================
R2DStateMachine r2dSM(pinBUZZ, BUZZER_ON_MS);

// ===================== SENSORES (dirección + ruedas) =====================
SteeringSensor steer(steerAdcLeft, steerAdcCenter, steerAdcRight);

// Pulsos de las ruedas (incrementados por interrupción).
volatile uint32_t wheelPulsesL = 0, wheelPulsesR = 0;
static void isrWheelL() { wheelPulsesL++; }
static void isrWheelR() { wheelPulsesR++; }
WheelSpeed wheelL(wheelPulsesL, wheelTeeth);
WheelSpeed wheelR(wheelPulsesR, wheelTeeth);

// ===================== BUFFERS DE TELEMETRÍA =====================
uint16_t CANAppsState[4];
uint16_t CANBrakeState[4];
uint8_t  CANVCUSignals[8];
uint8_t  CANVCUDiag[8];

// ===================== MODO DE CONTROL / SIMULACIÓN =====================
ControlMode controlMode = MODE_CAN;   // Modo por defecto: CAN (el coche corre en CAN).
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
  Serial.setTimeout(50);                // parseInt() no bloquea 1 s con ruido en el USART.
  resetCause = readResetCause();        // Causa del último reset (antes de tocar nada).
  lastDebug = millis();                 // Marca de tiempo inicial para debug.
  lastBMSMsg = millis();                // Marca de tiempo inicial para watchdog del BMS (SDC).

  // Configuración de pines.
  pinMode(pinStart, INPUT_PULLUP);
  pinMode(pinBUZZ, OUTPUT);
  pinMode(pinR2D_Digital, OUTPUT);
  digitalWrite(pinR2D_Digital, LOW);   // Estado seguro: DriveEnable LOW al arrancar.
  // (Recomendado: pull-down hardware en pinR2D_Digital para que esté LOW durante el boot/reset.)

  // Velocidad de rueda: entradas de pulsos por interrupción (flanco de subida).
  // ⚠ INPUT_PULLUP por si el SNDH es open-collector; si la PCB ya lleva pull, usar INPUT.
  pinMode(pinWheelL, INPUT_PULLUP);
  pinMode(pinWheelR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pinWheelL), isrWheelL, RISING);
  attachInterrupt(digitalPinToInterrupt(pinWheelR), isrWheelR, RISING);

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
    if (bmsWatchdogExpired(lastBMSMsg, millis(), BMS_WD_MS)) {
      stsSDC = false;   // fail-safe: sin tramas del BMS, el SDC no se considera presente
    }

    rawApps1   = adc.read(MCP3208::Channel::SINGLE_2);
    rawApps2   = adc.read(MCP3208::Channel::SINGLE_3);
    stsBrake   = adc.read(MCP3208::Channel::SINGLE_4);
    stsBrake2  = adc.read(MCP3208::Channel::SINGLE_5);
    stsVbatRAW = adc.read(MCP3208::Channel::SINGLE_6);
    steer.update(adc.read(chSteer));   // PSC-360 (dirección)

    // Velocidad de rueda + dirección → CAN 0x48D cada wheelWindowMs.
    static uint32_t tWheel = 0;
    if (millis() - tWheel >= wheelWindowMs) {
      tWheel = millis();
      wheelL.updateRpm();
      wheelR.updateRpm();
      uint16_t d[4] = { (uint16_t)(int16_t)steer.percent(),
                        (uint16_t)steer.raw(),
                        (uint16_t)wheelL.rpm(),
                        (uint16_t)wheelR.rpm() };
      CAN.setPacket(idSteerWheels, d, 4);   // la emite el CAN.send() de controlInverter
    }
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
