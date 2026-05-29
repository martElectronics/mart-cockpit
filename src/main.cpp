#include <Arduino.h>              // Librería base de Arduino: funciones de GPIO, Serial, millis(), etc.
#include <MART_CAN.h>             // Librería MART_CAN: gestión de comunicación CAN (envío/recepción de frames).
#include <SPI.h>                  // Librería SPI: comunicación con periféricos por bus SPI (ej. ADC).
#include <EEPROM.h>               // Librería EEPROM: lectura/escritura de memoria no volátil.
#include "global.h"               // Archivo propio del proyecto (probablemente define estructuras y constantes).
#include <Mcp320x.h>              // Librería para ADC MCP3208 (conversor analógico-digital de 12 bits).
#include <Adafruit_NeoPixel.h>    // Librería para controlar LEDs RGB tipo NeoPixel.
#include "PairedAnalogSensor.h"   // Par de sensores APPS: filtrado, escalado, plausibilidad y autocalibración.

// ===================== CONSTANTES =====================
#define CAN_SPEED_KBPS 125        // Velocidad del bus CAN en kbps (125 kbps).
#define NODE_ID 1                 // ID del nodo en la red CAN (para direccionamiento).
#define EEPROM_SIZE 512           // Tamaño de la EEPROM disponible en el ESP32.
#define EEPROM_ADDR_ACMAX 0       // Dirección en EEPROM donde se guarda el límite de corriente AC.
#define EEPROM_ADDR_DCMAX 10      // Dirección en EEPROM para corriente DC máxima.
#define EEPROM_ADDR_RPMAX 20      // Dirección en EEPROM para RPM máximo.
#define DEBUG_PERIOD_MS 500       // Periodo de refresco del debug por puerto serie (ms).
#define INVERTER_WD_MS 1000       // Tiempo de watchdog para comunicación con el inversor (ms).
#define BUZZER_ON_MS 2000         // Tiempo que suena el buzzer al activar R2D (ms).

// --- BMS por CAN (bms_master_26, rama testing) ---
#define ID_BMS_STATUS 10          // ID 10 (0x0A): Estado general del BMS. DLC 1, ~800 ms.
#define BMS_SDC_BIT   2           // BMS_SDC = byte 0, bit 2 (SDC presente). Confirmado con bms_master_26.
#define BMS_WD_MS     2500        // Sin trama del BMS en este tiempo -> SDC se considera NO presente (fail-safe). ID 10 llega cada ~800 ms.

#define UNUSED_BYTE  0xFF         // Valor especial para indicar "no usado" en arrays tipo byte.
#define UNUSED_SHORT ((int16_t)0x7FFF) // Valor especial para "no usado" en arrays tipo short (32767).
#define UNUSED_INT32 0xFFFFFFFF   // Valor especial para "no usado" en arrays tipo int32_t.

// ===================== PROTOTIPOS =====================
// Declaraciones de funciones para que el compilador las conozca antes de usarlas.
PairedAnalogSensorConfig buildAppsConfig();
bool R2D(bool sdc, bool start, bool brake);

void guardarEEPROM(int direccion, int valor);
int leerEEPROM(int direccion);
void cargarConfiguracionesEEPROM();
void resetEEPROM();
void mostrarConfiguracionesEEPROM();
void aplicarConfiguraciones();

void runSimulation();
void controlInverter();
void watchdogCAN();
void readInverterStatus();
void debug();

void serialMenu();
void configurarLimitesEEPROM_interactivo();
void processMenu();

// ===================== HARDWARE =====================
#define LED_PIN 48                // Pin donde está conectado el LED NeoPixel.
#define LED_COUNT 1               // Número de LEDs NeoPixel.
#define SPI_CS 10                 // Pin de chip select para el ADC MCP3208.
#define ADC_VREF 3300             // Voltaje de referencia del ADC (mV).
#define ADC_CLK 160000            // Frecuencia de reloj SPI para el ADC.

CAN_BUS CAN(HardwareType::Transciever, CAN_SPEED_KBPS, NODE_ID); // Objeto CAN con transceptor, velocidad y ID.
MCP3208 adc(ADC_VREF, SPI_CS);                                   // Objeto ADC MCP3208.
Adafruit_NeoPixel pixels(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800); // Objeto para controlar el LED RGB.

// GPIOs confirmados (pines físicos del ESP32 asignados a señales del coche).
int pinTSON = 21, pinStart = 15, pinBUZZ = 8, pinTSON_EXT = 9, pinSDC = 16, pinR2D_Digital = 2;

// ===================== CONFIGURACIONES =====================
// Variables de estado y configuración del sistema.
PairedAnalogSensorConfig appsCfg = buildAppsConfig();  // Configuración del par APPS (definición más abajo).
PairedAnalogSensor appsSensor(appsCfg);                // Par de sensores APPS: filtrado + plausibilidad + autocal.
uint16_t rawApps1 = 0, rawApps2 = 0;                   // Lecturas crudas de APPS1/APPS2 (ADC o simuladas).
SensorState appsState = SensorState::NORMAL;           // Estado de plausibilidad del par APPS.
bool stsStart, stsR2D;                   // Estados de Start y Ready-to-Drive. (TSON: próxima iteración de PCB)
bool stsSDC = false;                     // SDC (shutdown circuit) recibido del BMS por CAN (ID 10, byte 0, bit 2).
int stsBrake, stsBrake2, stsVbatRAW;     // Lecturas de freno y voltaje de batería.
byte canBMSStatus[1];                    // Buffer de recepción del estado del BMS (ID 10).
uint32_t lastInverterMsg = 0;            // Timestamp del último mensaje recibido del inversor.
uint32_t lastBMSMsg = 0;                 // Timestamp de la última trama de estado del BMS (para watchdog SDC).
uint32_t lastDebug = 0;                  // Timestamp del último debug enviado por serie.

int cfgBrakeTH = 550;                    // Umbral de freno (valor ADC).

int cfgCurrentACMAX = 190;               // Corriente AC máxima (Apk).
int cfgCurrentDCMAX = 60;                // Corriente DC máxima (Adc).
int cfgRPMax        = 1500;              // RPM máximo (ERPM target).

bool debugEnabled = false;   // Por defecto, debug desactivado


// ===================== IDS CAN =====================
// Identificadores CAN para comandos y estados del inversor.
uint32_t idCmdRPM             = combineInts(0x1C, NODE_ID);
uint32_t idCmdEN              = combineInts(0x24, NODE_ID);
uint32_t idCmdCurrentPCTG     = combineInts(0x1E, NODE_ID);
uint32_t idCmdSetMaxACCurrent = combineInts(0x20, NODE_ID);
uint32_t idCmdSetMaxDCCurrent = combineInts(0x22, NODE_ID);

uint32_t id0StsInverter = combineInts(0x00, NODE_ID);
uint32_t id1StsInverter = combineInts(0x01, NODE_ID);
uint32_t id2StsInverter = combineInts(0x02, NODE_ID);
uint32_t id3StsInverter = combineInts(0x03, NODE_ID);
uint32_t id4StsInverter = combineInts(0x04, NODE_ID);

// ===================== ESTADOS VCU =====================
// Identificadores CAN para publicar estados de la VCU.
unsigned long int idAPPSState  = 1163;
unsigned long int idBrakeState = 1164;
unsigned long int idVCUSignals = 1166;

uint16_t CANAppsState[4];   // Buffer para enviar estado de APPS.
uint16_t CANBrakeState[4];  // Buffer para enviar estado de freno.
uint8_t  CANVCUSignals[8];  // Buffer para enviar señales de la VCU.

// ===================== COMANDOS =====================
// Buffers de datos que se envían al inversor por CAN.
int32_t cmdDataRPM[2]          = {0, (int32_t)UNUSED_INT32};
int16_t cmdDataCurrent[4]      = {0, UNUSED_SHORT, UNUSED_SHORT, UNUSED_SHORT};
int16_t cmdDataCurrentACMax[4] = {0, UNUSED_SHORT, UNUSED_SHORT, UNUSED_SHORT};
int16_t cmdDataCurrentDCMax[4] = {0, UNUSED_SHORT, UNUSED_SHORT, UNUSED_SHORT};
byte    cmdDataDriveEN[8]      = {0, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE};

// ===================== MODO DE CONTROL =====================
enum ControlMode { MODE_CAN, MODE_DIRECT }; // Dos modos de control: por CAN o directo (GPIO).
ControlMode controlMode = MODE_DIRECT;   // ← empieza en modo DIRECTO;         // Por defecto, se usa CAN.

// ===================== SIMULACIÓN =====================
// Perfiles de simulación para pruebas sin hardware real.
enum SimProfile {
  SIM_OFF = 0,             // Sin simulación.
  SIM_APPS_STEP = 1,       // Simulación de acelerador progresivo.
  SIM_FAULT_OVERVOLTAGE = 2, // Simulación de fallo por sobretensión.
  SIM_FAULT_UNDERVOLTAGE = 3, // Simulación de fallo por subtensión.
  SIM_FAULT_CTRL_OVERTEMP = 4, // Simulación de fallo por sobretemperatura del controlador.
  SIM_FAULT_MOTOR_OVERTEMP = 5, // Simulación de fallo por sobretemperatura del motor.
  SIM_RANDOM = 6           // Simulación aleatoria de valores.
};
SimProfile simProfile = SIM_OFF; // Estado inicial: simulación desactivada.
uint32_t simStepT = 0;           // Timestamp auxiliar para pasos de simulación.
int simAppsProgress = 0;         // Progreso de simulación de APPS.

// ===================== FUNCIONES AUX =====================
// Construye la configuración del par APPS (calibración, escalado, filtrado, plausibilidad).
PairedAnalogSensorConfig buildAppsConfig() {
  PairedAnalogSensorConfig c;
  // APPS1 (normal: el ADC sube con el pedal). Reposo 1055 -> Fondo 1935.
  c.cfgSensor1.cfgAdcMinNormal    = 1055;
  c.cfgSensor1.cfgAdcMaxNormal    = 1935;
  c.cfgSensor1.cfgScaledOutputMin = 0;
  c.cfgSensor1.cfgScaledOutputMax = 1000;
  c.cfgSensor1.cfgFilterType      = FilterType::EWMA;
  c.cfgSensor1.cfgFilterAlpha     = 0.2;
  // APPS2 (INVERSO: el ADC baja con el pedal). Reposo 2290 -> Fondo 2068.
  c.cfgSensor2.cfgAdcMinNormal    = 2290;
  c.cfgSensor2.cfgAdcMaxNormal    = 2068;
  c.cfgSensor2.cfgScaledOutputMin = 0;
  c.cfgSensor2.cfgScaledOutputMax = 1000;
  c.cfgSensor2.cfgFilterType      = FilterType::EWMA;
  c.cfgSensor2.cfgFilterAlpha     = 0.2;
  // Coherencia del par: implausible si difieren >10% durante >100 ms (FSAE T.4.2.4).
  c.cfgMaxDeviationPercent = 10.0;
  c.cfgDeviationTimeout    = 100;

  // Auto-calibración por voltaje (AnalogSensor): rellenar cuando haya datos de
  // caracterización del APPS a varios voltajes de batería LV. Ejemplo:
  //   c.cfgSensor1.cfgVoltageCalibrationTable = {
  //       {11.5f, 1040, 1920}, {12.0f, 1055, 1935}, {13.0f, 1075, 1960} };
  //   c.cfgSensor2.cfgVoltageCalibrationTable = {
  //       {11.5f, 2300, 2075}, {12.0f, 2290, 2068}, {13.0f, 2280, 2060} };
  // Con la tabla vacía se usan los límites estáticos cfgAdcMinNormal/MaxNormal.
  return c;
}

// Convierte la lectura cruda de batería (stsVbatRAW) a voltios para la auto-calibración.
// TODO: ajustar VBAT_DIVIDER al divisor real de la placa. Mientras las tablas de voltaje
// estén vacías este valor no afecta (AnalogSensor usa los límites estáticos).
static float vbatVolts() {
  const float VBAT_DIVIDER = 1.0f;                              // relación real Vbat/Vadc (PENDIENTE)
  float vadc = (stsVbatRAW * (ADC_VREF / 1000.0f)) / 4095.0f;   // ADC_VREF en mV -> V en el pin
  return vadc * VBAT_DIVIDER;
}
// Precondición: SDC presente (recibido del BMS por CAN). Con el SDC activo, al pulsar
// Start con el freno pisado -> Ready-to-Drive (con buzzer). Si el SDC cae en cualquier
// momento, vuelve a reposo. (TSON se añadirá en la próxima iteración de PCB.)
bool R2D(bool sdc, bool start, bool brake) {
  static int step = 0;                  // Estado interno de la máquina de estados (0=idle, 10=espera, 20=activo).
  static uint32_t tAux = millis();      // Marca de tiempo para controlar el buzzer.
  bool r2d = false;                     // Valor de salida: indica si el sistema está en Ready-to-Drive.

  // Apaga el buzzer si ya pasó el tiempo definido.
  if ((millis() - tAux) >= BUZZER_ON_MS) digitalWrite(pinBUZZ, false);

  switch (step) {
    case 0:                             // Estado inicial: espera a que el SDC esté presente.
      if (sdc) step = 10;
      break;

    case 10:                            // Estado de espera: requiere que el SDC siga presente.
      if (!sdc) step = 0;               // Si el SDC se cae, vuelve a estado inicial.
      else if (start && brake) {        // Si se pulsa Start y el freno está presionado:
        tAux = millis();                // Guarda tiempo actual.
        digitalWrite(pinBUZZ, true);    // Activa buzzer.
        step = 20;                      // Pasa a estado Ready-to-Drive.
      }
      break;

    case 20:                            // Estado activo: Ready-to-Drive.
      if (!sdc) step = 0;               // Si el SDC se cae, vuelve a estado inicial.
      r2d = true;                       // Señal de salida: sistema listo para conducir.
      break;
  }
  return r2d;
}


// ===================== EEPROM =====================
void guardarEEPROM(int direccion, int valor) { EEPROM.put(direccion, valor); }
int  leerEEPROM(int direccion) { int valor; EEPROM.get(direccion, valor); return valor; }
void cargarConfiguracionesEEPROM() { /* ... como antes ... */ }
void resetEEPROM() { /* ... */ }
void mostrarConfiguracionesEEPROM() { /* ... */ }
void aplicarConfiguraciones() { /* ... */ }

void setup() {
  Serial.begin(115200);                 // Inicializa puerto serie para debug.                
  lastDebug = millis();                 // Marca de tiempo inicial para debug.
  lastInverterMsg = millis();           // Marca de tiempo inicial para watchdog CAN.
  lastBMSMsg = millis();                // Marca de tiempo inicial para watchdog del BMS (SDC).

  // Configuración de pines.
  pinMode(pinStart, INPUT_PULLUP);
  pinMode(pinTSON, INPUT);
  pinMode(pinTSON_EXT, INPUT);
  pinMode(pinBUZZ, OUTPUT);
  pinMode(pinSDC, INPUT);
  pinMode(pinR2D_Digital, OUTPUT);
  digitalWrite(pinR2D_Digital, LOW);

  // Inicializa EEPROM y carga configuraciones.
  EEPROM.begin(EEPROM_SIZE);
  cargarConfiguracionesEEPROM();
  mostrarConfiguracionesEEPROM();

  // Inicializa SPI para el ADC MCP3208.
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  SPISettings settings(ADC_CLK, MSBFIRST, SPI_MODE0);
  SPI.begin();
  SPI.beginTransaction(settings);

  // Inicializa LED NeoPixel.
  pixels.begin();
  pixels.clear();
  pixels.show();

  // La auto-calibración del APPS la hace AnalogSensor internamente por tabla de voltaje
  // (ver buildAppsConfig). Con la tabla vacía se usan los límites estáticos.
  aplicarConfiguraciones();

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
}


// ===================== SIMULACIÓN =====================
void runSimulation() {
  stsSDC = true;        // En simulación damos el SDC por presente para poder probar el R2D.
  lastBMSMsg = millis();
  switch (simProfile) {
    case SIM_APPS_STEP:
      if (millis() - simStepT > 50) {
        simStepT = millis();
        simAppsProgress = min(simAppsProgress + 10, 1000);
      }
      rawApps1 = map(simAppsProgress, 0, 1000, 1100, 1900);
      rawApps2 = map(simAppsProgress, 0, 1000, 2300, 2050);
      stsBrake = 300; stsBrake2 = 300; stsVbatRAW = 360;
      { short fakeTempsOK[4] = {350, 280, 0, (short)UNUSED_SHORT};
        CAN.setPacket(id2StsInverter, fakeTempsOK, 4);
        byte fakeDrive[8] = { (byte)(simAppsProgress/10), 0, 0, 1, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, 23 };
        CAN.setPacket(id4StsInverter, fakeDrive, 8);
      }
      break;

    case SIM_FAULT_OVERVOLTAGE:
    case SIM_FAULT_UNDERVOLTAGE:
    case SIM_FAULT_CTRL_OVERTEMP:
    case SIM_FAULT_MOTOR_OVERTEMP: {
      uint8_t faultCode = 0;
      if (simProfile == SIM_FAULT_OVERVOLTAGE)       faultCode = 1;
      else if (simProfile == SIM_FAULT_UNDERVOLTAGE) faultCode = 2;
      else if (simProfile == SIM_FAULT_CTRL_OVERTEMP)faultCode = 5;
      else if (simProfile == SIM_FAULT_MOTOR_OVERTEMP)faultCode = 6;

      rawApps1 = 1600;
      rawApps2 = 2150;
      stsBrake = 300; stsBrake2 = 300; stsVbatRAW = 360;

      short fakeTempsFail[4] = {400, 350, (short)faultCode, (short)UNUSED_SHORT};
      CAN.setPacket(id2StsInverter, fakeTempsFail, 4);
      { byte fakeDrive[8] = {50, 0, 0, 0, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, 23};
        CAN.setPacket(id4StsInverter, fakeDrive, 8);
      }
      break;
    }

    case SIM_RANDOM:
    default:
      rawApps1 = random(1000, 2000);
      rawApps2 = random(1000, 2000);
      stsBrake   = random(200, 800);
      stsBrake2  = random(200, 800);
      stsVbatRAW = random(300, 400);
      { short fakeTempsRnd[4] = {350, 280, 0, (short)UNUSED_SHORT};
        CAN.setPacket(id2StsInverter, fakeTempsRnd, 4);
        byte fakeDrive[8] = { (byte)random(0, 100), 0, 0, 1, UNUSED_BYTE, UNUSED_BYTE, UNUSED_BYTE, 23 };
        CAN.setPacket(id4StsInverter, fakeDrive, 8);
      }
      break;
  }
}

// ===================== CONTROL INVERSOR =====================
void controlInverter() {
  stsStart = !digitalRead(pinStart);
  // stsSDC se actualiza en loop() desde el BMS por CAN (ya no se lee TSON por GPIO).

  // Procesa el par APPS: filtra, escala (0..1000) y detecta implausibilidad
  // (corto a GND/VCC, fuera de rango y desviación >10% entre ambos durante >100 ms).
  float appsMeanF, appsMeanS, appsSensF, appsSensS;
  appsSensor.update(rawApps1, rawApps2, vbatVolts(), appsMeanF, appsMeanS, appsSensF, appsSensS, appsState);
  bool appsOk = (appsState == SensorState::NORMAL);
  int  appsThrottle = (int)appsMeanS;   // Consigna 0..1000 (la clase ya devuelve 0 si hay implausibilidad).

  stsR2D  = R2D(stsSDC, stsStart, (stsBrake2 >= cfgBrakeTH));
  digitalWrite(pinR2D_Digital, stsR2D ? HIGH : LOW);

  if (stsR2D && appsOk) {
    if (controlMode == MODE_CAN) {
      cmdDataDriveEN[0] = 1;
      CAN.setPacket(idCmdEN, cmdDataDriveEN, 1);

      int currentTarget = appsThrottle;
      cmdDataCurrent[0]      = (int16_t)currentTarget;
      cmdDataCurrentACMax[0] = (int16_t)(cfgCurrentACMAX * 10);
      cmdDataCurrentDCMax[0] = (int16_t)(cfgCurrentDCMAX * 10);
      CAN.setPacket(idCmdCurrentPCTG, cmdDataCurrent, 2);
      CAN.setPacket(idCmdSetMaxACCurrent, cmdDataCurrentACMax, 4);
      CAN.setPacket(idCmdSetMaxDCCurrent, cmdDataCurrentDCMax, 4);

      cmdDataRPM[0] = map(appsThrottle, 0, 1000, 0, cfgRPMax * 10);
      CAN.setPacket(idCmdRPM, cmdDataRPM, 2);

    } else {
      digitalWrite(pinR2D_Digital, HIGH);
    }
  } else {
    if (controlMode == MODE_CAN) {
      cmdDataDriveEN[0] = 0;
      CAN.DataOUT.removePacket(idCmdEN);
      CAN.DataOUT.removePacket(idCmdCurrentPCTG);
      CAN.DataOUT.removePacket(idCmdSetMaxACCurrent);
      CAN.DataOUT.removePacket(idCmdSetMaxDCCurrent);
      CAN.DataOUT.removePacket(idCmdRPM);
    } else {
      digitalWrite(pinR2D_Digital, LOW);
    }
  }

  // Publicación de estados VCU
  CANAppsState[0] = (uint16_t)appsSensor.getSensor1().getScaledValue();
  CANAppsState[1] = (uint16_t)appsSensor.getSensor2().getScaledValue();
  CANAppsState[2] = rawApps1;
  CANAppsState[3] = rawApps2;

  CANBrakeState[0] = 0;
  CANBrakeState[1] = 0;
  CANBrakeState[2] = stsBrake;
  CANBrakeState[3] = stsBrake2;

  CANVCUSignals[0] = (uint8_t)stsVbatRAW;
  CANVCUSignals[1] = (uint8_t)stsSDC;
  CANVCUSignals[2] = (uint8_t)stsStart;
  CANVCUSignals[6] = (uint8_t)stsR2D;

  CAN.setPacket(idAPPSState, CANAppsState, 4);
  CAN.setPacket(idBrakeState, CANBrakeState, 4);
  CAN.setPacket(idVCUSignals, CANVCUSignals, 8);
  CAN.send();
}

// ===================== WATCHDOG CAN =====================
void watchdogCAN() {
  if (controlMode == MODE_CAN && !CAN.config.simulating && (millis() - lastInverterMsg > INVERTER_WD_MS)) {
    Serial.println("[WD] Inverter RX timeout. Modo seguro.");
    cmdDataDriveEN[0] = 0;
    CAN.DataOUT.removePacket(idCmdEN);
    CAN.DataOUT.removePacket(idCmdCurrentPCTG);
    CAN.DataOUT.removePacket(idCmdSetMaxACCurrent);
    CAN.DataOUT.removePacket(idCmdSetMaxDCCurrent);
    CAN.DataOUT.removePacket(idCmdRPM);
    digitalWrite(pinR2D_Digital, LOW);
    CAN.send();
  }
}

// ===================== LECTURA DE ESTADO DEL INVERSOR =====================
void readInverterStatus() {
  static uint32_t tAux = millis();

  if (CAN.getPacket(id2StsInverter, stsInverterCAN_22_FULL, 8)) {
    lastInverterMsg = millis();
    stsInverterCAN_FaultCode = stsInverterCAN_22_FULL[4];
  }
  if (CAN.getPacket(id4StsInverter, stsInverterCAN_24_FULL, 8)) {
    lastInverterMsg = millis();
    stsInverterCAN_DriveEnable = stsInverterCAN_24_FULL[3];
  }

  if ((millis() - tAux) >= 1000) {
    Serial.print("Fault: "); Serial.println(getErrorMessage(stsInverterCAN_FaultCode));
    if (stsInverterCAN_DriveEnable == 1)      Serial.println("Drive enable OK");
    else if (stsInverterCAN_DriveEnable == 0) Serial.println("Drive enable FAIL");
    else                                      Serial.println("Drive enable UNKNOWN");

    if (stsInverterCAN_FaultCode != 0) pixels.setPixelColor(0, pixels.Color(255, 0, 0));
    else pixels.setPixelColor(0, pixels.Color(0, 0, 0));
    pixels.show();

    tAux = millis();
  }
}
// ===================== DEBUG =====================
void debug() {
  if ((millis() - lastDebug) < DEBUG_PERIOD_MS) return;
  lastDebug = millis();

  uint32_t canAgeMs = millis() - lastInverterMsg;

  Serial.println("\n=== DEBUG VCU ===");
  Serial.print("Mode: "); Serial.println(controlMode == MODE_CAN ? "CAN" : "DIRECT");
  Serial.print("R2D: "); Serial.print(stsR2D ? "ON" : "OFF");
  Serial.print(" | APPS status: ");
  switch (appsState) {
    case SensorState::NORMAL:         Serial.print("OK"); break;
    case SensorState::PENDING:        Serial.print("PENDING"); break;
    case SensorState::IMPLAUSIBILITY: Serial.print("IMPLAUSIBLE"); break;
  }
  Serial.print(" | SDC: "); Serial.print(stsSDC);
  Serial.print(" (BMS age "); Serial.print(millis() - lastBMSMsg); Serial.print(" ms)");
  Serial.print(" | START: "); Serial.println(stsStart);

  Serial.print("APPS1 analog: "); Serial.print(appsSensor.getSensor1().getRawValue());
  Serial.print(" | scaled: ");   Serial.println(appsSensor.getSensor1().getScaledValue());
  Serial.print("APPS2 analog: "); Serial.print(appsSensor.getSensor2().getRawValue());
  Serial.print(" | scaled: ");    Serial.println(appsSensor.getSensor2().getScaledValue());
  Serial.print("APPS throttle (media): "); Serial.println(appsSensor.getMeanScaledValue());

  Serial.print("Brake1: "); Serial.print(stsBrake);
  Serial.print(" | Brake2: "); Serial.println(stsBrake2);
  Serial.print("Vbat RAW: "); Serial.println(stsVbatRAW);

  Serial.print("Limits -> ACmax: "); Serial.print(cfgCurrentACMAX);
  Serial.print(" | DCmax: ");        Serial.print(cfgCurrentDCMAX);
  Serial.print(" | RPMmax: ");       Serial.println(cfgRPMax);

  Serial.print("Inverter -> DriveEnable: ");
  if (stsInverterCAN_DriveEnable == 1)      Serial.print("ON");
  else if (stsInverterCAN_DriveEnable == 0) Serial.print("OFF");
  else                                      Serial.print("UNKNOWN");
  Serial.print(" | Fault: "); Serial.print(stsInverterCAN_FaultCode);
  Serial.print(" ("); Serial.print(getErrorMessage(stsInverterCAN_FaultCode)); Serial.print(")");
  Serial.print(" | last msg age: "); Serial.print(canAgeMs); Serial.println(" ms");

  Serial.print("Sim: ");
  if (CAN.config.simulating) {
    Serial.print("ON | profile="); Serial.println((int)simProfile);
  } else {
    Serial.println("OFF");
  }
  Serial.println("-------------------------");
}

// ===================== MENÚ =====================
void serialMenu() {
  Serial.println("\n=== MENÚ DIAGNÓSTICO VCU ===");
  Serial.println("8. Configurar límites y guardar en EEPROM");
  Serial.println("11. Resetear EEPROM a valores por defecto");
  Serial.println("12. Mostrar configuraciones actuales (EEPROM)");
  Serial.println("13. Reaplicar configuraciones EEPROM al inversor");
  Serial.println("14. Activar/Desactivar modo simulación");
  Serial.println("15. Seleccionar perfil de simulación");
  Serial.println("16. Seleccionar modo de control (CAN / DIRECTO)");
  Serial.println("17. Activar/Desactivar debug");
  Serial.println("0. Salir del menú");
  Serial.print("Opción: ");
}

void configurarLimitesEEPROM_interactivo() {
  Serial.print("Corriente AC máx (Apk): ");  while (!Serial.available()) {}
  cfgCurrentACMAX = Serial.parseInt(); guardarEEPROM(EEPROM_ADDR_ACMAX, cfgCurrentACMAX);

  Serial.print("Corriente DC máx (Adc): "); while (!Serial.available()) {}
  cfgCurrentDCMAX = Serial.parseInt(); guardarEEPROM(EEPROM_ADDR_DCMAX, cfgCurrentDCMAX);

  Serial.print("RPM máx (ERPM target): "); while (!Serial.available()) {}
  cfgRPMax = Serial.parseInt(); guardarEEPROM(EEPROM_ADDR_RPMAX, cfgRPMax);

  aplicarConfiguraciones();
}

void processMenu() {
  if (!Serial.available()) return;

  // Leemos el primer carácter
  char c = Serial.peek();   // Miramos sin consumir
  if (isAlpha(c)) {         // Si es una letra
    Serial.read();          // Consumimos la letra
    if (c == 'M' || c == 'm') {
      serialMenu();         // Mostrar menú
      return;               // Salimos
    }
  }

  // Si no era letra, tratamos como número
  int opcion = Serial.parseInt();
  Serial.println();

  switch (opcion) {
    case 8: configurarLimitesEEPROM_interactivo(); break;
    case 11: resetEEPROM(); break;
    case 12: mostrarConfiguracionesEEPROM(); break;
    case 13: aplicarConfiguraciones(); break;
    case 14:
      CAN.config.simulating = !CAN.config.simulating;
      if (!CAN.config.simulating) simProfile = SIM_OFF;
      else { simProfile = SIM_APPS_STEP; simAppsProgress = 0; simStepT = millis(); }
      Serial.println(CAN.config.simulating ? "Modo SIMULACIÓN activado" : "Modo SIMULACIÓN desactivado");
      break;
    case 15:
      Serial.println("Selecciona perfil: 1=APPS_STEP, 2=OVERVOLTAGE, 3=UNDERVOLTAGE, 4=CTRL_OVERTEMP, 5=MOTOR_OVERTEMP, 6=RANDOM");
      while (!Serial.available()) {}
      { int sel = Serial.parseInt();
        if (sel >= 1 && sel <= 6) simProfile = (SimProfile)sel;
        else simProfile = SIM_APPS_STEP;
        Serial.print("Perfil sim: "); Serial.println((int)simProfile);
      }
      break;
    case 16:
      Serial.println("Selecciona modo: 1=CAN, 2=Directo");
      while (!Serial.available()) {}
      { int sel = Serial.parseInt();
        controlMode = (sel == 2) ? MODE_DIRECT : MODE_CAN;
        Serial.println(controlMode == MODE_CAN ? "Modo CAN activado" : "Modo DIRECTO activado");
      }
      break;
    case 17:
      debugEnabled = !debugEnabled;
      Serial.println(debugEnabled ? "Debug activado" : "Debug desactivado");
      break;

    case 0: Serial.println("Menú cerrado."); break;
    default: Serial.println("Opción inválida."); break;
  }

  while (Serial.available()) Serial.read();
  serialMenu();
}