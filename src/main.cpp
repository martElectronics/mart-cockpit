#include <Arduino.h>              // Librería base de Arduino: funciones de GPIO, Serial, millis(), etc.
#include <MART_CAN.h>             // Librería MART_CAN: gestión de comunicación CAN (envío/recepción de frames).
#include <SPI.h>                  // Librería SPI: comunicación con periféricos por bus SPI (ej. ADC).
#include <EEPROM.h>               // Librería EEPROM: lectura/escritura de memoria no volátil.
#include "global.h"               // Archivo propio del proyecto (probablemente define estructuras y constantes).
#include <Mcp320x.h>              // Librería para ADC MCP3208 (conversor analógico-digital de 12 bits).
#include <Adafruit_NeoPixel.h>    // Librería para controlar LEDs RGB tipo NeoPixel.

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

#define UNUSED_BYTE  0xFF         // Valor especial para indicar "no usado" en arrays tipo byte.
#define UNUSED_SHORT ((int16_t)0x7FFF) // Valor especial para "no usado" en arrays tipo short (32767).
#define UNUSED_INT32 0xFFFFFFFF   // Valor especial para "no usado" en arrays tipo int32_t.

// ===================== PROTOTIPOS =====================
// Declaraciones de funciones para que el compilador las conozca antes de usarlas.
void configureAPPS();
bool R2D(bool tson, bool start, bool brake);
int apps(int valAPPS1, int valAPPS2, int difMAX, int max, int valDesc);

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
struct sensorData apps1Data, apps2Data; // Datos de los dos sensores APPS (pedal acelerador).
bool stsTSON, stsStart, stsR2D, stsAPPS; // Estados de TSON, Start, Ready-to-Drive y APPS.
int stsBrake, stsBrake2, stsVbatRAW;     // Lecturas de freno y voltaje de batería.
uint32_t lastInverterMsg = 0;            // Timestamp del último mensaje recibido del inversor.
uint32_t lastDebug = 0;                  // Timestamp del último debug enviado por serie.

int cfgBrakeTH = 550;                    // Umbral de freno (valor ADC).
int cfgAPPSdiff = 100;                   // Diferencia máxima permitida entre APPS1 y APPS2.
int cfgAPPSdesc = 100;                   // Valor mínimo para detectar desconexión de APPS.

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
// Configuración inicial de los sensores APPS (valores de calibración).
void configureAPPS() {
  apps1Data.valAnalogUP   = 1055; // Valor ADC cuando APPS1 está en reposo.
  apps1Data.valAnalogDOWN = 1935; // Valor ADC cuando APPS1 está a fondo.
  apps2Data.valAnalogUP   = 2290; // Valor ADC cuando APPS2 está en reposo.
  apps2Data.valAnalogDOWN = 2068; // Valor ADC cuando APPS2 está a fondo.
  apps1Data.valScaledUP = apps2Data.valScaledUP = 0;     // Escalado mínimo.
  apps1Data.valScaledDOWN = apps2Data.valScaledDOWN = 1000; // Escalado máximo.
}
bool R2D(bool tson, bool start, bool brake) {
  static int step = 0;                  // Estado interno de la máquina de estados (0=idle, 10=espera, 20=activo).
  static uint32_t tAux = millis();      // Marca de tiempo para controlar el buzzer.
  bool r2d = false;                     // Valor de salida: indica si el sistema está en Ready-to-Drive.

  // Apaga el buzzer si ya pasó el tiempo definido.
  if ((millis() - tAux) >= BUZZER_ON_MS) digitalWrite(pinBUZZ, false);

  switch (step) {
    case 0:                             // Estado inicial: espera a que TSON esté activo.
      if (tson) step = 10;
      break;

    case 10:                            // Estado de espera: requiere que TSON siga activo.
      if (!tson) step = 0;              // Si TSON se desactiva, vuelve a estado inicial.
      else if (start && brake) {        // Si se pulsa Start y el freno está presionado:
        tAux = millis();                // Guarda tiempo actual.
        digitalWrite(pinBUZZ, true);    // Activa buzzer.
        step = 20;                      // Pasa a estado Ready-to-Drive.
      }
      break;

    case 20:                            // Estado activo: Ready-to-Drive.
      if (!tson) step = 0;              // Si TSON se desactiva, vuelve a estado inicial.
      r2d = true;                       // Señal de salida: sistema listo para conducir.
      break;
  }
  return r2d;
}


int apps(int valAPPS1, int valAPPS2, int difMAX, int max, int valDesc) {
  int val = abs(valAPPS1 - valAPPS2);   // Diferencia entre los dos sensores APPS.

  if ((valAPPS1 <= valDesc) || (valAPPS2 <= valDesc)) return -1; // Si alguno está por debajo del umbral → desconectado.
  else if (val >= difMAX) return 1;     // Si la diferencia entre ambos supera el máximo permitido → implausible.
  else return 0;                        // Si todo está correcto → OK.
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

  // Configura sensores APPS y aplica configuraciones al inversor.
  configureAPPS();
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
    if (controlMode == MODE_CAN) {
      if (CAN.getPacket(id2StsInverter, stsInverterCAN_22_FULL, 8) ||
          CAN.getPacket(id4StsInverter, stsInverterCAN_24_FULL, 8)) {
        lastInverterMsg = millis();
      }
    }
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
  switch (simProfile) {
    case SIM_APPS_STEP:
      if (millis() - simStepT > 50) {
        simStepT = millis();
        simAppsProgress = min(simAppsProgress + 10, 1000);
      }
      apps1Data.valAnalog = map(simAppsProgress, 0, 1000, 1100, 1900);
      apps2Data.valAnalog = map(simAppsProgress, 0, 1000, 2300, 2050);
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

      apps1Data.valAnalog = 1600;
      apps2Data.valAnalog = 2150;
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
      apps1Data.valAnalog = random(1000, 2000);
      apps2Data.valAnalog = random(1000, 2000);
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
  stsTSON  = digitalRead(pinTSON);

  apps1Data.valScaled = map(apps1Data.valAnalog, apps1Data.valAnalogUP, apps1Data.valAnalogDOWN, 0, 1000);
  apps2Data.valScaled = map(apps2Data.valAnalog, apps2Data.valAnalogUP, apps2Data.valAnalogDOWN, 0, 1000);

  stsAPPS = apps(apps1Data.valScaled, apps2Data.valScaled, cfgAPPSdiff, 0, cfgAPPSdesc);
  stsR2D  = R2D(stsTSON, stsStart, (stsBrake2 >= cfgBrakeTH));
  digitalWrite(pinR2D_Digital, stsR2D ? HIGH : LOW);

  if (stsR2D && stsAPPS == 0) {
    if (controlMode == MODE_CAN) {
      cmdDataDriveEN[0] = 1;
      CAN.setPacket(idCmdEN, cmdDataDriveEN, 1);

      int currentTarget = apps2Data.valScaled;
      cmdDataCurrent[0]      = (int16_t)currentTarget;
      cmdDataCurrentACMax[0] = (int16_t)(cfgCurrentACMAX * 10);
      cmdDataCurrentDCMax[0] = (int16_t)(cfgCurrentDCMAX * 10);
      CAN.setPacket(idCmdCurrentPCTG, cmdDataCurrent, 2);
      CAN.setPacket(idCmdSetMaxACCurrent, cmdDataCurrentACMax, 4);
      CAN.setPacket(idCmdSetMaxDCCurrent, cmdDataCurrentDCMax, 4);

      cmdDataRPM[0] = map(apps1Data.valScaled, 0, 1000, 0, cfgRPMax * 10);
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
  CANAppsState[0] = apps1Data.valScaled;
  CANAppsState[1] = apps2Data.valScaled;
  CANAppsState[2] = apps1Data.valAnalog;
  CANAppsState[3] = apps2Data.valAnalog;

  CANBrakeState[0] = 0;
  CANBrakeState[1] = 0;
  CANBrakeState[2] = stsBrake;
  CANBrakeState[3] = stsBrake2;

  CANVCUSignals[0] = (uint8_t)stsVbatRAW;
  CANVCUSignals[1] = (uint8_t)stsTSON;
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
  if (stsAPPS == 0) Serial.print("OK");
  else if (stsAPPS == 1) Serial.print("DIF (implausible)");
  else if (stsAPPS == -1) Serial.print("DESC (desconectado)");
  else Serial.print("UNKNOWN");
  Serial.print(" | TSON: "); Serial.print(stsTSON);
  Serial.print(" | START: "); Serial.println(stsStart);

  Serial.print("APPS1 analog: "); Serial.print(apps1Data.valAnalog);
  Serial.print(" | scaled: ");   Serial.println(apps1Data.valScaled);
  Serial.print("APPS2 analog: "); Serial.print(apps2Data.valAnalog);
  Serial.print(" | scaled: ");    Serial.println(apps2Data.valScaled);

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