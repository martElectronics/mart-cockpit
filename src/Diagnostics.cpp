#include "Vcu.h"

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
  Serial.println("14. Activar/Desactivar modo simulación");
  Serial.println("15. Seleccionar perfil de simulación");
  Serial.println("16. Seleccionar modo de control (CAN / DIRECTO)");
  Serial.println("17. Activar/Desactivar debug");
  Serial.println("0. Salir del menú");
  Serial.print("Opción: ");
}

void processMenu() {
  if (!Serial.available()) return;

  // Seguridad: el menú (que puede bloquear esperando entrada) solo se usa en parado.
  if (stsR2D) {
    Serial.println("Menú bloqueado: R2D activo.");
    while (Serial.available()) Serial.read();
    return;
  }

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
    case 14:
      CAN.config.simulating = !CAN.config.simulating;
      if (!CAN.config.simulating) simProfile = SIM_OFF;
      else { simProfile = SIM_APPS_STEP; simAppsProgress = 0; simStepT = millis(); }
      Serial.println(CAN.config.simulating ? "Modo SIMULACIÓN activado" : "Modo SIMULACIÓN desactivado");
      break;
    case 15:
      Serial.println("Selecciona perfil: 1=APPS_STEP, 2=OVERVOLTAGE, 3=UNDERVOLTAGE, 4=CTRL_OVERTEMP, 5=MOTOR_OVERTEMP, 6=RANDOM");
      while (!Serial.available()) { esp_task_wdt_reset(); }
      { int sel = Serial.parseInt();
        if (sel >= 1 && sel <= 6) simProfile = (SimProfile)sel;
        else simProfile = SIM_APPS_STEP;
        Serial.print("Perfil sim: "); Serial.println((int)simProfile);
      }
      break;
    case 16:
      Serial.println("Selecciona modo: 1=CAN, 2=Directo");
      while (!Serial.available()) { esp_task_wdt_reset(); }
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
