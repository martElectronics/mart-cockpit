#include "Vcu.h"

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

  stsR2D = R2D(stsSDC, stsStart, (stsBrake2 >= cfgBrakeTH));

  // ---- Fuente única de verdad para habilitar par ----
  // Una sola condición gobierna TANTO el pin digital DriveEnable COMO el comando CAN.
  // (Antes el pin seguía a stsR2D a secas e ignoraba el APPS en modo CAN.)
  bool inverterFault = (stsInverterCAN_FaultCode != 0);
  bool driveEnabled  = stsR2D && appsOk && !inverterFault;

  digitalWrite(pinR2D_Digital, driveEnabled ? HIGH : LOW);

  if (controlMode == MODE_CAN) {
    if (driveEnabled) {
      cmdDataDriveEN[0] = 1;
      CAN.setPacket(idCmdEN, cmdDataDriveEN, 1);

      cmdDataCurrent[0]      = (int16_t)appsThrottle;
      cmdDataCurrentACMax[0] = (int16_t)(cfgCurrentACMAX * 10);
      cmdDataCurrentDCMax[0] = (int16_t)(cfgCurrentDCMAX * 10);
      CAN.setPacket(idCmdCurrentPCTG, cmdDataCurrent, 2);
      CAN.setPacket(idCmdSetMaxACCurrent, cmdDataCurrentACMax, 4);
      CAN.setPacket(idCmdSetMaxDCCurrent, cmdDataCurrentDCMax, 4);
      // Control SOLO por corriente (Set Relative current 0x1E). NO se manda Set ERPM (0x1C):
      // el manual DTI conmuta el inversor a control por velocidad con 0x1C, y mezclarlo con
      // 0x1E lo haría cambiar de modo cada ciclo. (idCmdRPM queda disponible si algún día se
      // quiere control por velocidad.)
    } else {
      cmdDataDriveEN[0] = 0;
      CAN.DataOUT.removePacket(idCmdEN);
      CAN.DataOUT.removePacket(idCmdCurrentPCTG);
      CAN.DataOUT.removePacket(idCmdSetMaxACCurrent);
      CAN.DataOUT.removePacket(idCmdSetMaxDCCurrent);
    }
  }
  // En MODE_DIRECT el par lo gobierna solo el pin digital (ya fijado arriba).
  // Si el CAN del BMS cae, stsSDC pasa a false (watchdog) -> stsR2D false -> pin LOW.

  // Publicación de estados VCU
  CANAppsState[0] = (uint16_t)appsSensor.getSensor1().getScaledValue();
  CANAppsState[1] = (uint16_t)appsSensor.getSensor2().getScaledValue();
  CANAppsState[2] = rawApps1;
  CANAppsState[3] = rawApps2;

  CANBrakeState[0] = 0;
  CANBrakeState[1] = 0;
  CANBrakeState[2] = stsBrake;
  CANBrakeState[3] = stsBrake2;

  // Layout idVCUSignals (1166): b0-1 Vbat_raw (u16 BE), b2 SDC, b3 Start, b4 R2D,
  // b5 estado APPS (0=NORMAL,1=IMPLAUSIBLE,2=PENDING). ACTUALIZAR config del receptor.
  uint16_t vbatRaw = (uint16_t)stsVbatRAW;
  CANVCUSignals[0] = (uint8_t)(vbatRaw >> 8);
  CANVCUSignals[1] = (uint8_t)(vbatRaw & 0xFF);
  CANVCUSignals[2] = (uint8_t)stsSDC;
  CANVCUSignals[3] = (uint8_t)stsStart;
  CANVCUSignals[4] = (uint8_t)stsR2D;
  CANVCUSignals[5] = (uint8_t)appsState;

  CAN.setPacket(idAPPSState, CANAppsState, 4);
  CAN.setPacket(idBrakeState, CANBrakeState, 4);
  CAN.setPacket(idVCUSignals, CANVCUSignals, 8);

  // ---- Diagnóstico VCU (idVCUDiag 1160) para post-mortem ----
  // b0 causa-no-par, b1 tipo implausibilidad APPS, b2 flags comms/modo, b3 causa de reset,
  // b4 fault code inversor, b5 throttle %, b6-7 heartbeat (u16 BE; se congela si el loop muere).
  bool bmsStale = (millis() - lastBMSMsg) > BMS_WD_MS;
  bool invFresh = (millis() - lastInverterMsg) <= INVERTER_WD_MS;
  uint8_t faultCause = 0;                              // 0 = conduciendo / OK
  if      (driveEnabled)                       faultCause = 0;
  else if (bmsStale)                           faultCause = 5;  // comms BMS perdidas
  else if (!stsSDC)                            faultCause = 2;  // SDC abierto
  else if (inverterFault)                      faultCause = 4;  // fallo del inversor
  else if (appsState != SensorState::NORMAL)   faultCause = 3;  // APPS implausible
  else if (!stsR2D)                            faultCause = 1;  // sin R2D (esperando start+freno)

  CANVCUDiag[0] = faultCause;
  CANVCUDiag[1] = (uint8_t)appsSensor.getImplausibilityType();
  CANVCUDiag[2] = (uint8_t)((!bmsStale ? 0x01 : 0) | (invFresh ? 0x02 : 0)
                | (CAN.config.simulating ? 0x04 : 0) | (controlMode == MODE_CAN ? 0x08 : 0)
                | (debugEnabled ? 0x10 : 0) | (driveEnabled ? 0x20 : 0));
  CANVCUDiag[3] = resetCause;
  CANVCUDiag[4] = stsInverterCAN_FaultCode;
  CANVCUDiag[5] = (uint8_t)(appsThrottle / 10);        // 0..100 %
  CANVCUDiag[6] = (uint8_t)(heartbeat >> 8);
  CANVCUDiag[7] = (uint8_t)(heartbeat & 0xFF);
  CAN.setPacket(idVCUDiag, CANVCUDiag, 8);

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

    // (Indicador LED retirado en STM32; el AMS/rojo lo gestiona el HW del SDC.)

    tAux = millis();
  }
}
