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

  // ---- Límite dinámico de potencia (conforme baja V_dc del pack) ----
  // Capa el throttle relativo (0..1000) a la corriente AC máx que permite el
  // fusible/Max Wattage al voltaje actual. Solo con telemetría fresca y plausible
  // del inversor; si falta, se delega en los límites internos del DTI.
  if (inverter.isFresh(INVERTER_WD_MS) && inverter.dcVoltage() > 10.0f) {
    float vdc    = inverter.dcVoltage();
    float iAcMax = powerLimiter.maxAcCurrent(vdc, inverter.erpm());
    int   cap    = (int)((iAcMax / (float)cfgCurrentACMAX) * 1000.0f);
    if (appsThrottle > cap) appsThrottle = cap;
    // Subtensión con corte SUAVE (rampa) en vez de corte duro: par pleno en
    // Vmin+ramp, par nulo en Vmin. Evita el tironeo cuando el pack hace sag bajo
    // carga (cae por debajo → corta → recupera → vuelve → cae...).
    appsThrottle = (int)(appsThrottle * underVoltageScale(vdc, cfgVPackMinOp, cfgVPackRampV));
  }

  stsR2D = r2dSM.update(stsSDC, stsStart, (stsBrake2 >= cfgBrakeTH));

  // ---- Fuente única de verdad para habilitar par ----
  // Una sola condición gobierna TANTO el pin digital DriveEnable COMO el comando CAN.
  // (Antes el pin seguía a stsR2D a secas e ignoraba el APPS en modo CAN.)
  bool inverterFault = inverter.hasFault();
  // En modo CAN el par exige telemetría FRESCA del inversor: si deja de transmitir,
  // hasFault() devuelve el último fault conocido (probablemente 0) y driveEnabled
  // quedaría true → sendCommands() remete los paquetes y pone el pin a HIGH, y acto
  // seguido watchdogCAN() los retira y lo baja: aleteo cada ciclo. Incluir la frescura
  // aquí lo evita y deja watchdogCAN() como cinturón redundante.
  bool invAlive      = (controlMode != MODE_CAN) || inverter.isFresh(INVERTER_WD_MS);
  bool driveEnabled  = stsR2D && appsOk && !inverterFault && invAlive;

  digitalWrite(pinR2D_Digital, driveEnabled ? HIGH : LOW);

  // Control SOLO por corriente (Set Relative current). NO se manda Set ERPM: el
  // manual DTI conmuta el inversor a control por velocidad con ese comando, y
  // mezclarlo lo haría cambiar de modo cada ciclo.
  if (controlMode == MODE_CAN) {
    inverter.sendCommands(driveEnabled, (int16_t)appsThrottle,
                          (int16_t)(cfgCurrentACMAXApk * 10),
                          (int16_t)(cfgCurrentDCMAX * 10));
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
  bool invFresh = inverter.isFresh(INVERTER_WD_MS);
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
  CANVCUDiag[4] = inverter.faultCode();
  CANVCUDiag[5] = (uint8_t)(appsThrottle / 10);        // 0..100 %
  CANVCUDiag[6] = (uint8_t)(heartbeat >> 8);
  CANVCUDiag[7] = (uint8_t)(heartbeat & 0xFF);
  CAN.setPacket(idVCUDiag, CANVCUDiag, 8);

  CAN.send();
}

// ===================== WATCHDOG CAN =====================
void watchdogCAN() {
  if (controlMode == MODE_CAN && !CAN.config.simulating && !inverter.isFresh(INVERTER_WD_MS)) {
    Serial.println("[WD] Inverter RX timeout. Modo seguro.");
    inverter.stop();
    digitalWrite(pinR2D_Digital, LOW);
    CAN.send();
  }
}

// ===================== LECTURA DE ESTADO DEL INVERSOR =====================
void readInverterStatus() {
  static uint32_t tAux = millis();

  inverter.readStatus();

  if ((millis() - tAux) >= 1000) {
    Serial.print("Fault: "); Serial.println(getErrorMessage(inverter.faultCode()));
    if (inverter.driveEnable() == 1)      Serial.println("Drive enable OK");
    else if (inverter.driveEnable() == 0) Serial.println("Drive enable FAIL");
    else                                  Serial.println("Drive enable UNKNOWN");

    // (Indicador LED retirado en STM32; el AMS/rojo lo gestiona el HW del SDC.)

    tAux = millis();
  }
}
