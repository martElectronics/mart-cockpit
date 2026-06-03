#pragma once
//==============================================================================
// InverterController — Encapsula la E/S CAN del inversor DTI HV-500.
//   · Lee el estado (fault code, drive enable) de las tramas 0x441 / 0x481.
//   · Manda los comandos de tracción (drive enable, corriente %, límites AC/DC)
//     o los retira (modo seguro → el inversor entra en timeout).
// Sustituye los globales sueltos (cmdData*, stsInverterCAN_FaultCode/DriveEnable)
// agrupando estado + comportamiento con dueño claro.
//==============================================================================
#include <Arduino.h>
#include <MART_CAN.h>

class InverterController {
public:
  InverterController(CAN_BUS& can,
                     uint32_t idStsMain, uint32_t idStsTemps, uint32_t idStsMisc,
                     uint32_t idCmdEN, uint32_t idCmdCurrent,
                     uint32_t idCmdMaxAC, uint32_t idCmdMaxDC)
    : _can(can), _idStsMain(idStsMain), _idStsTemps(idStsTemps), _idStsMisc(idStsMisc),
      _idCmdEN(idCmdEN), _idCmdCurrent(idCmdCurrent),
      _idCmdMaxAC(idCmdMaxAC), _idCmdMaxDC(idCmdMaxDC) {}

  // Lee las tramas de estado del inversor: 0x401 (eRPM b0-3, Vin b6-7), 0x441
  // (temps/fault, byte 4 = fault code) y 0x481 (throttle/brake/driveEN, byte 3 =
  // drive enable). Refresca el timestamp de "vivo". Devuelve true si llegó alguna.
  bool readStatus() {
    bool got = false;
    if (_can.getPacket(_idStsMain, _rxMain, 8)) {           // 0x401: eRPM (int32 BE) + Vin (int16 BE)
      _erpm = (int32_t)(((uint32_t)_rxMain[0] << 24) | ((uint32_t)_rxMain[1] << 16)
                      | ((uint32_t)_rxMain[2] << 8)  |  (uint32_t)_rxMain[3]);
      _dcVoltage = (int16_t)(((uint16_t)_rxMain[6] << 8) | _rxMain[7]);  // V (escala 1; confirmado manual V2.5 §3.6, ej. 0x0186=390 V)
      _lastMsg = millis(); got = true;
    }
    if (_can.getPacket(_idStsTemps, _rxTemps, 8)) { _faultCode   = _rxTemps[4]; _lastMsg = millis(); got = true; }
    if (_can.getPacket(_idStsMisc,  _rxMisc,  8)) { _driveEnable = _rxMisc[3];  _lastMsg = millis(); got = true; }
    return got;
  }

  // Manda los comandos de tracción (enable + corriente% + límites) o los retira.
  // Valores de límite ya escalados ×10 (Apk / Adc).
  void sendCommands(bool enable, int16_t throttle, int16_t maxAC_x10, int16_t maxDC_x10) {
    if (!enable) { stop(); return; }
    _cmdDriveEN[0] = 1;
    _can.setPacket(_idCmdEN, _cmdDriveEN, 1);
    _cmdCurrent[0] = throttle;
    _cmdMaxAC[0]   = maxAC_x10;
    _cmdMaxDC[0]   = maxDC_x10;
    // 1 sola palabra int16 (bytes 0-1); setPacket rellena 2-7 con 0xFF, como pide
    // el manual §4.2 ("NOT USED, fill with FFs"). Antes se mandaban 2/4 -> dejaba
    // centinelas 0x7F en bytes "no usados".
    _can.setPacket(_idCmdCurrent, _cmdCurrent, 1);
    _can.setPacket(_idCmdMaxAC,   _cmdMaxAC,   1);
    _can.setPacket(_idCmdMaxDC,   _cmdMaxDC,   1);
  }

  // Retira todos los comandos (modo seguro): el inversor deja de recibir y entra
  // en timeout → free running.
  void stop() {
    _cmdDriveEN[0] = 0;
    _can.DataOUT.removePacket(_idCmdEN);
    _can.DataOUT.removePacket(_idCmdCurrent);
    _can.DataOUT.removePacket(_idCmdMaxAC);
    _can.DataOUT.removePacket(_idCmdMaxDC);
  }

  uint8_t  faultCode()   const         { return _faultCode; }
  uint8_t  driveEnable() const         { return _driveEnable; }
  float    dcVoltage()   const         { return (float)_dcVoltage; }   // V del pack (0x401)
  float    erpm()        const         { return (float)_erpm; }        // velocidad eléctrica (0x401)
  uint32_t lastMsg()     const         { return _lastMsg; }
  bool     hasFault()    const         { return _faultCode != 0; }
  bool     isFresh(uint32_t toMs) const { return (millis() - _lastMsg) <= toMs; }

private:
  CAN_BUS&       _can;
  const uint32_t _idStsMain, _idStsTemps, _idStsMisc;
  const uint32_t _idCmdEN, _idCmdCurrent, _idCmdMaxAC, _idCmdMaxDC;

  uint8_t  _faultCode   = 0;
  uint8_t  _driveEnable = 0;
  int32_t  _erpm        = 0;
  int16_t  _dcVoltage   = 0;
  uint32_t _lastMsg     = 0;
  byte     _rxMain[8]   = {0};
  byte     _rxTemps[8]  = {0};
  byte     _rxMisc[8]   = {0};
  // Buffers de comando: slot 0 = dato; resto centinela "no usado" (igual que antes).
  int16_t  _cmdCurrent[4] = {0, (int16_t)0x7FFF, (int16_t)0x7FFF, (int16_t)0x7FFF};
  int16_t  _cmdMaxAC[4]   = {0, (int16_t)0x7FFF, (int16_t)0x7FFF, (int16_t)0x7FFF};
  int16_t  _cmdMaxDC[4]   = {0, (int16_t)0x7FFF, (int16_t)0x7FFF, (int16_t)0x7FFF};
  byte     _cmdDriveEN[8] = {0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
};
