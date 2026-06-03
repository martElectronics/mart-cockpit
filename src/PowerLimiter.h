#pragma once
//==============================================================================
// PowerLimiter — Límite dinámico de corriente AC por la potencia DC disponible.
// Conforme baja el voltaje del pack, la potencia con el fusible fijo (125 A) cae;
// esta clase calcula la corriente AC máxima permitida para no superar el fusible
// ni el Maximum Wattage del DTI:
//
//   i_ac_max = (V_dc · I_fuse · η) / (K_T · ω_mec)      [limitado por potencia]
//   i_ac_max = min(i_ac_max, I_motor_max)               [nunca sobre el motor]
//
// A baja velocidad ω→0 dispararía la división: se clampea a I_motor (físicamente
// la corriente DC a baja velocidad es despreciable, no amenaza al fusible).
// SIN field weakening: el rango de operación queda por debajo de la velocidad base
// (Max ERPM < base), y el DTI gestiona el FW internamente.
//==============================================================================
#include <Arduino.h>

class PowerLimiter {
public:
  PowerLimiter(float ktEff, float iFuseMax, float iMotorMax,
               float etaInv, float pMaxW, uint8_t polePairs)
    : _kt(ktEff), _iFuse(iFuseMax), _iMotor(iMotorMax),
      _eta(etaInv), _pMax(pMaxW), _poles(polePairs) {}

  // v_dc [V], erpm [eléctricas]. Devuelve la corriente AC máx permitida [Arms].
  float maxAcCurrent(float vDc, float erpm) const {
    float pBat = vDc * _iFuse * _eta;                       // potencia disponible (W)
    float pLim = (pBat < _pMax) ? pBat : _pMax;             // tope del DTI
    float wMec = (erpm / (float)_poles) * (2.0f * PI / 60.0f); // rad/s mecánicos
    float iPow = (wMec > 50.0f) ? (pLim / (_kt * wMec)) : _iMotor;
    return (iPow < _iMotor) ? iPow : _iMotor;
  }

private:
  const float   _kt, _iFuse, _iMotor, _eta, _pMax;
  const uint8_t _poles;
};
