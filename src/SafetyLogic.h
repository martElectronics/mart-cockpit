#pragma once
//==============================================================================
// SafetyLogic — funciones PURAS de seguridad del VCU, SIN dependencias de
// Arduino/HAL/CAN. Reciben datos y devuelven datos: nada de estado global ni I/O.
// Así se pueden testear en native (host) de forma determinista. El firmware
// (main.cpp / InverterControl.cpp) las llama; los tests las incluyen directas.
//==============================================================================

// ¿Ha expirado el watchdog del BMS? true si han pasado MÁS de 'timeout' ms desde
// la última trama del BMS (last). Con true, el VCU debe considerar el SDC NO
// presente (fail-safe: sin tramas, no se fía). La resta en unsigned es robusta
// ante el wrap de millis() (~49 días).
inline bool bmsWatchdogExpired(unsigned long last, unsigned long now, unsigned long timeout) {
    return (now - last) > timeout;
}

// Factor [0..1] de corte SUAVE por subtensión del pack, a multiplicar por el
// throttle:
//   vdc <= vmin              -> 0   (corte total)
//   vmin < vdc < vmin+ramp   -> rampa lineal
//   vdc >= vmin+ramp         -> 1   (par pleno)
// Evita el tironeo del corte duro cuando el pack hace sag bajo carga (cae por
// debajo de Vmin → corta → recupera → vuelve → cae...).
inline float underVoltageScale(float vdc, float vmin, float ramp) {
    if (ramp <= 0.0f)        return (vdc < vmin) ? 0.0f : 1.0f; // sin rampa -> corte duro
    if (vdc <= vmin)         return 0.0f;
    if (vdc >= vmin + ramp)  return 1.0f;
    return (vdc - vmin) / ramp;
}
