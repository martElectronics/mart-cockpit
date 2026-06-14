#pragma once
//==============================================================================
// SafetyLogic — funciones PURAS de seguridad del VCU, SIN dependencias de
// Arduino/HAL/CAN. Reciben datos y devuelven datos: nada de estado global ni I/O.
// Así se pueden testear en native (host) de forma determinista. El firmware
// (main.cpp / InverterControl.cpp) las llama; los tests las incluyen directas.
//==============================================================================
#include <cstdint>

// ¿Ha expirado el watchdog del BMS? true si han pasado MÁS de 'timeout' ms desde
// la última trama del BMS (last). Con true, el VCU debe considerar el SDC NO
// presente (fail-safe: sin tramas, no se fía).
//
// Tipos uint32_t a propósito: millis() en STM32 devuelve uint32_t, y la resta en
// 32 bits envuelve correctamente al pasar los ~49 días (el wrap de millis()).
// Si la firma fuese 'unsigned long', en un host LP64 (Linux/Mac, long=64 bits) la
// resta NO envolvería como en el target -> el test del wrap daría falso negativo.
// Con uint32_t el comportamiento es idéntico en target y en cualquier host.
// El cast del return es defensivo (por si se compila con tipos más anchos).
inline bool bmsWatchdogExpired(uint32_t last, uint32_t now, uint32_t timeout) {
    return (uint32_t)(now - last) > timeout;
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
