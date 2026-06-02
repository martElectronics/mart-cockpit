#pragma once
//==============================================================================
// WheelSpeed — Velocidad de rueda por conteo de pulsos (sensor Hall SNDH-H3L-G01
// sobre rueda fónica). Un ISR externo incrementa el contador volátil; updateRpm()
// lo lee sobre la ventana transcurrida y calcula las RPM de rueda.
//   RPM = (pulsos / dientes) / (ventana_min)
//==============================================================================
#include <Arduino.h>

class WheelSpeed {
public:
  WheelSpeed(volatile uint32_t& counter, uint16_t teeth)
    : _counter(counter), _teeth(teeth) {}

  // Llamar cada ventana fija (p.ej. WHEEL_WINDOW_MS). Devuelve RPM de rueda.
  float updateRpm() {
    uint32_t now = millis();
    noInterrupts();
    uint32_t pulses = _counter; _counter = 0;
    interrupts();
    uint32_t dtMs = now - _tLast;
    _tLast = now;
    if (dtMs == 0 || _teeth == 0) return _rpm;
    _rpm = (pulses / (float)_teeth) * (60000.0f / (float)dtMs);
    return _rpm;
  }
  float rpm() const { return _rpm; }

private:
  volatile uint32_t& _counter;
  const uint16_t _teeth;
  uint32_t _tLast = 0;
  float    _rpm   = 0;
};
