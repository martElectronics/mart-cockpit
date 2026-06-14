#pragma once
//==============================================================================
// Shim mínimo de Arduino.h para compilar/testear AnalogSensor y PairedAnalogSensor
// en HOST (entorno native_test). Solo aporta los tipos y millis() que usan esas
// clases. El test define millis() para poder controlar el tiempo.
//==============================================================================
#include <cstdint>
#include <cstddef>
#include <cmath>
#include <algorithm>

typedef uint8_t byte;

// El test (o el runner) debe definir millis(). Permite simular el paso del tiempo
// para validar los timeouts de implausibilidad/desviación de forma determinista.
unsigned long millis();

// --- GPIO digital: no-op en HOST, pero registra el último valor escrito por pin
//     en _pinState para poder verificar salidas (p. ej. el buzzer de R2D) en los
//     tests. Sin efecto sobre las clases que no usan GPIO. ---
#define LOW           0
#define HIGH          1
#define INPUT         0
#define OUTPUT        1
#define INPUT_PULLUP  2

inline uint8_t _pinState[64] = {0};   // último valor escrito a cada pin (índice = nº pin)
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t pin, uint8_t val) { if (pin < 64) _pinState[pin] = val; }
inline int  digitalRead(uint8_t pin) { return pin < 64 ? _pinState[pin] : 0; }
