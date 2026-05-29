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
