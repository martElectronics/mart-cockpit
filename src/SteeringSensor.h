#pragma once
//==============================================================================
// SteeringSensor — PSC-360 (potenciómetro de hilo / string pot) que mide la
// dirección. Convierte la cuenta ADC a % de -100 (izquierda) a +100 (derecha)
// con 3 puntos de calibración (izq / centro / der).
//==============================================================================
#include <Arduino.h>

class SteeringSensor {
public:
  SteeringSensor(int adcLeft, int adcCenter, int adcRight)
    : _left(adcLeft), _center(adcCenter), _right(adcRight) {}

  void update(int raw) { _raw = raw; }
  int  raw()     const { return _raw; }
  int  percent() const {
    int p = (_raw <= _center) ? map(_raw, _left, _center, -100, 0)
                              : map(_raw, _center, _right,  0, 100);
    return constrain(p, -100, 100);
  }

private:
  const int _left, _center, _right;
  int _raw = 0;
};
