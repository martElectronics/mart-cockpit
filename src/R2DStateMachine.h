#pragma once
//==============================================================================
// R2DStateMachine — Máquina de estados Ready-to-Drive.
//   idle ──(SDC presente)──> wait ──(Start + freno)──> active (R2D + buzzer)
//   Si el SDC cae en cualquier estado, vuelve a idle.
// El estado (paso + temporizador de buzzer) es explícito (antes vivía oculto en
// 'static' dentro de la función R2D()).
//==============================================================================
#include <Arduino.h>

class R2DStateMachine {
public:
  R2DStateMachine(uint8_t pinBuzzer, uint32_t buzzerOnMs)
    : _pinBuzzer(pinBuzzer), _buzzerOnMs(buzzerOnMs) {}

  // Llamar cada ciclo. Devuelve true si está en Ready-to-Drive.
  bool update(bool sdc, bool start, bool brake) {
    if ((millis() - _tBuzz) >= _buzzerOnMs) digitalWrite(_pinBuzzer, LOW);  // apaga el buzzer

    switch (_step) {
      case Step::IDLE:
        if (sdc) _step = Step::WAIT;
        break;
      case Step::WAIT:
        if (!sdc) _step = Step::IDLE;                 // se cayó el SDC → reposo
        else if (start && brake) {                    // Start con freno pisado → R2D
          _tBuzz = millis();
          digitalWrite(_pinBuzzer, HIGH);             // pitido de R2D
          _step = Step::ACTIVE;
        }
        break;
      case Step::ACTIVE:
        if (!sdc) _step = Step::IDLE;                 // se cayó el SDC → reposo
        break;
    }
    return _step == Step::ACTIVE;
  }

  void reset() { _step = Step::IDLE; }

private:
  enum class Step { IDLE, WAIT, ACTIVE };
  const uint8_t  _pinBuzzer;
  const uint32_t _buzzerOnMs;
  Step     _step  = Step::IDLE;
  uint32_t _tBuzz = 0;
};
