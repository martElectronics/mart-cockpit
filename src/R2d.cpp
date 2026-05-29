#include "Vcu.h"

// Precondición: SDC presente (recibido del BMS por CAN). Con el SDC activo, al pulsar
// Start con el freno pisado -> Ready-to-Drive (con buzzer). Si el SDC cae en cualquier
// momento, vuelve a reposo. (TSON se añadirá en la próxima iteración de PCB.)
bool R2D(bool sdc, bool start, bool brake) {
  static int step = 0;                  // Estado interno de la máquina de estados (0=idle, 10=espera, 20=activo).
  static uint32_t tAux = millis();      // Marca de tiempo para controlar el buzzer.
  bool r2d = false;                     // Valor de salida: indica si el sistema está en Ready-to-Drive.

  // Apaga el buzzer si ya pasó el tiempo definido.
  if ((millis() - tAux) >= BUZZER_ON_MS) digitalWrite(pinBUZZ, false);

  switch (step) {
    case 0:                             // Estado inicial: espera a que el SDC esté presente.
      if (sdc) step = 10;
      break;

    case 10:                            // Estado de espera: requiere que el SDC siga presente.
      if (!sdc) step = 0;               // Si el SDC se cae, vuelve a estado inicial.
      else if (start && brake) {        // Si se pulsa Start y el freno está presionado:
        tAux = millis();                // Guarda tiempo actual.
        digitalWrite(pinBUZZ, true);    // Activa buzzer.
        step = 20;                      // Pasa a estado Ready-to-Drive.
      }
      break;

    case 20:                            // Estado activo: Ready-to-Drive.
      if (!sdc) step = 0;               // Si el SDC se cae, vuelve a estado inicial.
      r2d = true;                       // Señal de salida: sistema listo para conducir.
      break;
  }
  return r2d;
}
