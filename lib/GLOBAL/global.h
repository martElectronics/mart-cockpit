#ifndef __GLOBAL
#define __GLOBAL
#include <Arduino.h>

// Header compartido. Todo lo definible va 'inline' (C++17) para poder incluirlo
// desde varios .cpp sin violar la ODR.

// ---- Códigos de fallo del inversor (DTI HV-500) ----
inline const char* errorMessages[] = {
  "NO FAULTS",
  "Overvoltage",
  "Undervoltage",
  "DRV Error",
  "ABS. Overcurrent",
  "CTLR Overtemp.",
  "Motor Overtemp.",
  "Sensor wire fault",
  "Sensor general fault",
  "CAN Command error",
  "Analog input error"
};

inline const char* getErrorMessage(uint8_t errorCode) {
  if (errorCode < sizeof(errorMessages) / sizeof(errorMessages[0]))
    return errorMessages[errorCode];
  return "Unknown error code";
}

// (El estado del inversor recibido por CAN — fault code, drive enable — vive en
//  la clase InverterController, no en globales sueltos.)

// ID CAN = (PacketID << 5) | NodeID (modo Standard del HV-500).
constexpr uint32_t combineInts(uint32_t int1, uint32_t int2) {
  return (static_cast<uint32_t>(int1) << 5) + int2;
}

#endif
