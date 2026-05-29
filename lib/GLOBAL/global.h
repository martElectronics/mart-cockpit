
#ifndef __GLOBAL
#define __GLOBAL
#include <Arduino.h>

// NOTA: este header DEFINE variables y funciones. Para que sea seguro incluirlo desde
// varios .cpp sin violar la ODR, todo lo definible se marca 'inline' (C++17).
// Muchos de los stsInverterCAN_* de abajo están sin usar; se podrían podar en una
// limpieza futura.

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

// Function to get the error message based on the error code
inline const char* getErrorMessage(uint8_t errorCode) {
  if (errorCode < sizeof(errorMessages) / sizeof(errorMessages[0])) {
    return errorMessages[errorCode];
  }
  return "Unknown error code";
}

//**APPS**//
struct sensorData
{
  // Configuración
  int pin;
  uint16_t valAnalogUP, valAnalogDOWN;
  uint16_t valScaledUP, valScaledDOWN;
  uint16_t range;

  // Runtime
  uint16_t valAnalog, valScaled, valAvg, valDifference;
};

//INVERTER

// Packet ID 0x20: ERPM, Duty, Input Voltage
inline int32_t stsInverterCAN_EPRM;      // Electrical RPM
inline int16_t stsInverterCAN_DutyCycle; // Duty cycle in percentage (multiplied by 10)
inline int16_t stsInverterCAN_InputVoltage; // Input voltage in volts

// Packet ID 0x21: AC Current, DC Current
inline int16_t stsInverterCAN_ACCurrent; // AC current in Amperes (multiplied by 10)
inline int16_t stsInverterCAN_DCCurrent; // DC current in Amperes (multiplied by 10)

// Packet ID 0x22: Controller Temp., Motor Temp., Fault code
inline int16_t stsInverterCAN_ControllerTemp; // Controller temperature in °C (multiplied by 10)
inline int16_t stsInverterCAN_MotorTemp;      // Motor temperature in °C (multiplied by 10)
inline uint8_t stsInverterCAN_FaultCode;      // Fault code

// Packet ID 0x23: Id, Iq values
inline int32_t stsInverterCAN_Id; // FOC algorithm component Id (multiplied by 100)
inline int32_t stsInverterCAN_Iq; // FOC algorithm component Iq (multiplied by 100)

// Packet ID 0x24: Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits, CAN map version
inline int8_t stsInverterCAN_ThrottleSignal;   // Throttle signal in percentage
inline int8_t stsInverterCAN_BrakeSignal;      // Brake signal in percentage
inline uint8_t stsInverterCAN_DigitalInputs;   // 4 bits representing the digital inputs
inline uint8_t stsInverterCAN_DigitalOutputs;  // 4 bits representing the digital outputs
inline uint8_t stsInverterCAN_DriveEnable;     // Drive enable status (0: disabled, 1: enabled)
inline uint8_t stsInverterCAN_CANMapVersion;   // CAN map version
// Packet ID 0x20: ERPM, Duty, Input Voltage
inline int32_t stsInverterCAN_20[1];  // Electrical RPM
inline int16_t stsInverterCAN_20_1[1];  // Duty cycle in percentage (multiplied by 10)
inline int16_t stsInverterCAN_20_2[1];  // Input voltage in volts

// Packet ID 0x21: AC Current, DC Current
inline int16_t stsInverterCAN_21[2];  // Array for AC and DC current
// stsInverterCAN_21[0] - AC current in Amperes (multiplied by 10)
// stsInverterCAN_21[1] - DC current in Amperes (multiplied by 10)

// Packet ID 0x22: Controller Temp., Motor Temp., Fault code
inline int16_t stsInverterCAN_22[2];  // Array for Controller and Motor temperature
// stsInverterCAN_22[0] - Controller temperature in °C (multiplied by 10)
// stsInverterCAN_22[1] - Motor temperature in °C (multiplied by 10)
inline uint8_t stsInverterCAN_22_FaultCode[1];  // Fault code
inline byte stsInverterCAN_22_FULL[8];

// Packet ID 0x23: Id, Iq values
inline int32_t stsInverterCAN_23[2];  // Array for Id and Iq values
// stsInverterCAN_23[0] - FOC algorithm component Id (multiplied by 100)
// stsInverterCAN_23[1] - FOC algorithm component Iq (multiplied by 100)

// Packet ID 0x24: Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits, CAN map version
inline int8_t stsInverterCAN_24_1[2];  // Array for Throttle and Brake signal
// stsInverterCAN_24_1[0] - Throttle signal in percentage
// stsInverterCAN_24_1[1] - Brake signal in percentage
inline uint8_t stsInverterCAN_24_DigitalInputs[1];  // Digital Inputs (4 bits)
inline uint8_t stsInverterCAN_24_DigitalOutputs[1];  // Digital Outputs (4 bits)
inline uint8_t stsInverterCAN_24_2[1];  // Drive enable status (0: disabled, 1: enabled)
inline uint8_t stsInverterCAN_24_3[1];  // CAN map version
inline byte stsInverterCAN_24_FULL[8];

inline void assignstsInverterCANValues() {
    // Packet ID 0x20: ERPM, Duty, Input Voltage
    stsInverterCAN_EPRM = stsInverterCAN_20[0];
    stsInverterCAN_DutyCycle = stsInverterCAN_20_1[0];
    stsInverterCAN_InputVoltage = stsInverterCAN_20_2[0];

    // Packet ID 0x21: AC Current, DC Current
    stsInverterCAN_ACCurrent = stsInverterCAN_21[0];
    stsInverterCAN_DCCurrent = stsInverterCAN_21[1];

    // Packet ID 0x22: Controller Temp., Motor Temp., Fault code
    stsInverterCAN_ControllerTemp = stsInverterCAN_22[0];
    stsInverterCAN_MotorTemp = stsInverterCAN_22[1];
    stsInverterCAN_FaultCode = stsInverterCAN_22_FaultCode[0];

    // Packet ID 0x23: Id, Iq values
    stsInverterCAN_Id = stsInverterCAN_23[0];
    stsInverterCAN_Iq = stsInverterCAN_23[1];

    // Packet ID 0x24: Throttle signal, Brake signal, Digital I/Os, Drive enable, Limit status bits, CAN map version
    stsInverterCAN_ThrottleSignal = stsInverterCAN_24_1[0];
    stsInverterCAN_BrakeSignal = stsInverterCAN_24_1[1];

    for (int i = 0; i < 4; i++) {
        // stsInverterCAN_DigitalInputs[i] = stsInverterCAN_24_DigitalInputs[i];
        // stsInverterCAN_DigitalOutputs[i] = stsInverterCAN_24_DigitalOutputs[i];
    }

    stsInverterCAN_DriveEnable = stsInverterCAN_24_2[0];
    stsInverterCAN_CANMapVersion = stsInverterCAN_24_3[0];
}




inline void intToByteArray(int a, byte *byteArray)
{
  // Initialize all elements of the byte array to zero
  for (int i = 0; i < 8; i++)
  {
    byteArray[i] = 0;
  }

  // Convert the integer to a big-endian byte array
  byteArray[3] = (a >> 24) & 0xFF;
  byteArray[2] = (a >> 16) & 0xFF;
  byteArray[1] = (a >> 8) & 0xFF;
  byteArray[0] = a & 0xFF;
}
// Function to combine two integers into an unsigned long int
inline uint32_t combineInts(uint32_t int1, uint32_t int2)
{
  return (static_cast<uint32_t>(int1) << 5) + int2;
  // unsigned long finalID = (PID << 8) | NodeID;
  // return finalID;
}

// Function to print an unsigned long int in hexadecimal format
inline void printHex(unsigned long int num)
{
  char hexString[9]; // 8 digits for 32 bits + null terminator
  sprintf(hexString, "%08lX", num);
  Serial.println(hexString);
}



#endif
