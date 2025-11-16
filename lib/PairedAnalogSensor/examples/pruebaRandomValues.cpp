#include <Arduino.h>
#include "AnalogSensor.h"
#include "PairedAnalogSensor.h"

PairedAnalogSensorConfig pairedConfig;
AnalogSensorConfig cfgSensor1;
AnalogSensorConfig cfgSensor2;

PairedAnalogSensor* pairedSensor;

uint16_t rawValue1;
uint16_t rawValue2;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Configuracion por defecto
  // --- Configuración del SENSOR 1 (APPS1) ---

  cfgSensor1.cfgSensorVoltage = 4.5;
  cfgSensor1.cfgAdcResolution = 4095;
  cfgSensor1.cfgScaledOutputMin = 0; 
  cfgSensor1.cfgScaledOutputMax = 100; 
  cfgSensor1.cfgAdcMinNormal = 1035;    
  cfgSensor1.cfgAdcMaxNormal = 1985;
  cfgSensor1.cfgLowerMarginPercent = 10.0; 
  cfgSensor1.cfgUpperMarginPercent = 5.0; 
  cfgSensor1.cfgAdcShortGND = 10;
  cfgSensor1.cfgAdcShortVCC = 4085;
  cfgSensor1.cfgImplausibilityTimeout = 100;
  cfgSensor1.cfgFilterType = FilterType::EWMA;
  cfgSensor1.cfgFilterSize = 10;
  cfgSensor1.cfgFilterAlpha = 0.6;              // Factor de suavizado para EWMA (0 < alpha < 1)

  // --- Configuración del SENSOR 2 (APPS2) ---

  cfgSensor2.cfgSensorVoltage = 1.54;            // Voltaje de operación del sensor (e.g., 3.3, 5.0)
  cfgSensor2.cfgAdcResolution = 4095;        // Resolución del ADC (e.g., 4095 para 12-bit)
  cfgSensor2.cfgScaledOutputMin = 0;         // Valor mínimo de la salida escalada
  cfgSensor2.cfgScaledOutputMax = 100;       // Valor máximo de la salida escalada
  cfgSensor2.cfgAdcMinNormal = 2290;        // Valor del ADC con el sensor en reposo
  cfgSensor2.cfgAdcMaxNormal = 2055;        // Valor del ADC con el sensor actuado al máximo
  cfgSensor2.cfgLowerMarginPercent = 10.0;       // Porcentaje inferior donde la salida es cfgScaledOutputMin
  cfgSensor2.cfgUpperMarginPercent = 5.0;       // Porcentaje superior donde la salida es cfgScaledOutputMax
  cfgSensor2.cfgAdcShortGND = 10;            // Umbral para cortocircuito a GND
  cfgSensor2.cfgAdcShortVCC = 4085;          // Umbral para cortocircuito a VCC
  cfgSensor2.cfgImplausibilityTimeout = 100;
  cfgSensor2.cfgFilterType = FilterType::EWMA;
  cfgSensor2.cfgFilterSize = 10;              // Tamaño para Block Avg o Sliding Window
  cfgSensor2.cfgFilterAlpha = 0.6;              // Factor de suavizado para EWMA (0 < alpha < 1)
  
  
  pairedConfig.cfgSensor1 = cfgSensor1;
  pairedConfig.cfgSensor2 = cfgSensor2;
  pairedConfig.cfgMaxDeviationPercent = 10.0;

  pairedSensor = new PairedAnalogSensor(pairedConfig);

  uint16_t rawValue1 = 1500;
  uint16_t rawValue2 = 1500;
  
  Serial.println("Ejemplo Básico de PairedAnalogSensor");
}

void loop() {
  uint16_t rawValue1 = rawValue1 + ((uint16_t) random(-500, 500));
  uint16_t rawValue2 = rawValue2 + ((uint16_t) random(-500, 500));

  // Valores medios
  float meanFilteredValue;
  float meanScaledValue;
  // Valores sensor sensible
  float sensitiveFilteredValue;
  float sensitiveScaledValue;
  // Estado sensor
  SensorState state;

  
  pairedSensor->update(rawValue1, rawValue2, 
    meanFilteredValue, meanScaledValue, 
    sensitiveFilteredValue, sensitiveScaledValue, state);

  // Obtener y mostrar la información del sensor
  Serial.print("\tRaw sensor1: ");
  Serial.print(rawValue1, 2);       // Usamos la variable local
  Serial.print("\tRaw sensor2: ");
  Serial.print(rawValue2, 2);       // Usamos la variable local
  Serial.print("\tMean filtered: ");
  Serial.print(meanFilteredValue, 2);       // Usamos la variable local
  Serial.print("\tMean Scaled: ");
  Serial.print(meanScaledValue, 2);         // Usamos la variable local
  Serial.print("\tSensitive filtered: ");
  Serial.print(sensitiveFilteredValue, 2);       // Usamos la variable local
  Serial.print("\tSensitive Scaled: ");
  Serial.print(sensitiveScaledValue, 2);         // Usamos la variable local
  Serial.print("\tState: ");
  
  // Imprimir el estado de forma legible
  switch (state) {
    case SensorState::NORMAL:
      Serial.print("NORMAL");
      break;
    case SensorState::PENDING:
      Serial.print("PENDING");
      break;
    case SensorState::IMPLAUSIBILITY:
      Serial.print("IMPLAUSIBILITY (");
      switch(pairedSensor->getImplausibilityType()){
        case PairedImplausibilityType::SENSOR1_FAULT: Serial.print("SENSOR1_FAULT"); break;
        case PairedImplausibilityType::SENSOR2_FAULT: Serial.print("SENSOR2_FAULT"); break;
        case PairedImplausibilityType::SENSOR1y2_FAULT: Serial.print("SENSOR1y2_FAULT"); break;
        case PairedImplausibilityType::DEVIATION_FAULT: Serial.print("DEVIATION_FAULT"); break;
        default: break;
      }
      Serial.print(")");
      break;
  }
  Serial.println();

  delay(50);
}