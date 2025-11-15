#ifndef PAIRED_ANALOG_SENSOR_H
#define PAIRED_ANALOG_SENSOR_H

#include "AnalogSensor.h"

// Enum para el tipo de implausibilidad en un par de sensores
enum class PairedImplausibilityType {
    NONE,
    SENSOR1_FAULT,        // Fallo individual del sensor 1 (corto, fuera de rango)
    SENSOR2_FAULT,        // Fallo individual del sensor 2
    SENSOR1y2_FAULT,      // Fallo individual de ambos sensores
    DEVIATION_FAULT       // Los sensores difieren más de lo permitido
};

// Estructura de configuración para el par de sensores
struct PairedAnalogSensorConfig {
    AnalogSensorConfig cfgSensor1; // Configuración para el primer sensor
    AnalogSensorConfig cfgSensor2; // Configuración para el segundo sensor

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

    // --- Configuración de la Coherencia del PAR ---
    float cfgMaxDeviationPercent = 10.0;
};

class PairedAnalogSensor {
public:
    // Constructor que recibe la configuración completa para el par
    PairedAnalogSensor(const PairedAnalogSensorConfig& config);
    ~PairedAnalogSensor(); // Destructor para liberar memoria

    // Método principal para actualizar con las lecturas de ambos sensores
    void update(uint16_t rawValue1, uint16_t rawValue2, 
        float& meanFilteredValue, float& meanScaledValue, 
        float& sensitiveFilteredValue, float& sensitiveScaledValue, 
        SensorState& state);

    // Getters para valores medios
    float getMeanScaledValue() const;
    float getMeanFilteredValue() const;

    // Getters para valores del sensible
    float getSensitiveScaledValue() const;
    float getSensitiveFilteredValue() const;

    // Estados generales del pair
    SensorState getSensorState() const;
    PairedImplausibilityType getImplausibilityType() const;

    // Getters para acceder a los sensores individuales (útil para depuración)
    const AnalogSensor& getSensor1() const;
    const AnalogSensor& getSensor2() const;

private:
    // Atributos privados
    const PairedAnalogSensorConfig& mConfig;
    AnalogSensor* mSensor1;
    AnalogSensor* mSensor2;
    AnalogSensor* mSensitive; // Para poder obtener los valores más sensibles sin tener que almacenarlos dos veces, los obtienes llamando a los metodos

    float mFilteredValue;
    float mScaledValue;

    SensorState mState;
    PairedImplausibilityType mImplausibilityType;
    
    // Método privado para la lógica de comprobación
    void mCheckPlausibility();
};

#endif // PAIRED_ANALOG_SENSOR_H