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