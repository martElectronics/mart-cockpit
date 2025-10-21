#ifndef PAIRED_ANALOG_SENSOR_H
#define PAIRED_ANALOG_SENSOR_H

#include "AnalogSensor.h"

// Enum para el tipo de implausibilidad en un par de sensores
enum class PairedImplausibilityType {
    NONE,
    SENSOR1_FAULT,        // Fallo individual del sensor 1 (corto, fuera de rango)
    SENSOR2_FAULT,        // Fallo individual del sensor 2
    DEVIATION_FAULT       // Los sensores difieren más de lo permitido
};

// Estructura de configuración para el par de sensores
struct PairedAnalogSensorConfig {
    AnalogSensorConfig cfgSensor1; // Configuración para el primer sensor
    AnalogSensorConfig cfgSensor2; // Configuración para el segundo sensor

    // Porcentaje máximo de desviación permitido entre las salidas escaladas de los dos sensores
    // Por ejemplo, 10.0 para una diferencia máxima del 10%
    float cfgMaxDeviationPercent = 10.0;
};

class PairedAnalogSensor {
public:
    // Constructor que recibe la configuración completa para el par
    PairedAnalogSensor(const PairedAnalogSensorConfig& config);
    ~PairedAnalogSensor(); // Destructor para liberar memoria

    // Método principal para actualizar con las lecturas de ambos sensores
    void update(uint16_t rawValue1, uint16_t rawValue2);

    // Getters para el estado del par
    float getAverageValue() const;
    SensorState getSensorState() const;
    PairedImplausibilityType getImplausibilityType() const;

    // Getters para acceder a los sensores individuales (útil para depuración)
    const AnalogSensor& getSensor1() const;
    const AnalogSensor& getSensor2() const;

private:
    // Atributos privados
    PairedAnalogSensorConfig mConfig;
    AnalogSensor* mSensor1;
    AnalogSensor* mSensor2;

    float mAverageValue;
    SensorState mState;
    PairedImplausibilityType mImplausibilityType;
    
    // Método privado para la lógica de comprobación
    void mCheckPlausibility();
};

#endif // PAIRED_ANALOG_SENSOR_H