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

    // fix #3: la desviación entre sensores debe persistir este tiempo (ms) antes de
    // declararse implausible (regla FSAE: >10% durante >100 ms).
    uint32_t cfgDeviationTimeout = 100;
};

class PairedAnalogSensor {
public:
    // Constructor que recibe la configuración completa para el par
    PairedAnalogSensor(const PairedAnalogSensorConfig& config);

    // fix #5: la clase posee punteros internos (mSensitive); prohibimos la copia para
    // evitar dobles liberaciones / punteros colgantes.
    PairedAnalogSensor(const PairedAnalogSensor&) = delete;
    PairedAnalogSensor& operator=(const PairedAnalogSensor&) = delete;

    // Método principal para actualizar con las lecturas de ambos sensores
    void update(uint16_t rawValue1, uint16_t rawValue2,
        float& meanFilteredValue, float& meanScaledValue,
        float& sensitiveFilteredValue, float& sensitiveScaledValue,
        SensorState& state);

    // --- Autocalibración ---
    // Calibra el punto de reposo / fondo de ambos sensores de forma atómica: sólo
    // aplica si AMBAS lecturas son válidas (si una falla, no toca ninguna).
    bool calibrateRest(uint16_t rawValue1, uint16_t rawValue2);
    bool calibrateFull(uint16_t rawValue1, uint16_t rawValue2);

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
    PairedAnalogSensorConfig mConfig;     // fix #4: por valor, no por referencia
    AnalogSensor mSensor1;                // fix #5: por valor, sin new/delete
    AnalogSensor mSensor2;
    AnalogSensor* mSensitive;             // puntero observador a mSensor1 o mSensor2

    float mFilteredValue;
    float mScaledValue;

    SensorState mState;
    PairedImplausibilityType mImplausibilityType;

    // fix #3: temporización de la desviación entre sensores
    bool mDeviationTiming;
    uint32_t mDeviationStartTime;

    // Método privado para la lógica de comprobación
    void mCheckPlausibility();
};

#endif // PAIRED_ANALOG_SENSOR_H
