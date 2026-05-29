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

    // La desviación entre sensores debe persistir este tiempo (ms) antes de declararse
    // implausible (regla FSAE: >10% durante >100 ms).
    uint32_t cfgDeviationTimeout = 100;
};

// Empareja dos AnalogSensor (de la ESP32-Global-Library) y añade la comprobación de
// coherencia entre ambos. Cada sensor hace su propio filtrado, escalado, plausibilidad
// individual y auto-calibración por tabla de voltaje (si se configura).
class PairedAnalogSensor {
public:
    PairedAnalogSensor(const PairedAnalogSensorConfig& config);

    // La clase posee un puntero observador (mSensitive); prohibimos la copia.
    PairedAnalogSensor(const PairedAnalogSensor&) = delete;
    PairedAnalogSensor& operator=(const PairedAnalogSensor&) = delete;

    // Actualiza con las lecturas de ambos sensores y el voltaje del sistema (para la
    // auto-calibración dinámica de AnalogSensor). Pasa currentVoltage < 0 para usar
    // los límites estáticos (sin compensación por voltaje).
    void update(uint16_t rawValue1, uint16_t rawValue2, float currentVoltage,
        float& meanFilteredValue, float& meanScaledValue,
        float& sensitiveFilteredValue, float& sensitiveScaledValue,
        SensorState& state);

    // Sobrecarga sin voltaje (límites estáticos).
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

    // Estados generales del par
    SensorState getSensorState() const;
    PairedImplausibilityType getImplausibilityType() const;

    // Getters para acceder a los sensores individuales (útil para depuración)
    const AnalogSensor& getSensor1() const;
    const AnalogSensor& getSensor2() const;

private:
    PairedAnalogSensorConfig mConfig;     // Configuración por valor.
    AnalogSensor mSensor1;                // Sensores por valor (sin new/delete).
    AnalogSensor mSensor2;
    AnalogSensor* mSensitive;             // Puntero observador a mSensor1 o mSensor2.

    float mFilteredValue;
    float mScaledValue;

    SensorState mState;
    PairedImplausibilityType mImplausibilityType;

    // Temporización de la desviación entre sensores
    bool mDeviationTiming;
    uint32_t mDeviationStartTime;

    void mCheckPlausibility();
};

#endif // PAIRED_ANALOG_SENSOR_H
