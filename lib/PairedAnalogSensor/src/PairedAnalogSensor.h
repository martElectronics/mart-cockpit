// --- START OF FILE PairedAnalogSensor.h ---

#ifndef PAIRED_ANALOG_SENSOR_H
#define PAIRED_ANALOG_SENSOR_H

#include "AnalogSensor.h"

/**
 * @brief Enumeración que define los tipos de fallos de plausibilidad en el par de sensores.
 */
enum class PairedImplausibilityType {
    NONE,               ///< El sistema funciona correctamente. No hay fallos.
    SENSOR1_FAULT,      ///< Fallo eléctrico individual del Sensor 1 (corto a tierra/VCC o circuito abierto).
    SENSOR2_FAULT,      ///< Fallo eléctrico individual del Sensor 2.
    SENSOR1y2_FAULT,    ///< Fallo eléctrico simultáneo en ambos sensores.
    DEVIATION_FAULT     ///< Error de coherencia: La diferencia entre sensores supera el % permitido.
};

/**
 * @brief Estructura de configuración para inicializar el par de sensores redundantes.
 */
struct PairedAnalogSensorConfig {
    AnalogSensorConfig cfgSensor1; ///< Configuración completa para el primer sensor.
    AnalogSensorConfig cfgSensor2; ///< Configuración completa para el segundo sensor.

    /**
     * @brief Porcentaje máximo de desviación permitido entre ambos sensores.
     * @details Calculado sobre el rango de salida (Full Scale) del Sensor 1.
     *          Ejemplo: 10.0 significa un 10% de tolerancia.
     */
    float cfgMaxDeviationPercent = 10.0f;
};

/**
 * @class PairedAnalogSensor
 * @brief Clase para gestionar un par de sensores analógicos redundantes (Seguridad Funcional).
 * 
 * @details Esta clase encapsula dos objetos `AnalogSensor`. Su función principal es:
 *          1. Leer ambos sensores.
 *          2. Verificar errores eléctricos individuales.
 *          3. Verificar la coherencia (plausibilidad) entre ambos valores.
 *          4. Proporcionar un valor medio seguro o un estado de error.
 */
class PairedAnalogSensor {
public:
    /**
     * @brief Constructor de la clase.
     * @param config Estructura con la configuración de ambos sensores y la tolerancia de desviación.
     */
    PairedAnalogSensor(const PairedAnalogSensorConfig& config);

    /**
     * @brief Destructor de la clase. Libera la memoria de los sensores internos.
     */
    ~PairedAnalogSensor();

    /**
     * @brief Actualiza el estado de los sensores con nuevas lecturas ADC.
     * 
     * @details Realiza la lectura, filtrado, escalado y comprobación de errores.
     *          Si el estado resultante no es `NORMAL`, los valores de salida se fuerzan a 0.0.
     * 
     * @param rawValue1 [in] Valor crudo (ADC) del primer sensor.
     * @param rawValue2 [in] Valor crudo (ADC) del segundo sensor.
     * @param meanFilteredValue [out] Promedio de los valores filtrados de ambos sensores (0 si hay error).
     * @param meanScaledValue [out] Promedio de los valores escalados (físicos) de ambos sensores (0 si hay error).
     * @param sensitiveFilteredValue [out] Valor filtrado del sensor considerado más sensible.
     * @param sensitiveScaledValue [out] Valor escalado del sensor considerado más sensible.
     * @param state [out] Estado global resultante del par de sensores.
     */
    void update(uint16_t rawValue1, uint16_t rawValue2, 
        float& meanFilteredValue, float& meanScaledValue, 
        float& sensitiveFilteredValue, float& sensitiveScaledValue, 
        SensorState& state);

    /**
     * @brief Obtiene el valor físico promedio actual.
     * @return Valor float promedio (o 0.0 si hay error).
     */
    float getMeanScaledValue() const;

    /**
     * @brief Obtiene el valor filtrado promedio actual.
     * @return Valor float promedio (o 0.0 si hay error).
     */
    float getMeanFilteredValue() const;

    /**
     * @brief Obtiene el valor físico del sensor más sensible.
     * @return Valor float del sensor con mayor rango ADC.
     */
    float getSensitiveScaledValue() const;

    /**
     * @brief Obtiene el valor filtrado del sensor más sensible.
     * @return Valor float del sensor con mayor rango ADC.
     */
    float getSensitiveFilteredValue() const;

    /**
     * @brief Obtiene el estado global del par de sensores.
     * @return SensorState (NORMAL o IMPLAUSIBILITY).
     */
    SensorState getSensorState() const;

    /**
     * @brief Obtiene el tipo específico de fallo detectado.
     * @return PairedImplausibilityType indicando si es fallo de sensor 1, 2 o desviación.
     */
    PairedImplausibilityType getImplausibilityType() const;

    /**
     * @brief Acceso de solo lectura al objeto del primer sensor interno.
     * @return Referencia constante al Sensor 1.
     */
    const AnalogSensor& getSensor1() const;

    /**
     * @brief Acceso de solo lectura al objeto del segundo sensor interno.
     * @return Referencia constante al Sensor 2.
     */
    const AnalogSensor& getSensor2() const;

private:
    const PairedAnalogSensorConfig& mConfig; ///< Referencia a la configuración.
    AnalogSensor* mSensor1;     ///< Puntero al primer sensor.
    AnalogSensor* mSensor2;     ///< Puntero al segundo sensor.
    AnalogSensor* mSensitive;   ///< Puntero auxiliar al sensor con mayor resolución/rango.

    float mFilteredValue;       ///< Valor medio filtrado almacenado.
    float mScaledValue;         ///< Valor medio escalado almacenado.

    SensorState mState;         ///< Estado actual del sistema redundante.
    PairedImplausibilityType mImplausibilityType; ///< Detalle del error actual.
    
    /**
     * @brief Lógica interna para verificar la plausibilidad.
     * @details Compara los estados individuales y la desviación porcentual entre los valores escalados.
     *          Actualiza `mState` y `mImplausibilityType`.
     */
    void mCheckPlausibility();
};

#endif // PAIRED_ANALOG_SENSOR_H