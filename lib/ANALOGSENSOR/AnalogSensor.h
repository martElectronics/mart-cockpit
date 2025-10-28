#ifndef ANALOG_SENSOR_H
#define ANALOG_SENSOR_H

#include <Arduino.h>
#include <vector>
#include <numeric>

// Enum para los tipos de implausibilidad según la normativa T11.9
enum class ImplausibilityType {
    NONE,
    SHORT_TO_GND,       // T11.9.2.a
    SHORT_TO_VCC,       // T11.9.2.b
    OUT_OF_RANGE        // T11.9.2.c
};

// Enum para el estado actual del sensor
enum class SensorState {
    NORMAL,
    IMPLAUSIBILITY,
    PENDING
};

// Enum para seleccionar el método de filtrado
enum class FilterType {
    NO_FILTER,
    BLOCK_AVG,          // Media de un bloque de muestras
    SLIDING_WINDOW,     // Media móvil
    EWMA               // Media móvil exponencialmente ponderada (IIR)
};

// Estructura para la configuración del sensor
struct AnalogSensorConfig {
    // Parámetros generales
    float cfgSensorVoltage = 5.0;            // Voltaje de operación del sensor (e.g., 3.3, 5.0)
    uint16_t cfgAdcResolution = 4095;        // Resolución del ADC (e.g., 4095 para 12-bit)

    // Parámetros de escalado
    uint16_t cfgScaledOutputMin = 0;         // Valor mínimo de la salida escalada
    uint16_t cfgScaledOutputMax = 100;       // Valor máximo de la salida escalada

    // Límites del rango de operación normal (valores del ADC)
    uint16_t cfgAdcMinNormal = 500;          // Valor del ADC con el sensor en reposo
    uint16_t cfgAdcMaxNormal = 3500;         // Valor del ADC con el sensor actuado al máximo

    // Márgenes de seguridad (en porcentaje del rango de operación)
    float cfgLowerMarginPercent = 5.0;       // Porcentaje inferior donde la salida es cfgScaledOutputMin
    float cfgUpperMarginPercent = 5.0;       // Porcentaje superior donde la salida es cfgScaledOutputMax

    // Límites para detección de implausibilidades (valores del ADC)
    uint16_t cfgAdcShortGND = 10;            // Umbral para cortocircuito a GND
    uint16_t cfgAdcShortVCC = 4085;          // Umbral para cortocircuito a VCC

    // Tiempo (ms) que una implausibilidad debe estar presente para ser notificada
    uint32_t cfgImplausibilityTimeout = 100;

    // Configuración del filtro
    FilterType cfgFilterType = FilterType::NO_FILTER;
    uint8_t cfgFilterSize = 10;              // Tamaño para Block Avg o Sliding Window
    float cfgFilterAlpha = 0.1;              // Factor de suavizado para EWMA (0 < alpha < 1)
};

class AnalogSensor {
public:
    // Constructor: recibe la configuración por referencia
    AnalogSensor(const AnalogSensorConfig& config);

    // Método principal para actualizar el estado con una nueva lectura del ADC
    // Para devolver los valores lo hace por los parametros pasados por referencia
    void update(uint16_t rawValue, float& filteredValue, float& scaledValue, SensorState& state);

    // Getters para obtener los valores procesados y el estado
    float getScaledValue() const;
    float getFilteredValue() const;
    SensorState getSensorState() const;
    ImplausibilityType getImplausibilityType() const;
    uint16_t getRawValue() const;

private:
    // Atributos privados (prefijo 'm' según convenio MART)
    const AnalogSensorConfig& mConfig;
    uint16_t mRawValue;
    float mFilteredValue;
    float mScaledValue;
    SensorState mState;
    ImplausibilityType mImplausibilityType;

    // Variables para el filtrado
    std::vector<uint16_t> mFilterBuffer;
    uint8_t mFilterBufferIndex;
    
    // Variables para la temporización de implausibilidades
    uint32_t mImplausibilityStartTime;
    bool mIsImplausibilityTiming;
    ImplausibilityType mPendingImplausibility;

    // Métodos privados de ayuda
    void mProcessFilter(uint16_t rawValue);
    void mProcessScaling();
    void mProcessImplausibility();
};

#endif // ANALOG_SENSOR_H