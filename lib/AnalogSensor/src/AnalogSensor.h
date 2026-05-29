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

    // Límites del rango de operación normal (valores del ADC).
    // OJO: son semántica REPOSO / FONDO, NO mínimo/máximo numérico.
    //   cfgAdcMinNormal = lectura ADC con el sensor en REPOSO (pedal suelto)
    //   cfgAdcMaxNormal = lectura ADC con el sensor a FONDO   (pedal pisado)
    // La clase soporta sensores en sentido opuesto por el signo de (Max - Min):
    //   - Sensor normal  (sube con el pedal): cfgAdcMinNormal < cfgAdcMaxNormal
    //   - Sensor INVERSO (baja con el pedal): cfgAdcMinNormal > cfgAdcMaxNormal
    // En el inverso, cfgAdcMinNormal (reposo) es numéricamente MAYOR que cfgAdcMaxNormal
    // (fondo). Configurarlo al revés invierte el escalado -> 100% en reposo: ¡PELIGRO!
    uint16_t cfgAdcMinNormal = 500;          // Lectura ADC en REPOSO
    uint16_t cfgAdcMaxNormal = 3500;         // Lectura ADC a FONDO

    // Márgenes de seguridad (en porcentaje del rango de operación)
    float cfgLowerMarginPercent = 5.0;       // Porcentaje inferior donde la salida es cfgScaledOutputMin
    float cfgUpperMarginPercent = 5.0;       // Porcentaje superior donde la salida es cfgScaledOutputMax

    // Límites para detección de implausibilidades (valores del ADC)
    uint16_t cfgAdcShortGND = 10;            // Umbral para cortocircuito a GND
    uint16_t cfgAdcShortVCC = 4085;          // Umbral para cortocircuito a VCC

    // Tolerancia (cuentas de ADC) que se añade al rango de operación normal antes de
    // declarar OUT_OF_RANGE. Evita falsos positivos por ruido cuando el sensor está
    // en reposo justo sobre cfgAdcMinNormal (fix #6: el rango fuera-de-rango es más
    // ancho que el rango útil).
    uint16_t cfgAdcRangeTolerance = 50;

    // Tiempo (ms) que una implausibilidad debe estar presente para ser notificada
    uint32_t cfgImplausibilityTimeout = 100;

    // Banda (cuentas de ADC) dentro de la cual se acepta una autocalibración respecto
    // al valor configurado por defecto. Si la lectura de calibración se aleja más que
    // esto (p.ej. pedal pisado en el arranque), la autocalibración se rechaza.
    uint16_t cfgCalibrationBand = 300;

    // Configuración del filtro
    FilterType cfgFilterType = FilterType::NO_FILTER;
    uint8_t cfgFilterSize = 10;              // Tamaño para Block Avg o Sliding Window
    float cfgFilterAlpha = 0.1;              // Factor de suavizado para EWMA (0 < alpha < 1)
};

class AnalogSensor {
public:
    // Constructor: copia la configuración (fix #4: se guarda por valor, no por
    // referencia, para evitar referencias colgantes y permitir recalibrar en caliente)
    AnalogSensor(const AnalogSensorConfig& config);

    // Método principal para actualizar el estado con una nueva lectura del ADC
    // Para devolver los valores lo hace por los parametros pasados por referencia
    void update(uint16_t rawValue, float& filteredValue, float& scaledValue, SensorState& state);

    // --- Autocalibración ---
    // Comprueban si una lectura promediada es válida para calibrar sin aplicarla.
    bool isRestCalibrationValid(uint16_t averagedRaw) const;
    bool isFullCalibrationValid(uint16_t averagedRaw) const;
    // Aplican la calibración del punto de reposo / fondo. Devuelven true sólo si la
    // lectura es plausible (no en corto y dentro de cfgCalibrationBand del valor por
    // defecto). Si devuelven false, no modifican nada.
    bool calibrateRest(uint16_t averagedRaw);
    bool calibrateFull(uint16_t averagedRaw);

    // Getters para obtener los valores procesados y el estado
    float getScaledValue() const;
    float getFilteredValue() const;
    SensorState getSensorState() const;
    ImplausibilityType getImplausibilityType() const;
    uint16_t getRawValue() const;
    uint16_t getAdcMinNormal() const;
    uint16_t getAdcMaxNormal() const;

private:
    // Atributos privados (prefijo 'm' según convenio MART)
    AnalogSensorConfig mConfig;
    uint16_t mRawValue;
    float mFilteredValue;
    float mScaledValue;
    SensorState mState;
    ImplausibilityType mImplausibilityType;

    // Variables para el filtrado
    std::vector<uint16_t> mFilterBuffer;
    uint8_t mFilterBufferIndex;
    uint8_t mFilterSamplesFilled;   // fix #7: nº de muestras válidas durante el warmup
    bool mFilterInitialized;        // fix #7: evita comparar floats con == 0.0 en EWMA

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
