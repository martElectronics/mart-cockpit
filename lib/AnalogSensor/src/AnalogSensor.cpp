#include "AnalogSensor.h"

// Constructor que inicializa la clase con la configuración proporcionada
AnalogSensor::AnalogSensor(const AnalogSensorConfig& config)
    : mConfig(config),
      mRawValue(0),
      mFilteredValue(0.0),
      mScaledValue(0.0),
      mState(SensorState::NORMAL),
      mImplausibilityType(ImplausibilityType::NONE),
      mFilterBufferIndex(0),
      mFilterSamplesFilled(0),
      mFilterInitialized(false),
      mImplausibilityStartTime(0),
      mIsImplausibilityTiming(false),
      mPendingImplausibility(ImplausibilityType::NONE)
{
    // Inicializa el buffer del filtro con el tamaño adecuado
    if (mConfig.cfgFilterType == FilterType::BLOCK_AVG || mConfig.cfgFilterType == FilterType::SLIDING_WINDOW) {
        mFilterBuffer.resize(mConfig.cfgFilterSize, 0);
    }
}

// Método principal llamado en cada ciclo para procesar la nueva lectura
void AnalogSensor::update(uint16_t rawValue, float& filteredValue, float& scaledValue, SensorState& state) {
    mRawValue = rawValue;

    // 1. Filtrar la señal
    mProcessFilter(rawValue);

    // 2. Escalar la señal filtrada
    mProcessScaling();

    // 3. Comprobar implausibilidades
    mProcessImplausibility();

    // Devuelve los valores
    filteredValue = getFilteredValue();
    scaledValue = getScaledValue();
    state = getSensorState();
}

// --- Autocalibración ---

bool AnalogSensor::isRestCalibrationValid(uint16_t averagedRaw) const {
    // No aceptar si está en corto (a GND o VCC)
    if (averagedRaw <= mConfig.cfgAdcShortGND || averagedRaw >= mConfig.cfgAdcShortVCC) {
        return false;
    }
    // Sólo aceptar si está cerca del reposo esperado: protege contra arrancar con el
    // pedal pisado o con un sensor averiado.
    int diff = static_cast<int>(averagedRaw) - static_cast<int>(mConfig.cfgAdcMinNormal);
    return (abs(diff) <= static_cast<int>(mConfig.cfgCalibrationBand));
}

bool AnalogSensor::isFullCalibrationValid(uint16_t averagedRaw) const {
    if (averagedRaw <= mConfig.cfgAdcShortGND || averagedRaw >= mConfig.cfgAdcShortVCC) {
        return false;
    }
    int diff = static_cast<int>(averagedRaw) - static_cast<int>(mConfig.cfgAdcMaxNormal);
    return (abs(diff) <= static_cast<int>(mConfig.cfgCalibrationBand));
}

bool AnalogSensor::calibrateRest(uint16_t averagedRaw) {
    if (!isRestCalibrationValid(averagedRaw)) {
        return false;
    }
    mConfig.cfgAdcMinNormal = averagedRaw;
    return true;
}

bool AnalogSensor::calibrateFull(uint16_t averagedRaw) {
    if (!isFullCalibrationValid(averagedRaw)) {
        return false;
    }
    mConfig.cfgAdcMaxNormal = averagedRaw;
    return true;
}

// Implementación del filtrado
void AnalogSensor::mProcessFilter(uint16_t rawValue) {
    switch (mConfig.cfgFilterType) {
        case FilterType::NO_FILTER:
            mFilteredValue = rawValue;
            break;

        case FilterType::BLOCK_AVG:
            mFilterBuffer[mFilterBufferIndex++] = rawValue;
            if (mFilterBufferIndex >= mConfig.cfgFilterSize) {
                long sum = 0;
                for (uint16_t val : mFilterBuffer) {
                    sum += val;
                }
                mFilteredValue = static_cast<float>(sum) / mConfig.cfgFilterSize;
                mFilterBufferIndex = 0; // Reset index
            } else if (!mFilterInitialized) {
                // Antes de completar el primer bloque, evita devolver 0 (fix #7)
                mFilteredValue = rawValue;
            }
            mFilterInitialized = true;
            break;

        case FilterType::SLIDING_WINDOW:
            mFilterBuffer[mFilterBufferIndex++] = rawValue;
            if (mFilterBufferIndex >= mConfig.cfgFilterSize) {
                mFilterBufferIndex = 0; // Circular buffer
            }
            // fix #7: promedia sólo las muestras realmente capturadas durante el
            // warmup, en vez de incluir los ceros iniciales del buffer.
            if (mFilterSamplesFilled < mConfig.cfgFilterSize) {
                mFilterSamplesFilled++;
            }
            {
                long sum = 0;
                for (uint8_t i = 0; i < mFilterSamplesFilled; ++i) {
                    sum += mFilterBuffer[i];
                }
                mFilteredValue = static_cast<float>(sum) / mFilterSamplesFilled;
            }
            mFilterInitialized = true;
            break;

        case FilterType::EWMA:
            // fix #7: usa un flag de inicialización en vez de comparar el float con 0.0
            if (!mFilterInitialized) {
                mFilteredValue = rawValue;
                mFilterInitialized = true;
            } else {
                mFilteredValue = (mConfig.cfgFilterAlpha * rawValue) + ((1.0 - mConfig.cfgFilterAlpha) * mFilteredValue);
            }
            break;
    }
}

// Implementación del escalado lineal con márgenes de seguridad
void AnalogSensor::mProcessScaling() {
    // Calcula el rango de ADC. Puede ser negativo si el sensor es inverso.
    float rangeNormal = mConfig.cfgAdcMaxNormal - mConfig.cfgAdcMinNormal;

    // Evitar división por cero si el rango es estrictamente cero (sensor atascado)
    if (rangeNormal == 0) {
        mScaledValue = mConfig.cfgScaledOutputMin;
        return;
    }

    // 1. Determinar los límites de los márgenes
    // Los márgenes se calculan *siempre* desde los valores cfgAdcMinNormal y cfgAdcMaxNormal.
    // Para sensores inversos (rangeNormal < 0) la fórmula se ajusta automáticamente.
    float lowerMarginLimit = mConfig.cfgAdcMinNormal + (rangeNormal * mConfig.cfgLowerMarginPercent / 100.0);
    float upperMarginLimit = mConfig.cfgAdcMaxNormal - (rangeNormal * mConfig.cfgUpperMarginPercent / 100.0);

    // 2. Aplicar la lógica de los márgenes
    if (rangeNormal > 0) { // Sensor normal (ADC de menor a mayor)
        if (mFilteredValue <= lowerMarginLimit) {
            mScaledValue = mConfig.cfgScaledOutputMin;
        } else if (mFilteredValue >= upperMarginLimit) {
            mScaledValue = mConfig.cfgScaledOutputMax;
        } else {
            mScaledValue = mConfig.cfgScaledOutputMin +
                           (mFilteredValue - lowerMarginLimit) * (mConfig.cfgScaledOutputMax - mConfig.cfgScaledOutputMin) /
                           (upperMarginLimit - lowerMarginLimit);
        }
    } else { // Sensor inverso (rangeNormal < 0)
        if (mFilteredValue >= lowerMarginLimit) {
            mScaledValue = mConfig.cfgScaledOutputMin;
        } else if (mFilteredValue <= upperMarginLimit) {
            mScaledValue = mConfig.cfgScaledOutputMax;
        } else {
            // El denominador negativo invierte la pendiente automáticamente.
            mScaledValue = mConfig.cfgScaledOutputMin +
                           (mFilteredValue - lowerMarginLimit) * (mConfig.cfgScaledOutputMax - mConfig.cfgScaledOutputMin) /
                           (upperMarginLimit - lowerMarginLimit);
        }
    }
}

// Implementación de la detección de implausibilidades (no bloqueante)
void AnalogSensor::mProcessImplausibility() {
    ImplausibilityType currentImplausibility = ImplausibilityType::NONE;

    // Rango de operación real (soporta sensores inversos)
    int adcMin = std::min(mConfig.cfgAdcMinNormal, mConfig.cfgAdcMaxNormal);
    int adcMax = std::max(mConfig.cfgAdcMinNormal, mConfig.cfgAdcMaxNormal);

    // fix #6: OUT_OF_RANGE usa una banda más ancha que el rango útil (± tolerancia)
    // para no disparar por ruido cuando el sensor reposa sobre el límite.
    int outOfRangeLow = adcMin - static_cast<int>(mConfig.cfgAdcRangeTolerance);
    int outOfRangeHigh = adcMax + static_cast<int>(mConfig.cfgAdcRangeTolerance);

    if (mRawValue <= mConfig.cfgAdcShortGND) {
        currentImplausibility = ImplausibilityType::SHORT_TO_GND;
    } else if (mRawValue >= mConfig.cfgAdcShortVCC) {
        currentImplausibility = ImplausibilityType::SHORT_TO_VCC;
    } else if (static_cast<int>(mRawValue) < outOfRangeLow || static_cast<int>(mRawValue) > outOfRangeHigh) {
        currentImplausibility = ImplausibilityType::OUT_OF_RANGE;
    }

    if (currentImplausibility != ImplausibilityType::NONE) {
        if (!mIsImplausibilityTiming) {
            // Inicia el temporizador para una nueva implausibilidad detectada
            mIsImplausibilityTiming = true;
            mImplausibilityStartTime = millis();
            mPendingImplausibility = currentImplausibility;
        } else if (currentImplausibility != mPendingImplausibility) {
            // Si el tipo de implausibilidad cambia, reinicia el temporizador
            mImplausibilityStartTime = millis();
            mPendingImplausibility = currentImplausibility;
        }

        // Comprueba si ha pasado el tiempo de timeout (resta unsigned -> wraparound seguro)
        if (millis() - mImplausibilityStartTime > mConfig.cfgImplausibilityTimeout) {
            mState = SensorState::IMPLAUSIBILITY;
            mImplausibilityType = mPendingImplausibility;
        }
    } else {
        // Si la señal vuelve a ser normal, resetea el estado y el temporizador
        mIsImplausibilityTiming = false;
        mState = SensorState::NORMAL;
        mImplausibilityType = ImplausibilityType::NONE;
        mPendingImplausibility = ImplausibilityType::NONE;
    }
}

// Getters
float AnalogSensor::getScaledValue() const { return mScaledValue; }
float AnalogSensor::getFilteredValue() const { return mFilteredValue; }
SensorState AnalogSensor::getSensorState() const {
    // Devuelve PENDING mientras se está temporizando una posible implausibilidad
    if (mIsImplausibilityTiming) {
        return SensorState::PENDING;
    }
    return mState;
}
ImplausibilityType AnalogSensor::getImplausibilityType() const { return mImplausibilityType; }
uint16_t AnalogSensor::getRawValue() const { return mRawValue; }
uint16_t AnalogSensor::getAdcMinNormal() const { return mConfig.cfgAdcMinNormal; }
uint16_t AnalogSensor::getAdcMaxNormal() const { return mConfig.cfgAdcMaxNormal; }
