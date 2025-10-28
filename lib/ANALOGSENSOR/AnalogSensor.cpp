#include "AnalogSensor.h"

// Constructor que inicializa la clase con la configuración proporcionada
// La estructura AnalogSensorConfig es guardada como una referencia constante
AnalogSensor::AnalogSensor(const AnalogSensorConfig& config)
    : mConfig(config),
      mRawValue(0),
      mFilteredValue(0.0),
      mScaledValue(0.0),
      mState(SensorState::NORMAL),
      mImplausibilityType(ImplausibilityType::NONE),
      mFilterBufferIndex(0),
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
            }
            // Mientras no se llena el bloque, se mantiene el valor anterior
            break;

        case FilterType::SLIDING_WINDOW:
            mFilterBuffer[mFilterBufferIndex++] = rawValue;
            if (mFilterBufferIndex >= mConfig.cfgFilterSize) {
                mFilterBufferIndex = 0; // Circular buffer
            }
            {
                long sum = 0;
                for (uint16_t val : mFilterBuffer) {
                    sum += val;
                }
                mFilteredValue = static_cast<float>(sum) / mConfig.cfgFilterSize;
            }
            break;

        case FilterType::EWMA:
            // Si es el primer valor, se inicializa directamente
            if (mFilteredValue == 0.0) {
                 mFilteredValue = rawValue;
            } else {
                 mFilteredValue = (mConfig.cfgFilterAlpha * rawValue) + ((1.0 - mConfig.cfgFilterAlpha) * mFilteredValue);
            }
            break;
    }
}

// Implementación del escalado lineal con márgenes de seguridad
void AnalogSensor::mProcessScaling() {
    float rangeNormal = mConfig.cfgAdcMaxNormal - mConfig.cfgAdcMinNormal;
    if (rangeNormal <= 0) { // Evitar división por cero
        mScaledValue = mConfig.cfgScaledOutputMin;
        return;
    }

    float lowerMarginLimit = mConfig.cfgAdcMinNormal + (rangeNormal * mConfig.cfgLowerMarginPercent / 100.0);
    float upperMarginLimit = mConfig.cfgAdcMaxNormal - (rangeNormal * mConfig.cfgUpperMarginPercent / 100.0);

    if (mFilteredValue <= lowerMarginLimit) {
        mScaledValue = mConfig.cfgScaledOutputMin;
    } else if (mFilteredValue >= upperMarginLimit) {
        mScaledValue = mConfig.cfgScaledOutputMax;
    } else {
        // Mapeo lineal entre los límites de los márgenes
        mScaledValue = mConfig.cfgScaledOutputMin + 
                       (mFilteredValue - lowerMarginLimit) * 
                       (mConfig.cfgScaledOutputMax - mConfig.cfgScaledOutputMin) / 
                       (upperMarginLimit - lowerMarginLimit);
    }
}

// Implementación de la detección de implausibilidades (no bloqueante)
void AnalogSensor::mProcessImplausibility() {
    ImplausibilityType currentImplausibility = ImplausibilityType::NONE;

    if (mRawValue <= mConfig.cfgAdcShortGND) {
        currentImplausibility = ImplausibilityType::SHORT_TO_GND;
    } else if (mRawValue >= mConfig.cfgAdcShortVCC) {
        currentImplausibility = ImplausibilityType::SHORT_TO_VCC;
    } else if (mRawValue < mConfig.cfgAdcMinNormal || mRawValue > mConfig.cfgAdcMaxNormal) {
        currentImplausibility = ImplausibilityType::OUT_OF_RANGE;
    }

    if (currentImplausibility != ImplausibilityType::NONE) {
        if (!mIsImplausibilityTiming) {
            // Inicia el temporizador para una nueva implausibilidad detectada
            mIsImplausibilityTiming = true;
            mImplausibilityStartTime = millis();
            mPendingImplausibility = currentImplausibility;
        } else if (currentImplausibility != mPendingImplausibility) {
            // Si el tipo de implausibility cambia, reinicia el temporizador
            mImplausibilityStartTime = millis();
            mPendingImplausibility = currentImplausibility;
        }
        
        // Comprueba si ha pasado el tiempo de timeout
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
    // Devolverá un estado PENDING solo cuando se este esperando un nuevo dato 
    // para decidir si existe una implausibilidad
    if (mIsImplausibilityTiming) {
        return SensorState::PENDING;
    }
    return mState; 
}

ImplausibilityType AnalogSensor::getImplausibilityType() const { return mImplausibilityType; }

uint16_t AnalogSensor::getRawValue() const { return mRawValue; }