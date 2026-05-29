#include "PairedAnalogSensor.h"
#include <cmath> // Para std::abs

PairedAnalogSensor::PairedAnalogSensor(const PairedAnalogSensorConfig& config)
    : mConfig(config),
      mSensor1(mConfig.cfgSensor1),
      mSensor2(mConfig.cfgSensor2),
      mSensitive(nullptr),
      mFilteredValue(0.0),
      mScaledValue(0.0),
      mState(SensorState::NORMAL),
      mImplausibilityType(PairedImplausibilityType::NONE),
      mDeviationTiming(false),
      mDeviationStartTime(0)
{
    // Selecciona el sensor con mayor rango de ADC (más resolución) como "sensible".
    float sensitiSensor1 = mConfig.cfgSensor1.cfgAdcMaxNormal - mConfig.cfgSensor1.cfgAdcMinNormal;
    float sensitiSensor2 = mConfig.cfgSensor2.cfgAdcMaxNormal - mConfig.cfgSensor2.cfgAdcMinNormal;
    if (fabsf(sensitiSensor1) >= fabsf(sensitiSensor2)) {
        mSensitive = &mSensor1;
    } else {
        mSensitive = &mSensor2;
    }
}

void PairedAnalogSensor::update(uint16_t rawValue1, uint16_t rawValue2, float currentVoltage,
        float& meanFilteredValue, float& meanScaledValue,
        float& sensitiveFilteredValue, float& sensitiveScaledValue,
        SensorState& state) {

    float filteredValue1, scaledValue1;
    SensorState state1;
    float filteredValue2, scaledValue2;
    SensorState state2;

    // 1. Actualizar cada sensor individualmente (con compensación por voltaje).
    mSensor1.update(rawValue1, currentVoltage, filteredValue1, scaledValue1, state1);
    mSensor2.update(rawValue2, currentVoltage, filteredValue2, scaledValue2, state2);

    // 2. Comprobar la plausibilidad entre ambos.
    mCheckPlausibility();

    // 3. Calcular los valores medios sólo si el estado es normal.
    if (mState == SensorState::NORMAL) {
        mScaledValue = (scaledValue1 + scaledValue2) / 2.0f;
        mFilteredValue = (filteredValue1 + filteredValue2) / 2.0f;
    } else {
        // En caso de fallo, la salida debe ser segura (0 para el acelerador).
        mScaledValue = 0.0f;
        mFilteredValue = 0.0f;
    }

    // 4. Valores a devolver.
    meanScaledValue = getMeanScaledValue();
    meanFilteredValue = getMeanFilteredValue();
    sensitiveFilteredValue = getSensitiveFilteredValue();
    sensitiveScaledValue = getSensitiveScaledValue();
    state = getSensorState();
}

void PairedAnalogSensor::update(uint16_t rawValue1, uint16_t rawValue2,
        float& meanFilteredValue, float& meanScaledValue,
        float& sensitiveFilteredValue, float& sensitiveScaledValue,
        SensorState& state) {
    // Sin compensación por voltaje: límites estáticos.
    update(rawValue1, rawValue2, -1.0f,
           meanFilteredValue, meanScaledValue,
           sensitiveFilteredValue, sensitiveScaledValue, state);
}

void PairedAnalogSensor::mCheckPlausibility() {
    // Primero, comprobar si alguno de los sensores ha fallado individualmente.
    if (mSensor1.getSensorState() == SensorState::IMPLAUSIBILITY
        && mSensor2.getSensorState() == SensorState::IMPLAUSIBILITY) {
        mDeviationTiming = false;
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR1y2_FAULT;
        return;
    }
    if (mSensor1.getSensorState() == SensorState::IMPLAUSIBILITY) {
        mDeviationTiming = false;
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR1_FAULT;
        return;
    }
    if (mSensor2.getSensorState() == SensorState::IMPLAUSIBILITY) {
        mDeviationTiming = false;
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR2_FAULT;
        return;
    }

    // Si ambos sensores están bien, comprobar la desviación entre ellos.
    float scaledValue1 = mSensor1.getScaledValue();
    float scaledValue2 = mSensor2.getScaledValue();

    // Rango de salida del primer sensor como referencia para el porcentaje.
    float outputRange = mConfig.cfgSensor1.cfgScaledOutputMax - mConfig.cfgSensor1.cfgScaledOutputMin;
    if (outputRange <= 0) { // Evitar división por cero.
        mDeviationTiming = false;
        mState = SensorState::NORMAL;
        mImplausibilityType = PairedImplausibilityType::NONE;
        return;
    }

    float deviation = std::abs(scaledValue1 - scaledValue2);
    float deviationPercent = (deviation / outputRange) * 100.0f;

    // La desviación debe mantenerse cfgDeviationTimeout ms antes de cortar.
    if (deviationPercent > mConfig.cfgMaxDeviationPercent) {
        if (!mDeviationTiming) {
            mDeviationTiming = true;
            mDeviationStartTime = millis();
        }
        if (millis() - mDeviationStartTime > mConfig.cfgDeviationTimeout) {
            mState = SensorState::IMPLAUSIBILITY;
            mImplausibilityType = PairedImplausibilityType::DEVIATION_FAULT;
        } else {
            mState = SensorState::NORMAL;
            mImplausibilityType = PairedImplausibilityType::NONE;
        }
    } else {
        mDeviationTiming = false;
        mState = SensorState::NORMAL;
        mImplausibilityType = PairedImplausibilityType::NONE;
    }
}

// Getters
float PairedAnalogSensor::getMeanScaledValue() const { return mScaledValue; }
float PairedAnalogSensor::getMeanFilteredValue() const { return mFilteredValue; }
float PairedAnalogSensor::getSensitiveScaledValue() const { return mSensitive->getScaledValue(); }
float PairedAnalogSensor::getSensitiveFilteredValue() const { return mSensitive->getFilteredValue(); }
SensorState PairedAnalogSensor::getSensorState() const { return mState; }
PairedImplausibilityType PairedAnalogSensor::getImplausibilityType() const { return mImplausibilityType; }
const AnalogSensor& PairedAnalogSensor::getSensor1() const { return mSensor1; }
const AnalogSensor& PairedAnalogSensor::getSensor2() const { return mSensor2; }
