#include "PairedAnalogSensor.h"
#include <cmath> // Para std::abs

PairedAnalogSensor::PairedAnalogSensor(const PairedAnalogSensorConfig& config)
    : mConfig(config),
      mAverageValue(0.0),
      mState(SensorState::NORMAL),
      mImplausibilityType(PairedImplausibilityType::NONE)
{
    // Creación dinámica de los sensores internos
    mSensor1 = new AnalogSensor(mConfig.cfgSensor1);
    mSensor2 = new AnalogSensor(mConfig.cfgSensor2);
}

PairedAnalogSensor::~PairedAnalogSensor() {
    // Liberar la memoria de los objetos creados dinámicamente
    delete mSensor1;
    delete mSensor2;
}

void PairedAnalogSensor::update(uint16_t rawValue1, uint16_t rawValue2) {
    // 1. Actualizar cada sensor individualmente
    // Se crean variables locales para cumplir con la nueva firma de AnalogSensor::update.
    // Aunque no usemos estas variables locales directamente aquí, la llamada actualiza
    // el estado interno de mSensor1 y mSensor2, que es lo que necesitamos.
    float filtered1, scaled1;
    SensorState state1;
    mSensor1->update(rawValue1, filtered1, scaled1, state1);

    float filtered2, scaled2;
    SensorState state2;
    mSensor2->update(rawValue2, filtered2, scaled2, state2);

    // 2. Comprobar la plausibilidad entre ambos
    // Esta función ya usa los getters, por lo que funcionará correctamente
    // con el estado interno actualizado de los sensores.
    mCheckPlausibility();

    // 3. Calcular el valor medio solo si el estado es normal
    if (mState == SensorState::NORMAL) {
        mAverageValue = (mSensor1->getScaledValue() + mSensor2->getScaledValue()) / 2.0f;
    } else {
        // En caso de fallo, el valor de salida debe ser seguro (ej. 0 para el acelerador)
        mAverageValue = 0.0f;
    }
}

void PairedAnalogSensor::mCheckPlausibility() {
    // Primero, comprobar si alguno de los sensores ha fallado individualmente
    if (mSensor1->getSensorState() == SensorState::IMPLAUSIBILITY) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR1_FAULT;
        return;
    }
    if (mSensor2->getSensorState() == SensorState::IMPLAUSIBILITY) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR2_FAULT;
        return;
    }

    // Si ambos sensores están bien, comprobar la desviación entre ellos
    float scaledValue1 = mSensor1->getScaledValue();
    float scaledValue2 = mSensor2->getScaledValue();
    
    // Usamos el rango del primer sensor como referencia para el cálculo del porcentaje
    float outputRange = mConfig.cfgSensor1.cfgScaledOutputMax - mConfig.cfgSensor1.cfgScaledOutputMin;
    if (outputRange <= 0) { // Evitar división por cero
        mState = SensorState::NORMAL;
        mImplausibilityType = PairedImplausibilityType::NONE;
        return;
    }

    float deviation = std::abs(scaledValue1 - scaledValue2);
    float deviationPercent = (deviation / outputRange) * 100.0f;

    if (deviationPercent > mConfig.cfgMaxDeviationPercent) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::DEVIATION_FAULT;
    } else {
        // Si todo es correcto
        mState = SensorState::NORMAL;
        mImplausibilityType = PairedImplausibilityType::NONE;
    }
}

// Getters
float PairedAnalogSensor::getAverageValue() const { return mAverageValue; }
SensorState PairedAnalogSensor::getSensorState() const { return mState; }
PairedImplausibilityType PairedAnalogSensor::getImplausibilityType() const { return mImplausibilityType; }
const AnalogSensor& PairedAnalogSensor::getSensor1() const { return *mSensor1; }
const AnalogSensor& PairedAnalogSensor::getSensor2() const { return *mSensor2; }