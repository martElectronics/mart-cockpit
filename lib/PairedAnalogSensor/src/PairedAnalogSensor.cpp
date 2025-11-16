#include "PairedAnalogSensor.h"
#include <cmath> // Para std::abs

PairedAnalogSensor::PairedAnalogSensor(const PairedAnalogSensorConfig& config)
    : mConfig(config),
      mFilteredValue(0.0),
      mScaledValue(0.0),
      mState(SensorState::NORMAL),
      mImplausibilityType(PairedImplausibilityType::NONE)
{
    // Creación dinámica de los sensores internos
    mSensor1 = new AnalogSensor(mConfig.cfgSensor1);
    mSensor2 = new AnalogSensor(mConfig.cfgSensor2);

    // Obtener el sensor más sensible
    float sensitiSensor1 = mConfig.cfgSensor1.cfgAdcMaxNormal - mConfig.cfgSensor1.cfgAdcMinNormal;
    float sensitiSensor2 = mConfig.cfgSensor2.cfgAdcMaxNormal - mConfig.cfgSensor2.cfgAdcMinNormal;
    if (sensitiSensor1 >= sensitiSensor2) {
        mSensitive = mSensor1;
    } else {
        mSensitive = mSensor2;
    }
}

PairedAnalogSensor::~PairedAnalogSensor() {
    // Liberar la memoria de los objetos creados dinámicamente
    delete mSensor1;
    delete mSensor2;
}

void PairedAnalogSensor::update(uint16_t rawValue1, uint16_t rawValue2,
        float& meanFilteredValue, float& meanScaledValue, 
        float& sensitiveFilteredValue, float& sensitiveScaledValue, 
        SensorState& state) {

    // Valores sensor 1
    float filteredValue1;
    float scaledValue1;
    SensorState state1;
    // Valores sensor 2
    float filteredValue2;
    float scaledValue2;
    SensorState state2;
    
    // 1. Actualizar cada sensor individualmente
    mSensor1->update(rawValue1, filteredValue1, scaledValue1, state1);
    mSensor2->update(rawValue2, filteredValue2, scaledValue2, state2);

    // 2. Comprobar la plausibilidad entre ambos
    // Esta función ya usa los getters, por lo que funcionará correctamente
    // con el estado interno actualizado de los sensores.
    mCheckPlausibility();

    // 3. Calcular el valor medio solo si el estado es normal
    if (mState == SensorState::NORMAL) {
        mScaledValue = (filteredValue1 + filteredValue2) / 2.0f;
        mFilteredValue = (scaledValue1 + scaledValue2) / 2.0f;
    } else {
        // En caso de fallo, el valor de salida debe ser seguro (ej. 0 para el acelerador)
        mScaledValue = 0.0f;
        mFilteredValue = 0.0f;
    }

    // Valores a devolver

    meanScaledValue = getMeanScaledValue();
    meanFilteredValue = getMeanFilteredValue();
    sensitiveFilteredValue = getSensitiveScaledValue();
    sensitiveScaledValue = getSensitiveFilteredValue();
    state = getSensorState();
}

void PairedAnalogSensor::mCheckPlausibility() {
    // Primero, comprobar si alguno de los sensores ha fallado individualmente
    if (mSensor1->getSensorState() == SensorState::IMPLAUSIBILITY
        && mSensor2->getSensorState() == SensorState::IMPLAUSIBILITY) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR1y2_FAULT;
        return;
    }
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
float PairedAnalogSensor::getMeanScaledValue() const { return mScaledValue; }

float PairedAnalogSensor::getMeanFilteredValue() const { return mFilteredValue; }

float PairedAnalogSensor::getSensitiveScaledValue() const { return mSensitive->getScaledValue(); }

float PairedAnalogSensor::getSensitiveFilteredValue() const { return mSensitive->getFilteredValue();  }

SensorState PairedAnalogSensor::getSensorState() const { return mState; }

PairedImplausibilityType PairedAnalogSensor::getImplausibilityType() const { return mImplausibilityType; }

const AnalogSensor& PairedAnalogSensor::getSensor1() const { return *mSensor1; }

const AnalogSensor& PairedAnalogSensor::getSensor2() const { return *mSensor2; }