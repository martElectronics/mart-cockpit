// --- START OF FILE PairedAnalogSensor.cpp ---

#include "PairedAnalogSensor.h"
#include <cmath> // Para std::abs

PairedAnalogSensor::PairedAnalogSensor(const PairedAnalogSensorConfig& config)
    : mConfig(config),
      mSensor1(nullptr),
      mSensor2(nullptr),
      mSensitive(nullptr),
      mFilteredValue(0.0f),
      mScaledValue(0.0f),
      mState(SensorState::NORMAL),
      mImplausibilityType(PairedImplausibilityType::NONE)
{
    // Creación dinámica de los sensores internos
    mSensor1 = new AnalogSensor(mConfig.cfgSensor1);
    mSensor2 = new AnalogSensor(mConfig.cfgSensor2);

    // Determinar cuál es el sensor "sensible" (el de mayor rango ADC)
    // Se asume que mayor rango de entrada implica mejor resolución/sensibilidad.
    float rangeSensor1 = mConfig.cfgSensor1.cfgAdcMaxNormal - mConfig.cfgSensor1.cfgAdcMinNormal;
    float rangeSensor2 = mConfig.cfgSensor2.cfgAdcMaxNormal - mConfig.cfgSensor2.cfgAdcMinNormal;

    if (rangeSensor1 >= rangeSensor2) {
        mSensitive = mSensor1;
    } else {
        mSensitive = mSensor2;
    }
}

PairedAnalogSensor::~PairedAnalogSensor() {
    // Liberar la memoria de los objetos creados dinámicamente
    if (mSensor1 != nullptr) delete mSensor1;
    if (mSensor2 != nullptr) delete mSensor2;
}

void PairedAnalogSensor::update(uint16_t rawValue1, uint16_t rawValue2,
        float& meanFilteredValue, float& meanScaledValue, 
        float& sensitiveFilteredValue, float& sensitiveScaledValue, 
        SensorState& state) {

    // Variables temporales para capturar la salida de los sensores individuales
    float valFiltered1, valScaled1;
    SensorState state1;

    float valFiltered2, valScaled2;
    SensorState state2;
    
    // 1. Actualizar cada sensor individualmente
    mSensor1->update(rawValue1, valFiltered1, valScaled1, state1);
    mSensor2->update(rawValue2, valFiltered2, valScaled2, state2);

    // 2. Comprobar la plausibilidad entre ambos
    // Esta función actualiza mState y mImplausibilityType basándose en los estados 
    // y valores escalados actuales de mSensor1 y mSensor2.
    mCheckPlausibility();

    // 3. Calcular el valor medio y gestionar estado de error
    if (mState == SensorState::NORMAL) {
        // Cálculo correcto: promedio de escalados a escalado, promedio de filtrados a filtrado
        mScaledValue = (valScaled1 + valScaled2) / 2.0f;
        mFilteredValue = (valFiltered1 + valFiltered2) / 2.0f;
    } else {
        // En caso de fallo, el valor de salida se fuerza a seguro (0.0)
        mScaledValue = 0.0f;
        mFilteredValue = 0.0f;
    }

    // 4. Asignar valores de retorno a las referencias
    meanScaledValue = getMeanScaledValue();
    meanFilteredValue = getMeanFilteredValue();
    
    sensitiveScaledValue = getSensitiveScaledValue();
    sensitiveFilteredValue = getSensitiveFilteredValue();
    
    state = getSensorState();
}

void PairedAnalogSensor::mCheckPlausibility() {
    SensorState s1State = mSensor1->getSensorState();
    SensorState s2State = mSensor2->getSensorState();

    // 1. Comprobar fallos eléctricos/individuales
    bool s1Fault = (s1State == SensorState::IMPLAUSIBILITY);
    bool s2Fault = (s2State == SensorState::IMPLAUSIBILITY);

    if (s1Fault && s2Fault) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR1y2_FAULT;
        return;
    }
    if (s1Fault) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR1_FAULT;
        return;
    }
    if (s2Fault) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::SENSOR2_FAULT;
        return;
    }

    // 2. Si ambos sensores están eléctricamente bien, comprobar la desviación
    float scaledVal1 = mSensor1->getScaledValue();
    float scaledVal2 = mSensor2->getScaledValue();
    
    // Usamos el rango de salida del primer sensor como referencia (Full Scale)
    float outputRange = mConfig.cfgSensor1.cfgScaledOutputMax - mConfig.cfgSensor1.cfgScaledOutputMin;
    
    // Evitar división por cero si la configuración es errónea
    if (std::abs(outputRange) < 1e-5f) { 
        mState = SensorState::NORMAL; // O IMPLAUSIBILITY si se considera config error
        mImplausibilityType = PairedImplausibilityType::NONE;
        return;
    }

    float deviation = std::abs(scaledVal1 - scaledVal2);
    float deviationPercent = (deviation / outputRange) * 100.0f;

    if (deviationPercent > mConfig.cfgMaxDeviationPercent) {
        mState = SensorState::IMPLAUSIBILITY;
        mImplausibilityType = PairedImplausibilityType::DEVIATION_FAULT;
    } else {
        // Todo correcto
        mState = SensorState::NORMAL;
        mImplausibilityType = PairedImplausibilityType::NONE;
    }
}

// --- Getters ---

float PairedAnalogSensor::getMeanScaledValue() const { 
    return mScaledValue; 
}

float PairedAnalogSensor::getMeanFilteredValue() const { 
    return mFilteredValue; 
}

float PairedAnalogSensor::getSensitiveScaledValue() const { 
    // Si mSensitive no está inicializado (no debería pasar), devolver 0 o manejar error
    if (mSensitive) return mSensitive->getScaledValue();
    return 0.0f;
}

float PairedAnalogSensor::getSensitiveFilteredValue() const { 
    if (mSensitive) return mSensitive->getFilteredValue();
    return 0.0f;
}

SensorState PairedAnalogSensor::getSensorState() const { 
    return mState; 
}

PairedImplausibilityType PairedAnalogSensor::getImplausibilityType() const { 
    return mImplausibilityType; 
}

const AnalogSensor& PairedAnalogSensor::getSensor1() const { 
    return *mSensor1; 
}

const AnalogSensor& PairedAnalogSensor::getSensor2() const { 
    return *mSensor2; 
}