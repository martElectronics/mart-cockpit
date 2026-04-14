// shared_data.h
#pragma once
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

struct InverterCmd {
    int currentTarget;
    int RPMtarget;
    int cfgCurrentACMAX;
    int cfgCurrentDCMAX;
    bool driveEnable;
    bool ctrlByPctg;
};

// Datos que enviaremos a la telemetría/VCU y usamos para el estado
struct VCU_Telemetry {
    uint16_t apps1_scaled;
    uint16_t apps2_scaled;
    uint16_t apps1_analog;
    uint16_t apps2_analog;
    int brake_analog;
    int brake2_analog;
    int vbat_raw;
    bool stsR2D;
    bool stsAPPS_ok;
};

extern SemaphoreHandle_t dataMutex;
extern InverterCmd cmdData;
extern VCU_Telemetry vcuData;