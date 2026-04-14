// vehicle_control.cpp
#include "vehicle_control.h"
#include "shared_data.h"
#include "config.h"       // ¡ESTO SOLUCIONA LOS ERRORES DE 'PIN_...' y 'CFG_RPMAX'!
#include "driver/gpio.h"
#include <Arduino.h>      // Nos da acceso a millis() y map() para la lógica de pedales

static const char* TAG = "VEHICLE_CTRL";

long map_val(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

bool check_apps(int apps1, int apps2) {
    // Lógica de implausibilidad (ajustar con tus valores)
    return (apps1 > 700 && apps1 < 2300) && (apps2 > 1700 && apps2 < 2700);
}

bool check_r2d(bool sdc_ok, bool start_pressed, bool brake_pressed, bool apps_ok) {
    static int step = 0;
    static uint32_t tAux = 0;
    bool r2d_active = false;

    // Apagar Buzzer pasado el tiempo
    if ((millis() - tAux) >= TIME_SOUND_R2D) {
        gpio_set_level((gpio_num_t)PIN_BUZZ, 0);
    }

    switch (step) {
        case 0:
            if (sdc_ok && apps_ok) step = 10;
            break;
        case 10:
            if (!(sdc_ok && apps_ok)) {
                step = 0;
            }
            else if (start_pressed && brake_pressed) {
                tAux = millis();
                gpio_set_level((gpio_num_t)PIN_BUZZ, 1);
                step = 20;
            }
            break;
        case 20:
            if (!(sdc_ok && apps_ok)) {
                step = 0;
            } else {
                r2d_active = true;
            }
            break;
    }
    return r2d_active;
}

void control_task(void *pvParameters) {
    Serial.println("Control_Task: Iniciada en Núcleo 1");

    while (1) {
        // 1. LECTURA DE HARDWARE (Aquí usarías tu lectura SPI del MCP3208)
        bool startBtn = gpio_get_level((gpio_num_t)PIN_START) == 0;
        
        // Mock de lecturas (reemplazar con: adc.read(MCP3208::Channel::SINGLE_x))
        int adc_apps1 = 1500; 
        int adc_apps2 = 2000;
        int adc_brake = 800;
        int adc_sdc = 2100;

        // 2. PROCESAMIENTO
        int apps1_scaled = map_val(adc_apps1, 1935, 1055, 1000, 0);
        int apps2_scaled = map_val(adc_apps2, 2068, 2290, 1000, 0);
        bool apps_ok = check_apps(adc_apps1, adc_apps2);
        bool sdc_ok = adc_sdc > 2000;

        bool r2d = check_r2d(sdc_ok, startBtn, (adc_brake > 550), apps_ok);

        // 3. ACTUALIZACIÓN SEGURA DE VARIABLES HACIA EL CAN BUS (MUTEX)
        if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE) {
            
            // Llenamos los datos para telemetría (Soluciona el error 'vState was not declared')
            vcuData.apps1_analog = adc_apps1;
            vcuData.apps2_analog = adc_apps2;
            vcuData.apps1_scaled = apps1_scaled;
            vcuData.apps2_scaled = apps2_scaled;
            vcuData.stsR2D = r2d;
            vcuData.stsAPPS_ok = apps_ok;

            // Preparamos los comandos del motor
            if (r2d && apps_ok) {
                cmdData.driveEnable = true;
                cmdData.RPMtarget = map_val(apps1_scaled, 1000, 0, 0, CFG_RPMAX * 10);
                cmdData.currentTarget = apps2_scaled;
                cmdData.cfgCurrentACMAX = 190; // Tus valores por defecto
                cmdData.cfgCurrentDCMAX = 60;
                cmdData.ctrlByPctg = true;
            } else {
                cmdData.driveEnable = false;
            }
            
            xSemaphoreGive(dataMutex);
        }

        // Salida digital de hardware
        gpio_set_level((gpio_num_t)PIN_R2D_DIGITAL, cmdData.driveEnable ? 1 : 0);

        // Retardo para ceder tiempo a la CPU y que corra a ~100Hz
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}