#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "shared_data.h"
#include "can_manager.h"
#include "vehicle_control.h"

SemaphoreHandle_t dataMutex;
InverterCmd cmdData = {0};
VCU_Telemetry vcuData = {0};

void setup() {
    // Arduino ya ha inicializado el hardware en este punto, así que omitimos initArduino()
    Serial.begin(115200);
    Serial.println("VCU Booting Dual-Core...");

    // 1. Inicializar el Mutex (Crucial para no mezclar memoria entre los dos núcleos)
    dataMutex = xSemaphoreCreateMutex();

    // 2. Iniciar tarea de CAN BUS (Anclada al Núcleo 0 - Protocol CPU)
    // El núcleo 0 se dedicará en exclusiva a que MART_CAN envíe mensajes sin latencia
    xTaskCreatePinnedToCore(
        can_task,         
        "CAN_Task",       
        8192,             // Stack amplio (Las clases de Arduino como Serial/CAN consumen memoria)
        NULL,             
        5,                // Prioridad alta (Crítico para el vehículo)
        NULL,             
        0                 // <-- NÚCLEO 0
    );

    // 3. Iniciar tarea de Control y Lógica (Anclada al Núcleo 1 - Application CPU)
    // Aquí se ejecutarán tus lecturas de ADC/MCP3208, comprobación del R2D y APPS
    xTaskCreatePinnedToCore(
        control_task,     
        "Control_Task",   
        8192,             
        NULL,             
        4,                
        NULL,             
        1                 // <-- NÚCLEO 1
    );
}

void loop() {
    // Como hemos delegado todo a nuestras tareas de FreeRTOS independientes, 
    // el núcleo de Arduino ya no necesita ejecutar su bucle habitual.
    // Con esta instrucción eliminamos la tarea 'loop' original para liberar CPU y Memoria.
    vTaskDelete(NULL);
}