/*
 * Copyright (C) 2017 Universitat Oberta de Catalunya - http://www.uoc.edu/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Universitat Oberta de Catalunya nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*----------------------------------------------------------------------------*/
// Includes standard
#include <stdio.h>
#include <stdint.h>

// Includes MSP432 Launchpad
#include "msp432_launchpad_board.h"

// Includes FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*----------------------------------------------------------------------------*/
// Definicion de prioridades de tareas
#define prvLED_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)

/*----------------------------------------------------------------------------*/

// Prototipos de funciones privadas
static void prvLedTask(void *pvParameters);

/*----------------------------------------------------------------------------*/
// Semaoforo global
SemaphoreHandle_t semphr_button;

/*----------------------------------------------------------------------------*/
int main(void)
{
    // Inicializacion MSP432 Launchpad
    board_init();

    // Creamos semaforo
    semphr_button = xSemaphoreCreateBinary();

    if (semphr_button != NULL) {
        // Creacion de tarea RedLedTask
        xTaskCreate(prvLedTask,   // Puntero a la funcion que implementa la tarea
                "LedTask",              // Nombre descriptivo de la tarea
                configMINIMAL_STACK_SIZE,  // Tamaño del stack de la tarea
                NULL,                      // Argumentos de la tarea
                prvLED_TASK_PRIORITY,  // Prioridad de la tarea
                NULL);

        // Puesta en marcha de las tareas creadas
        vTaskStartScheduler();
    }
    // Solo llega aqui si no hay suficiente memoria
    // para iniciar el scheduler
    return 0;
}

/*----------------------------------------------------------------------------*/

static void prvLedTask(void *pvParameters)
{
    // Calcula el tiempo de activacion del LED (en ticks)
    // a partir del tiempo en milisegundos
    static const TickType_t xBlinkOn = pdMS_TO_TICKS(500);

    // La tarea se repite en un bucle infinito
    for (;;)
    {
        // Esperamos a coger el semaforo
        if (xSemaphoreTake(semphr_button, portMAX_DELAY) == pdTRUE)
        {
            // On LED verde
            led_green_on();

            // Bloquea la tarea durante el tiempo de On del LED
            vTaskDelay(xBlinkOn);

            // Off LED verde
            led_green_off();
        }
    }
}

/*----------------------------------------------------------------------------*/

// Rutina de Servicio a Interrupcion (ISR) del PORT1
void PORT1_IRQHandler(void)
{
    uint32_t status;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // Lee el estado de la interrupcion generada por GPIO_PORT_P1
    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);


    // Reset del flag de interrupcion del pin que la genera
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    // Chequea si la interrupcion la genero el pin P1.1
    if (status & GPIO_PIN1)
    {
        xSemaphoreGiveFromISR(semphr_button, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}
