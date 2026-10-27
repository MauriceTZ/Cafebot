#pragma once
#include "config.h"
#include "control.h"

// El control corre en una TAREA de FreeRTOS, no en la interrupción de un timer:
// dentro de una interrupción no se puede esperar un semáforo, usar colas sin
// FromISR, pedir memoria (new/delete) ni llamar a ledcWrite. Una tarea puede
// hacer todo eso, y vTaskDelayUntil la repite a un ritmo fijo.
void tarea_control(void*) {
  TickType_t ultimo_despertar = xTaskGetTickCount();
  const TickType_t periodo = pdMS_TO_TICKS(1000 / TIMER_TRIGGER_FREQUENCY);
  for (;;) {
    // El mutex protege la posición, la velocidad y el objetivo, que el loop
    // (comandos por serial) también lee y modifica.
    if (xSemaphoreTake(mutex_pos_carro, portMAX_DELAY)) {
      control_carro();
      xSemaphoreGive(mutex_pos_carro);
    }
    vTaskDelayUntil(&ultimo_despertar, periodo);
  }
}

// Llamar DESPUÉS de control_begin() (que crea el mutex y la cola).
void timer_begin() {
  // Prioridad 3 (> loop, que es 1) en el núcleo 1, el mismo del loop.
  xTaskCreatePinnedToCore(tarea_control, "control", 8192, NULL, 3, NULL, 1);
}
