#pragma once
#include "config.h"
#include "control.h"

// Interrupt Service Routine
// Se dispara cada vez que el timer llega al valor de disparo
void IRAM_ATTR timer_ISR() {
    control_carro();
}

void timer_begin() {
    timer_Cfg = timerBegin(TIMER_TICK_FREQUENCY);
    timerAttachInterrupt(timer_Cfg, &timer_ISR);
    timerAlarm(timer_Cfg, TIMER_TICK_FREQUENCY / TIMER_TRIGGER_FREQUENCY, true, 0);
}