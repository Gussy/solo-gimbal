#include "PM_Sensorless.h"
#include "hardware/timing.h"
#include <stdbool.h>
#include <hardware/uart.h>
#include "string.h"
#include <stdio.h>

#define CYCLES_PER_MICRO 80UL
#define CYCLES_PER_MILLI (CYCLES_PER_MICRO*1000)

static uint32_t timer_tick_count = 0;
static uint32_t scheduler_timer_tick_count = 0;

static struct SchedTask* scheduled_tasks = 0;
static size_t num_scheduled_tasks = 0;

void scheduler_init(struct SchedTask* tasks, size_t num_tasks) {
    CpuTimer0Regs.PRD.all = mSec1;
    scheduled_tasks = tasks;
    num_scheduled_tasks = num_tasks;
}

void check_timer_tick() {
    if (CpuTimer0Regs.TCR.bit.TIF == 1) {
        CpuTimer0Regs.TCR.bit.TIF = 1;
        timer_tick_count++;
    }
}

void run_scheduler() {
    check_timer_tick();
    if (scheduled_tasks != 0 && scheduler_timer_tick_count != timer_tick_count) {
        while(scheduler_timer_tick_count != timer_tick_count) {
            uint8_t i;
            for(i=0; i<num_scheduled_tasks; i++) {
                if (scheduler_timer_tick_count-scheduled_tasks[i].last_run_ms >= scheduled_tasks[i].interval_ms) {
                    (*scheduled_tasks[i].task_func)();
                    scheduled_tasks[i].last_run_ms = scheduler_timer_tick_count;
                }
            }
            scheduler_timer_tick_count++;
        }
    }
}

uint32_t millis(void) {
    check_timer_tick();
    return timer_tick_count;
}

uint32_t micros(void) {
    check_timer_tick();
    return timer_tick_count*1000UL + (CpuTimer0Regs.PRD.all-CpuTimer0Regs.TIM.all)/CYCLES_PER_MICRO;
}
