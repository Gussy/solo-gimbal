#include "PM_Sensorless.h"
#include "hardware/timing.h"
#include <stdbool.h>
#include <hardware/uart.h>
#include "string.h"
#include <stdio.h>

#define CYCLES_PER_MICRO 80UL
#define CYCLES_PER_MILLI (CYCLES_PER_MICRO*1000)

static uint64_t cycle_count_on_tick = 0;

void timer_init(void) {
    CpuTimer0Regs.PRD.all = UINT32_MAX;
}

bool check_timer_tick(void) {
    if (CpuTimer0Regs.TCR.bit.TIF == 1) {
        CpuTimer0Regs.TCR.bit.TIF = 1;
        cycle_count_on_tick += CpuTimer0Regs.PRD.all;
        return true;
    }
    return false;
}

uint64_t cycles_64(void) {
    uint32_t cycles_since_tick = CpuTimer0Regs.PRD.all-CpuTimer0Regs.TIM.all;

    if (check_timer_tick()) {
        cycles_since_tick = CpuTimer0Regs.PRD.all-CpuTimer0Regs.TIM.all;
    }

    return cycle_count_on_tick + cycles_since_tick;
}

uint32_t millis(void) {
    return cycles_64()/CYCLES_PER_MILLI;
}

uint32_t micros(void) {
    return cycles_64()/CYCLES_PER_MICRO;
}

static struct SchedTask* scheduled_tasks = 0;
static size_t num_scheduled_tasks = 0;

void scheduler_init(struct SchedTask* tasks, size_t num_tasks) {
    scheduled_tasks = tasks;
    num_scheduled_tasks = num_tasks;
}

void run_scheduler() {
    static uint32_t last_run_ms = 0;
    uint32_t tnow_ms = millis();
    if (scheduled_tasks != 0 && tnow_ms != last_run_ms) {
        uint8_t i;
        for(i=0; i<num_scheduled_tasks; i++) {
            uint32_t ms_since_task = tnow_ms-scheduled_tasks[i].last_run_ms;
            if (ms_since_task >= scheduled_tasks[i].interval_ms) {
                (*scheduled_tasks[i].task_func)();
                scheduled_tasks[i].last_run_ms = tnow_ms-(ms_since_task%scheduled_tasks[i].interval_ms);
            }
        }
        last_run_ms = tnow_ms;
    }
}
