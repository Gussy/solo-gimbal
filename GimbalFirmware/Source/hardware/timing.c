#include "PM_Sensorless.h"
#include "hardware/timing.h"

uint32_t get_cycle_count_32(void) {
    return CpuTimer2Regs.TIM.all;
}

#define CYCLES_PER_MICRO 80UL
#define CYCLES_PER_MILLI (CYCLES_PER_MICRO*1000)

static uint32_t last_cycles = 0;
static uint64_t cycle_count_64 = 0;

void update_cycle_count(void) {
    uint32_t tnow_cycles = get_cycle_count_32();
    if (tnow_cycles < last_cycles) {
        cycle_count_64 += UINT32_MAX;
    }
    last_cycles = tnow_cycles;
    cycle_count_64 &= 0xFFFFFFFF00000000UL;
    cycle_count_64 += tnow_cycles;
}

uint32_t millis(void) {
    update_cycle_count();
    return (uint32_t)(cycle_count_64/CYCLES_PER_MILLI);
}

uint32_t micros(void) {
    update_cycle_count();
    return (uint32_t)(cycle_count_64/CYCLES_PER_MICRO);
}
