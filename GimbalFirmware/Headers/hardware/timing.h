#ifndef HARDWARE_TIMING_H_
#define HARDWARE_TIMING_H_

#include <inttypes.h>
#include <stddef.h>

/*-----------------------------------------------------------------------------
      Specify the clock rate of the CPU (SYSCLKOUT) in nS.

      Take into account the input clock frequency and the PLL multiplier
      selected in step 1.

      Use one of the values provided, or define your own.
      The trailing L is required tells the compiler to treat
      the number as a 64-bit value.

      Only one statement should be uncommented.

      Example:   80 MHz devices:
                 CLKIN is a 10 MHz crystal or internal 10 MHz oscillator

                 In step 1 the user specified PLLCR = 0x16 for a
                 80 MHz CPU clock (SYSCLKOUT = 80 MHz).

                 In this case, the CPU_RATE will be 12.500L
                 Uncomment the line: #define CPU_RATE 12.500L

-----------------------------------------------------------------------------*/
#define CPU_RATE    12.500L // for a 80MHz CPU clock speed (SYSCLKOUT)

#define DELAY_US(A) DSP28x_usDelay(((((long double) A * 1000.0L) / (long double)CPU_RATE) - 9.0L) / 5.0L)

typedef void (*task_func_t)(void);
struct SchedTask {
    task_func_t task_func;
    uint16_t interval_ms;
    uint32_t last_run_ms;
};

// timing functions
void timer_init(void);
uint64_t cycles_64(void);
uint32_t millis(void);
uint32_t micros(void);

// scheduler
void scheduler_init(struct SchedTask* tasks, size_t num_tasks);
void run_scheduler();

#endif /* HARDWARE_TIMING_H_ */
