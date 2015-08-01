#ifndef PLL_H_
#define PLL_H_

#include "F2806x_Device.h" // Included for Uint16 typedef

#define DIVSEL_BY_4     0
#define DIVSEL_BY_2     2
#define DIVSEL_BY_1     3

// SYSTEM CLOCK speed based on external 20MHz crystal
#define PLL_90MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x9
#define PLL_80MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x8
#define PLL_70MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x7
#define PLL_60MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x6
#define PLL_50MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x5
#define PLL_40MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x4
#define PLL_30MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x3
#define PLL_20MHZ_SYSTEM_CLOCK_20MHZ_XTAL 0x2


// Call with one of the values above
void PLLset(Uint16 val);

#endif /* PLL_H_ */
