#ifndef WATCHDOG_H_
#define WATCHDOG_H_

void watchdog_disable();
void watchdog_enable();
void watchdog_device_reset();
void watchdog_immediate_device_reset();

#endif /* WATCHDOG_H_ */
