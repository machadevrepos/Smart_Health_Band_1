#ifndef SHB_POWER_H_
#define SHB_POWER_H_

#include <stdint.h>
#include <zephyr/kernel.h>

enum shb_power_event {
	SHB_POWER_EVENT_BMI_WAKE        = BIT(0),
	SHB_POWER_EVENT_MAX32664_VITALS = BIT(1),
	SHB_POWER_EVENT_BMI_GESTURE     = BIT(2),
};

int shb_power_init(void);
void shb_power_signal(uint32_t events);
uint32_t shb_power_wait_for_wake(k_timeout_t timeout);

#endif
