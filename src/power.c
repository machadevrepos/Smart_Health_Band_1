#include <zephyr/logging/log.h>
#include "power.h"

LOG_MODULE_REGISTER(shb_power, CONFIG_SHB_LOG_LEVEL);

static struct k_event shb_wake_events;

int shb_power_init(void)
{
	k_event_init(&shb_wake_events);
	return 0;
}

void shb_power_signal(uint32_t events)
{
	k_event_post(&shb_wake_events, events);
}

uint32_t shb_power_wait_for_wake(k_timeout_t timeout)
{
	return k_event_wait(&shb_wake_events,
			    SHB_POWER_EVENT_BMI_WAKE |
			    SHB_POWER_EVENT_MAX32664_VITALS |
			    SHB_POWER_EVENT_BMI_GESTURE,
			    true, timeout);
}
