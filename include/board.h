#ifndef SHB_BOARD_H_
#define SHB_BOARD_H_

#include <stdbool.h>
#include <zephyr/kernel.h>

enum shb_led_id {
	SHB_LED_TEMP = 0,
	SHB_LED_BMI,
	SHB_LED_HEART,
	SHB_LED_COUNT,
};

int shb_board_init(void);
void shb_board_log_pin_summary(void);
int shb_board_led_set(enum shb_led_id led, bool on);
int shb_board_led_pulse(enum shb_led_id led, k_timeout_t duration);

#endif
