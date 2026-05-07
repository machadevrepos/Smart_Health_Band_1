#ifndef SHB_BOARD_H_
#define SHB_BOARD_H_

#include <stdbool.h>
#include <zephyr/kernel.h>

enum shb_led_id {
	SHB_LED_TEMP = 0,
	SHB_LED_BMI,
	SHB_LED_COUNT,
};

int shb_board_init(void);
int shb_board_led_pulse(enum shb_led_id led, k_timeout_t duration);

/*
 * Alert haptic: used for user-facing notifications/alerts. Runs for the given
 * duration and leaves shb_board_haptic_alert_is_active() true until it expires
 * or is explicitly stopped via shb_board_haptic_alert_stop(). Single-tap during
 * an active alert acknowledges and stops it.
 */
int shb_board_haptic_alert_start(k_timeout_t duration);
int shb_board_haptic_alert_stop(void);
bool shb_board_haptic_alert_is_active(void);

/*
 * Feedback haptic: short confirmation pulse (e.g. double-tap ack). Does NOT
 * mark the alert state as active so it won't be mistaken for an alert pending
 * acknowledgement.
 */
int shb_board_haptic_feedback_pulse(k_timeout_t duration);

#endif
