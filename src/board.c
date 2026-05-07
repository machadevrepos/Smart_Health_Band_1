#include <errno.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "board.h"

LOG_MODULE_REGISTER(shb_board, CONFIG_SHB_LOG_LEVEL);

struct shb_led_ctx {
	struct gpio_dt_spec spec;
	struct k_work_delayable off_work;
	bool ready;
};

static struct shb_led_ctx shb_leds[SHB_LED_COUNT] = {
	[SHB_LED_TEMP] = {
		.spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(tmp_led), gpios, { 0 }),
	},
	[SHB_LED_BMI] = {
		.spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(bmi_led), gpios, { 0 }),
	},
	[SHB_LED_HEART] = {
		.spec = GPIO_DT_SPEC_GET_OR(DT_ALIAS(heart_led), gpios, { 0 }),
	},
};

static const struct gpio_dt_spec shb_haptic_spec =
	GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), haptic_gpios);
static struct k_work_delayable shb_haptic_off_work;
static bool shb_haptic_ready;
/*
 * 1 while an alert haptic is running (shb_board_haptic_alert_start).
 * 0 when cleared by explicit stop, by the off-work handler, or for feedback
 * pulses (which never set this flag).
 */
static atomic_t shb_haptic_alert_active;

static void shb_led_off_work(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);

	for (size_t i = 0; i < SHB_LED_COUNT; ++i) {
		if (&shb_leds[i].off_work == delayable && shb_leds[i].ready) {
			(void)gpio_pin_set_dt(&shb_leds[i].spec, 0);
		}
	}
}

static void shb_haptic_off_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	if (shb_haptic_ready) {
		(void)gpio_pin_set_dt(&shb_haptic_spec, 0);
	}
	atomic_clear(&shb_haptic_alert_active);
}

int shb_board_init(void)
{
	for (size_t i = 0; i < SHB_LED_COUNT; ++i) {
		k_work_init_delayable(&shb_leds[i].off_work, shb_led_off_work);
		if (!gpio_is_ready_dt(&shb_leds[i].spec)) {
			LOG_WRN("LED %u GPIO not ready or not present", (unsigned int)i);
			continue;
		}

		if (gpio_pin_configure_dt(&shb_leds[i].spec, GPIO_OUTPUT_INACTIVE) == 0) {
			shb_leds[i].ready = true;
		}
	}

	k_work_init_delayable(&shb_haptic_off_work, shb_haptic_off_work_fn);
	if (gpio_is_ready_dt(&shb_haptic_spec) &&
	    gpio_pin_configure_dt(&shb_haptic_spec, GPIO_OUTPUT_INACTIVE) == 0) {
		shb_haptic_ready = true;
	}

	return 0;
}

int shb_board_led_pulse(enum shb_led_id led, k_timeout_t duration)
{
	int ret;

	if ((led >= SHB_LED_COUNT) || !shb_leds[led].ready) {
		return -ENODEV;
	}

	ret = gpio_pin_set_dt(&shb_leds[led].spec, 1);
	if (ret < 0) {
		return ret;
	}

	return k_work_schedule(&shb_leds[led].off_work, duration);
}

static int shb_haptic_start_locked(k_timeout_t duration)
{
	int ret = gpio_pin_set_dt(&shb_haptic_spec, 1);

	if (ret < 0) {
		return ret;
	}

	/* Reschedule cancels any prior pending off-work so the duration resets
	 * cleanly whether we are starting an alert, extending one, or firing
	 * a feedback pulse while the motor is idle.
	 */
	return k_work_reschedule(&shb_haptic_off_work, duration);
}

int shb_board_haptic_alert_start(k_timeout_t duration)
{
	if (!shb_haptic_ready) {
		return -ENODEV;
	}

	atomic_set(&shb_haptic_alert_active, 1);
	return shb_haptic_start_locked(duration);
}

int shb_board_haptic_alert_stop(void)
{
	if (!shb_haptic_ready) {
		return -ENODEV;
	}

	(void)k_work_cancel_delayable(&shb_haptic_off_work);
	atomic_clear(&shb_haptic_alert_active);
	return gpio_pin_set_dt(&shb_haptic_spec, 0);
}

bool shb_board_haptic_alert_is_active(void)
{
	return atomic_get(&shb_haptic_alert_active) != 0;
}

int shb_board_haptic_feedback_pulse(k_timeout_t duration)
{
	if (!shb_haptic_ready) {
		return -ENODEV;
	}

	/*
	 * Don't disturb an ongoing alert. A feedback pulse is a short UX
	 * confirmation; letting it preempt the alert would cancel the motor
	 * early and silently clear the alert state via the shared off-work.
	 * If an alert is already running, skip the confirmation.
	 */
	if (atomic_get(&shb_haptic_alert_active) != 0) {
		return -EBUSY;
	}

	return shb_haptic_start_locked(duration);
}
