#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "shb/board.h"

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

static void shb_led_off_work(struct k_work *work)
{
	struct k_work_delayable *delayable = k_work_delayable_from_work(work);

	for (size_t i = 0; i < SHB_LED_COUNT; ++i) {
		if ((&shb_leds[i].off_work == delayable) && shb_leds[i].ready) {
			(void)gpio_pin_set_dt(&shb_leds[i].spec, 0);
		}
	}
}

int shb_board_init(void)
{
	for (size_t i = 0; i < SHB_LED_COUNT; ++i) {
		k_work_init_delayable(&shb_leds[i].off_work, shb_led_off_work);
		if (!gpio_is_ready_dt(&shb_leds[i].spec)) {
			continue;
		}

		if (gpio_pin_configure_dt(&shb_leds[i].spec, GPIO_OUTPUT_INACTIVE) == 0) {
			shb_leds[i].ready = true;
		}
	}

	return 0;
}

void shb_board_log_pin_summary(void)
{
	LOG_INF("Board target: Smart Health Band BMD-350/nRF52832 custom PCB overlay");
	LOG_INF("Main I2C bus: SCL=P0.00 SDA=P0.04");
	LOG_INF("TMP117 pins: ALERT=P0.11 LED=P0.08");
	LOG_INF("BMI270 pins: INT1=P0.31 INT2=P0.30 LED=P0.13");
	LOG_INF("MAX32664D pins: MFIO=P0.07 only; RSTN is NOT driven by firmware");
	LOG_INF("Biometric LED: P0.12");
}

int shb_board_led_set(enum shb_led_id led, bool on)
{
	if ((led >= SHB_LED_COUNT) || !shb_leds[led].ready) {
		return 0;
	}

	return gpio_pin_set_dt(&shb_leds[led].spec, on ? 1 : 0);
}

int shb_board_led_pulse(enum shb_led_id led, k_timeout_t duration)
{
	int ret;

	if ((led >= SHB_LED_COUNT) || !shb_leds[led].ready) {
		return 0;
	}

	ret = gpio_pin_set_dt(&shb_leds[led].spec, 1);
	if (ret < 0) {
		return ret;
	}

	return k_work_schedule(&shb_leds[led].off_work, duration);
}
