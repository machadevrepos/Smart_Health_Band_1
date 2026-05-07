#include <errno.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "shb/ble.h"
#include "shb/board.h"
#include "shb/bmi270.h"
#include "shb/max32664.h"
#include "shb/power.h"
#include "shb/tmp117.h"

LOG_MODULE_REGISTER(shb_main, CONFIG_SHB_LOG_LEVEL);

struct shb_sensor_fmt {
	const char *sign;
	uint32_t whole;
	uint32_t frac;
};

static struct shb_sensor_fmt shb_sensor_fmt_4(const struct sensor_value *value)
{
	int64_t micro = ((int64_t)value->val1 * 1000000LL) + value->val2;
	uint64_t abs_micro = (micro < 0) ? (uint64_t)(-micro) : (uint64_t)micro;
	struct shb_sensor_fmt formatted = {
		.sign = (micro < 0) ? "-" : "",
		.whole = (uint32_t)(abs_micro / 1000000ULL),
		.frac = (uint32_t)((abs_micro % 1000000ULL) / 100ULL),
	};

	return formatted;
}

static void shb_log_temperature(void)
{
	struct sensor_value temp;
	struct shb_sensor_fmt formatted;
	int ret = shb_tmp117_read_temperature(&temp);

	if (ret < 0) {
		LOG_ERR("TMP117 read failed: %d", ret);
		return;
	}

	formatted = shb_sensor_fmt_4(&temp);
	(void)shb_board_led_pulse(SHB_LED_TEMP, K_MSEC(50));
	(void)shb_ble_notify_temperature(&temp);
	LOG_INF("TMP117: %s%u.%04u C",
		formatted.sign,
		(unsigned int)formatted.whole,
		(unsigned int)formatted.frac);
}

static void shb_bmi270_irq_wake_cb(void)
{
	shb_power_signal(SHB_POWER_EVENT_BMI_MOTION);
}

static void shb_log_bmi270(void)
{
	struct shb_bmi270_sample sample;
	struct shb_bmi270_motion_state motion;
	struct shb_sensor_fmt accel_x;
	struct shb_sensor_fmt accel_y;
	struct shb_sensor_fmt accel_z;
	struct shb_sensor_fmt gyro_x;
	struct shb_sensor_fmt gyro_y;
	struct shb_sensor_fmt gyro_z;
	int ret = shb_bmi270_read_sample(&sample);

	if (ret < 0) {
		LOG_ERR("BMI270 sample read failed: %d", ret);
		return;
	}

	(void)shb_bmi270_process_motion(&sample, &motion);
	(void)shb_ble_notify_motion(&motion);

	accel_x = shb_sensor_fmt_4(&sample.accel_xyz[0]);
	accel_y = shb_sensor_fmt_4(&sample.accel_xyz[1]);
	accel_z = shb_sensor_fmt_4(&sample.accel_xyz[2]);
	gyro_x = shb_sensor_fmt_4(&sample.gyro_xyz[0]);
	gyro_y = shb_sensor_fmt_4(&sample.gyro_xyz[1]);
	gyro_z = shb_sensor_fmt_4(&sample.gyro_xyz[2]);

	LOG_INF("BMI270 accel: X=%s%u.%04u Y=%s%u.%04u Z=%s%u.%04u m/s^2 | gyro: X=%s%u.%04u Y=%s%u.%04u Z=%s%u.%04u dps | state=%s accel-delta=%u mg gyro-peak=%u mdps",
		accel_x.sign, (unsigned int)accel_x.whole, (unsigned int)accel_x.frac,
		accel_y.sign, (unsigned int)accel_y.whole, (unsigned int)accel_y.frac,
		accel_z.sign, (unsigned int)accel_z.whole, (unsigned int)accel_z.frac,
		gyro_x.sign, (unsigned int)gyro_x.whole, (unsigned int)gyro_x.frac,
		gyro_y.sign, (unsigned int)gyro_y.whole, (unsigned int)gyro_y.frac,
		gyro_z.sign, (unsigned int)gyro_z.whole, (unsigned int)gyro_z.frac,
		motion.moving ? "moving" : "still",
		(unsigned int)motion.accel_delta_mg,
		(unsigned int)motion.gyro_peak_mdps);

	if ((motion.events & SHB_BMI270_EVENT_MOTION) != 0U) {
		(void)shb_board_led_pulse(SHB_LED_BMI, K_MSEC(50));
		LOG_INF("BMI270 motion detected");
	}

	if ((motion.events & SHB_BMI270_EVENT_SIGNIFICANT_MOTION) != 0U) {
		(void)shb_board_led_pulse(SHB_LED_BMI, K_MSEC(120));
		LOG_INF("BMI270 significant motion detected");
	}

	if ((motion.events & SHB_BMI270_EVENT_NO_MOTION) != 0U) {
		LOG_INF("BMI270 no-motion detected");
	}
}

static void shb_handle_max32664_vitals_event(void)
{
	struct shb_max32664_result latest = { 0 };

	shb_max32664_get_latest(&latest);
	(void)shb_ble_notify_vitals(&latest);
	if (latest.valid) {
		(void)shb_board_led_pulse(SHB_LED_HEART, K_MSEC(60));
	}
}

static void shb_handle_power_events(void)
{
	uint32_t events = shb_power_wait_for_wake(K_NO_WAIT);

	if ((events & SHB_POWER_EVENT_BMI_MOTION) != 0U) {
		LOG_INF("BMI270 INT1 any-motion interrupt observed");
		(void)shb_board_led_pulse(SHB_LED_BMI, K_MSEC(40));
	}

	if ((events & SHB_POWER_EVENT_MAX32664_VITALS) != 0U) {
		shb_handle_max32664_vitals_event();
	}
}

int main(void)
{
	int ret;
	bool max_session_enabled = false;

	LOG_INF("Smart Health Band critical-fix runtime boot");

	ret = shb_board_init();
	if (ret < 0) {
		LOG_ERR("Board initialization failed: %d", ret);
		return ret;
	}

	shb_board_log_pin_summary();

	ret = shb_power_init();
	if (ret < 0) {
		LOG_ERR("Power event initialization failed: %d", ret);
		return ret;
	}

	ret = shb_ble_init();
	if (ret < 0) {
		LOG_ERR("BLE initialization failed: %d", ret);
	} else {
		LOG_INF("BLE initialized");
	}

	ret = shb_tmp117_init();
	if (ret < 0) {
		LOG_ERR("TMP117 missing or misconfigured: %d", ret);
	} else {
		shb_log_temperature();
	}

	ret = shb_bmi270_init(shb_bmi270_irq_wake_cb);
	if (ret < 0) {
		LOG_ERR("BMI270 chip/config failure: %d", ret);
	} else {
		if (IS_ENABLED(CONFIG_SHB_BMI_ANYMOTION_WAKE)) {
			ret = shb_bmi270_configure_any_motion();
			if (ret < 0) {
				LOG_ERR("BMI270 interrupt setup failed: %d", ret);
			}
		}

		LOG_INF("Taking initial BMI270 sample");
		shb_log_bmi270();
	}

	LOG_INF("Starting MAX32664 initialization without firmware-driven RSTN");
	ret = shb_max32664_init();
	if (ret < 0) {
		LOG_ERR("MAX32664 init failed: %d", ret);
		LOG_ERR("If this fails consistently, check that MAX32664 RSTN has a hardware pull-up/shared reset and MFIO defaults high at power-up.");
	} else {
		ret = shb_max32664_start_session();
		if (ret < 0) {
			LOG_ERR("MAX32664 vitals session start failed: %d", ret);
		} else {
			max_session_enabled = true;
			LOG_INF("MAX32664 vitals session started");
		}
	}

	LOG_INF("Polling TMP117 and BMI270 every %d ms; BLE notifications active when connected",
		CONFIG_SHB_SENSOR_POLL_INTERVAL_MS);

	while (1) {
		if (shb_tmp117_is_ready()) {
			shb_log_temperature();
		}

		if (shb_bmi270_is_ready()) {
			shb_log_bmi270();
		}

		if (max_session_enabled || IS_ENABLED(CONFIG_SHB_BMI_ANYMOTION_WAKE)) {
			shb_handle_power_events();
		}

		shb_power_enter_idle();
		k_sleep(K_MSEC(CONFIG_SHB_SENSOR_POLL_INTERVAL_MS));
	}
}
