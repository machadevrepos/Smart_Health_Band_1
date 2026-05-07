#include <stdio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ble.h"
#include "board.h"
#include "bmi270.h"
#include "max32664.h"
#include "power.h"
#include "tmp117.h"

LOG_MODULE_REGISTER(shb_main, CONFIG_SHB_LOG_LEVEL);

static int32_t shb_last_temp_mc;
static bool shb_have_temp;
static uint32_t shb_last_gesture_events;
static int64_t shb_last_gesture_ms;

static const char *shb_motion_label(const struct shb_bmi270_motion_state *motion)
{
	if (motion->significant_active) {
		return "SIG. MOTION";
	}

	return motion->moving ? "MOTION" : "IDLE";
}

static const char *shb_gesture_label(void)
{
	if ((shb_last_gesture_events == 0U) ||
	    ((k_uptime_get() - shb_last_gesture_ms) > 2000)) {
		return "NONE";
	}

	if ((shb_last_gesture_events & SHB_BMI270_EVENT_DOUBLE_TAP) != 0U) {
		return "TAP_DOUBLE";
	}

	if ((shb_last_gesture_events & SHB_BMI270_EVENT_TAP) != 0U) {
		return "TAP_SINGLE";
	}

	return "NONE";
}

static void shb_log_snapshot(const char *reason)
{
	struct shb_bmi270_motion_state motion = { 0 };
	struct shb_max32664_result latest = { 0 };
	char temp_buf[16] = "--";
	char hr_buf[16] = "--";
	char spo2_buf[16] = "--";
	char bp_buf[16] = "--";

	if (shb_have_temp) {
		int32_t temp_mc = shb_last_temp_mc;
		int32_t abs_mc = (temp_mc < 0) ? -temp_mc : temp_mc;

		(void)snprintf(temp_buf, sizeof(temp_buf), "%s%d.%02d C",
			       (temp_mc < 0) ? "-" : "",
			       abs_mc / 1000, (abs_mc % 1000) / 10);
	}

	if (shb_bmi270_is_ready()) {
		shb_bmi270_get_motion_state(&motion);
	}

	shb_max32664_get_latest(&latest);
	if (latest.valid) {
		(void)snprintf(hr_buf, sizeof(hr_buf), "%u.%u bpm",
			       (unsigned int)(latest.hr_x10 / 10U),
			       (unsigned int)(latest.hr_x10 % 10U));
		(void)snprintf(spo2_buf, sizeof(spo2_buf), "%u.%u%%",
			       (unsigned int)(latest.spo2_x10 / 10U),
			       (unsigned int)(latest.spo2_x10 % 10U));
		(void)snprintf(bp_buf, sizeof(bp_buf), "%u/%u mmHg",
			       (unsigned int)latest.sys_bp,
			       (unsigned int)latest.dia_bp);
	}

	LOG_INF("Status (%s): Temp=%s | Motion=%s | Gesture=%s | HR=%s | SpO2=%s | BP=%s",
		reason, temp_buf, shb_motion_label(&motion), shb_gesture_label(),
		hr_buf, spo2_buf, bp_buf);
}

static void shb_update_temperature(void)
{
	struct sensor_value temp;
	int ret = shb_tmp117_read_temperature(&temp);

	if (ret < 0) {
		LOG_ERR("TMP117 read failed: %d", ret);
		return;
	}

	(void)shb_board_led_pulse(SHB_LED_TEMP, K_MSEC(50));
	int32_t temp_mc = (int32_t)(temp.val1 * 1000 + temp.val2 / 1000);

	shb_last_temp_mc = temp_mc;
	shb_have_temp = true;
	shb_ble_notify_temperature(temp_mc);
}

static void shb_bmi270_irq_wake_cb(void)
{
	shb_power_signal(SHB_POWER_EVENT_BMI_WAKE);
}

static bool shb_poll_bmi270(void)
{
	/*
	 * Per product spec (Samuel, PoC): no motion/wrist-raise/significant-motion
	 * BLE events — taps are the only gesture. We still run the accel through
	 * shb_bmi270_process_motion() to refresh the internal idle/moving state
	 * used by the snapshot logger, but do not publish those events anywhere.
	 */
	struct shb_bmi270_sample sample;
	struct shb_bmi270_motion_state motion;
	int ret = shb_bmi270_read_sample(&sample);

	if (ret < 0) {
		LOG_ERR("BMI270 read failed: %d", ret);
		return false;
	}

	(void)shb_bmi270_process_motion(&sample, &motion);
	return false;
}

static void shb_handle_max32664_vitals_event(void)
{
	struct shb_max32664_result latest = { 0 };

	shb_max32664_get_latest(&latest);
	if (latest.valid) {
		(void)shb_board_led_pulse(SHB_LED_BMI, K_MSEC(60));
		shb_ble_notify_vitals(latest.hr_x10, latest.spo2_x10,
				      latest.sys_bp, latest.dia_bp);
		shb_log_snapshot("vitals");
	}
}

static void shb_handle_power_events(k_timeout_t timeout)
{
	uint32_t events = shb_power_wait_for_wake(timeout);

	if ((events & SHB_POWER_EVENT_BMI_GESTURE) != 0U) {
		uint32_t gestures = shb_bmi270_consume_gesture_events();

		if (gestures != 0U) {
			/*
			 * Product spec (Samuel):
			 *   Single tap + alert active -> stop vibration,
			 *                                send ALERT_ACKNOWLEDGED
			 *   Single tap + idle         -> send TAP_SINGLE (no haptic)
			 *   Double tap                -> medium feedback haptic,
			 *                                send TAP_DOUBLE
			 * Double-tap is checked first because its bit coexists with
			 * any pending TAP bit that survived in the atomic.
			 */
			const char *event_name = NULL;

			if ((gestures & SHB_BMI270_EVENT_DOUBLE_TAP) != 0U) {
				LOG_INF("Gesture: TAP_DOUBLE");
				(void)shb_board_haptic_feedback_pulse(K_MSEC(80));
				event_name = "TAP_DOUBLE";
			} else if ((gestures & SHB_BMI270_EVENT_TAP) != 0U) {
				if (shb_board_haptic_alert_is_active()) {
					LOG_INF("Gesture: TAP_SINGLE during alert -> ALERT_ACKNOWLEDGED");
					(void)shb_board_haptic_alert_stop();
					event_name = "ALERT_ACKNOWLEDGED";
				} else {
					LOG_INF("Gesture: TAP_SINGLE");
					event_name = "TAP_SINGLE";
				}
			}

			shb_last_gesture_events = gestures;
			shb_last_gesture_ms = k_uptime_get();
			if (event_name != NULL) {
				shb_ble_notify_event(event_name);
			}
			shb_log_snapshot("gesture");
		}
	}

	if ((events & SHB_POWER_EVENT_MAX32664_VITALS) != 0U) {
		shb_handle_max32664_vitals_event();
	}
}

int main(void)
{
	int ret;
	bool max_session_enabled = false;
	const int64_t poll_interval_ms = CONFIG_SHB_SENSOR_POLL_INTERVAL_MS;
	int64_t next_poll_ms;

	ret = shb_board_init();
	if (ret < 0) {
		LOG_ERR("Board init failed: %d", ret);
		return ret;
	}

	ret = shb_power_init();
	if (ret < 0) {
		LOG_ERR("Power init failed: %d", ret);
		return ret;
	}

	ret = shb_ble_init();
	if (ret < 0) {
		LOG_ERR("BLE init failed: %d", ret);
	}

	ret = shb_tmp117_init();
	if (ret < 0) {
		LOG_ERR("TMP117 init failed: %d", ret);
	} else {
		shb_update_temperature();
	}

	ret = shb_bmi270_init(shb_bmi270_irq_wake_cb);
	if (ret < 0) {
		LOG_ERR("BMI270 init failed: %d", ret);
	} else {
		if (IS_ENABLED(CONFIG_SHB_BMI_ANYMOTION_WAKE)) {
			ret = shb_bmi270_configure_any_motion();
			if (ret < 0) {
				LOG_ERR("BMI270 interrupt setup failed: %d", ret);
			}
		}
	}

	ret = shb_max32664_init();
	if (ret < 0) {
		LOG_ERR("MAX32664 init failed: %d", ret);
	} else {
		ret = shb_max32664_start_session();
		if (ret < 0) {
			LOG_ERR("MAX32664 session start failed: %d", ret);
		} else {
			max_session_enabled = true;
		}
	}

	next_poll_ms = k_uptime_get() + poll_interval_ms;
	shb_log_snapshot("startup");

	while (1) {
		int64_t now = k_uptime_get();
		int64_t wait_ms = (next_poll_ms > now) ? (next_poll_ms - now) : 0LL;

		if (max_session_enabled || IS_ENABLED(CONFIG_SHB_BMI_ANYMOTION_WAKE)) {
			shb_handle_power_events(K_MSEC(wait_ms));
		} else {
			k_sleep(K_MSEC(wait_ms));
		}

		now = k_uptime_get();
		if (now >= next_poll_ms) {
			bool motion_snapshot_logged = false;

			if (shb_tmp117_is_ready()) {
				shb_update_temperature();
			}

			if (shb_bmi270_is_ready()) {
				motion_snapshot_logged = shb_poll_bmi270();
			}

			if (!motion_snapshot_logged) {
				shb_log_snapshot("poll");
			}
			next_poll_ms = now + poll_interval_ms;
		}
	}
}
