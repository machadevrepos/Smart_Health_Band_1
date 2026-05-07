#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>
#include "bmi270.h"
#include "power.h"

LOG_MODULE_REGISTER(shb_bmi270, CONFIG_SHB_LOG_LEVEL);

#define SHB_GRAVITY_UMS2_PER_G 9806650LL

static const struct device *const shb_bmi270_dev = DEVICE_DT_GET(DT_NODELABEL(bmi270));
static struct sensor_trigger shb_motion_trigger = {
	.type = SENSOR_TRIG_MOTION,
	.chan = SENSOR_CHAN_ACCEL_XYZ,
};
static shb_bmi270_wake_cb_t shb_motion_wake_cb;
static bool shb_bmi270_ready;
static bool shb_bmi270_motion_seen;
static struct k_work_delayable shb_bmi270_watchdog_work;
static struct shb_bmi270_motion_state shb_motion_state;
static struct sensor_value shb_prev_accel[3];
static bool shb_prev_accel_valid;
static int64_t shb_motion_started_ms;
static int64_t shb_last_motion_ms;
static bool shb_no_motion_reported;

static atomic_t            shb_gesture_events_atom;
static struct k_spinlock   shb_tap_lock;
static bool                shb_tap_pending;
static int64_t             shb_first_tap_ms;
static int64_t             shb_last_event_ms;   /* refractory reference */
static struct k_work_delayable shb_tap_confirm_work;

static int64_t shb_sensor_value_to_micro(const struct sensor_value *value)
{
	return ((int64_t)value->val1 * 1000000LL) + value->val2;
}

static uint32_t shb_abs_u32_from_i64(int64_t value)
{
	return (uint32_t)((value < 0) ? -value : value);
}

static uint32_t shb_accel_micro_to_mg(uint32_t accel_micro)
{
	return (uint32_t)(((uint64_t)accel_micro * 1000ULL) / SHB_GRAVITY_UMS2_PER_G);
}

static uint32_t shb_compute_accel_delta_micro(const struct sensor_value accel_xyz[3])
{
	uint32_t peak_delta = 0U;

	if (!shb_prev_accel_valid) {
		return 0U;
	}

	for (size_t axis = 0; axis < 3U; ++axis) {
		int64_t delta = shb_sensor_value_to_micro(&accel_xyz[axis]) -
				shb_sensor_value_to_micro(&shb_prev_accel[axis]);
		uint32_t abs_delta = shb_abs_u32_from_i64(delta);

		if (abs_delta > peak_delta) {
			peak_delta = abs_delta;
		}
	}

	return peak_delta;
}

static uint32_t shb_compute_gyro_peak_micro(const struct sensor_value gyro_xyz[3])
{
	uint32_t peak = 0U;

	for (size_t axis = 0; axis < 3U; ++axis) {
		uint32_t abs_axis = shb_abs_u32_from_i64(shb_sensor_value_to_micro(&gyro_xyz[axis]));
		if (abs_axis > peak) {
			peak = abs_axis;
		}
	}

	return peak;
}

static void shb_tap_confirm_handler(struct k_work *work)
{
	ARG_UNUSED(work);
	k_spinlock_key_t key = k_spin_lock(&shb_tap_lock);
	bool pending = shb_tap_pending;
	shb_tap_pending = false;
	if (pending) {
		shb_last_event_ms = k_uptime_get();
	}
	k_spin_unlock(&shb_tap_lock, key);

	if (pending) {
		atomic_or(&shb_gesture_events_atom, SHB_BMI270_EVENT_TAP);
		shb_power_signal(SHB_POWER_EVENT_BMI_GESTURE);
		LOG_DBG("Single tap confirmed");
	}
}

static void shb_bmi270_watchdog_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!shb_bmi270_motion_seen) {
		LOG_WRN("BMI270 interrupt not observed within %d s; check INT1 (P0.31)",
			CONFIG_SHB_BMI_INTERRUPT_WATCHDOG_S);
	}
}

/*
 * Integer square root (Newton's method). Used to compute |a| from
 * squared accel magnitude without pulling in libm.
 */
static uint64_t shb_isqrt_u64(uint64_t n)
{
	uint64_t x;
	uint64_t y;

	if (n == 0U) {
		return 0U;
	}
	x = n;
	y = (x + 1U) / 2U;
	while (y < x) {
		x = y;
		y = (x + n / x) / 2U;
	}
	return x;
}

/*
 * Return |a|-1g in millig for the current sample, where
 * |a| = sqrt(ax^2 + ay^2 + az^2). Positive when accel exceeds gravity,
 * negative when below. Returns 0 on I2C / fetch error (caller treats
 * a zero excursion as not-a-tap).
 */
static int32_t shb_read_excursion_mg(void)
{
	struct sensor_value accel[3];
	int64_t ax_um, ay_um, az_um;
	int64_t ax_mg, ay_mg, az_mg;
	uint64_t sq_mg2;
	uint64_t mag_mg;

	if (sensor_sample_fetch(shb_bmi270_dev) < 0) {
		return 0;
	}
	if (sensor_channel_get(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, accel) < 0) {
		return 0;
	}

	ax_um = shb_sensor_value_to_micro(&accel[0]);
	ay_um = shb_sensor_value_to_micro(&accel[1]);
	az_um = shb_sensor_value_to_micro(&accel[2]);

	ax_mg = (ax_um * 1000LL) / SHB_GRAVITY_UMS2_PER_G;
	ay_mg = (ay_um * 1000LL) / SHB_GRAVITY_UMS2_PER_G;
	az_mg = (az_um * 1000LL) / SHB_GRAVITY_UMS2_PER_G;

	sq_mg2 = (uint64_t)(ax_mg * ax_mg) +
		 (uint64_t)(ay_mg * ay_mg) +
		 (uint64_t)(az_mg * az_mg);
	mag_mg = shb_isqrt_u64(sq_mg2);

	return (int32_t)((int64_t)mag_mg - 1000LL);
}

/*
 * Impulsive-tap classifier. A real finger tap produces a short spike
 * where total acceleration briefly leaves 1 g and returns within
 * ~25 ms. Sustained events — wrist rotation, finger pressure on the
 * PPG sensor, walking — keep |a| near 1 g on some axis orientation,
 * so either the excursion is small OR it does not decay.
 *
 * Two-sample test:
 *   1. |a|-1g at IRQ time must exceed IMPULSE_MG.
 *   2. Sleep 25 ms, read again.
 *   3. (|excursion_1| - |excursion_2|) must exceed DECAY_MG.
 *
 * Runs in the BMI270 trigger thread where blocking I2C + k_msleep
 * are safe (CONFIG_BMI270_TRIGGER_OWN_THREAD=y).
 */
static bool shb_motion_is_tap_like(void)
{
	int32_t exc1 = shb_read_excursion_mg();
	int32_t abs1 = (exc1 < 0) ? -exc1 : exc1;
	int32_t exc2;
	int32_t abs2;

	if (abs1 < CONFIG_SHB_BMI_TAP_IMPULSE_MG) {
		return false;
	}

	k_msleep(25);

	exc2 = shb_read_excursion_mg();
	abs2 = (exc2 < 0) ? -exc2 : exc2;

	return (abs1 - abs2) >= CONFIG_SHB_BMI_TAP_DECAY_MG;
}

static void shb_bmi270_motion_callback(const struct device *dev,
				       const struct sensor_trigger *trigger)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(trigger);
	int64_t now;
	k_spinlock_key_t key;

	shb_bmi270_motion_seen = true;
	if (CONFIG_SHB_BMI_INTERRUPT_WATCHDOG_S > 0) {
		(void)k_work_cancel_delayable(&shb_bmi270_watchdog_work);
	}

	if (shb_motion_wake_cb != NULL) {
		shb_motion_wake_cb();
	}

	/*
	 * Reject non-impulsive motion (waves, wrist raises, walking). Only
	 * impulsive events above the peak threshold qualify as tap candidates.
	 */
	if (!shb_motion_is_tap_like()) {
		return;
	}

	/*
	 * Tap / double-tap state machine.
	 *   REFRACTORY_MS     — blanking window after any confirmed emission,
	 *                       absorbs post-impact ringing.
	 *   MIN_GAP_MS        — minimum spacing between two taps of a double.
	 *   DOUBLE_TAP_GAP_MS — maximum spacing; also the confirm timer for
	 *                       turning a pending first tap into a SINGLE_TAP.
	 */
	now = k_uptime_get();
	key = k_spin_lock(&shb_tap_lock);

	if ((shb_last_event_ms != 0) &&
	    ((now - shb_last_event_ms) < CONFIG_SHB_BMI_TAP_REFRACTORY_MS)) {
		k_spin_unlock(&shb_tap_lock, key);
		return;
	}

	if (shb_tap_pending) {
		int64_t gap = now - shb_first_tap_ms;

		if (gap < CONFIG_SHB_BMI_DOUBLE_TAP_MIN_GAP_MS) {
			k_spin_unlock(&shb_tap_lock, key);
			return;
		}

		if (gap <= CONFIG_SHB_BMI_DOUBLE_TAP_GAP_MS) {
			shb_tap_pending   = false;
			shb_last_event_ms = now;
			k_spin_unlock(&shb_tap_lock, key);
			(void)k_work_cancel_delayable(&shb_tap_confirm_work);
			atomic_or(&shb_gesture_events_atom, SHB_BMI270_EVENT_DOUBLE_TAP);
			shb_power_signal(SHB_POWER_EVENT_BMI_GESTURE);
			LOG_DBG("Double-tap (gap=%lld ms)", gap);
			return;
		}

		/* Window expired — start fresh */
		shb_tap_pending  = true;
		shb_first_tap_ms = now;
		k_spin_unlock(&shb_tap_lock, key);
		(void)k_work_reschedule(&shb_tap_confirm_work,
					K_MSEC(CONFIG_SHB_BMI_DOUBLE_TAP_GAP_MS));
		return;
	}

	shb_tap_pending  = true;
	shb_first_tap_ms = now;
	k_spin_unlock(&shb_tap_lock, key);
	(void)k_work_reschedule(&shb_tap_confirm_work,
				K_MSEC(CONFIG_SHB_BMI_DOUBLE_TAP_GAP_MS));
}

static int shb_bmi270_configure_accel_path(void)
{
	struct sensor_value full_scale = { .val1 = 2, .val2 = 0 };
	struct sensor_value sampling_freq = { .val1 = 50, .val2 = 0 };
	struct sensor_value oversampling = { .val1 = 4, .val2 = 0 };
	int ret;

	ret = sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_FULL_SCALE, &full_scale);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_OVERSAMPLING, &oversampling);
	if (ret < 0) {
		return ret;
	}

	return sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
			       SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
}

static int shb_bmi270_configure_gyro_path(void)
{
	struct sensor_value full_scale = { .val1 = 500, .val2 = 0 };
	struct sensor_value sampling_freq = { .val1 = 100, .val2 = 0 };
	struct sensor_value oversampling = { .val1 = 1, .val2 = 0 };
	int ret;

	ret = sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_FULL_SCALE, &full_scale);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_GYRO_XYZ,
			      SENSOR_ATTR_OVERSAMPLING, &oversampling);
	if (ret < 0) {
		return ret;
	}

	return sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_GYRO_XYZ,
			       SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);
}

int shb_bmi270_init(shb_bmi270_wake_cb_t wake_cb)
{
	int ret;

	if (!device_is_ready(shb_bmi270_dev)) {
		LOG_ERR("BMI270 device is not ready on I2C address 0x68");
		return -ENODEV;
	}

	shb_motion_wake_cb = wake_cb;
	ret = shb_bmi270_configure_accel_path();
	if (ret < 0) {
		LOG_ERR("BMI270 accel path configuration failed: %d", ret);
		return ret;
	}

	ret = shb_bmi270_configure_gyro_path();
	if (ret < 0) {
		LOG_ERR("BMI270 gyro path configuration failed: %d", ret);
		return ret;
	}

	shb_bmi270_ready = true;
	shb_bmi270_motion_seen = false;
	shb_prev_accel_valid = false;
	memset(&shb_motion_state, 0, sizeof(shb_motion_state));
	shb_motion_started_ms = 0;
	shb_last_motion_ms = 0;
	shb_no_motion_reported = false;
	k_work_init_delayable(&shb_bmi270_watchdog_work, shb_bmi270_watchdog_handler);
	atomic_clear(&shb_gesture_events_atom);
	k_work_init_delayable(&shb_tap_confirm_work, shb_tap_confirm_handler);
	shb_tap_pending  = false;
	shb_first_tap_ms = 0;
	shb_last_event_ms = 0;
	LOG_INF("BMI270 ready");

	return 0;
}

int shb_bmi270_configure_any_motion(void)
{
	struct sensor_value duration = {
		.val1 = CONFIG_SHB_BMI_ANYMOTION_DURATION_MS,
		.val2 = 0,
	};
	struct sensor_value threshold = {
		.val1 = 0,
		.val2 = CONFIG_SHB_BMI_ANYMOTION_THRESHOLD_MG * 1000,
	};
	int ret;

	if (!shb_bmi270_ready) {
		return -ENODEV;
	}

	ret = sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SLOPE_DUR, &duration);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_attr_set(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ,
			      SENSOR_ATTR_SLOPE_TH, &threshold);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_trigger_set(shb_bmi270_dev, &shb_motion_trigger,
				 shb_bmi270_motion_callback);
	if (ret < 0) {
		return ret;
	}

	if (CONFIG_SHB_BMI_INTERRUPT_WATCHDOG_S > 0) {
		(void)k_work_schedule(&shb_bmi270_watchdog_work,
				      K_SECONDS(CONFIG_SHB_BMI_INTERRUPT_WATCHDOG_S));
	}

	LOG_INF("BMI270 INT1 armed (%d mg / %d ms)",
		CONFIG_SHB_BMI_ANYMOTION_THRESHOLD_MG,
		CONFIG_SHB_BMI_ANYMOTION_DURATION_MS);
	return 0;
}

int shb_bmi270_read_accel(struct sensor_value accel_xyz[3])
{
	int ret;

	if (!shb_bmi270_ready) {
		return -ENODEV;
	}

	ret = sensor_sample_fetch(shb_bmi270_dev);
	if (ret < 0) {
		return ret;
	}

	return sensor_channel_get(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, accel_xyz);
}

int shb_bmi270_read_gyro(struct sensor_value gyro_xyz[3])
{
	int ret;

	if (!shb_bmi270_ready) {
		return -ENODEV;
	}

	ret = sensor_sample_fetch(shb_bmi270_dev);
	if (ret < 0) {
		return ret;
	}

	return sensor_channel_get(shb_bmi270_dev, SENSOR_CHAN_GYRO_XYZ, gyro_xyz);
}

int shb_bmi270_read_sample(struct shb_bmi270_sample *sample)
{
	int ret;

	if ((sample == NULL) || !shb_bmi270_ready) {
		return -EINVAL;
	}

	ret = sensor_sample_fetch(shb_bmi270_dev);
	if (ret < 0) {
		return ret;
	}

	ret = sensor_channel_get(shb_bmi270_dev, SENSOR_CHAN_ACCEL_XYZ, sample->accel_xyz);
	if (ret < 0) {
		return ret;
	}

	return sensor_channel_get(shb_bmi270_dev, SENSOR_CHAN_GYRO_XYZ, sample->gyro_xyz);
}

uint32_t shb_bmi270_process_motion(const struct shb_bmi270_sample *sample,
				   struct shb_bmi270_motion_state *state_out)
{
	uint32_t events = SHB_BMI270_EVENT_NONE;
	uint32_t accel_delta_micro;
	uint32_t accel_threshold_micro;
	uint32_t gyro_peak_micro;
	uint32_t gyro_threshold_micro;
	bool moving_now;
	int64_t now_ms;

	if (sample == NULL) {
		return SHB_BMI270_EVENT_NONE;
	}

	accel_delta_micro = shb_compute_accel_delta_micro(sample->accel_xyz);
	accel_threshold_micro = (uint32_t)(((uint64_t)CONFIG_SHB_BMI_MOTION_ACCEL_THRESHOLD_MG *
					      SHB_GRAVITY_UMS2_PER_G) / 1000ULL);
	gyro_peak_micro = shb_compute_gyro_peak_micro(sample->gyro_xyz);
	gyro_threshold_micro = (uint32_t)CONFIG_SHB_BMI_MOTION_GYRO_THRESHOLD_MDPS * 1000U;
	moving_now = (accel_delta_micro >= accel_threshold_micro) ||
		     (gyro_peak_micro >= gyro_threshold_micro);
	now_ms = k_uptime_get();

	shb_motion_state.events = SHB_BMI270_EVENT_NONE;
	shb_motion_state.accel_delta_mg = shb_accel_micro_to_mg(accel_delta_micro);
	shb_motion_state.gyro_peak_mdps = gyro_peak_micro / 1000U;
	shb_motion_state.moving = moving_now;

	if (moving_now) {
		if (shb_motion_started_ms == 0) {
			shb_motion_started_ms = now_ms;
		}

		if (!shb_motion_state.significant_active &&
		    ((now_ms - shb_motion_started_ms) >= CONFIG_SHB_BMI_SIG_MOTION_DURATION_MS)) {
			events |= SHB_BMI270_EVENT_SIGNIFICANT_MOTION;
			shb_motion_state.significant_active = true;
		}

		if ((shb_last_motion_ms == 0) || !shb_prev_accel_valid || shb_no_motion_reported) {
			events |= SHB_BMI270_EVENT_MOTION;
		}

		shb_last_motion_ms = now_ms;
		shb_no_motion_reported = false;
	} else {
		if ((shb_last_motion_ms != 0) && !shb_no_motion_reported &&
		    ((now_ms - shb_last_motion_ms) >= CONFIG_SHB_BMI_NO_MOTION_DURATION_MS)) {
			events |= SHB_BMI270_EVENT_NO_MOTION;
			shb_no_motion_reported = true;
			shb_motion_state.significant_active = false;
			shb_motion_started_ms = 0;
		}
	}

	shb_motion_state.events = events;
	memcpy(shb_prev_accel, sample->accel_xyz, sizeof(shb_prev_accel));
	shb_prev_accel_valid = true;

	if (state_out != NULL) {
		*state_out = shb_motion_state;
	}

	return events;
}

void shb_bmi270_get_motion_state(struct shb_bmi270_motion_state *state_out)
{
	if (state_out != NULL) {
		*state_out = shb_motion_state;
	}
}

uint32_t shb_bmi270_consume_gesture_events(void)
{
	return (uint32_t)atomic_clear(&shb_gesture_events_atom);
}

bool shb_bmi270_is_ready(void)
{
	return shb_bmi270_ready;
}
