#ifndef SHB_BMI270_H_
#define SHB_BMI270_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util.h>

typedef void (*shb_bmi270_wake_cb_t)(void);

enum shb_bmi270_motion_event {
	SHB_BMI270_EVENT_NONE               = 0,
	SHB_BMI270_EVENT_MOTION             = BIT(0),
	SHB_BMI270_EVENT_NO_MOTION          = BIT(1),
	SHB_BMI270_EVENT_SIGNIFICANT_MOTION = BIT(2),
	SHB_BMI270_EVENT_TAP                = BIT(3),
	SHB_BMI270_EVENT_DOUBLE_TAP         = BIT(4),
};

struct shb_bmi270_sample {
	struct sensor_value accel_xyz[3];
	struct sensor_value gyro_xyz[3];
};

struct shb_bmi270_motion_state {
	bool moving;
	bool significant_active;
	uint32_t events;
	uint32_t accel_delta_mg;
	uint32_t gyro_peak_mdps;
};

int shb_bmi270_init(shb_bmi270_wake_cb_t wake_cb);
int shb_bmi270_configure_any_motion(void);
int shb_bmi270_read_accel(struct sensor_value accel_xyz[3]);
int shb_bmi270_read_gyro(struct sensor_value gyro_xyz[3]);
int shb_bmi270_read_sample(struct shb_bmi270_sample *sample);
uint32_t shb_bmi270_process_motion(const struct shb_bmi270_sample *sample,
				   struct shb_bmi270_motion_state *state_out);
void shb_bmi270_get_motion_state(struct shb_bmi270_motion_state *state_out);
uint32_t shb_bmi270_consume_gesture_events(void);
bool shb_bmi270_is_ready(void);

#endif
