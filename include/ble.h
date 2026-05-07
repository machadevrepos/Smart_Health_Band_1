#ifndef SHB_BLE_H_
#define SHB_BLE_H_

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/toolchain.h>
#include "shb/bmi270.h"
#include "shb/max32664.h"

struct shb_ble_vitals_payload {
	uint8_t valid;
	uint8_t status;
	uint8_t progress;
	uint8_t sys_bp;
	uint8_t dia_bp;
	uint16_t hr_x10;
	uint16_t spo2_x10;
} __packed;

struct shb_ble_temp_payload {
	int32_t temp_mdeg_c;
} __packed;

struct shb_ble_motion_payload {
	uint8_t moving;
	uint8_t significant_active;
	uint16_t reserved;
	uint32_t events;
	uint32_t accel_delta_mg;
	uint32_t gyro_peak_mdps;
} __packed;

int shb_ble_init(void);
bool shb_ble_is_connected(void);
int shb_ble_notify_vitals(const struct shb_max32664_result *result);
int shb_ble_notify_temperature(const struct sensor_value *temperature);
int shb_ble_notify_motion(const struct shb_bmi270_motion_state *motion);

#endif
