#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/byteorder.h>

#include "shb/ble.h"

LOG_MODULE_REGISTER(shb_ble, CONFIG_SHB_LOG_LEVEL);

/* 128-bit vendor service UUIDs: f364xxxx-8f22-4c9f-9a58-8d62c6f2b001 */
#define BT_UUID_SHB_SERVICE_VAL \
	BT_UUID_128_ENCODE(0xf3640000, 0x8f22, 0x4c9f, 0x9a58, 0x8d62c6f2b001)
#define BT_UUID_SHB_VITALS_VAL \
	BT_UUID_128_ENCODE(0xf3640001, 0x8f22, 0x4c9f, 0x9a58, 0x8d62c6f2b001)
#define BT_UUID_SHB_TEMP_VAL \
	BT_UUID_128_ENCODE(0xf3640002, 0x8f22, 0x4c9f, 0x9a58, 0x8d62c6f2b001)
#define BT_UUID_SHB_MOTION_VAL \
	BT_UUID_128_ENCODE(0xf3640003, 0x8f22, 0x4c9f, 0x9a58, 0x8d62c6f2b001)

static struct bt_uuid_128 shb_service_uuid = BT_UUID_INIT_128(BT_UUID_SHB_SERVICE_VAL);
static struct bt_uuid_128 shb_vitals_uuid = BT_UUID_INIT_128(BT_UUID_SHB_VITALS_VAL);
static struct bt_uuid_128 shb_temp_uuid = BT_UUID_INIT_128(BT_UUID_SHB_TEMP_VAL);
static struct bt_uuid_128 shb_motion_uuid = BT_UUID_INIT_128(BT_UUID_SHB_MOTION_VAL);

static struct bt_conn *shb_current_conn;
static bool shb_ble_ready;

static struct shb_ble_vitals_payload shb_latest_vitals;
static struct shb_ble_temp_payload shb_latest_temp;
static struct shb_ble_motion_payload shb_latest_motion;

static ssize_t read_vitals(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
	const struct shb_ble_vitals_payload *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	const struct shb_ble_temp_payload *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

static ssize_t read_motion(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				  void *buf, uint16_t len, uint16_t offset)
{
	const struct shb_ble_motion_payload *value = attr->user_data;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value, sizeof(*value));
}

BT_GATT_SERVICE_DEFINE(shb_svc,
	BT_GATT_PRIMARY_SERVICE(&shb_service_uuid),

	BT_GATT_CHARACTERISTIC(&shb_vitals_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_vitals, NULL, &shb_latest_vitals),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(&shb_temp_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_temp, NULL, &shb_latest_temp),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	BT_GATT_CHARACTERISTIC(&shb_motion_uuid.uuid,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		read_motion, NULL, &shb_latest_motion),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE)
);

#define SHB_ATTR_VITALS_VALUE 2
#define SHB_ATTR_TEMP_VALUE 5
#define SHB_ATTR_MOTION_VALUE 8

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err != 0U) {
		LOG_WRN("BLE connection failed: 0x%02x", err);
		return;
	}

	shb_current_conn = bt_conn_ref(conn);
	LOG_INF("BLE connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	ARG_UNUSED(conn);

	if (shb_current_conn != NULL) {
		bt_conn_unref(shb_current_conn);
		shb_current_conn = NULL;
	}

	LOG_INF("BLE disconnected: 0x%02x", reason);
}

BT_CONN_CB_DEFINE(shb_conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

int shb_ble_init(void)
{
	int ret;

	ret = bt_enable(NULL);
	if (ret != 0) {
		LOG_ERR("bt_enable failed: %d", ret);
		return ret;
	}

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	ret = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
	if (ret != 0) {
		LOG_ERR("BLE advertising start failed: %d", ret);
		return ret;
	}

	shb_ble_ready = true;
	LOG_INF("BLE advertising as %s", CONFIG_BT_DEVICE_NAME);
	return 0;
}

bool shb_ble_is_connected(void)
{
	return shb_current_conn != NULL;
}

int shb_ble_notify_vitals(const struct shb_max32664_result *result)
{
	if ((result == NULL) || !shb_ble_ready) {
		return -EINVAL;
	}

	shb_latest_vitals.valid = result->valid ? 1U : 0U;
	shb_latest_vitals.status = result->status;
	shb_latest_vitals.progress = result->progress;
	shb_latest_vitals.sys_bp = result->sys_bp;
	shb_latest_vitals.dia_bp = result->dia_bp;
	shb_latest_vitals.hr_x10 = sys_cpu_to_le16(result->hr_x10);
	shb_latest_vitals.spo2_x10 = sys_cpu_to_le16(result->spo2_x10);

	if (shb_current_conn == NULL) {
		return 0;
	}

	return bt_gatt_notify(NULL, &shb_svc.attrs[SHB_ATTR_VITALS_VALUE],
			      &shb_latest_vitals, sizeof(shb_latest_vitals));
}

int shb_ble_notify_temperature(const struct sensor_value *temperature)
{
	int64_t micro_c;

	if ((temperature == NULL) || !shb_ble_ready) {
		return -EINVAL;
	}

	micro_c = ((int64_t)temperature->val1 * 1000000LL) + temperature->val2;
	shb_latest_temp.temp_mdeg_c = (int32_t)(micro_c / 1000LL);

	if (shb_current_conn == NULL) {
		return 0;
	}

	return bt_gatt_notify(NULL, &shb_svc.attrs[SHB_ATTR_TEMP_VALUE],
			      &shb_latest_temp, sizeof(shb_latest_temp));
}

int shb_ble_notify_motion(const struct shb_bmi270_motion_state *motion)
{
	if ((motion == NULL) || !shb_ble_ready) {
		return -EINVAL;
	}

	shb_latest_motion.moving = motion->moving ? 1U : 0U;
	shb_latest_motion.significant_active = motion->significant_active ? 1U : 0U;
	shb_latest_motion.reserved = 0U;
	shb_latest_motion.events = sys_cpu_to_le32(motion->events);
	shb_latest_motion.accel_delta_mg = sys_cpu_to_le32(motion->accel_delta_mg);
	shb_latest_motion.gyro_peak_mdps = sys_cpu_to_le32(motion->gyro_peak_mdps);

	if (shb_current_conn == NULL) {
		return 0;
	}

	return bt_gatt_notify(NULL, &shb_svc.attrs[SHB_ATTR_MOTION_VALUE],
			      &shb_latest_motion, sizeof(shb_latest_motion));
}
