#include <stdio.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "ble.h"

LOG_MODULE_REGISTER(shb_ble, CONFIG_SHB_LOG_LEVEL);

/*
 * Custom 128-bit service/characteristic UUIDs:
 *   Service:  4fafc201-1fb5-459e-8fcc-c5c9c331914b
 *   Temp:     4fafc202-1fb5-459e-8fcc-c5c9c331914b
 *   Gesture:  4fafc203-1fb5-459e-8fcc-c5c9c331914b
 *   Vitals:   4fafc204-1fb5-459e-8fcc-c5c9c331914b
 */
#define SHB_BLE_UUID_SVC_VAL \
	BT_UUID_128_ENCODE(0x4fafc201, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914b)
#define SHB_BLE_UUID_TEMP_VAL \
	BT_UUID_128_ENCODE(0x4fafc202, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914b)
#define SHB_BLE_UUID_GESTURE_VAL \
	BT_UUID_128_ENCODE(0x4fafc203, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914b)
#define SHB_BLE_UUID_VITALS_VAL \
	BT_UUID_128_ENCODE(0x4fafc204, 0x1fb5, 0x459e, 0x8fcc, 0xc5c9c331914b)

#define SHB_BLE_UUID_SVC     BT_UUID_DECLARE_128(SHB_BLE_UUID_SVC_VAL)
#define SHB_BLE_UUID_TEMP    BT_UUID_DECLARE_128(SHB_BLE_UUID_TEMP_VAL)
#define SHB_BLE_UUID_GESTURE BT_UUID_DECLARE_128(SHB_BLE_UUID_GESTURE_VAL)
#define SHB_BLE_UUID_VITALS  BT_UUID_DECLARE_128(SHB_BLE_UUID_VITALS_VAL)

#define SHB_BLE_TEMP_BUF_LEN    16U
#define SHB_BLE_GESTURE_BUF_LEN 24U
#define SHB_BLE_VITALS_BUF_LEN  64U

static char    shb_temp_buf[SHB_BLE_TEMP_BUF_LEN]       = "0.0 C";
static uint8_t shb_temp_len                               = 5U;
static char    shb_gesture_buf[SHB_BLE_GESTURE_BUF_LEN]  = "IDLE";
static uint8_t shb_gesture_len                            = 4U;
static char    shb_vitals_buf[SHB_BLE_VITALS_BUF_LEN]    = "HR: 0.0 bpm | SpO2: 0.0% | BP: 0/0 mmHg";
static uint8_t shb_vitals_len                             = 40U;

/*
 * Attribute indices — must stay in sync with BT_GATT_SERVICE_DEFINE below.
 * BT_GATT_CHARACTERISTIC expands to exactly 2 attributes (declaration + value).
 *
 *  [0]  Primary service declaration
 *  [1]  Temperature char declaration
 *  [2]  Temperature char value       <- SHB_BLE_TEMP_ATTR_IDX
 *  [3]  Temperature CCC
 *  [4]  Temperature CPF
 *  [5]  Gesture char declaration
 *  [6]  Gesture char value           <- SHB_BLE_GESTURE_ATTR_IDX
 *  [7]  Gesture CCC
 *  [8]  Gesture CPF
 *  [9]  Vitals char declaration
 *  [10] Vitals char value            <- SHB_BLE_VITALS_ATTR_IDX
 *  [11] Vitals CCC
 *  [12] Vitals CPF
 */
#define SHB_BLE_TEMP_ATTR_IDX    2U
#define SHB_BLE_GESTURE_ATTR_IDX 6U
#define SHB_BLE_VITALS_ATTR_IDX  10U

extern const struct bt_gatt_service_static shb_ble_svc;

static bool shb_temp_notify_en;
static bool shb_gesture_notify_en;
static bool shb_vitals_notify_en;

static struct bt_conn *shb_ble_conn;

static const struct bt_gatt_cpf shb_cpf_utf8 = {
	.format      = 0x19U,
	.exponent    = 0,
	.unit        = 0x0000U,
	.name_space  = 0x01U,
	.description = 0x0000U,
};

static ssize_t shb_read_temp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			     void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 shb_temp_buf, shb_temp_len);
}

static ssize_t shb_read_gesture(struct bt_conn *conn, const struct bt_gatt_attr *attr,
				void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 shb_gesture_buf, shb_gesture_len);
}

static ssize_t shb_read_vitals(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			       void *buf, uint16_t len, uint16_t offset)
{
	return bt_gatt_attr_read(conn, attr, buf, len, offset,
				 shb_vitals_buf, shb_vitals_len);
}

static void shb_temp_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	shb_temp_notify_en = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("BLE temp notify %s", shb_temp_notify_en ? "on" : "off");
	if (shb_temp_notify_en) {
		(void)bt_gatt_notify(NULL, &shb_ble_svc.attrs[SHB_BLE_TEMP_ATTR_IDX],
				     shb_temp_buf, shb_temp_len);
	}
}

static void shb_gesture_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	shb_gesture_notify_en = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("BLE gesture notify %s", shb_gesture_notify_en ? "on" : "off");
	if (shb_gesture_notify_en) {
		(void)bt_gatt_notify(NULL, &shb_ble_svc.attrs[SHB_BLE_GESTURE_ATTR_IDX],
				     shb_gesture_buf, shb_gesture_len);
	}
}

static void shb_vitals_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	shb_vitals_notify_en = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("BLE vitals notify %s", shb_vitals_notify_en ? "on" : "off");
	if (shb_vitals_notify_en) {
		(void)bt_gatt_notify(NULL, &shb_ble_svc.attrs[SHB_BLE_VITALS_ATTR_IDX],
				     shb_vitals_buf, shb_vitals_len);
	}
}

BT_GATT_SERVICE_DEFINE(shb_ble_svc,
	BT_GATT_PRIMARY_SERVICE(SHB_BLE_UUID_SVC),

	BT_GATT_CHARACTERISTIC(SHB_BLE_UUID_TEMP,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		shb_read_temp, NULL, NULL),
	BT_GATT_CCC(shb_temp_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(BT_UUID_GATT_CPF, BT_GATT_PERM_READ,
			   bt_gatt_attr_read_cpf, NULL, (void *)&shb_cpf_utf8),

	BT_GATT_CHARACTERISTIC(SHB_BLE_UUID_GESTURE,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		shb_read_gesture, NULL, NULL),
	BT_GATT_CCC(shb_gesture_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(BT_UUID_GATT_CPF, BT_GATT_PERM_READ,
			   bt_gatt_attr_read_cpf, NULL, (void *)&shb_cpf_utf8),

	BT_GATT_CHARACTERISTIC(SHB_BLE_UUID_VITALS,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ,
		shb_read_vitals, NULL, NULL),
	BT_GATT_CCC(shb_vitals_ccc_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_DESCRIPTOR(BT_UUID_GATT_CPF, BT_GATT_PERM_READ,
			   bt_gatt_attr_read_cpf, NULL, (void *)&shb_cpf_utf8),
);

static const struct bt_data shb_ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR),
	BT_DATA(BT_DATA_NAME_COMPLETE,
		CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param shb_adv_param = {
	.id           = BT_ID_DEFAULT,
	.sid          = 0U,
	.secondary_max_skip = 0U,
	.options      = BT_LE_ADV_OPT_CONNECTABLE,
	.interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
	.interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
	.peer         = NULL,
};

static void shb_ble_start_adv(void)
{
	int err = bt_le_adv_start(&shb_adv_param,
				  shb_ad, ARRAY_SIZE(shb_ad),
				  NULL, 0);

	if (err != 0 && err != -EALREADY) {
		LOG_ERR("BLE advertising start failed: %d", err);
	} else {
		LOG_INF("BLE advertising as \"%s\"", CONFIG_BT_DEVICE_NAME);
	}
}

static void shb_ble_connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	if (err != 0U) {
		LOG_WRN("BLE connection failed: %u", err);
		return;
	}

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("BLE connected: %s", addr);

	shb_ble_conn = bt_conn_ref(conn);
}

static void shb_ble_disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
	LOG_INF("BLE disconnected: %s (reason 0x%02x)", addr, reason);

	if (shb_ble_conn != NULL) {
		bt_conn_unref(shb_ble_conn);
		shb_ble_conn = NULL;
	}

	shb_temp_notify_en    = false;
	shb_gesture_notify_en = false;
	shb_vitals_notify_en  = false;

	shb_ble_start_adv();
}

BT_CONN_CB_DEFINE(shb_ble_conn_cb) = {
	.connected    = shb_ble_connected,
	.disconnected = shb_ble_disconnected,
};

int shb_ble_init(void)
{
	int err = bt_enable(NULL);

	if (err != 0) {
		LOG_ERR("BLE enable failed: %d", err);
		return err;
	}

	LOG_INF("BLE stack ready");
	shb_ble_start_adv();
	return 0;
}

void shb_ble_notify_temperature(int32_t temp_mc)
{
	bool negative  = temp_mc < 0;
	int32_t abs_mc = negative ? -temp_mc : temp_mc;
	unsigned int whole = (unsigned int)(abs_mc / 1000U);
	unsigned int frac  = (unsigned int)((abs_mc % 1000U) / 100U);

	int n = snprintf(shb_temp_buf, sizeof(shb_temp_buf),
			 "%s%u.%u C", negative ? "-" : "", whole, frac);

	if (n <= 0 || n >= (int)sizeof(shb_temp_buf)) {
		return;
	}
	shb_temp_len = (uint8_t)n;

	if (!shb_temp_notify_en) {
		return;
	}
	(void)bt_gatt_notify(NULL, &shb_ble_svc.attrs[SHB_BLE_TEMP_ATTR_IDX],
			     shb_temp_buf, shb_temp_len);
}

void shb_ble_notify_event(const char *event_name)
{
	size_t label_len;

	if (event_name == NULL) {
		return;
	}

	label_len = strlen(event_name);
	if (label_len > sizeof(shb_gesture_buf)) {
		label_len = sizeof(shb_gesture_buf);
	}
	memcpy(shb_gesture_buf, event_name, label_len);
	shb_gesture_len = (uint8_t)label_len;

	LOG_INF("BLE event: %.*s", (int)label_len, shb_gesture_buf);

	if (!shb_gesture_notify_en) {
		return;
	}
	(void)bt_gatt_notify(NULL, &shb_ble_svc.attrs[SHB_BLE_GESTURE_ATTR_IDX],
			     shb_gesture_buf, shb_gesture_len);
}

void shb_ble_notify_vitals(uint16_t hr_x10, uint16_t spo2_x10,
			   uint8_t sys_bp, uint8_t dia_bp)
{
	unsigned int hr_whole  = hr_x10 / 10U;
	unsigned int hr_frac   = hr_x10 % 10U;
	unsigned int sp_whole  = spo2_x10 / 10U;
	unsigned int sp_frac   = spo2_x10 % 10U;

	int n = snprintf(shb_vitals_buf, sizeof(shb_vitals_buf),
			 "HR: %u.%u bpm | SpO2: %u.%u%% | BP: %u/%u mmHg",
			 hr_whole, hr_frac, sp_whole, sp_frac, sys_bp, dia_bp);

	if (n <= 0 || n >= (int)sizeof(shb_vitals_buf)) {
		return;
	}
	shb_vitals_len = (uint8_t)n;

	if (!shb_vitals_notify_en) {
		return;
	}
	(void)bt_gatt_notify(NULL, &shb_ble_svc.attrs[SHB_BLE_VITALS_ATTR_IDX],
			     shb_vitals_buf, shb_vitals_len);
}
