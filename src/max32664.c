#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys/util.h>

#include "max32664.h"
#include "power.h"

LOG_MODULE_REGISTER(shb_max32664, CONFIG_SHB_LOG_LEVEL);

#define HUB_I2C_NODE DT_NODELABEL(i2c0)
#define USER_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_NODE_EXISTS(HUB_I2C_NODE), "i2c0 node required for MAX32664");
BUILD_ASSERT(DT_NODE_HAS_PROP(USER_NODE, max32664_mfio_gpios),
	     "zephyr,user.max32664-mfio-gpios missing from overlay");
BUILD_ASSERT(DT_NODE_HAS_PROP(USER_NODE, max32664_rstn_gpios),
	     "zephyr,user.max32664-rstn-gpios missing from overlay");

static const struct device *const shb_hub_i2c = DEVICE_DT_GET(HUB_I2C_NODE);
static const struct gpio_dt_spec shb_hub_mfio =
	GPIO_DT_SPEC_GET(USER_NODE, max32664_mfio_gpios);
static const struct gpio_dt_spec shb_hub_rstn =
	GPIO_DT_SPEC_GET(USER_NODE, max32664_rstn_gpios);

#define HUB_I2C_ADDR 0x55U
#define HUB_RETRY_COUNT 6
#define HUB_DEFAULT_DELAY_MS 2
#define HUB_STARTUP_MS 1100
#define HUB_RESET_PULSE_MS 20
#define HUB_GPIO_SETTLE_MS 5
#define HUB_SMALL_CFG_MAX 16U

#define HUB_ST_SUCCESS 0x00U
#define HUB_ST_ERR_UNAVAIL_CMD 0x01U
#define HUB_ST_ERR_UNAVAIL_FUNC 0x02U
#define HUB_ST_ERR_TRY_AGAIN 0xFEU

#define HUB_FAM_R_STATUS 0x00U
#define HUB_FAM_W_MODE 0x01U
#define HUB_FAM_R_MODE 0x02U
#define HUB_FAM_W_COMMCHAN 0x10U
#define HUB_FAM_R_OUTPUTFIFO 0x12U
#define HUB_FAM_R_READREG 0x41U
#define HUB_FAM_R_REGATTRIBS 0x42U
#define HUB_FAM_W_SENSORMODE 0x44U
#define HUB_FAM_W_ALGOCONFIG 0x50U
#define HUB_FAM_R_ALGOCONFIG 0x51U
#define HUB_FAM_W_ALGOMODE 0x52U
#define HUB_FAM_R_IDENTITY 0xFFU

#define HUB_IDX_STATUS 0x00U
#define HUB_IDX_MODE 0x00U
#define HUB_IDX_OUTPUTMODE 0x00U
#define HUB_IDX_FIFO_THRESHOLD 0x01U
#define HUB_IDX_NUM_SAMPLES 0x00U
#define HUB_IDX_READ_FIFO 0x01U
#define HUB_IDX_FWVERSION 0x03U

#define HUB_MODE_RUNTIME 0x00U
#define HUB_MODE_FIFO_RESET 0x04U

#define HUB_OUT_PAUSE 0x00U
#define HUB_OUT_SENSOR_ONLY 0x01U
#define HUB_OUT_SENSOR_ALGO 0x03U

#define SENSOR_IDX_MAX30101 0x03U
#define SENSOR_ENABLE 0x01U
#define SENSOR_DISABLE 0x00U

#define ALGO_IDX_AGC 0x00U
#define ALGO_IDX_BPT 0x04U
#define ALGO_DISABLE 0x00U
#define ALGO_ENABLE 0x01U
#define BPT_CAL 0x01U
#define BPT_EST 0x02U

#define CFG_BP_SYS_REF 0x01U
#define CFG_BP_DIA_REF 0x02U
#define CFG_BP_CAL_VECTOR 0x03U
#define CFG_BP_EST_DATE 0x04U
#define CFG_BP_SPO2_COEFS 0x06U

#define MAX30101_REG_PART_ID 0xFFU
#define MAX30101_EXPECTED_ID 0x15U

#define PPG_SAMPLE_BYTES 12U
#define BPT_SAMPLE_BYTES 23U
#define BPT_CAL_VECTOR_BYTES 824U
#define FIFO_MAX_SAMPLES 15U
#define FIFO_BUF_SIZE (1U + (FIFO_MAX_SAMPLES * BPT_SAMPLE_BYTES))
#define RAW_FIFO_BUF_SIZE (1U + (FIFO_MAX_SAMPLES * PPG_SAMPLE_BYTES))
#define ALGO_CFG_BUF_SIZE (1U + BPT_CAL_VECTOR_BYTES)
#define BPT_FIFO_THRESHOLD 0x0FU
#define RAW_FIFO_THRESHOLD 0x01U

#define CAL_MAGIC 0x42505456U
/* CAL_VERSION 2: corrected SpO2 coefficients to Maxim defaults (UG6921 §2.1) */
#define CAL_VERSION 2U
#define CAL_NVS_ID 1U

/*
 * SpO2 calibration coefficients sent to the hub (UG6921 Table 3, cfg index 0x06).
 * Format: round(10^5 × float_coef), transmitted as three little-endian int32 values.
 * Maxim defaults: a=1.5958422, b=-34.659664, c=112.68987
 */
#define SPO2_COEF_A CONFIG_SHB_MAX32664_SPO2_COEF_A
#define SPO2_COEF_B CONFIG_SHB_MAX32664_SPO2_COEF_B
#define SPO2_COEF_C CONFIG_SHB_MAX32664_SPO2_COEF_C

#define FLOW_START_MS 100U
#define LOOP_MS 100U
#define WAIT_LOG_INTERVAL 20U
#define CAL_LOG_INTERVAL 10U
#define MAX_XPORT_ERRORS 5U
#define HR_DELTA_X10 5U
#define SPO2_DELTA_X10 1U
#define BP_DELTA 2U

struct bpt_sample {
	uint32_t led1_ir;
	uint32_t led2_red;
	uint32_t led3;
	uint32_t led4;
	uint8_t bp_status;
	uint8_t progress;
	uint16_t hr_x10;
	uint8_t sys_bp;
	uint8_t dia_bp;
	uint16_t spo2_x10;
	uint16_t r_x1000;
};

struct cal_blob {
	uint32_t magic;
	uint16_t version;
	uint16_t payload_len;
	int32_t spo2_coefs[3];
	uint8_t bp_sys_refs[3];
	uint8_t bp_dia_refs[3];
	uint8_t reserved[2];
	uint8_t vector[BPT_CAL_VECTOR_BYTES];
	uint32_t crc32;
};

static bool shb_hub_ready;
static bool shb_hub_vector_loaded;
static bool shb_hub_raw_streaming;
static bool shb_max_thread_running;

static int32_t shb_spo2_coefs[3] = { SPO2_COEF_A, SPO2_COEF_B, SPO2_COEF_C };
static uint8_t shb_bp_sys_refs[3];
static uint8_t shb_bp_dia_refs[3];
static uint32_t shb_est_date[2] = { 180828U, 163808U };

static uint8_t shb_fifo_buf[FIFO_BUF_SIZE];
static uint8_t shb_raw_fifo_buf[RAW_FIFO_BUF_SIZE];
static uint8_t shb_algo_cfg_buf[ALGO_CFG_BUF_SIZE];
static uint8_t shb_saved_vector[BPT_CAL_VECTOR_BYTES];
static struct cal_blob shb_cal_blob;

static struct nvs_fs shb_cal_nvs;
static bool shb_cal_nvs_ready;

static struct shb_max32664_result shb_latest;
static K_MUTEX_DEFINE(shb_result_mtx);

K_THREAD_STACK_DEFINE(shb_max32664_stack, CONFIG_SHB_MAX32664_THREAD_STACK_SIZE);
static struct k_thread shb_max32664_td;

static void shb_max32664_thread(void *p1, void *p2, void *p3);

static void hub_recover_bus(const char *reason)
{
	int err = i2c_recover_bus(shb_hub_i2c);

	if ((err != 0) && (err != -ENOSYS) && (err != -ENOTSUP)) {
		LOG_WRN("I2C recovery failed (%s): %d", reason, err);
	}
}

static int hub_write(const uint8_t *tx, size_t tx_len, int delay_ms, uint8_t *status_out)
{
	int err = 0;

	for (int i = 0; i < HUB_RETRY_COUNT; ++i) {
		err = i2c_write(shb_hub_i2c, tx, tx_len, HUB_I2C_ADDR);
		if (err == 0) {
			break;
		}
		hub_recover_bus("write");
		k_msleep(2);
	}
	if (err != 0) {
		return -EIO;
	}

	for (int i = 0; i < HUB_RETRY_COUNT; ++i) {
		k_msleep(delay_ms);
		err = i2c_read(shb_hub_i2c, status_out, 1, HUB_I2C_ADDR);
		if ((err == 0) && (*status_out != HUB_ST_ERR_TRY_AGAIN)) {
			return 0;
		}
		if (err != 0) {
			hub_recover_bus("status poll");
		}
	}

	return -EIO;
}

static int hub_read(const uint8_t *cmd, size_t cmd_len,
		    uint8_t *rx, size_t rx_len, int delay_ms)
{
	for (int i = 0; i < HUB_RETRY_COUNT; ++i) {
		int err = i2c_write(shb_hub_i2c, cmd, cmd_len, HUB_I2C_ADDR);

		if (err != 0) {
			hub_recover_bus("read cmd");
			k_msleep(2);
			continue;
		}

		k_msleep(delay_ms);
		err = i2c_read(shb_hub_i2c, rx, rx_len, HUB_I2C_ADDR);
		if ((err == 0) && (rx[0] != HUB_ST_ERR_TRY_AGAIN)) {
			return 0;
		}
		if (err != 0) {
			hub_recover_bus("read resp");
		}
	}

	return -EIO;
}

static int hub_read_mode(uint8_t *mode)
{
	uint8_t cmd[] = { HUB_FAM_R_MODE, HUB_IDX_MODE };
	uint8_t rx[2] = { 0 };
	int err = hub_read(cmd, sizeof(cmd), rx, sizeof(rx), HUB_DEFAULT_DELAY_MS);

	if ((err == 0) && (rx[0] == HUB_ST_SUCCESS)) {
		*mode = rx[1];
		return 0;
	}

	return (err != 0) ? err : -EIO;
}

static int hub_write_mode(uint8_t mode_val, int delay_ms)
{
	uint8_t cmd[] = { HUB_FAM_W_MODE, HUB_IDX_MODE, mode_val };
	uint8_t status = 0U;
	int err = hub_write(cmd, sizeof(cmd), delay_ms, &status);

	return ((err == 0) && (status == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_read_status(uint8_t *hub_status)
{
	uint8_t cmd[] = { HUB_FAM_R_STATUS, HUB_IDX_STATUS };
	uint8_t rx[2] = { 0 };
	int err = hub_read(cmd, sizeof(cmd), rx, sizeof(rx), HUB_DEFAULT_DELAY_MS);

	if ((err == 0) && (rx[0] == HUB_ST_SUCCESS)) {
		*hub_status = rx[1];
		return 0;
	}

	return (err != 0) ? err : -EIO;
}

static int hub_read_identity(uint8_t index, uint8_t *data, size_t data_len)
{
	uint8_t cmd[] = { HUB_FAM_R_IDENTITY, index };
	uint8_t rx[8] = { 0 };
	size_t rx_len = data_len + 1U;
	int err;

	if (rx_len > sizeof(rx)) {
		return -EINVAL;
	}

	err = hub_read(cmd, sizeof(cmd), rx, rx_len, HUB_DEFAULT_DELAY_MS);
	if (err != 0) {
		return err;
	}

	if ((rx[0] == HUB_ST_ERR_UNAVAIL_CMD) || (rx[0] == HUB_ST_ERR_UNAVAIL_FUNC)) {
		return -ENOTSUP;
	}
	if (rx[0] != HUB_ST_SUCCESS) {
		return -EIO;
	}

	memcpy(data, &rx[1], data_len);
	return 0;
}

static int hub_set_output_mode(uint8_t out_mode)
{
	uint8_t cmd[] = { HUB_FAM_W_COMMCHAN, HUB_IDX_OUTPUTMODE, out_mode };
	uint8_t status = 0U;
	int err = hub_write(cmd, sizeof(cmd), HUB_DEFAULT_DELAY_MS, &status);

	return ((err == 0) && (status == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_set_fifo_threshold(uint8_t threshold)
{
	uint8_t cmd[] = { HUB_FAM_W_COMMCHAN, HUB_IDX_FIFO_THRESHOLD, threshold };
	uint8_t status = 0U;
	int err = hub_write(cmd, sizeof(cmd), HUB_DEFAULT_DELAY_MS, &status);

	return ((err == 0) && (status == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_set_sensor(bool enabled)
{
	uint8_t cmd[] = {
		HUB_FAM_W_SENSORMODE, SENSOR_IDX_MAX30101,
		enabled ? SENSOR_ENABLE : SENSOR_DISABLE,
	};
	uint8_t status = 0U;
	int err = hub_write(cmd, sizeof(cmd), 40, &status);

	return ((err == 0) && (status == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_set_algo_mode(uint8_t algo, uint8_t mode, int delay_ms)
{
	uint8_t cmd[] = { HUB_FAM_W_ALGOMODE, algo, mode };
	uint8_t status = 0U;
	int err = hub_write(cmd, sizeof(cmd), delay_ms, &status);

	return ((err == 0) && (status == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_set_algo_cfg(uint8_t algo, uint8_t cfg_idx,
			    const uint8_t *data, size_t data_len, int delay_ms)
{
	uint8_t tx[3U + HUB_SMALL_CFG_MAX] = {
		HUB_FAM_W_ALGOCONFIG, algo, cfg_idx,
	};
	uint8_t status = 0U;
	int err;

	if (data_len > HUB_SMALL_CFG_MAX) {
		return -EINVAL;
	}

	if ((data != NULL) && (data_len > 0U)) {
		memcpy(&tx[3], data, data_len);
	}

	err = hub_write(tx, 3U + data_len, delay_ms, &status);
	return ((err == 0) && (status == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_read_algo_cfg(uint8_t algo, uint8_t cfg_idx,
			     uint8_t *out, size_t out_len, int delay_ms)
{
	uint8_t cmd[] = { HUB_FAM_R_ALGOCONFIG, algo, cfg_idx };
	int err;

	if ((out == NULL) || (out_len > BPT_CAL_VECTOR_BYTES)) {
		return -EINVAL;
	}

	err = hub_read(cmd, sizeof(cmd), shb_algo_cfg_buf, 1U + out_len, delay_ms);
	if ((err == 0) && (shb_algo_cfg_buf[0] == HUB_ST_SUCCESS)) {
		memcpy(out, &shb_algo_cfg_buf[1], out_len);
		return 0;
	}

	return (err != 0) ? err : -EIO;
}

static int hub_read_num_samples(uint8_t *n)
{
	uint8_t cmd[] = { HUB_FAM_R_OUTPUTFIFO, HUB_IDX_NUM_SAMPLES };
	uint8_t rx[2] = { 0 };
	int err = hub_read(cmd, sizeof(cmd), rx, sizeof(rx), HUB_DEFAULT_DELAY_MS);

	if ((err == 0) && (rx[0] == HUB_ST_SUCCESS)) {
		*n = rx[1];
		return 0;
	}

	return (err != 0) ? err : -EIO;
}

static int hub_read_fifo_samples(uint8_t count, size_t sample_size,
				 uint8_t *rx, size_t rx_len)
{
	uint8_t cmd[] = { HUB_FAM_R_OUTPUTFIFO, HUB_IDX_READ_FIFO };
	size_t expected = 1U + ((size_t)count * sample_size);
	int err;

	if (rx_len < expected) {
		return -EINVAL;
	}

	err = hub_read(cmd, sizeof(cmd), rx, expected, 10);
	return ((err == 0) && (rx[0] == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_drain_fifo(size_t sample_size, uint8_t *latest_out,
			  uint8_t *avail_out, uint8_t *read_out)
{
	uint8_t avail = 0U;
	uint8_t total = 0U;
	int err = hub_read_num_samples(&avail);

	if (err != 0) {
		return err;
	}

	*avail_out = avail;
	*read_out = 0U;

	if (avail == 0U) {
		return -EAGAIN;
	}

	while (avail > 0U) {
		uint8_t to_read = MIN(avail, (uint8_t)FIFO_MAX_SAMPLES);
		size_t last_off;

		err = hub_read_fifo_samples(to_read, sample_size,
					    (sample_size == PPG_SAMPLE_BYTES) ? shb_raw_fifo_buf : shb_fifo_buf,
					    (sample_size == PPG_SAMPLE_BYTES) ? sizeof(shb_raw_fifo_buf) : sizeof(shb_fifo_buf));
		if (err != 0) {
			return err;
		}

		last_off = 1U + ((to_read - 1U) * sample_size);
		memcpy(latest_out,
		       ((sample_size == PPG_SAMPLE_BYTES) ? shb_raw_fifo_buf : shb_fifo_buf) + last_off,
		       sample_size);
		total += to_read;

		err = hub_read_num_samples(&avail);
		if (err != 0) {
			return err;
		}
	}

	*read_out = total;
	return 0;
}

static int hub_load_cal_vector(const uint8_t *vec, size_t vec_len)
{
	static uint8_t tx[3U + BPT_CAL_VECTOR_BYTES];
	uint8_t status = 0U;
	int err;

	if ((vec == NULL) || (vec_len != BPT_CAL_VECTOR_BYTES)) {
		return -EINVAL;
	}

	tx[0] = HUB_FAM_W_ALGOCONFIG;
	tx[1] = ALGO_IDX_BPT;
	tx[2] = CFG_BP_CAL_VECTOR;
	memcpy(&tx[3], vec, vec_len);

	err = hub_write(tx, sizeof(tx), 10, &status);
	return ((err == 0) && (status == HUB_ST_SUCCESS)) ? 0 : ((err != 0) ? err : -EIO);
}

static int hub_read_sensor_reg(uint8_t sensor_idx, uint8_t reg, uint32_t *value)
{
	uint8_t cmd_attr[] = { HUB_FAM_R_REGATTRIBS, sensor_idx };
	uint8_t rx_attr[3] = { 0 };
	uint8_t reg_width = 1U;
	uint8_t cmd[] = { HUB_FAM_R_READREG, sensor_idx, reg };
	uint8_t rx[5] = { 0 };
	uint32_t assembled = 0U;
	int err;

	err = hub_read(cmd_attr, sizeof(cmd_attr), rx_attr, sizeof(rx_attr), HUB_DEFAULT_DELAY_MS);
	if ((err == 0) && (rx_attr[0] == HUB_ST_SUCCESS)) {
		reg_width = rx_attr[1];
	}
	if ((reg_width == 0U) || (reg_width > 4U)) {
		return -EINVAL;
	}

	err = hub_read(cmd, sizeof(cmd), rx, reg_width + 1U, HUB_DEFAULT_DELAY_MS);
	if ((err != 0) || (rx[0] != HUB_ST_SUCCESS)) {
		return (err != 0) ? err : -EIO;
	}

	for (uint8_t i = 0U; i < reg_width; ++i) {
		assembled = (assembled << 8) | rx[1U + i];
	}

	*value = assembled;
	return 0;
}

static int hub_enter_runtime_mode(void)
{
	int err;

	err = gpio_pin_configure_dt(&shb_hub_rstn, GPIO_OUTPUT_ACTIVE);
	if (err != 0) {
		return err;
	}

	err = gpio_pin_configure_dt(&shb_hub_mfio, GPIO_OUTPUT_INACTIVE);
	if (err != 0) {
		return err;
	}

	k_msleep(HUB_RESET_PULSE_MS);

	err = gpio_pin_set_dt(&shb_hub_mfio, 0);
	if (err != 0) {
		return err;
	}

	k_msleep(HUB_GPIO_SETTLE_MS);

	err = gpio_pin_set_dt(&shb_hub_rstn, 0);
	if (err != 0) {
		return err;
	}

	k_msleep(HUB_STARTUP_MS);

	err = gpio_pin_configure_dt(&shb_hub_mfio, GPIO_INPUT | GPIO_PULL_UP);
	if (err != 0) {
		return err;
	}

	err = gpio_pin_configure_dt(&shb_hub_rstn, GPIO_INPUT | GPIO_PULL_UP);
	if (err != 0) {
		return err;
	}

	hub_recover_bus("runtime reset");
	k_msleep(HUB_GPIO_SETTLE_MS);
	return 0;
}

static int hub_ensure_runtime_mode(void)
{
	uint8_t mode = 0U;
	int err = hub_read_mode(&mode);

	if (err != 0) {
		return err;
	}

	if (mode == HUB_MODE_FIFO_RESET) {
		err = hub_write_mode(HUB_MODE_RUNTIME, 10);
		if (err != 0) {
			return err;
		}
		k_msleep(20);
		err = hub_read_mode(&mode);
		if (err != 0) {
			return err;
		}
	}

	if (mode != HUB_MODE_RUNTIME) {
		LOG_ERR("MAX32664 unexpected mode: 0x%02x", mode);
		return -EIO;
	}

	return 0;
}

static int hub_reset_fifo(void)
{
	int err = hub_write_mode(HUB_MODE_FIFO_RESET, HUB_DEFAULT_DELAY_MS);

	if (err != 0) {
		return err;
	}

	k_msleep(10);
	return hub_ensure_runtime_mode();
}

static void enc_u32_le(uint32_t v, uint8_t out[4])
{
	out[0] = (uint8_t)(v & 0xFFU);
	out[1] = (uint8_t)((v >> 8) & 0xFFU);
	out[2] = (uint8_t)((v >> 16) & 0xFFU);
	out[3] = (uint8_t)((v >> 24) & 0xFFU);
}

static void enc_i32_le(int32_t v, uint8_t out[4])
{
	enc_u32_le((uint32_t)v, out);
}

static int hub_set_datetime(void)
{
	uint8_t payload[8];

	enc_u32_le(shb_est_date[0], &payload[0]);
	enc_u32_le(shb_est_date[1], &payload[4]);
	return hub_set_algo_cfg(ALGO_IDX_BPT, CFG_BP_EST_DATE, payload, sizeof(payload), 30);
}

static int hub_set_bp_refs(void)
{
	int err;

	err = hub_set_algo_cfg(ALGO_IDX_BPT, CFG_BP_SYS_REF,
			       shb_bp_sys_refs, sizeof(shb_bp_sys_refs), 30);
	if (err != 0) {
		return err;
	}

	return hub_set_algo_cfg(ALGO_IDX_BPT, CFG_BP_DIA_REF,
				shb_bp_dia_refs, sizeof(shb_bp_dia_refs), 30);
}

static int hub_set_spo2_coefs(void)
{
	uint8_t payload[12];
	int err;

	enc_i32_le(shb_spo2_coefs[0], &payload[0]);
	enc_i32_le(shb_spo2_coefs[1], &payload[4]);
	enc_i32_le(shb_spo2_coefs[2], &payload[8]);

	err = hub_set_algo_cfg(ALGO_IDX_BPT, CFG_BP_SPO2_COEFS, payload, sizeof(payload), 30);
	if (err != 0) {
		return err;
	}

	LOG_INF("MAX32664 SpO2 coefs: (%d, %d, %d)",
		shb_spo2_coefs[0], shb_spo2_coefs[1], shb_spo2_coefs[2]);
	return 0;
}

static void parse_bpt_sample(const uint8_t *b, struct bpt_sample *s)
{
	s->led1_ir = ((uint32_t)b[0] << 16) | ((uint32_t)b[1] << 8) | b[2];
	s->led2_red = ((uint32_t)b[3] << 16) | ((uint32_t)b[4] << 8) | b[5];
	s->led3 = ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 8) | b[8];
	s->led4 = ((uint32_t)b[9] << 16) | ((uint32_t)b[10] << 8) | b[11];
	s->bp_status = b[12];
	s->progress = b[13];
	s->hr_x10 = ((uint16_t)b[14] << 8) | b[15];
	s->sys_bp = b[16];
	s->dia_bp = b[17];
	s->spo2_x10 = ((uint16_t)b[18] << 8) | b[19];
	s->r_x1000 = ((uint16_t)b[20] << 8) | b[21];
}

static bool sample_publishable(const struct bpt_sample *s)
{
	return (s->bp_status == 2U) && (s->progress >= 100U) &&
	       (s->hr_x10 > 0U) && (s->spo2_x10 > 0U) &&
	       (s->sys_bp > 0U) && (s->dia_bp > 0U);
}

static uint16_t u16_absdiff(uint16_t a, uint16_t b)
{
	return (a > b) ? (a - b) : (b - a);
}

static uint8_t u8_absdiff(uint8_t a, uint8_t b)
{
	return (a > b) ? (a - b) : (b - a);
}

static void publish_result(const struct bpt_sample *s)
{
	k_mutex_lock(&shb_result_mtx, K_FOREVER);
	shb_latest.valid = true;
	shb_latest.hr_x10 = s->hr_x10;
	shb_latest.spo2_x10 = s->spo2_x10;
	shb_latest.sys_bp = s->sys_bp;
	shb_latest.dia_bp = s->dia_bp;
	shb_latest.status = s->bp_status;
	shb_latest.progress = s->progress;
	k_mutex_unlock(&shb_result_mtx);

	LOG_INF("MAX32664 vitals: HR=%u.%u bpm SpO2=%u.%u%% BP=%u/%u mmHg",
		(unsigned int)(s->hr_x10 / 10U), (unsigned int)(s->hr_x10 % 10U),
		(unsigned int)(s->spo2_x10 / 10U), (unsigned int)(s->spo2_x10 % 10U),
		(unsigned int)s->sys_bp, (unsigned int)s->dia_bp);

	shb_power_signal(SHB_POWER_EVENT_MAX32664_VITALS);
}

static void log_bpt_status(const char *phase, const struct bpt_sample *s)
{
	switch (s->bp_status) {
	case 0U:
		LOG_INF("MAX32664 %s: place finger on sensor (progress=%u%% IR=%u RED=%u)",
			phase, (unsigned int)s->progress,
			(unsigned int)s->led1_ir, (unsigned int)s->led2_red);
		break;
	case 1U:
		LOG_INF("MAX32664 %s: stabilizing (progress=%u%% IR=%u RED=%u)",
			phase, (unsigned int)s->progress,
			(unsigned int)s->led1_ir, (unsigned int)s->led2_red);
		break;
	case 2U:
		LOG_INF("MAX32664 %s: good signal (progress=%u%% IR=%u RED=%u)",
			phase, (unsigned int)s->progress,
			(unsigned int)s->led1_ir, (unsigned int)s->led2_red);
		break;
	case 3U:
		LOG_WRN("MAX32664 %s: weak signal - reposition finger (progress=%u%% IR=%u RED=%u)",
			phase, (unsigned int)s->progress,
			(unsigned int)s->led1_ir, (unsigned int)s->led2_red);
		break;
	case 4U:
		LOG_WRN("MAX32664 %s: motion detected - hold still (progress=%u%% IR=%u RED=%u)",
			phase, (unsigned int)s->progress,
			(unsigned int)s->led1_ir, (unsigned int)s->led2_red);
		break;
	case 5U:
		LOG_ERR("MAX32664 %s: measurement failed (progress=%u%% IR=%u RED=%u)",
			phase, (unsigned int)s->progress,
			(unsigned int)s->led1_ir, (unsigned int)s->led2_red);
		break;
	default:
		LOG_INF("MAX32664 %s: status=%u progress=%u%% IR=%u RED=%u",
			phase, (unsigned int)s->bp_status, (unsigned int)s->progress,
			(unsigned int)s->led1_ir, (unsigned int)s->led2_red);
		break;
	}
}

static uint32_t cal_blob_crc(const struct cal_blob *b)
{
	return crc32_ieee((const uint8_t *)b, offsetof(struct cal_blob, crc32));
}

static int cal_init(void)
{
	struct flash_pages_info page;
	int err;

	shb_cal_nvs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if (!device_is_ready(shb_cal_nvs.flash_device)) {
		return -ENODEV;
	}

	shb_cal_nvs.offset = FIXED_PARTITION_OFFSET(storage_partition);
	err = flash_get_page_info_by_offs(shb_cal_nvs.flash_device, shb_cal_nvs.offset, &page);
	if (err != 0) {
		return err;
	}

	shb_cal_nvs.sector_size = page.size;
	shb_cal_nvs.sector_count = FIXED_PARTITION_SIZE(storage_partition) / page.size;
	if (shb_cal_nvs.sector_count < 2U) {
		return -ENOSPC;
	}

	err = nvs_mount(&shb_cal_nvs);
	if (err == 0) {
		shb_cal_nvs_ready = true;
	}

	return err;
}

static int cal_save(const uint8_t vector[BPT_CAL_VECTOR_BYTES])
{
	ssize_t written;

	if (!shb_cal_nvs_ready) {
		return -ENODEV;
	}

	shb_cal_blob.magic = CAL_MAGIC;
	shb_cal_blob.version = CAL_VERSION;
	shb_cal_blob.payload_len = BPT_CAL_VECTOR_BYTES;
	memcpy(shb_cal_blob.spo2_coefs, shb_spo2_coefs, sizeof(shb_cal_blob.spo2_coefs));
	memcpy(shb_cal_blob.bp_sys_refs, shb_bp_sys_refs, sizeof(shb_cal_blob.bp_sys_refs));
	memcpy(shb_cal_blob.bp_dia_refs, shb_bp_dia_refs, sizeof(shb_cal_blob.bp_dia_refs));
	memcpy(shb_cal_blob.vector, vector, BPT_CAL_VECTOR_BYTES);
	shb_cal_blob.crc32 = cal_blob_crc(&shb_cal_blob);

	written = nvs_write(&shb_cal_nvs, CAL_NVS_ID, &shb_cal_blob, sizeof(shb_cal_blob));
	if (written < 0) {
		return (int)written;
	}
	if ((size_t)written != sizeof(shb_cal_blob)) {
		return -EIO;
	}

	return 0;
}

static int cal_load(uint8_t vector_out[BPT_CAL_VECTOR_BYTES])
{
	ssize_t n;

	if (!shb_cal_nvs_ready) {
		return -ENODEV;
	}

	n = nvs_read(&shb_cal_nvs, CAL_NVS_ID, &shb_cal_blob, sizeof(shb_cal_blob));
	if (n == 0) {
		return -ENOENT;
	}
	if (n < 0) {
		return (int)n;
	}
	if ((size_t)n != sizeof(shb_cal_blob)) {
		return -EINVAL;
	}

	if ((shb_cal_blob.magic != CAL_MAGIC) ||
	    (shb_cal_blob.version != CAL_VERSION) ||
	    (shb_cal_blob.payload_len != BPT_CAL_VECTOR_BYTES)) {
		return -EINVAL;
	}
	if (shb_cal_blob.crc32 != cal_blob_crc(&shb_cal_blob)) {
		LOG_WRN("MAX32664 saved calibration CRC mismatch - discarding");
		return -EBADMSG;
	}

	memcpy(vector_out, shb_cal_blob.vector, BPT_CAL_VECTOR_BYTES);
	memcpy(shb_spo2_coefs, shb_cal_blob.spo2_coefs, sizeof(shb_spo2_coefs));
	memcpy(shb_bp_sys_refs, shb_cal_blob.bp_sys_refs, sizeof(shb_bp_sys_refs));
	memcpy(shb_bp_dia_refs, shb_cal_blob.bp_dia_refs, sizeof(shb_bp_dia_refs));
	return 0;
}

static int start_calibration(void)
{
	const bool agc = IS_ENABLED(CONFIG_SHB_MAX32664_AGC_ENABLED);
	int err;

	err = hub_reset_fifo();
	if (err != 0) {
		return err;
	}
	err = hub_set_datetime();
	if (err != 0) {
		return err;
	}
	err = hub_set_bp_refs();
	if (err != 0) {
		return err;
	}
	err = hub_set_spo2_coefs();
	if (err != 0) {
		return err;
	}
	err = hub_set_output_mode(HUB_OUT_SENSOR_ALGO);
	if (err != 0) {
		return err;
	}
	err = hub_set_fifo_threshold(BPT_FIFO_THRESHOLD);
	if (err != 0) {
		return err;
	}
	err = hub_set_algo_mode(ALGO_IDX_AGC, agc ? ALGO_ENABLE : ALGO_DISABLE, 40);
	if (err != 0) {
		return err;
	}
	err = hub_set_sensor(true);
	if (err != 0) {
		return err;
	}
	err = hub_set_algo_mode(ALGO_IDX_BPT, BPT_CAL, 60);
	if (err != 0) {
		return err;
	}

	k_msleep(FLOW_START_MS);
	LOG_INF("MAX32664 calibration started (%s) - SYS=%u/%u/%u DIA=%u/%u/%u",
		agc ? "AGC on" : "AGC off",
		(unsigned int)shb_bp_sys_refs[0], (unsigned int)shb_bp_sys_refs[1], (unsigned int)shb_bp_sys_refs[2],
		(unsigned int)shb_bp_dia_refs[0], (unsigned int)shb_bp_dia_refs[1], (unsigned int)shb_bp_dia_refs[2]);
	return 0;
}

static int start_estimation(const uint8_t vector[BPT_CAL_VECTOR_BYTES])
{
	const bool agc = IS_ENABLED(CONFIG_SHB_MAX32664_AGC_ENABLED);
	int err;

	err = hub_reset_fifo();
	if (err != 0) {
		return err;
	}
	err = hub_load_cal_vector(vector, BPT_CAL_VECTOR_BYTES);
	if (err != 0) {
		return err;
	}
	err = hub_set_datetime();
	if (err != 0) {
		return err;
	}
	err = hub_set_spo2_coefs();
	if (err != 0) {
		return err;
	}
	err = hub_set_output_mode(HUB_OUT_SENSOR_ALGO);
	if (err != 0) {
		return err;
	}
	err = hub_set_fifo_threshold(BPT_FIFO_THRESHOLD);
	if (err != 0) {
		return err;
	}
	err = hub_set_algo_mode(ALGO_IDX_AGC, agc ? ALGO_ENABLE : ALGO_DISABLE, 40);
	if (err != 0) {
		return err;
	}
	err = hub_set_sensor(true);
	if (err != 0) {
		return err;
	}
	err = hub_set_algo_mode(ALGO_IDX_BPT, BPT_EST, 60);
	if (err != 0) {
		return err;
	}

	k_msleep(FLOW_START_MS);
	LOG_INF("MAX32664 estimation started (%s)", agc ? "AGC on" : "AGC off");
	return 0;
}

static int run_calibration_loop(uint8_t vector_out[BPT_CAL_VECTOR_BYTES])
{
	uint8_t raw[BPT_SAMPLE_BYTES] = { 0 };
	uint32_t xport_errors = 0U;
	uint32_t empty_polls = 0U;
	uint32_t status_logs = 0U;
	uint8_t last_status = 0xFFU;
	uint8_t last_progress = 0xFFU;
	const int64_t cal_deadline_ms = (CONFIG_SHB_MAX32664_CAL_TIMEOUT_S > 0)
		? (k_uptime_get() + (int64_t)CONFIG_SHB_MAX32664_CAL_TIMEOUT_S * 1000LL)
		: 0;
	int err;

	while (true) {
		if ((cal_deadline_ms != 0) && (k_uptime_get() >= cal_deadline_ms)) {
			LOG_WRN("MAX32664 calibration timed out after %d s "
				"(progress=%u%% last_status=%u) - aborting",
				CONFIG_SHB_MAX32664_CAL_TIMEOUT_S,
				(unsigned int)last_progress,
				(unsigned int)last_status);
			(void)hub_set_sensor(false);
			(void)hub_set_algo_mode(ALGO_IDX_BPT, ALGO_DISABLE, 40);
			(void)hub_set_algo_mode(ALGO_IDX_AGC, ALGO_DISABLE, 40);
			(void)hub_set_output_mode(HUB_OUT_PAUSE);
			(void)hub_reset_fifo();
			return -ETIMEDOUT;
		}
		uint8_t avail = 0U;
		uint8_t read_total = 0U;
		uint8_t hub_status = 0U;
		struct bpt_sample s = { 0 };

		err = hub_read_status(&hub_status);
		if (err != 0) {
			if (++xport_errors >= MAX_XPORT_ERRORS) {
				LOG_ERR("MAX32664 calibration: hub poll failed: %d", err);
				return err;
			}
			k_msleep(LOOP_MS);
			continue;
		}
		xport_errors = 0U;

		err = hub_drain_fifo(BPT_SAMPLE_BYTES, raw, &avail, &read_total);
		if (err == -EAGAIN) {
			if ((++empty_polls % WAIT_LOG_INTERVAL) == 0U) {
				LOG_INF("MAX32664 calibration: waiting for finger contact...");
			}
			k_msleep(LOOP_MS);
			continue;
		}
		if (err != 0) {
			LOG_ERR("MAX32664 calibration: FIFO read failed: %d", err);
			return err;
		}

		empty_polls = 0U;
		parse_bpt_sample(raw, &s);

		if ((s.bp_status == 2U) && (s.progress >= 100U)) {
			LOG_INF("MAX32664 calibration complete - reading vector");
			break;
		}

		if ((s.bp_status != last_status) || (s.progress != last_progress) ||
		    ((++status_logs % CAL_LOG_INTERVAL) == 0U)) {
			log_bpt_status("calibration", &s);
			last_status = s.bp_status;
			last_progress = s.progress;
		}

		k_msleep(LOOP_MS);
	}

	err = hub_set_sensor(false);
	if (err != 0) {
		return err;
	}
	err = hub_set_algo_mode(ALGO_IDX_BPT, ALGO_DISABLE, 40);
	if (err != 0) {
		return err;
	}
	err = hub_read_algo_cfg(ALGO_IDX_BPT, CFG_BP_CAL_VECTOR, vector_out, BPT_CAL_VECTOR_BYTES, 20);
	if (err != 0) {
		return err;
	}
	err = hub_set_algo_mode(ALGO_IDX_AGC, ALGO_DISABLE, 40);
	if (err != 0) {
		return err;
	}
	err = hub_set_output_mode(HUB_OUT_PAUSE);
	if (err != 0) {
		return err;
	}

	return hub_reset_fifo();
}

static int run_estimation_loop(void)
{
	uint8_t raw[BPT_SAMPLE_BYTES] = { 0 };
	uint32_t xport_errors = 0U;
	uint32_t empty_polls = 0U;
	struct bpt_sample prev = { 0 };
	bool have_prev = false;

	while (true) {
		uint8_t avail = 0U;
		uint8_t read_total = 0U;
		uint8_t hub_status = 0U;
		struct bpt_sample s = { 0 };
		int err;

		err = hub_read_status(&hub_status);
		if (err != 0) {
			if (++xport_errors >= MAX_XPORT_ERRORS) {
				LOG_ERR("MAX32664 estimation: hub poll failed: %d", err);
				return err;
			}
			k_msleep(LOOP_MS);
			continue;
		}
		xport_errors = 0U;

		err = hub_drain_fifo(BPT_SAMPLE_BYTES, raw, &avail, &read_total);
		if (err == -EAGAIN) {
			if ((++empty_polls % WAIT_LOG_INTERVAL) == 0U) {
				LOG_INF("MAX32664: waiting for finger contact...");
			}
			k_msleep(LOOP_MS);
			continue;
		}
		if (err != 0) {
			if (++xport_errors >= MAX_XPORT_ERRORS) {
				LOG_ERR("MAX32664 estimation: FIFO read failed: %d", err);
				return err;
			}
			k_msleep(LOOP_MS);
			continue;
		}

		empty_polls = 0U;
		parse_bpt_sample(raw, &s);

		if (sample_publishable(&s)) {
			bool changed = !have_prev ||
				u16_absdiff(s.hr_x10, prev.hr_x10) >= HR_DELTA_X10 ||
				u16_absdiff(s.spo2_x10, prev.spo2_x10) >= SPO2_DELTA_X10 ||
				u8_absdiff(s.sys_bp, prev.sys_bp) >= BP_DELTA ||
				u8_absdiff(s.dia_bp, prev.dia_bp) >= BP_DELTA;

			if (changed) {
				publish_result(&s);
			}
		} else if (!have_prev || (s.bp_status != prev.bp_status)) {
			log_bpt_status("estimation", &s);
		}

		prev = s;
		have_prev = true;
		k_msleep(LOOP_MS);
	}
}

static void restore_idle(void)
{
	(void)hub_set_sensor(false);
	(void)hub_set_algo_mode(ALGO_IDX_BPT, ALGO_DISABLE, 40);
	(void)hub_set_algo_mode(ALGO_IDX_AGC, ALGO_DISABLE, 40);
	(void)hub_set_output_mode(HUB_OUT_PAUSE);
	(void)hub_reset_fifo();
	shb_hub_raw_streaming = false;
}

static void shb_max32664_thread(void *p1, void *p2, void *p3)
{
	int err;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	if (!shb_hub_ready) {
		LOG_ERR("MAX32664 thread: hub not ready - exiting");
		shb_max_thread_running = false;
		return;
	}

	if (!shb_hub_vector_loaded) {
		LOG_INF("MAX32664: no saved calibration - starting calibration session");

		err = start_calibration();
		if (err != 0) {
			LOG_ERR("MAX32664: calibration start failed: %d", err);
			restore_idle();
			shb_max_thread_running = false;
			return;
		}

		err = run_calibration_loop(shb_saved_vector);
		if (err != 0) {
			LOG_ERR("MAX32664: calibration loop failed: %d", err);
			restore_idle();
			shb_max_thread_running = false;
			return;
		}

		err = cal_save(shb_saved_vector);
		if (err != 0) {
			LOG_WRN("MAX32664: calibration save failed (%d) - using in-memory vector", err);
		} else {
			LOG_INF("MAX32664: calibration vector saved to flash");
		}
		shb_hub_vector_loaded = true;
	}

	while (true) {
		LOG_INF("MAX32664: starting estimation");

		err = start_estimation(shb_saved_vector);
		if (err != 0) {
			LOG_ERR("MAX32664: estimation start failed: %d", err);
			restore_idle();
			k_msleep(2000);
		} else {
			err = run_estimation_loop();
			if (err == 0) {
				break;
			}
			LOG_WRN("MAX32664: estimation stopped (%d) - retrying", err);
			restore_idle();
			k_msleep(1000);
			if ((hub_enter_runtime_mode() != 0) || (hub_ensure_runtime_mode() != 0)) {
				LOG_ERR("MAX32664: could not recover hub - giving up");
				shb_max_thread_running = false;
				return;
			}
		}
	}

	shb_max_thread_running = false;
}

int shb_max32664_init(void)
{
	int err;
	uint8_t fw[3] = { 0 };
	uint32_t part_id = 0U;

	shb_bp_sys_refs[0] = CONFIG_SHB_MAX32664_BP_SYS_REF_0;
	shb_bp_sys_refs[1] = CONFIG_SHB_MAX32664_BP_SYS_REF_1;
	shb_bp_sys_refs[2] = CONFIG_SHB_MAX32664_BP_SYS_REF_2;
	shb_bp_dia_refs[0] = CONFIG_SHB_MAX32664_BP_DIA_REF_0;
	shb_bp_dia_refs[1] = CONFIG_SHB_MAX32664_BP_DIA_REF_1;
	shb_bp_dia_refs[2] = CONFIG_SHB_MAX32664_BP_DIA_REF_2;

	if (!device_is_ready(shb_hub_i2c)) {
		LOG_ERR("MAX32664: I2C controller not ready");
		return -ENODEV;
	}
	if (!gpio_is_ready_dt(&shb_hub_mfio) || !gpio_is_ready_dt(&shb_hub_rstn)) {
		LOG_ERR("MAX32664: GPIO controller not ready");
		return -ENODEV;
	}

	err = cal_init();
	if (err != 0) {
		LOG_WRN("MAX32664: NVS storage unavailable (%d) - calibration will run", err);
	} else {
		err = cal_load(shb_saved_vector);
		if (err == 0) {
			shb_hub_vector_loaded = true;
			LOG_INF("MAX32664: saved calibration vector loaded from flash");
		} else if (err != -ENOENT) {
			LOG_WRN("MAX32664: saved vector load failed (%d) - will recalibrate", err);
		}
	}

	err = hub_enter_runtime_mode();
	if (err != 0) {
		LOG_ERR("MAX32664: runtime mode entry failed: %d", err);
		return err;
	}

	err = hub_ensure_runtime_mode();
	if (err != 0) {
		LOG_ERR("MAX32664: hub not in runtime mode after reset: %d", err);
		return err;
	}

	err = hub_read_identity(HUB_IDX_FWVERSION, fw, sizeof(fw));
	if (err != 0) {
		LOG_ERR("MAX32664: firmware version read failed: %d", err);
		return err;
	}

	LOG_INF("MAX32664 ready - FW v%u.%u.%u", fw[0], fw[1], fw[2]);

	err = hub_read_sensor_reg(SENSOR_IDX_MAX30101, MAX30101_REG_PART_ID, &part_id);
	if (err == 0) {
		if ((uint8_t)part_id != MAX30101_EXPECTED_ID) {
			LOG_WRN("MAX30101: unexpected part ID 0x%02x", (uint8_t)part_id);
		} else {
			LOG_INF("MAX30101 part ID OK (0x%02x)", MAX30101_EXPECTED_ID);
		}
	} else {
		LOG_WRN("MAX30101: part ID unavailable (%d) - continuing", err);
	}

	shb_hub_ready = true;
	shb_hub_raw_streaming = false;
	memset(&shb_latest, 0, sizeof(shb_latest));
	return 0;
}

int shb_max32664_start_raw_stream(void)
{
	const bool agc = IS_ENABLED(CONFIG_SHB_MAX32664_AGC_ENABLED);
	int err;

	if (!shb_hub_ready) {
		return -ENODEV;
	}

	err = hub_reset_fifo();
	if (err != 0) {
		return err;
	}
	err = hub_set_output_mode(HUB_OUT_SENSOR_ONLY);
	if (err != 0) {
		return err;
	}
	err = hub_set_fifo_threshold(RAW_FIFO_THRESHOLD);
	if (err != 0) {
		return err;
	}
	err = hub_set_algo_mode(ALGO_IDX_AGC, agc ? ALGO_ENABLE : ALGO_DISABLE, 40);
	if ((err != 0) && agc) {
		LOG_WRN("MAX32664 raw stream: AGC enable failed (%d), continuing with AGC off", err);
		err = hub_set_algo_mode(ALGO_IDX_AGC, ALGO_DISABLE, 40);
	}
	if (err != 0) {
		return err;
	}
	err = hub_set_sensor(true);
	if (err != 0) {
		return err;
	}

	k_msleep(FLOW_START_MS);
	shb_hub_raw_streaming = true;
	LOG_INF("MAX32664 raw optical stream started (%s)", agc ? "AGC on" : "AGC off");
	return 0;
}

int shb_max32664_stop(void)
{
	if (!shb_hub_ready) {
		return -ENODEV;
	}

	restore_idle();
	return 0;
}

int shb_max32664_start_session(void)
{
	k_tid_t tid;

	if (!shb_hub_ready) {
		return -ENODEV;
	}
	if (shb_max_thread_running) {
		return 0;
	}

	if (shb_hub_raw_streaming) {
		restore_idle();
	}

	shb_max_thread_running = true;
	tid = k_thread_create(&shb_max32664_td,
			      shb_max32664_stack,
			      K_THREAD_STACK_SIZEOF(shb_max32664_stack),
			      shb_max32664_thread,
			      NULL, NULL, NULL,
			      CONFIG_SHB_MAX32664_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(tid, "max32664");
	return 0;
}

bool shb_max32664_is_ready(void)
{
	return shb_hub_ready;
}

void shb_max32664_get_latest(struct shb_max32664_result *result_out)
{
	if (result_out == NULL) {
		return;
	}

	k_mutex_lock(&shb_result_mtx, K_FOREVER);
	*result_out = shb_latest;
	k_mutex_unlock(&shb_result_mtx);
}
