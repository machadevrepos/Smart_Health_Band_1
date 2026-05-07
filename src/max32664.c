#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "max32664.h"
#include "power.h"

LOG_MODULE_REGISTER(shb_max32664, CONFIG_SHB_LOG_LEVEL);

#define HUB_I2C_NODE DT_NODELABEL(i2c0)
#define USER_NODE DT_PATH(zephyr_user)

BUILD_ASSERT(DT_NODE_EXISTS(HUB_I2C_NODE), "i2c0 node required for MAX32664");
BUILD_ASSERT(DT_NODE_HAS_PROP(USER_NODE, max32664_mfio_gpios),
	     "zephyr,user.max32664-mfio-gpios missing from overlay");

static const struct device *const shb_hub_i2c = DEVICE_DT_GET(HUB_I2C_NODE);
static const struct gpio_dt_spec shb_hub_mfio =
	GPIO_DT_SPEC_GET(USER_NODE, max32664_mfio_gpios);

#define HUB_I2C_ADDR 0x55U
#define HUB_RETRY_COUNT 8
#define HUB_DEFAULT_DELAY_MS 2
#define HUB_APP_BOOT_MS 1100
#define HUB_MFIO_WAKE_LOW_MS 5
#define HUB_POST_WAKE_MS 5

#define HUB_ST_SUCCESS 0x00U
#define HUB_ST_ERR_TRY_AGAIN_UG 0x05U
#define HUB_ST_ERR_TRY_AGAIN_FW 0xFEU

#define HUB_FAM_R_STATUS 0x00U
#define HUB_FAM_W_MODE 0x01U
#define HUB_FAM_R_MODE 0x02U
#define HUB_FAM_W_COMMCHAN 0x10U
#define HUB_FAM_R_OUTPUTFIFO 0x12U
#define HUB_FAM_R_READREG 0x41U
#define HUB_FAM_R_REGATTRIBS 0x42U
#define HUB_FAM_W_SENSORMODE 0x44U
#define HUB_FAM_W_ALGOMODE 0x52U

#define HUB_IDX_STATUS 0x00U
#define HUB_IDX_MODE 0x00U
#define HUB_IDX_OUTPUTMODE 0x00U
#define HUB_IDX_FIFO_THRESHOLD 0x01U
#define HUB_IDX_NUM_SAMPLES 0x00U
#define HUB_IDX_READ_FIFO 0x01U

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
#define BPT_EST 0x02U

#define MAX30101_REG_PART_ID 0xFFU
#define MAX30101_EXPECTED_ID 0x15U

#define PPG_SAMPLE_BYTES 12U
#define BPT_SAMPLE_BYTES 23U
#define FIFO_MAX_SAMPLES 15U
#define FIFO_BUF_SIZE (1U + (FIFO_MAX_SAMPLES * BPT_SAMPLE_BYTES))
#define RAW_FIFO_BUF_SIZE (1U + (FIFO_MAX_SAMPLES * PPG_SAMPLE_BYTES))
#define BPT_FIFO_THRESHOLD 0x01U
#define RAW_FIFO_THRESHOLD 0x01U
#define SESSION_THREAD_PERIOD_MS 100
#define MAX_XPORT_ERRORS 10

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

static bool shb_hub_ready;
static bool shb_hub_raw_streaming;
static bool shb_max_thread_running;
static uint8_t shb_fifo_buf[FIFO_BUF_SIZE];
static uint8_t shb_raw_fifo_buf[RAW_FIFO_BUF_SIZE];

static struct shb_max32664_result shb_latest;
static K_MUTEX_DEFINE(shb_result_mtx);

K_THREAD_STACK_DEFINE(shb_max32664_stack, CONFIG_SHB_MAX32664_THREAD_STACK_SIZE);
static struct k_thread shb_max32664_td;

static void shb_max32664_thread(void *p1, void *p2, void *p3);

static bool hub_status_is_busy(uint8_t status)
{
	return (status == HUB_ST_ERR_TRY_AGAIN_UG) || (status == HUB_ST_ERR_TRY_AGAIN_FW);
}

static void hub_recover_bus(const char *reason)
{
	int err = i2c_recover_bus(shb_hub_i2c);

	if ((err != 0) && (err != -ENOSYS) && (err != -ENOTSUP)) {
		LOG_WRN("I2C recovery failed (%s): %d", reason, err);
	}
}

static int hub_mfio_drive_app_high(void)
{
	if (!gpio_is_ready_dt(&shb_hub_mfio)) {
		return -ENODEV;
	}

	/* GPIO_ACTIVE_LOW in DTS means logical 0 drives the physical line high. */
	return gpio_pin_configure_dt(&shb_hub_mfio, GPIO_OUTPUT_INACTIVE);
}

static int hub_mfio_release_irq_input(void)
{
	if (!gpio_is_ready_dt(&shb_hub_mfio)) {
		return -ENODEV;
	}

	return gpio_pin_configure_dt(&shb_hub_mfio, GPIO_INPUT | GPIO_PULL_UP);
}

static int hub_mfio_wake_pulse(void)
{
	int err;

	if (!gpio_is_ready_dt(&shb_hub_mfio)) {
		return -ENODEV;
	}

	/* MFIO low can wake low-power MAX32664 variants. It is not used as reset. */
	err = gpio_pin_configure_dt(&shb_hub_mfio, GPIO_OUTPUT_ACTIVE);
	if (err != 0) {
		return err;
	}

	k_msleep(HUB_MFIO_WAKE_LOW_MS);
	err = hub_mfio_release_irq_input();
	if (err != 0) {
		return err;
	}

	k_msleep(HUB_POST_WAKE_MS);
	return 0;
}

static int hub_write(const uint8_t *tx, size_t tx_len, int delay_ms, uint8_t *status_out)
{
	int err = 0;

	if ((tx == NULL) || (tx_len == 0U) || (status_out == NULL)) {
		return -EINVAL;
	}

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
		if ((err == 0) && !hub_status_is_busy(*status_out)) {
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
	if ((cmd == NULL) || (cmd_len == 0U) || (rx == NULL) || (rx_len == 0U)) {
		return -EINVAL;
	}

	for (int i = 0; i < HUB_RETRY_COUNT; ++i) {
		int err = i2c_write(shb_hub_i2c, cmd, cmd_len, HUB_I2C_ADDR);

		if (err != 0) {
			hub_recover_bus("read cmd");
			k_msleep(2);
			continue;
		}

		k_msleep(delay_ms);
		err = i2c_read(shb_hub_i2c, rx, rx_len, HUB_I2C_ADDR);
		if ((err == 0) && !hub_status_is_busy(rx[0])) {
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

	if ((count == 0U) || (rx == NULL) || (rx_len < expected)) {
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
		uint8_t *buf = (sample_size == PPG_SAMPLE_BYTES) ? shb_raw_fifo_buf : shb_fifo_buf;
		size_t buf_len = (sample_size == PPG_SAMPLE_BYTES) ? sizeof(shb_raw_fifo_buf) : sizeof(shb_fifo_buf);

		err = hub_read_fifo_samples(to_read, sample_size, buf, buf_len);
		if (err != 0) {
			return err;
		}

		last_off = 1U + ((to_read - 1U) * sample_size);
		memcpy(latest_out, buf + last_off, sample_size);
		total += to_read;

		err = hub_read_num_samples(&avail);
		if (err != 0) {
			return err;
		}
	}

	*read_out = total;
	return 0;
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

static int hub_ensure_runtime_mode(void)
{
	uint8_t mode = 0U;
	int err;

	err = hub_mfio_wake_pulse();
	if (err != 0) {
		LOG_WRN("MFIO wake pulse failed: %d", err);
	}

	err = hub_read_mode(&mode);
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

static uint32_t parse_u24_be(const uint8_t *bytes)
{
	return ((uint32_t)bytes[0] << 16) |
	       ((uint32_t)bytes[1] << 8) |
	       bytes[2];
}

static void parse_raw_ppg_sample(const uint8_t *bytes, struct shb_max32664_raw_sample *sample)
{
	sample->led1_ir = parse_u24_be(&bytes[0]);
	sample->led2_red = parse_u24_be(&bytes[3]);
	sample->led3 = parse_u24_be(&bytes[6]);
	sample->led4 = parse_u24_be(&bytes[9]);
}

static void parse_bpt_sample(const uint8_t *b, struct bpt_sample *s)
{
	s->led1_ir = parse_u24_be(&b[0]);
	s->led2_red = parse_u24_be(&b[3]);
	s->led3 = parse_u24_be(&b[6]);
	s->led4 = parse_u24_be(&b[9]);
	s->bp_status = b[12];
	s->progress = b[13];
	s->hr_x10 = ((uint16_t)b[14] << 8) | b[15];
	s->sys_bp = b[16];
	s->dia_bp = b[17];
	s->spo2_x10 = ((uint16_t)b[18] << 8) | b[19];
	s->r_x1000 = ((uint16_t)b[20] << 8) | b[21];
}

static void update_latest_from_sample(const struct bpt_sample *s)
{
	struct shb_max32664_result result = {
		.valid = ((s->hr_x10 != 0U) || (s->spo2_x10 != 0U) || (s->sys_bp != 0U)),
		.hr_x10 = s->hr_x10,
		.spo2_x10 = s->spo2_x10,
		.sys_bp = s->sys_bp,
		.dia_bp = s->dia_bp,
		.status = s->bp_status,
		.progress = s->progress,
	};

	k_mutex_lock(&shb_result_mtx, K_FOREVER);
	shb_latest = result;
	k_mutex_unlock(&shb_result_mtx);

	if (result.valid) {
		LOG_INF("MAX32664 sample: HR=%u.%u bpm SpO2=%u.%u%% BP=%u/%u status=%u progress=%u",
			(unsigned int)(result.hr_x10 / 10U), (unsigned int)(result.hr_x10 % 10U),
			(unsigned int)(result.spo2_x10 / 10U), (unsigned int)(result.spo2_x10 % 10U),
			(unsigned int)result.sys_bp, (unsigned int)result.dia_bp,
			(unsigned int)result.status, (unsigned int)result.progress);
		shb_power_signal(SHB_POWER_EVENT_MAX32664_VITALS);
	}
}

int shb_max32664_init(void)
{
	uint8_t mode = 0U;
	uint8_t status = 0U;
	uint32_t part_id = 0U;
	int err;

	if (!device_is_ready(shb_hub_i2c)) {
		LOG_ERR("MAX32664 I2C bus is not ready");
		return -ENODEV;
	}

	err = hub_mfio_drive_app_high();
	if (err != 0) {
		LOG_ERR("MAX32664 MFIO configure failed: %d", err);
		return err;
	}

	/* Without a dedicated RSTN GPIO, firmware cannot force the boot sequence.
	 * Wait for the hub to finish power-on/shared-reset application startup.
	 */
	k_msleep(HUB_APP_BOOT_MS);
	hub_recover_bus("startup");
	(void)hub_mfio_release_irq_input();

	err = hub_read_mode(&mode);
	if (err != 0) {
		LOG_ERR("MAX32664 did not respond in application mode: %d", err);
		LOG_ERR("Do not drive BMD-350 P0.21 as hub reset. Power-cycle the board or verify MAX32664 RSTN/MFIO pull-ups.");
		return err;
	}

	if (mode != HUB_MODE_RUNTIME) {
		LOG_ERR("MAX32664 mode is 0x%02x, expected runtime 0x00", mode);
		return -EIO;
	}

	err = hub_read_status(&status);
	if (err != 0) {
		LOG_WRN("MAX32664 status read failed: %d", err);
	} else {
		LOG_INF("MAX32664 hub status: 0x%02x", status);
	}

	/* Sensor-side register read is a useful check that MAX32664D can see MAX30101. */
	err = hub_read_sensor_reg(SENSOR_IDX_MAX30101, MAX30101_REG_PART_ID, &part_id);
	if (err != 0) {
		LOG_WRN("MAX30101 part-id read through MAX32664 failed: %d", err);
	} else {
		LOG_INF("MAX30101 part id through hub: 0x%02x", (unsigned int)(part_id & 0xFFU));
		if ((part_id & 0xFFU) != MAX30101_EXPECTED_ID) {
			LOG_WRN("MAX30101 expected 0x%02x, got 0x%02x",
				MAX30101_EXPECTED_ID, (unsigned int)(part_id & 0xFFU));
		}
	}

	shb_hub_ready = true;
	shb_hub_raw_streaming = false;
	LOG_INF("MAX32664 ready using MFIO-only safe startup");
	return 0;
}

int shb_max32664_start_raw_stream(void)
{
	int err;

	if (!shb_hub_ready) {
		return -ENODEV;
	}

	err = shb_max32664_stop();
	if (err != 0) {
		return err;
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

	err = hub_set_sensor(true);
	if (err != 0) {
		return err;
	}

	shb_hub_raw_streaming = true;
	LOG_INF("MAX32664 raw PPG stream started");
	return 0;
}

int shb_max32664_read_raw_sample(struct shb_max32664_raw_sample *sample_out)
{
	uint8_t latest[PPG_SAMPLE_BYTES];
	uint8_t avail = 0U;
	uint8_t read_count = 0U;
	int err;

	if ((sample_out == NULL) || !shb_hub_ready || !shb_hub_raw_streaming) {
		return -EINVAL;
	}

	err = hub_drain_fifo(PPG_SAMPLE_BYTES, latest, &avail, &read_count);
	if (err != 0) {
		return err;
	}

	parse_raw_ppg_sample(latest, sample_out);
	return 0;
}

int shb_max32664_stop(void)
{
	int first_err = 0;
	int err;

	if (!shb_hub_ready) {
		return -ENODEV;
	}

	err = hub_set_algo_mode(ALGO_IDX_BPT, ALGO_DISABLE, 40);
	if ((err != 0) && (first_err == 0)) {
		first_err = err;
	}

	err = hub_set_algo_mode(ALGO_IDX_AGC, ALGO_DISABLE, 40);
	if ((err != 0) && (first_err == 0)) {
		first_err = err;
	}

	err = hub_set_sensor(false);
	if ((err != 0) && (first_err == 0)) {
		first_err = err;
	}

	err = hub_set_output_mode(HUB_OUT_PAUSE);
	if ((err != 0) && (first_err == 0)) {
		first_err = err;
	}

	shb_hub_raw_streaming = false;
	return first_err;
}

int shb_max32664_start_session(void)
{
	int err;

	if (!shb_hub_ready) {
		return -ENODEV;
	}

	err = shb_max32664_stop();
	if (err != 0) {
		LOG_WRN("MAX32664 stop-before-start returned %d", err);
	}

	err = hub_reset_fifo();
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

	if (IS_ENABLED(CONFIG_SHB_MAX32664_AGC_ENABLED)) {
		err = hub_set_algo_mode(ALGO_IDX_AGC, ALGO_ENABLE, 40);
		if (err != 0) {
			LOG_WRN("MAX32664 AGC enable failed: %d", err);
		}
	}

	err = hub_set_sensor(true);
	if (err != 0) {
		return err;
	}

	/* BPT estimation may report status/progress only until calibration is valid.
	 * Calibration handling will be restored in the next MAX32664-specific pass.
	 */
	err = hub_set_algo_mode(ALGO_IDX_BPT, BPT_EST, 40);
	if (err != 0) {
		return err;
	}

	if (!shb_max_thread_running) {
		shb_max_thread_running = true;
		k_thread_create(&shb_max32664_td, shb_max32664_stack,
				K_THREAD_STACK_SIZEOF(shb_max32664_stack),
				shb_max32664_thread, NULL, NULL, NULL,
				CONFIG_SHB_MAX32664_THREAD_PRIORITY, 0, K_NO_WAIT);
		k_thread_name_set(&shb_max32664_td, "max32664");
	}

	LOG_INF("MAX32664 vitals session started (MFIO-only, no firmware RSTN)");
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

static void shb_max32664_thread(void *p1, void *p2, void *p3)
{
	uint8_t latest[BPT_SAMPLE_BYTES];
	uint8_t avail = 0U;
	uint8_t read_count = 0U;
	int consecutive_errors = 0;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		int err = hub_drain_fifo(BPT_SAMPLE_BYTES, latest, &avail, &read_count);

		if (err == 0) {
			struct bpt_sample s;

			consecutive_errors = 0;
			parse_bpt_sample(latest, &s);
			update_latest_from_sample(&s);
		} else if (err == -EAGAIN) {
			consecutive_errors = 0;
		} else {
			consecutive_errors++;
			LOG_WRN("MAX32664 FIFO read failed: %d", err);
			if (consecutive_errors >= MAX_XPORT_ERRORS) {
				LOG_ERR("MAX32664 repeated transport errors; session remains running but data may be stale");
				consecutive_errors = 0;
			}
		}

		k_msleep(SESSION_THREAD_PERIOD_MS);
	}
}
