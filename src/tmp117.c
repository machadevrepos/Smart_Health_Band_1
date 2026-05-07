#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include "tmp117.h"

LOG_MODULE_REGISTER(shb_tmp117, CONFIG_SHB_LOG_LEVEL);

#define TMP117_REG_TEMP 0x00
#define TMP117_REG_CONFIG 0x01
#define TMP117_REG_DEVICE_ID 0x0F
#define TMP117_DEVICE_ID 0x0117
#define TMP117_TEMP_RESOLUTION_UC 78125

static const struct i2c_dt_spec shb_tmp117 = I2C_DT_SPEC_GET(DT_NODELABEL(tmp117));
static bool shb_tmp117_ready;

static int shb_tmp117_reg_read(uint8_t reg, uint16_t *value)
{
	uint8_t buf[2];
	int ret = i2c_write_read_dt(&shb_tmp117, &reg, sizeof(reg), buf, sizeof(buf));

	if (ret < 0) {
		return ret;
	}

	*value = ((uint16_t)buf[0] << 8) | buf[1];
	return 0;
}

int shb_tmp117_init(void)
{
	uint16_t dev_id = 0U;

	if (!i2c_is_ready_dt(&shb_tmp117)) {
		LOG_ERR("TMP117 bus is not ready");
		return -ENODEV;
	}

	if (shb_tmp117_reg_read(TMP117_REG_DEVICE_ID, &dev_id) < 0) {
		LOG_WRN("TMP117 device ID read failed, continuing with raw temperature reads");
	} else if (dev_id != TMP117_DEVICE_ID) {
		LOG_WRN("TMP117 device ID mismatch: 0x%04x", dev_id);
	}

	shb_tmp117_ready = true;
	LOG_INF("TMP117 ready");
	return 0;
}

int shb_tmp117_read_temperature(struct sensor_value *temperature)
{
	uint16_t raw;
	int16_t signed_raw;
	int32_t temp_uc;
	int ret;

	if ((temperature == NULL) || !shb_tmp117_ready) {
		return -EINVAL;
	}

	ret = shb_tmp117_reg_read(TMP117_REG_TEMP, &raw);
	if (ret < 0) {
		LOG_ERR("TMP117 read failed: %d", ret);
		return ret;
	}

	signed_raw = (int16_t)raw;
	temp_uc = (int32_t)(((int64_t)signed_raw * TMP117_TEMP_RESOLUTION_UC) / 10LL);
	temperature->val1 = temp_uc / 1000000;
	temperature->val2 = temp_uc % 1000000;

	return 0;
}

bool shb_tmp117_is_ready(void)
{
	return shb_tmp117_ready;
}
