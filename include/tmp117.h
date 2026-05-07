#ifndef SHB_TMP117_H_
#define SHB_TMP117_H_

#include <stdbool.h>
#include <zephyr/drivers/sensor.h>

int shb_tmp117_init(void);
int shb_tmp117_read_temperature(struct sensor_value *temperature);
bool shb_tmp117_is_ready(void);

#endif
