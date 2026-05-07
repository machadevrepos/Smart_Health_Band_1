#ifndef SHB_MAX32664_H_
#define SHB_MAX32664_H_

#include <stdbool.h>
#include <stdint.h>

struct shb_max32664_result {
	bool valid;
	uint16_t hr_x10;
	uint16_t spo2_x10;
	uint8_t sys_bp;
	uint8_t dia_bp;
	uint8_t status;
	uint8_t progress;
};

int shb_max32664_init(void);
int shb_max32664_start_session(void);
bool shb_max32664_is_ready(void);
void shb_max32664_get_latest(struct shb_max32664_result *result_out);

#endif
