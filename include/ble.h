#ifndef SHB_BLE_H_
#define SHB_BLE_H_

#include <stdbool.h>
#include <stdint.h>

int shb_ble_init(void);
void shb_ble_notify_temperature(int32_t temp_mc);
/*
 * Publish a named event on the gesture characteristic. Expected strings per
 * product spec: "TAP_SINGLE", "TAP_DOUBLE", "ALERT_ACKNOWLEDGED".
 * Caller-provided string is copied — safe to pass string literals or local
 * buffers. Truncates to characteristic buffer length if oversized.
 */
void shb_ble_notify_event(const char *event_name);
void shb_ble_notify_vitals(uint16_t hr_x10, uint16_t spo2_x10,
			    uint8_t sys_bp, uint8_t dia_bp);

#endif /* SHB_BLE_H_ */
