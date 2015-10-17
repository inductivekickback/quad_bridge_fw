#ifndef __BLE_CTL_H
#define __BLE_CTL_H

#define MIN_CONN_INTERVAL               (24UL) // Min conn interval (in units of 1.25ms)
#define MAX_CONN_INTERVAL               (24UL) // Max conn interval (in units of 1.25ms)
#define SLAVE_LATENCY                   (0UL)

/**
 * Inits the BLE interface for the Syma library.
 */
void ble_ctl_init(void);

/**
 * Returns true if the BLE module is currently connected.
 */
bool ble_ctl_connected(void);

#endif
