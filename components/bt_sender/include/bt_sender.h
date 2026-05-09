#ifndef BT_SENDER_H
#define BT_SENDER_H

#include <stdbool.h>
#include "sensorMsg.h"

/* Callback signature for subscription events */
typedef void (*ble_subscription_cb_t)(bool is_subscribed);

/**
 * @brief Initialize the NimBLE stack, setup GATT services, and start advertising.
 * @param cb Callback function triggered when a client subscribes/unsubscribes.
 */
void ble_server_init(ble_subscription_cb_t cb);

/**
 * @brief Send the sensor message over BLE to the connected device.
 * @param msg Pointer to the populated sensorMsg structure.
 */
void ble_server_send_sensor_msg(sensorMsg *msg);

#endif // BLE_SERVER_H