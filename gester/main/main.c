#include <stdio.h>
#include <stdbool.h>
#include "esp_log.h"
#include "bno055.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensorMsg.h"
#include "bt_sender.h"

QueueHandle_t sensor_queue;
bno055_ctx_t *global_bno_ctx = NULL;

/* Callback fired by the BLE task when a client subscribes or unsubscribes */
void on_ble_subscription_change(bool is_subscribed)
{
    if (global_bno_ctx) {
        if (is_subscribed) {
            ESP_LOGI("MAIN", "Waking up I2C bus - Client subscribed!");
        } else {
            ESP_LOGI("MAIN", "Sleeping I2C bus - Client unsubscribed!");
        }
        bno055_set_reading_enabled(global_bno_ctx, is_subscribed);
    }
}

void app_main(void)
{
    // 1. Initialize BLE with our callback
    ble_server_init(on_ble_subscription_change);

    sensor_queue = xQueueCreate(10, sizeof(sensorMsg));

    bno055_config_t bno_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_pin = GPIO_NUM_6,
        .scl_pin = GPIO_NUM_7,
        .queue = sensor_queue
    };

    // 2. Initialize the sensor
    global_bno_ctx = bno055_init(&bno_cfg);

    if (global_bno_ctx != NULL)
    {
        // Start with reading PAUSED (waiting for BLE client)
        bno055_set_reading_enabled(global_bno_ctx, false);
        xTaskCreate(bno055_task, "bno055_task", 4096, global_bno_ctx, 5, NULL);
    }
    else
    {
        ESP_LOGE("MAIN", "BNO055 Init Failed!");
    }

    sensorMsg msg;

    // 3. Main loop: Receive from Queue, Log, and transmit via BLE
    while(1) {
        // Because the BNO task only writes to this queue when subscribed, 
        // this xQueueReceive will safely block indefinitely when idle.
        if (xQueueReceive(sensor_queue, &msg, portMAX_DELAY) == pdTRUE) {
            
            // Optional: Log it
            ESP_LOGI("MAIN", "P: %f R:%f M:%f", msg.data.pitch, msg.data.roll, msg.data.gyro_mag);

            // Send the payload to the connected BLE client
            ble_server_send_sensor_msg(&msg);
        }
    }
}