#include <stdio.h>
#include "esp_log.h"
#include "bno055.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "sensorMsg.h"
QueueHandle_t sensor_queue;
void app_main(void)
{
    sensor_queue = xQueueCreate(10, sizeof(sensorMsg));

    BNO055Init(sensor_queue);

    while(1);
}
