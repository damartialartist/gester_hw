#include "bno055.h"
#include "sensorMsg.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>

#define I2C_BUS_FREQ_KHZ 100
#define BNO_ADDR 0x28

/* BNO055 quaternion output is in units of 1/16384 (2^14) */
#define QUAT_SCALE (1.0 / 16384.0)

/* Gyro scale: you set unit select bit 1 = rads, so raw / 900 = rad/s */
#define GYRO_SCALE_RPS (1.0 / 900.0)

#define RAD_TO_DEG (180.0 / M_PI)

static const char *TAG = "Imu";

i2c_master_dev_handle_t bno_handle;

QueueHandle_t data_queue;

/* ── Low-level I2C helpers (unchanged) ────────────────────────────────── */

esp_err_t write8(adafruit_bno055_reg_t reg, uint8_t value)
{
    uint8_t buffer[2] = {(uint8_t)reg, value};
    return i2c_master_transmit(bno_handle, buffer, 2, pdMS_TO_TICKS(100));
}

esp_err_t read8(adafruit_bno055_reg_t reg, uint8_t *buffer)
{
    uint8_t reg_addr = (uint8_t)reg;
    return i2c_master_transmit_receive(bno_handle, &reg_addr, 1, buffer, 1, pdMS_TO_TICKS(100));
}

esp_err_t readLen(adafruit_bno055_reg_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t reg_addr = (uint8_t)reg;
    return i2c_master_transmit_receive(bno_handle, &reg_addr, 1, buffer, len, pdMS_TO_TICKS(100));
}

esp_err_t setMode(adafruit_bno055_opmode_t mode)
{
    esp_err_t ret = write8(BNO055_OPR_MODE_ADDR, mode);
    vTaskDelay(pdMS_TO_TICKS(30));
    return ret;
}

/* ── Quaternion + Gyro reading ────────────────────────────────────────── */

esp_err_t bno055_read_quaternion(bno055_quat_t *quat)
{
    uint8_t buf[8];

    /* Registers 0x20–0x27: W_LSB, W_MSB, X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB */
    esp_err_t err = readLen(BNO055_QUATERNION_DATA_W_LSB_ADDR, buf, 8);
    if (err != ESP_OK)
    {
        return err;
    }

    int16_t raw_w = ((int16_t)buf[1] << 8) | buf[0];
    int16_t raw_x = ((int16_t)buf[3] << 8) | buf[2];
    int16_t raw_y = ((int16_t)buf[5] << 8) | buf[4];
    int16_t raw_z = ((int16_t)buf[7] << 8) | buf[6];

    quat->w = raw_w * QUAT_SCALE;
    quat->x = raw_x * QUAT_SCALE;
    quat->y = raw_y * QUAT_SCALE;
    quat->z = raw_z * QUAT_SCALE;

    return ESP_OK;
}

esp_err_t bno055_read_gyroscope(bno055_gyro_t *gyro)
{
    uint8_t buf[6];

    /* Registers 0x14–0x19: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB */
    esp_err_t err = readLen(BNO055_GYRO_DATA_X_LSB_ADDR, buf, 6);
    if (err != ESP_OK)
    {
        return err;
    }

    int16_t raw_x = ((int16_t)buf[1] << 8) | buf[0];
    int16_t raw_y = ((int16_t)buf[3] << 8) | buf[2];
    int16_t raw_z = ((int16_t)buf[5] << 8) | buf[4];

    gyro->x = raw_x * GYRO_SCALE_RPS;
    gyro->y = raw_y * GYRO_SCALE_RPS;
    gyro->z = raw_z * GYRO_SCALE_RPS;

    return ESP_OK;
}

esp_err_t bno055_read_imu_sample(bno055_imu_sample_t *sample)
{
    esp_err_t err;

    err = bno055_read_quaternion(&sample->quat);
    if (err != ESP_OK)
        return err;

    err = bno055_read_gyroscope(&sample->gyro);
    if (err != ESP_OK)
        return err;

    sample->timestamp = xTaskGetTickCount();
    return ESP_OK;
}

/* ── Quaternion math ──────────────────────────────────────────────────── */

bno055_quat_t quat_inverse(bno055_quat_t q)
{
    /* For a unit quaternion, inverse = conjugate */
    bno055_quat_t inv = {
        .w = q.w,
        .x = -q.x,
        .y = -q.y,
        .z = -q.z};
    return inv;
}

bno055_quat_t quat_multiply(bno055_quat_t a, bno055_quat_t b)
{
    bno055_quat_t result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
    return result;
}

void quat_to_pitch_roll(bno055_quat_t q, double *pitch, double *roll)
{
    /*
     * Tait-Bryan angles from quaternion (ZYX convention).
     *
     * pitch = rotation about Y axis (forward/back tilt)
     * roll  = rotation about X axis (left/right tilt)
     *
     * We skip yaw because we use gyro-integrated twist detection
     * for the layer switch, which is immune to magnetometer drift.
     */

    /* Pitch (Y axis) — clamped to avoid NaN at gimbal lock */
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (sinp >= 1.0)
        *pitch = 90.0;
    else if (sinp <= -1.0)
        *pitch = -90.0;
    else
        *pitch = asin(sinp) * RAD_TO_DEG;

    /* Roll (X axis) */
    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    *roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
}

/* ── Init + main read loop ────────────────────────────────────────────── */

int BNO055Init(QueueHandle_t queue)
{
    data_queue = queue;

    /* I2C Bus configuration */
    i2c_master_bus_config_t i2c_master_conf = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_6,
        .scl_io_num = GPIO_NUM_7,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags.enable_internal_pullup = false,
    };
    i2c_master_bus_handle_t i2c_master_bus = NULL;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_conf, &i2c_master_bus));

    /* Configure BNO055 bus behavior */
    i2c_device_config_t bno055_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO_ADDR,
        .scl_speed_hz = I2C_BUS_FREQ_KHZ * 1000,
        .flags.disable_ack_check = 0,
        .scl_wait_us = 0xffff,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus, &bno055_conf, &bno_handle));

    /* Verify BNO055 is on the bus */
    if (i2c_master_probe(i2c_master_bus, 0x28, 100) == ESP_OK)
    {
        ESP_LOGI(TAG, "Found device at 0x28");
    }

    /* Reset */
    vTaskDelay(pdMS_TO_TICKS(400));
    if (write8(BNO055_SYS_TRIGGER_ADDR, 0x20) != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to reset");
        return -1;
    }
    ESP_LOGI(TAG, "RST_SYS successful");

    vTaskDelay(pdMS_TO_TICKS(650));
    if (setMode(OPERATION_MODE_CONFIG) != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to set mode config");
        return -1;
    }
    ESP_LOGI(TAG, "Set mode config");
    vTaskDelay(pdMS_TO_TICKS(7));

    uint8_t chip_id;
    if (read8(BNO055_CHIP_ID_ADDR, &chip_id) != ESP_OK)
    {
        ESP_LOGE(TAG, "Unable to read chip ID");
        return -1;
    }
    ESP_LOGI(TAG, "Chip ID is: 0x%02x", chip_id);

    if (write8(BNO055_PAGE_ID_ADDR, 0x0) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set Page id 0");
        return -1;
    }

    uint8_t unitsel = (0 << 7) | // Orientation = Android
                      (0 << 4) | // Temperature = Celsius
                      (0 << 2) | // Euler = Degrees
                      (1 << 1) | // Gyro = Rads
                      (0 << 0);  // Accelerometer = m/s^2

    if (write8(BNO055_UNIT_SEL_ADDR, unitsel) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to select units");
        return -1;
    }
    ESP_LOGI(TAG, "Unit select successful");

    if (write8(BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P5) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to map axis");
        return -1;
    }
    ESP_LOGI(TAG, "Map axis select successful");

    if (write8(BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P5) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to map sign");
        return -1;
    }
    ESP_LOGI(TAG, "Map sign select successful");

    if (setMode(OPERATION_MODE_IMUPLUS) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set operation mode IMUPLUS");
        return -1;
    }
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "INIT SUCCESSFUL");

    /* ── Capture neutral orientation ────────────────────────────────── */
    /* Give the sensor fusion a moment to stabilize, then snapshot     */
    /* the current orientation as "hand at rest."                       */
    vTaskDelay(pdMS_TO_TICKS(500));

    bno055_quat_t neutral_quat;
    if (bno055_read_quaternion(&neutral_quat) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to capture neutral quaternion");
        return -1;
    }
    ESP_LOGI(TAG, "Neutral captured: w=%.4f x=%.4f y=%.4f z=%.4f",
             neutral_quat.w, neutral_quat.x, neutral_quat.y, neutral_quat.z);

    bno055_quat_t neutral_inv = quat_inverse(neutral_quat);

    /* ── Main read loop ─────────────────────────────────────────────── */

    sensorMsg msg = {0};
    while (1)
    {
        bno055_imu_sample_t sample;
        if (bno055_read_imu_sample(&sample) == ESP_OK)
        {
            /* Compute orientation relative to neutral.
             * relative = inverse(neutral) × current
             * This gives us "how far has the hand tilted from rest?" */
            bno055_quat_t relative = quat_multiply(neutral_inv, sample.quat);

            /* Extract pitch and roll from the relative quaternion.
             * These will be small angles (0–30° typically) so we stay
             * far away from gimbal lock. */
            double raw_pitch, raw_roll;
            quat_to_pitch_roll(relative, &raw_pitch, &raw_roll);

            double pitch = raw_roll;
            double roll = raw_pitch;

            /* Gyro magnitude for flick detection */
            double gyro_mag = sqrt(sample.gyro.x * sample.gyro.x +
                                   sample.gyro.y * sample.gyro.y +
                                   sample.gyro.z * sample.gyro.z) *
                              RAD_TO_DEG; /* convert to deg/s for threshold comparison */

            ESP_LOGI(TAG,
                     "P:%+6.1f  R:%+6.1f, gyro:%5.1f dps  ",
                     pitch, roll, gyro_mag);

            msg.data.gyro_mag = gyro_mag;
            msg.data.pitch = pitch;
            msg.data.roll = roll;

            if (xQueueSend(data_queue, &msg, 0) != pdPASS) {
                ESP_LOGW(TAG, "Queue full, dropping sample");
            }
        }
        else
        {
            ESP_LOGE(TAG, "Failed to read IMU sample");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}