#include "bno055.h"
#include "sensorMsg.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <math.h>
#include <stdlib.h>

#define I2C_BUS_FREQ_KHZ 400
#define BNO_ADDR 0x28

/* BNO055 quaternion output is in units of 1/16384 (2^14) */
#define QUAT_SCALE (1.0 / 16384.0)

/* Gyro scale: you set unit select bit 1 = rads, so raw / 900 = rad/s */
#define GYRO_SCALE_RPS (1.0 / 900.0)

#define RAD_TO_DEG (180.0 / M_PI)

static const char *TAG = "Imu";


/* ── Low-level I2C helpers ────────────────────────────────────────────── */

static esp_err_t bno055_write8(bno055_ctx_t *ctx, adafruit_bno055_reg_t reg, uint8_t value)
{
    uint8_t buffer[2] = {(uint8_t)reg, value};
    return i2c_master_transmit(ctx->i2c_dev, buffer, 2, pdMS_TO_TICKS(100));
}

static esp_err_t bno055_readLen(bno055_ctx_t *ctx, adafruit_bno055_reg_t reg, uint8_t *buffer, uint8_t len)
{
    uint8_t reg_addr = (uint8_t)reg;
    return i2c_master_transmit_receive(ctx->i2c_dev, &reg_addr, 1, buffer, len, pdMS_TO_TICKS(100));
}

static esp_err_t bno055_setMode(bno055_ctx_t *ctx, adafruit_bno055_opmode_t mode)
{
    esp_err_t ret = bno055_write8(ctx, BNO055_OPR_MODE_ADDR, mode);
    vTaskDelay(pdMS_TO_TICKS(30));
    return ret;
}

/* ── Quaternion + Gyro reading ────────────────────────────────────────── */

esp_err_t bno055_read_quaternion(bno055_ctx_t *ctx, bno055_quat_t *quat)
{
    uint8_t buf[8];
    esp_err_t err = bno055_readLen(ctx, BNO055_QUATERNION_DATA_W_LSB_ADDR, buf, 8);
    if (err != ESP_OK) return err;

    quat->w = (int16_t)(buf[0] | (buf[1] << 8)) * QUAT_SCALE;
    quat->x = (int16_t)(buf[2] | (buf[3] << 8)) * QUAT_SCALE;
    quat->y = (int16_t)(buf[4] | (buf[5] << 8)) * QUAT_SCALE;
    quat->z = (int16_t)(buf[6] | (buf[7] << 8)) * QUAT_SCALE;

    return ESP_OK;
}

esp_err_t bno055_read_gyroscope(bno055_ctx_t *ctx, bno055_gyro_t *gyro)
{
    uint8_t buf[6];
    esp_err_t err = bno055_readLen(ctx, BNO055_GYRO_DATA_X_LSB_ADDR, buf, 6);
    if (err != ESP_OK) return err;

    gyro->x = (int16_t)(buf[0] | (buf[1] << 8)) * GYRO_SCALE_RPS;
    gyro->y = (int16_t)(buf[2] | (buf[3] << 8)) * GYRO_SCALE_RPS;
    gyro->z = (int16_t)(buf[4] | (buf[5] << 8)) * GYRO_SCALE_RPS;

    return ESP_OK;
}

esp_err_t bno055_read_imu_sample(bno055_ctx_t *ctx, bno055_imu_sample_t *sample)
{
    if (bno055_read_quaternion(ctx, &sample->quat) != ESP_OK) return ESP_FAIL;
    if (bno055_read_gyroscope(ctx, &sample->gyro) != ESP_OK) return ESP_FAIL;

    sample->timestamp = xTaskGetTickCount();
    return ESP_OK;
}

/* ── Quaternion math ──────────────────────────────────────────────────── */

bno055_quat_t quat_inverse(bno055_quat_t q)
{
    bno055_quat_t inv = { .w = q.w, .x = -q.x, .y = -q.y, .z = -q.z };
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
    double sinp = 2.0 * (q.w * q.y - q.z * q.x);
    if (sinp >= 1.0)       *pitch = 90.0;
    else if (sinp <= -1.0) *pitch = -90.0;
    else                   *pitch = asin(sinp) * RAD_TO_DEG;

    double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    *roll = atan2(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
}

/* ── Initialization ───────────────────────────────────────────────────── */

bno055_ctx_t* bno055_init(const bno055_config_t *config)
{
    bno055_ctx_t *ctx = calloc(1, sizeof(bno055_ctx_t));
    if (!ctx) return NULL;

    ctx->data_queue = config->queue;
    ctx->reading_enabled = true;

    /* I2C Bus configuration */
    i2c_master_bus_config_t i2c_master_conf = {
        .i2c_port = config->i2c_port,
        .sda_io_num = config->sda_pin,
        .scl_io_num = config->scl_pin,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .flags.enable_internal_pullup = false,
    };
    
    i2c_master_bus_handle_t i2c_master_bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_conf, &i2c_master_bus));

    i2c_device_config_t bno055_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BNO_ADDR,
        .scl_speed_hz = I2C_BUS_FREQ_KHZ * 1000,
        .scl_wait_us = 0xffff,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_master_bus, &bno055_conf, &ctx->i2c_dev));

    /* Reset and configure */
    vTaskDelay(pdMS_TO_TICKS(400));
    if (bno055_write8(ctx, BNO055_SYS_TRIGGER_ADDR, 0x20) != ESP_OK) goto err;
    vTaskDelay(pdMS_TO_TICKS(650));

    if (bno055_setMode(ctx, OPERATION_MODE_CONFIG) != ESP_OK) goto err;
    vTaskDelay(pdMS_TO_TICKS(7));

    uint8_t chip_id;
    /* Replaced bno055_read8 with bno055_readLen */
    if (bno055_readLen(ctx, BNO055_CHIP_ID_ADDR, &chip_id, 1) != ESP_OK) goto err;
    ESP_LOGI(TAG, "Chip ID is: 0x%02x", chip_id);

    if (bno055_write8(ctx, BNO055_PAGE_ID_ADDR, 0x0) != ESP_OK) goto err;

    uint8_t unitsel = (0 << 7) | (0 << 4) | (0 << 2) | (1 << 1) | (0 << 0);
    if (bno055_write8(ctx, BNO055_UNIT_SEL_ADDR, unitsel) != ESP_OK) goto err;
    if (bno055_write8(ctx, BNO055_AXIS_MAP_CONFIG_ADDR, REMAP_CONFIG_P5) != ESP_OK) goto err;
    if (bno055_write8(ctx, BNO055_AXIS_MAP_SIGN_ADDR, REMAP_SIGN_P5) != ESP_OK) goto err;
    
    if (bno055_setMode(ctx, OPERATION_MODE_NDOF) != ESP_OK) goto err;
    vTaskDelay(pdMS_TO_TICKS(20));

    /* Capture neutral orientation */
    vTaskDelay(pdMS_TO_TICKS(500));
    bno055_quat_t neutral_quat;
    if (bno055_read_quaternion(ctx, &neutral_quat) != ESP_OK) goto err;
    
    ctx->neutral_inv = quat_inverse(neutral_quat);
    ESP_LOGI(TAG, "INIT SUCCESSFUL. Neutral captured.");
    
    return ctx;

err:
    ESP_LOGE(TAG, "Failed to initialize BNO055");
    /* Prevent memory leak by removing the I2C device handle before freeing context */
    if (ctx && ctx->i2c_dev) {
        i2c_master_bus_rm_device(ctx->i2c_dev); 
    }
    free(ctx);
    return NULL;
}

void bno055_set_reading_enabled(bno055_ctx_t *ctx, bool enable)
{
    if (ctx) {
        ctx->reading_enabled = enable;
    }
}

/* ── FreeRTOS Task (Main read loop) ───────────────────────────────────── */

void bno055_task(void *pvParameters)
{
    bno055_ctx_t *ctx = (bno055_ctx_t *)pvParameters;
    if (!ctx) {
        vTaskDelete(NULL);
        return;
    }

    sensorMsg msg = {0};
    bno055_imu_sample_t sample;

    /* --- Auto-Recenter Variables --- */
    int stationary_counter = 0;
    const int STATIONARY_THRESHOLD = 100;        /* 100 ticks @ 20ms = 2.0 seconds */
    const double GYRO_STILL_THRESHOLD_DPS = 10.0; /* Rotation under 2 deg/sec is considered "still" */

    while (1)
    {
        if (ctx->reading_enabled)
        {
            if (bno055_read_imu_sample(ctx, &sample) == ESP_OK)
            {
                /* 1. Calculate Gyro Magnitude first to check for activity */
                msg.data.gyro_mag = sqrt(sample.gyro.x * sample.gyro.x +
                                         sample.gyro.y * sample.gyro.y +
                                         sample.gyro.z * sample.gyro.z) * RAD_TO_DEG;

                /* 2. Auto-Recenter Logic */
                if (msg.data.gyro_mag < GYRO_STILL_THRESHOLD_DPS)
                {
                    stationary_counter++;
                    
                    if (stationary_counter == STATIONARY_THRESHOLD) 
                    {
                        /* We just hit the 2-second mark. Log it once. */
                        ESP_LOGI(TAG, "Inactivity detected. Auto-recentering zero point.");
                        ctx->neutral_inv = quat_inverse(sample.quat);
                    }
                    else if (stationary_counter > STATIONARY_THRESHOLD)
                    {
                        /* We are STILL inactive. Continuously update neutral_inv to 
                           silently absorb any slow IMU sensor drift while resting. */
                        ctx->neutral_inv = quat_inverse(sample.quat);
                        stationary_counter = STATIONARY_THRESHOLD + 1; /* Prevent integer overflow */
                    }
                }
                else
                {
                    /* Movement detected! Reset the stationary counter. */
                    stationary_counter = 0;
                }

                /* 3. Calculate Relative Orientation (using the potentially updated neutral_inv) */
                bno055_quat_t relative = quat_multiply(ctx->neutral_inv, sample.quat);

                /* 4. Convert to Pitch/Roll */
                quat_to_pitch_roll(relative, &msg.data.roll, &msg.data.pitch);

                /* 5. Send to Queue */
                if (xQueueSend(ctx->data_queue, &msg, 0) != pdPASS) {
                    ESP_LOGW(TAG, "Queue full, dropping sample");
                }
            }
            else
            {
                ESP_LOGE(TAG, "Failed to read IMU sample");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30)); /* Run at ~30Hz */
    }
}