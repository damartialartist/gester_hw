#pragma once

typedef union {
    struct {
        double pitch;
        double roll;
        double gyro_mag;

    } data;
    uint8_t bytes[sizeof(double) * 3];
} sensorMsg;