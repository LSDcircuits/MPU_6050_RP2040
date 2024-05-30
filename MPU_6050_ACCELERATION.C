#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include <math.h>

#define MPU6050_ADDR 0x68
#define MPU6050_RESET_REG 0x6B
#define MPU6050_ACCEL_START_REG 0x3B
#define SDA_PIN 4
#define SCL_PIN 5

void mpu6050_reset() {
    uint8_t buf[] = {MPU6050_RESET_REG, 0x00};
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);
}

void mpu6050_read_raw(int16_t accel[3], float acceleration[3], float* roll, float* pitch) {
    uint8_t buffer[14];
    uint8_t val = MPU6050_ACCEL_START_REG;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 14, false);
    // Accelerometer data
    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8) | buffer[(i * 2) + 1];
    }

    const float acc_scale = 1 / 16500.0; // Scale factor for ±2g
    // Calculate acceleration in m/s²
    for (int i = 0; i < 3; i++) {
        acceleration[i] = accel[i] * acc_scale * 9.81;
    }

    // Adjust for potential Z-axis offset
    acceleration[2] = acceleration[2] + 1.1; // Adjust the value as per your sensor's characteristics
    // Calculate Roll and Pitch
    *roll = atan2(acceleration[1], acceleration[2]) * (180.0 / M_PI);
    *pitch = atan2(-acceleration[0], sqrt(acceleration[1] * acceleration[1] + acceleration[2] * acceleration[2])) * (180.0 / M_PI);
}

int main() {
    stdio_init_all();
    // Initialize I2C
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    mpu6050_reset();

    int16_t accel[3];
    float acceleration[3] = {0, 0, 0};
    float roll, pitch;

    while (1) {
        mpu6050_read_raw(accel, acceleration, &roll, &pitch);
        printf("Acceleration (m/s²). X = %f, Y = %f, Z = %f\n", acceleration[0], acceleration[1], acceleration[2]);
        printf("Roll: %f degrees, Pitch: %f degrees\n", roll, pitch);
        sleep_ms(100);
    }
    return 0;
}