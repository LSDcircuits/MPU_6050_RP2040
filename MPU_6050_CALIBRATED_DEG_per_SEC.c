#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/time.h"
#include <math.h>
#define MPU6050_ADDR 0x68
#define MPU6050_RESET_REG 0x6B
#define MPU6050_GYRO_START_REG 0x43
#define SDA_PIN 4
#define SCL_PIN 5
#define NUM_SAMPLES 3000

float roundToOneDecimal(float number) {
    return roundf(number * 10) / 10;
}
void mpu6050_reset() {
    uint8_t buf[] = {MPU6050_RESET_REG, 0x00};
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);
}
void mpu6050_read_gyro(int16_t gyro[3], float gyro_dps[3]) {
    uint8_t buffer[6];
    uint8_t val = MPU6050_GYRO_START_REG;
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, false);
    const float gyro_scale = 131.0; // Scale factor for ±250 degrees/s
    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        gyro_dps[i] = gyro[i] / gyro_scale;
    }
}
void find_drift(float drift[3]) {
    int16_t gyro[3];
    float gyro_dps[3], sum[3] = {0.0, 0.0, 0.0};

    for (int j = 0; j < NUM_SAMPLES; j++) {
        mpu6050_read_gyro(gyro, gyro_dps);
        for (int i = 0; i < 3; i++) {
            sum[i] += gyro_dps[i];
        }
        sleep_ms(1); // Delay between samples
    }
    for (int i = 0; i < 3; i++) {
        drift[i] = sum[i] / NUM_SAMPLES;
    }
}
int main() {
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    mpu6050_reset();

    int16_t gyro[3];
    float gyro_dps[3], drift[3];
    // Calibrate for drift
    find_drift(drift);

    while (1) {
        mpu6050_read_gyro(gyro, gyro_dps);

        // Correct for drift
        for (int i = 0; i < 3; i++) {
            gyro_dps[i] = roundToOneDecimal(gyro_dps[i] - drift[i]);
        }

        printf("Corrected Gyro (degrees/s). X = %f, Y = %f, Z = %f\n", gyro_dps[0], gyro_dps[1], gyro_dps[2]);
        sleep_ms(100);
    }

    return 0;
}