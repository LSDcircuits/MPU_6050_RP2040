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

//code explanation
// 1. 

//thirngs to improve when returning the code for self and for the code, next part is to make a pid value output, based on what we have main issues ariseed from absolute get time, practice morer absolute get time
// make sure to also go over pointers, not sure why i use double

// unsure operations
// round to decimal, go over this
// gety absolute time
// general code flow 


float roundToOneDecimal(float number) {
    return roundf(number * 10) / 10;
}


// MPU6050 reset, read gyro, find drift functions...
double toRadians(double degrees) {
    return degrees * M_PI / 180.0;
}

//MPU6050 reset
void mpu6050_reset() {
    uint8_t buf[] = {MPU6050_RESET_REG, 0x00};
    i2c_write_blocking(i2c_default, MPU6050_ADDR, buf, 2, false);
}


//gyro sends 16 bytes in 8 bits so bitbanging is needed the I2B is 8 bit
//we take the 8 bits read and rearrange to form a 16 bit array to get a 16 bit buffer
void mpu6050_read_gyro(int16_t gyro[3], float gyro_dps[3]){
    uint8_t buffer[6];
    uint8_t val = MPU6050_GYRO_START_REG;

    //here goes the I2C writing and reading
    // logic (I2CPort, Adress_to_slave, byte_adress_sent, )
    i2c_write_blocking(i2c_default, MPU6050_ADDR, &val, 1, true);
    i2c_read_blocking(i2c_default, MPU6050_ADDR, buffer, 6, false);

    const float gyro_scale = 131.0; // Scale factor for ±250 degrees/s
    for (int i = 0; i < 3; i++) {

        //here goes the bit manipulation to seperate fifo 
        gyro[i] = (buffer[i * 2] << 8) | buffer[i * 2 + 1];
        gyro_dps[i] = gyro[i] / gyro_scale;
    }
}



//here the dirft 
void find_drift(float drift[3]) {
    int16_t gyro[3];
    float gyro_dps[3], sum[3] = {0.0, 0.0, 0.0};
// start sampling
    for (int j = 0; j < NUM_SAMPLES; j++) {
        mpu6050_read_gyro(gyro, gyro_dps);
        //initialize new loop 2D array 
        for (int i = 0; i < 3; i++) {
            sum[i] += gyro_dps[i];
        }
        sleep_ms(1); // Delay between samples
    }
    for (int i = 0; i < 3; i++) {
        drift[i] = sum[i] / NUM_SAMPLES;
    }
}



//go over get absolute time 
void integrate_to_angle(float angle[3], float drift[3]) {
    static absolute_time_t last_time = 0; // Use static to preserve the value across function calls
    int16_t gyro[3];
    float gyro_dps[3];
    // Initialize last_time during the first call
    if (last_time == 0) {
        last_time = get_absolute_time();
    }
    absolute_time_t current_time = get_absolute_time(); // Get the current time
    float time_diff = absolute_time_diff_us(last_time, current_time) / 1e6; // Calculate the time difference in seconds
    last_time = current_time; // Update last_time for the next call

    mpu6050_read_gyro(gyro, gyro_dps); // Read gyro data

    // Correct for drift and integrate to get the angle
    for (int i = 0; i < 3; i++) {
        gyro_dps[i] -= drift[i]; // Apply drift correction
        angle[i] += gyro_dps[i] * time_diff; // Integrate to get the angle
    }
}



// Note: This function assumes angles are given in degrees
void anglesToQuaternion(double roll, double pitch, double yaw, double* qw, double* qx, double* qy, double* qz) {
    double cy = cos(toRadians(yaw) * 0.5);
    double sy = sin(toRadians(yaw) * 0.5);
    double cp = cos(toRadians(pitch) * 0.5);
    double sp = sin(toRadians(pitch) * 0.5);
    double cr = cos(toRadians(roll) * 0.5);
    double sr = sin(toRadians(roll) * 0.5);

    *qw = cr * cp * cy + sr * sp * sy;
    *qx = sr * cp * cy - cr * sp * sy;
    *qy = cr * sp * cy + sr * cp * sy;
    *qz = cr * cp * sy - sr * sp * cy;
}



int main() {
    //setup baudrate & I2C clock frequency
    stdio_init_all();
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
    // Calibrate for drift
    mpu6050_reset();
    int16_t gyro[3];
    float drift[3], angle[3] = {0.0, 0.0, 0.0}; // Initialize the angle array
    find_drift(drift); // Calibrate for drift

    double qw, qx, qy, qz;
    while (1) {
        integrate_to_angle(angle, drift); // Assuming this updates `angle[]` correctly
        anglesToQuaternion(angle[0], angle[1], angle[2], &qw, &qx, &qy, &qz);
        printf("Quaternion: qw = %f, qx = %f, qy = %f, qz = %f\n", qw, qx, qy, qz);
        sleep_ms(100);
    }
    return 0;
}

