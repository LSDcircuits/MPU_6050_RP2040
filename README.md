Overview
This project demonstrates how to interface an MPU6050 sensor with an RP2040 microcontroller using bitbanging I2C protocol. The goal is to gather data from the MPU6050, such as acceleration and gyroscope readings, and process this data for inertial navigation applications.
Files
1. MPU_6050_ACCELERATION.C
2. MPU_6050_CALIBRATED_DEG_per_SEC.c
3. MPU_6050_DEGREES_CALIBRATED.C
4. MPU_6050_QUATERNION.c
Description

1. MPU_6050_ACCELERATION.C
* Purpose: This file reads raw acceleration data from the MPU6050 and computes the acceleration in m/s? along with roll and pitch angles.
* Key Functions:
* mpu6050_reset(): Resets the MPU6050.
* mpu6050_read_raw(): Reads raw acceleration data, calculates the acceleration in m/s?, and computes roll and pitch angles.
* main(): Initializes I2C, reads acceleration data continuously, and prints the acceleration and angles.

2. MPU_6050_CALIBRATED_DEG_per_SEC.c
* Purpose: This file handles the calibration of the gyroscope data to correct for drift and calculates angular velocity in degrees per second.
* Key Functions:
* find_drift(): Calibrates the gyroscope to find the drift.
* mpu6050_read_gyro(): Reads raw gyroscope data and computes the angular velocity in degrees per second.
* integrate_to_angle(): Integrates angular velocity to find the angle.
* main(): Initializes I2C, calibrates the gyroscope, and prints the angular velocity continuously.

3. MPU_6050_DEGREES_CALIBRATED.C
* Purpose: This file combines accelerometer and gyroscope data to compute the device's orientation in degrees.
* Key Functions:
* mpu6050_read_data(): Reads both accelerometer and gyroscope data.
* calculate_angles(): Calculates the orientation angles from the sensor data.
* main(): Initializes I2C, reads sensor data continuously, and prints the calculated angles.







4. MPU_6050_QUATERNION.c
* Purpose: This file converts the orientation angles into quaternions for more efficient computation in 3D space.
* Key Functions:
* anglesToQuaternion(): Converts roll, pitch, and yaw angles into a quaternion.
* main(): Initializes I2C, continuously computes quaternions from sensor data, and prints the quaternion values.



Usage
1. Hardware Setup:
* Connect the SDA and SCL pins of the MPU6050 to the corresponding I2C pins on the RP2040 (default: SDA_PIN=4, SCL_PIN=5).
* Ensure pull-up resistors are connected to the SDA and SCL lines.

2. Software Setup:
* Install the Pico SDK and configure your development environment.
* Clone this repository and navigate to the project directory.

3. Compiling and Running:
* Compile the code using the Pico SDK make system.
* Upload the compiled binary to the RP2040.
* Monitor the serial output to observe the sensor data.




Conclusion
This project serves as a foundational step towards building more complex inertial navigation systems using the RP2040 and MPU6050 sensor. By mastering the basics of reading and processing sensor data, future enhancements can be made to improve the performance and accuracy of the navigation system.
