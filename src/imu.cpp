#include "pins.h"
#include "imu.h"
#include <LSM6DSOSensor.h>

#include <SPI.h>

LSM6DSOSensor IMU(&SPI, PIN_IMU_CS, 1000000);
int32_t accelerometer[3];
int32_t gyroscope[3];

bool initIMU() {
    SPI.setCS(PIN_IMU_CS);
    SPI.setTX(PIN_IMU_TX);
    SPI.setRX(PIN_IMU_RX);
    SPI.setSCK(PIN_IMU_SCK);
    SPI.begin();

    int status = IMU.begin();
    if (status != 0)
        return false;

    if (IMU.Enable_G() != 0)
        return false;

    if (IMU.Enable_X() != 0)
        return false;
    return true;
}

bool imuRead(sensor_msgs__msg__Imu *imuMsg) {
    bool success = true;
    success &= IMU.Get_X_Axes(accelerometer) == 0;
    success &= IMU.Get_G_Axes(gyroscope) == 0;

    imuMsg->linear_acceleration.x = accelerometer[0] * 9.81 / 1000.0;
    imuMsg->linear_acceleration.y = accelerometer[1] * 9.81 / 1000.0;
    imuMsg->linear_acceleration.z = accelerometer[2] * 9.81 / 1000.0;

    imuMsg->angular_velocity.x = gyroscope[0] * (PI / 180.0) / 1000.0;
    imuMsg->angular_velocity.y = gyroscope[1] * (PI / 180.0) / 1000.0;
    imuMsg->angular_velocity.z = gyroscope[2] * (PI / 180.0) / 1000.0;

    imuMsg->orientation_covariance[0] = -1; // orientation data is not available

    return success;
}
