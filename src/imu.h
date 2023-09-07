#pragma once

#include <sensor_msgs/msg/imu.h>

bool initIMU();
bool imuRead(sensor_msgs__msg__Imu *imuMsg);