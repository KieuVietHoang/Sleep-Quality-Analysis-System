#ifndef HEART_RATE_SENSOR_H
#define HEART_RATE_SENSOR_H

#include <Arduino.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

// Các hàm public
void setupMAX30102();
void readSensorData();

// Getter để lấy giá trị ở main
int32_t getHeartRate();
int32_t getSpO2();
bool isDataValid();

#endif