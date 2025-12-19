#include "HeartRateSensor.h"

MAX30105 particleSensor;

uint32_t irBuffer[100]; 
uint32_t redBuffer[100];
int32_t bufferLength = 100;
int32_t spo2;
int8_t validSPO2;
int32_t heartRate;
int8_t validHeartRate;

void setupMAX30102() {
    if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
        Serial.println("MAX30102 not found!");
        while (1);
    }

    byte ledBrightness = 60; 
    byte sampleAverage = 4; 
    byte ledMode = 2; 
    int sampleRate = 100; 
    int pulseWidth = 411; 
    int adcRange = 4096; 

    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
}

void readSensorData() {
    // Đọc 100 mẫu
    for (byte i = 0 ; i < bufferLength ; i++) {
        while (particleSensor.available() == false) particleSensor.check();
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample();
    }
    // Tính toán bằng thuật toán của thư viện
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
}

int32_t getHeartRate() { return heartRate; }
int32_t getSpO2() { return spo2; }
bool isDataValid() { return (validHeartRate == 1 && validSPO2 == 1); }