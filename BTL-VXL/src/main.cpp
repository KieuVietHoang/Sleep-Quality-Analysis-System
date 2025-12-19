#include <Arduino.h>
#include "HeartRateSensor.h"

void setup() {
    Serial.begin(115200);
    setupMAX30102();
    Serial.println("System Ready!");
}

void loop() {
    Serial.println("Đang đo... Vui lòng giữ yên ngón tay.");
    readSensorData();

    if (isDataValid()) {
        Serial.printf("BPM: %d | SpO2: %d%%\n", getHeartRate(), getSpO2());
    } else {
        Serial.println("Dữ liệu không chính xác, thử lại...");
    }
    
    delay(100); 
}