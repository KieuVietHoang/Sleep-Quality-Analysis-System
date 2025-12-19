#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "max30102.h"

extern "C" void app_main()
{
    // configure max30102 with i2c instructions
    max30102_init();

    // Loại bỏ display_init() như yêu cầu

    // Tạo Task
    xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 5, NULL);
    
    // Loại bỏ Task draw_data vì không dùng màn hình
}