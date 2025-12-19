#ifndef MY_I2C_H
#define MY_I2C_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

// Các Define cấu hình I2C từ code gốc
#define I2C_ADDR_MAX30102 0x57
#define i2c_port 0
#define i2c_frequency 800000
#define i2c_gpio_sda 21
#define i2c_gpio_scl 22

// Khai báo hàm
esp_err_t i2c_init_custom(); // Đổi tên nhẹ để tránh trùng hàm hệ thống
int i2c_read_custom(uint8_t chip_addr, uint8_t data_addr, uint8_t *data_rd, size_t len);
int i2c_write_custom(int chip_addr, int data_addr, int wr_data);

#endif