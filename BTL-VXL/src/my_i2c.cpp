#include "my_i2c.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Các define nội bộ của I2C
#define I2C_MASTER_TX_BUF_DISABLE   0                       
#define I2C_MASTER_RX_BUF_DISABLE   0                       
#define WRITE_BIT                   I2C_MASTER_WRITE          
#define READ_BIT                    I2C_MASTER_READ          
#define ACK_CHECK_EN                0x1                 
#define ACK_CHECK_DIS               0x0               
#define ACK_VAL                     0x0                    
#define NACK_VAL                    0x1                  
#define LAST_NACK_VAL               0x2  

// Logic giữ nguyên từ code gốc
esp_err_t i2c_init_custom()
{   
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = i2c_gpio_sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = i2c_gpio_scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = i2c_frequency;
    
    // Sửa lỗi biên dịch C++: ép kiểu enum
    return i2c_param_config((i2c_port_t)i2c_port, &conf);
}

int i2c_read_custom(uint8_t chip_addr, uint8_t data_addr, uint8_t *data_rd, size_t len)
{ 
    i2c_init_custom();
    vTaskDelay(1);
    i2c_driver_install((i2c_port_t)i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, 
                       I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (len > 1) {
        i2c_master_read(cmd, data_rd, len - 1, (i2c_ack_type_t)ACK_VAL);
    }
    i2c_master_read_byte(cmd, data_rd + len - 1, (i2c_ack_type_t)NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    // Phần log giữ nguyên nhưng comment lại để đỡ spam nếu cần
    if (ret == ESP_OK) { 
         //for(int i=0; i<len; i++){ printf ("%d %x\n", i, data_rd[i]); }
    }
    else if (ret == ESP_ERR_TIMEOUT) { ESP_LOGW("", "Bus is busy");}
    else { ESP_LOGW("", "Read failed"); }
    
    i2c_driver_delete((i2c_port_t)i2c_port);
    vTaskDelay(2);
    return 0;
}

int i2c_write_custom(int chip_addr, int data_addr, int wr_data)
{
    i2c_init_custom();
    i2c_driver_install((i2c_port_t)i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, 
                       I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data_addr, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, wr_data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin((i2c_port_t)i2c_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    
    if (ret == ESP_OK) { //ESP_LOGI("", "Write OK addr %x  data %x\n", chip_addr, wr_data);
    } else if (ret == ESP_ERR_TIMEOUT) { ESP_LOGW("", "Bus is busy");
    } else { ESP_LOGW("", "Write Failed"); }
    
    i2c_driver_delete((i2c_port_t)i2c_port);
    return 0;
}