#include "max30102.h"
#include "my_i2c.h"
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Biến toàn cục
int irpower = 0,
    rpower = 0,
    lirpower = 0,
    lrpower = 0,
    finger_on_sensor = 0;

float heartrate = 0.0,
      pctspo2 = 100.0,
      meastime;

void max30102_init()
{
    i2c_init_custom();
    uint8_t data;
    data = ( 0x2 << 5);  //sample averaging 0=1,1=2,2=4,3=8,4=16,5+=32
    i2c_write_custom(I2C_ADDR_MAX30102, 0x08, data);
    data = 0x03;                //mode = red and ir samples
    i2c_write_custom(I2C_ADDR_MAX30102, 0x09, data);
    data = ( 0x3 << 5) + ( 0x3 << 2 ) + 0x3; //first and last 0x3, middle smap rate 0=50,1=100,etc
    i2c_write_custom(I2C_ADDR_MAX30102, 0x0a, data);
    
    // --- CHỈNH SỬA QUAN TRỌNG: GIẢM CƯỜNG ĐỘ LED ---
    // Code gốc là 0xd0 và 0xa0 (quá cao, gây sai số trên module rẻ)
    // Sửa thành 0x7F (trung bình) để tín hiệu rõ hơn.
    // Nếu vẫn sai, hãy thử giảm xuống 0x40 hoặc tăng lại 0xA0
    data = 0x7F; // ir pulse power
    i2c_write_custom(I2C_ADDR_MAX30102, 0x0c, data);
    
    data = 0x7F; // red pulse power
    i2c_write_custom(I2C_ADDR_MAX30102, 0x0d, data);
    
    // Cập nhật biến theo dõi để vòng lặp không ghi đè lại ngay lập tức
    irpower = 0x7F;
    rpower = 0x7F;
    lirpower = 0x7F;
    lrpower = 0x7F;
}

void max30102_task (void *pvParameters) {
    int cnt, samp, tcnt = 0;
    uint8_t rptr, wptr;
    uint8_t data;
    uint8_t regdata[256];
    
    // 1. KHỞI TẠO MẢNG BẰNG 0 ĐỂ TRÁNH GIÁ TRỊ RÁC
    float firxv[5] = {0}, firyv[5] = {0}, fredxv[5] = {0}, fredyv[5] = {0};
    
    float lastmeastime = 0;
    float hrarray[10] = {0}, spo2array[10] = {0};
    int hrarraycnt = 0;
    
    while (1) {
      if (lirpower != irpower) {
        data = (uint8_t) irpower;
        i2c_write_custom(I2C_ADDR_MAX30102, 0x0c, data);
        lirpower = irpower;
      }
      if (lrpower != rpower) {
        data = (uint8_t) rpower;
        i2c_write_custom(I2C_ADDR_MAX30102, 0x0d, data);
        lrpower = rpower;
      }
      i2c_read_custom(I2C_ADDR_MAX30102, 0x04, &wptr, 1);
      i2c_read_custom(I2C_ADDR_MAX30102, 0x06, &rptr, 1);

      samp = ((32 + wptr) - rptr) % 32;
      i2c_read_custom(I2C_ADDR_MAX30102, 0x07, regdata, 6 * samp);

      for (cnt = 0; cnt < samp; cnt++) {
        meastime = 0.01 * tcnt++;
        
        firxv[0] = firxv[1]; firxv[1] = firxv[2]; firxv[2] = firxv[3]; firxv[3] = firxv[4];
        firxv[4] = (1 / 3.48311) *
                   (256 * 256 * (regdata[6 * cnt + 0] % 4) + 256 * regdata[6 * cnt + 1] + regdata[6 * cnt + 2]);
        firyv[0] = firyv[1]; firyv[1] = firyv[2]; firyv[2] = firyv[3]; firyv[3] = firyv[4];
        firyv[4] = (firxv[0] + firxv[4]) - 2 * firxv[2]
                   + (-0.1718123813 * firyv[0]) + (0.3686645260 * firyv[1])
                   + (-1.1718123813 * firyv[2]) + (1.9738037992 * firyv[3]);

        fredxv[0] = fredxv[1]; fredxv[1] = fredxv[2]; fredxv[2] = fredxv[3]; fredxv[3] = fredxv[4];
        fredxv[4] = (1 / 3.48311) *
                    (256 * 256 * (regdata[6 * cnt + 3] % 4) + 256 * regdata[6 * cnt + 4] + regdata[6 * cnt + 5]);
        fredyv[0] = fredyv[1]; fredyv[1] = fredyv[2]; fredyv[2] = fredyv[3]; fredyv[3] = fredyv[4];
        fredyv[4] = (fredxv[0] + fredxv[4]) - 2 * fredxv[2]
                    + (-0.1718123813 * fredyv[0]) + (0.3686645260 * fredyv[1])
                    + (-1.1718123813 * fredyv[2]) + (1.9738037992 * fredyv[3]);

        if (-1.0 * firyv[4] >= 100 && -1.0 * firyv[2] > -1 * firyv[0] && -1.0 * firyv[2] > -1 * firyv[4])
        {
          if (meastime - lastmeastime < 0.5)
            continue;
          if (!(finger_on_sensor == 0 && meastime - lastmeastime > 2))
            finger_on_sensor = 1;

          hrarray[hrarraycnt % 5] = 60 / (meastime - lastmeastime);
          
          // 2. SỬA LỖI CHIA CHO 0: Kiểm tra mẫu số khác 0 trước khi chia
          float ac_ratio = 0;
          if (fredxv[4] != 0 && firxv[4] != 0) {
              // R = (AC_Red / DC_Red) / (AC_IR / DC_IR)
              ac_ratio = (fredyv[4] / fredxv[4]) / (firyv[4] / firxv[4]);
              spo2array[hrarraycnt % 5] = 110 - 25 * ac_ratio;
          } else {
              spo2array[hrarraycnt % 5] = 0; // Tín hiệu chưa ổn định
          }

          if (spo2array[hrarraycnt % 5] > 100)
            spo2array[hrarraycnt % 5] = 99.9;

          // In thêm giá trị Ratio (ac_ratio) để debug xem tại sao SpO2 sai
          printf("T=%6.2f | HR=%5.1f | SpO2=%5.1f | Ratio=%.2f\n", 
                 meastime, heartrate, pctspo2, ac_ratio);

          lastmeastime = meastime;
          hrarraycnt++;
          heartrate = (hrarray[0] + hrarray[1] + hrarray[2] + hrarray[3] + hrarray[4]) / 5;
          if (heartrate < 40 || heartrate > 150)
            heartrate = 0;

          pctspo2 = (spo2array[0] + spo2array[1] + spo2array[2] + spo2array[3] + spo2array[4]) / 5;
          if (pctspo2 < 50 || pctspo2 > 101)
            pctspo2 = 0;
        }
        if (heartrate && lastmeastime + 1.8 < 0.01 * tcnt)
          finger_on_sensor = 0;
      }
    }
}