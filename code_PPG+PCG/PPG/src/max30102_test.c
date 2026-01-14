#include <stdio.h>
#include <inttypes.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <string.h>
#include <math.h>
#include "driver/i2c.h"
#include "max30102.h"

// --- CẤU HÌNH ---
#define I2C_SDA_GPIO  21
#define I2C_SCL_GPIO  22
#define I2C_FREQ_HZ   400000 
#define SAMPLE_RATE   1000   
#define PULSE_WIDTH   118    
#define FFT_N         512    
#define PI            3.1415926535
#define SPO2_WINDOW   2000   

// Ngưỡng để xác định có đặt tay hay không (IR raw value)
// Với Pulse Width 118us, giá trị khi không có tay thường < 50000
// Khi có tay thường > 70000. Bạn có thể chỉnh số này nếu cần.
#define FINGER_THRESHOLD 50000 

// --- BIẾN TOÀN CỤC ---
i2c_dev_t dev;
struct max30102_record record;
QueueHandle_t dataQueue;
float global_spo2 = 0; 

// Cờ báo hiệu reset giữa 2 task
volatile bool reset_request = false;

typedef struct { float ir_for_fft; } ppg_data_t;
typedef struct { float r; float i; } complex_t;

// --- CẤU TRÚC BỘ LỌC IIR ---
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2; 
} IIR_Biquad;

void IIR_Design_Butterworth(IIR_Biquad *f, int type, float fc, float fs) {
    float omega = 2.0f * PI * fc / fs;
    float sn = sinf(omega);
    float cs = cosf(omega);
    float alpha = sn / (2.0f * 0.707f); 
    float a0;
    if (type == 0) { // Low Pass
        f->b0 = (1.0f - cs) / 2.0f; f->b1 = 1.0f - cs; f->b2 = (1.0f - cs) / 2.0f;
        a0 = 1.0f + alpha; f->a1 = -2.0f * cs; f->a2 = 1.0f - alpha;
    } else { // High Pass
        f->b0 = (1.0f + cs) / 2.0f; f->b1 = -(1.0f + cs); f->b2 = (1.0f + cs) / 2.0f;
        a0 = 1.0f + alpha; f->a1 = -2.0f * cs; f->a2 = 1.0f - alpha;
    }
    f->b0 /= a0; f->b1 /= a0; f->b2 /= a0; f->a1 /= a0; f->a2 /= a0;
    f->a1 = -f->a1; f->a2 = -f->a2;
    f->x1 = 0; f->x2 = 0; f->y1 = 0; f->y2 = 0;
}

// Reset bộ lọc về trạng thái ban đầu
void IIR_Reset(IIR_Biquad *f) {
    f->x1 = 0; f->x2 = 0; f->y1 = 0; f->y2 = 0;
}

// Lọc DC
typedef struct { float val; float alpha; } DC_Filter;
float DC_Update(DC_Filter *dc, float x) {
    dc->val = x * (1.0f - dc->alpha) + dc->val * dc->alpha;
    return dc->val;
}

float IIR_Update_B2(IIR_Biquad *f, float x) {
    float y = f->b0*x + f->b1*f->x1 + f->b2*f->x2 + f->a1*f->y1 + f->a2*f->y2;
    f->x2 = f->x1; f->x1 = x; f->y2 = f->y1; f->y1 = y;
    return y;
}

// --- FFT Function ---
void fft_iterative(complex_t *x, int n) {
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j) { complex_t temp = x[i]; x[i] = x[j]; x[j] = temp; }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float ang = 2.0f * PI / len * -1;
        complex_t wlen = { cosf(ang), sinf(ang) };
        for (int i = 0; i < n; i += len) {
            complex_t w = { 1, 0 };
            for (int j = 0; j < len / 2; j++) {
                complex_t u = x[i + j];
                complex_t v = { x[i + j + len / 2].r * w.r - x[i + j + len / 2].i * w.i,
                                x[i + j + len / 2].r * w.i + x[i + j + len / 2].i * w.r };
                x[i + j].r = u.r + v.r; x[i + j].i = u.i + v.i;
                x[i + j + len / 2].r = u.r - v.r; x[i + j + len / 2].i = u.i - v.i;
                float tmp_r = w.r * wlen.r - w.i * wlen.i;
                w.i = w.r * wlen.i + w.i * wlen.r; w.r = tmp_r;
            }
        }
    }
}

// --- TASK 1: SENSOR & FILTER ---
void sensor_task(void *pvParameter) {
    IIR_Biquad ir_hp, red_hp, ir_lp, red_lp;
    IIR_Design_Butterworth(&ir_hp, 1, 0.5f, SAMPLE_RATE);
    IIR_Design_Butterworth(&red_hp, 1, 0.5f, SAMPLE_RATE);
    IIR_Design_Butterworth(&ir_lp, 0, 20.0f, SAMPLE_RATE);
    IIR_Design_Butterworth(&red_lp, 0, 20.0f, SAMPLE_RATE);

    DC_Filter ir_dc_est = {0, 0.999f};
    DC_Filter red_dc_est = {0, 0.999f};

    float sum_ir_rms = 0, sum_red_rms = 0;
    int spo2_cnt = 0;
    float current_spo2 = 0;
    
    int downsample_cnt = 0;
    float downsample_accum = 0;
    
    // Biến trạng thái
    bool finger_present = false;

    memset(&dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(max30102_initDesc(&dev, 0, I2C_SDA_GPIO, I2C_SCL_GPIO));
    max30102_init(0x1F, 4, 3, 1000, 118, 8192, &record, &dev);

    while (1) {
        max30102_check(&record, &dev);
        while (max30102_available(&record)) {
            float ir_raw = (float)max30102_getFIFOIR(&record);
            float red_raw = (float)max30102_getFIFORed(&record);
            max30102_nextSample(&record);

            // --- KIỂM TRA ĐẶT TAY ---
            if (ir_raw < FINGER_THRESHOLD) {
                // Nếu đang có tay mà rút ra -> RESET NGAY LẬP TỨC
                if (finger_present) {
                    finger_present = false;
                    printf(">msg:No Finger - Resetting...\n");
                    
                    // Reset biến nội bộ
                    sum_ir_rms = 0; sum_red_rms = 0; spo2_cnt = 0;
                    current_spo2 = 0; global_spo2 = 0;
                    downsample_accum = 0; downsample_cnt = 0;
                    
                    // Reset bộ lọc để tránh nhiễu "đuôi" khi đặt tay lại
                    IIR_Reset(&ir_hp); IIR_Reset(&red_hp);
                    IIR_Reset(&ir_lp); IIR_Reset(&red_lp);
                    ir_dc_est.val = ir_raw; red_dc_est.val = red_raw; // Gán lại DC

                    // Reset hàng đợi và báo Task FFT
                    xQueueReset(dataQueue); 
                    reset_request = true; // Báo hiệu cho task kia

                    // Cập nhật lên Teleplot ngay về 0
                    printf(">BPM:0\n");
                    printf(">SPO2:0\n");
                    printf(">wave:0\n");
                }
                vTaskDelay(pdMS_TO_TICKS(10)); // Nghỉ chút đỡ tốn CPU
                continue; // Bỏ qua xử lý mẫu này
            } else {
                if (!finger_present) {
                    // Mới đặt tay vào
                    finger_present = true;
                    printf(">msg:Finger Detected!\n");
                }
            }

            // --- XỬ LÝ KHI CÓ TAY ---
            float ir_dc = DC_Update(&ir_dc_est, ir_raw);
            float red_dc = DC_Update(&red_dc_est, red_raw);

            float ir_ac = IIR_Update_B2(&ir_hp, ir_raw);
            float red_ac = IIR_Update_B2(&red_hp, red_raw);

            float ir_smooth = IIR_Update_B2(&ir_lp, ir_ac);
            float red_smooth = IIR_Update_B2(&red_lp, red_ac);

            // Vẽ sóng (Giảm tải in 1/2 mẫu)
            static int print_skip = 0;
            if (print_skip++ >= 1) { 
                printf(">wave:%.2f\n", -ir_smooth); 
                print_skip = 0;
            }

            // Tính SpO2
            sum_ir_rms += ir_smooth * ir_smooth;
            sum_red_rms += red_smooth * red_smooth;
            spo2_cnt++;

            if (spo2_cnt >= SPO2_WINDOW) {
                float R = (sqrtf(sum_red_rms / SPO2_WINDOW) / red_dc) / 
                          (sqrtf(sum_ir_rms / SPO2_WINDOW) / ir_dc);
                float spo2_val = 110.0f - 25.0f * R;
                if (spo2_val > 100) spo2_val = 100;
                if (spo2_val < 0) spo2_val = 0;
                
                if (spo2_val > 60) {
                    current_spo2 = (current_spo2 == 0) ? spo2_val : (current_spo2 * 0.9f + spo2_val * 0.1f);
                    global_spo2 = current_spo2;
                }
                sum_ir_rms = 0; sum_red_rms = 0; spo2_cnt = 0;
            }

            // Downsample gửi sang FFT
            downsample_accum += ir_smooth;
            downsample_cnt++;
            if (downsample_cnt >= 10) { 
                ppg_data_t msg;
                msg.ir_for_fft = downsample_accum / 10.0f;
                xQueueSend(dataQueue, &msg, 0);
                downsample_accum = 0;
                downsample_cnt = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- TASK 2: FFT PROCESSING ---
void processing_task(void *pvParameter) {
    complex_t *fft_buf = malloc(sizeof(complex_t) * FFT_N);
    int buf_idx = 0;
    float last_bpm = 0;
    ppg_data_t recv;
    int effective_sample_rate = SAMPLE_RATE / 10; 

    while (1) {
        // Kiểm tra tín hiệu Reset từ Sensor Task
        if (reset_request) {
            buf_idx = 0; // Xóa sạch bộ đệm FFT cũ
            last_bpm = 0;
            reset_request = false; // Đã xử lý xong
            // Xóa bộ nhớ đệm
            memset(fft_buf, 0, sizeof(complex_t) * FFT_N);
        }

        // Chờ dữ liệu (Timeout 100ms để còn check cờ reset nếu ko có dữ liệu)
        if (xQueueReceive(dataQueue, &recv, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Nếu vừa nhận được dữ liệu mà lại có yêu cầu reset (trường hợp hiếm gặp)
            if (reset_request) { 
                buf_idx = 0; 
                reset_request = false; 
                continue; 
            }

            fft_buf[buf_idx].r = recv.ir_for_fft;
            fft_buf[buf_idx].i = 0;
            buf_idx++;

            // Khi đủ 512 mẫu
            if (buf_idx >= FFT_N) {
                // FFT Process
                for (int i = 0; i < FFT_N; i++) {
                    float w = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_N - 1)));
                    fft_buf[i].r *= w;
                }
                fft_iterative(fft_buf, FFT_N);

                // Find Peak
                int b_min = (int)(0.7f * FFT_N / effective_sample_rate);
                int b_max = (int)(3.5f * FFT_N / effective_sample_rate);
                float max_m = 0; int p_bin = 0;

                for (int i = b_min; i <= b_max; i++) {
                    float m = fft_buf[i].r * fft_buf[i].r + fft_buf[i].i * fft_buf[i].i;
                    if (m > max_m) { max_m = m; p_bin = i; }
                }

                float bpm_raw = p_bin * ((float)effective_sample_rate / FFT_N) * 60.0f;
                
                if (bpm_raw > 40 && bpm_raw < 220) {
                    last_bpm = (last_bpm == 0) ? bpm_raw : (last_bpm * 0.8f + bpm_raw * 0.2f);
                    
                    printf(">BPM:%.1f\n", last_bpm);
                    printf(">SPO2:%.1f\n", global_spo2);

                    printf("******************************\n");
                    printf("* [UPDATE] BPM: %.1f        *\n", last_bpm);
                    printf("* SpO2: %.1f %%             *\n", global_spo2);
                    printf("******************************\n");
                }
                
                buf_idx = 0;
            }
        }
    }
}

void app_main(void) {
    i2cdev_init();
    dataQueue = xQueueCreate(128, sizeof(ppg_data_t));
    xTaskCreatePinnedToCore(sensor_task, "Sensor", 8192, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(processing_task, "FFT", 8192, NULL, 5, NULL, 0);
}