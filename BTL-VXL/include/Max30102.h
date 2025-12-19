#ifndef MAX30102_H
#define MAX30102_H

#ifdef __cplusplus
extern "C" {
#endif

// Khai báo biến extern để chia sẻ giữa các file
extern int irpower, rpower, lirpower, lrpower, finger_on_sensor;
extern float heartrate, pctspo2, meastime;

void max30102_init();
void max30102_task(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif