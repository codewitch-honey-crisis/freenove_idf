#ifndef FREENOVE_S3_DEVKIT_H
#define FREENOVE_S3_DEVKIT_H
#include <stdint.h>
#include <stddef.h>
#include "esp_attr.h"
enum {
    CAM_ALLOC_FB_PSRAM=(1<<0),
    CAM_ALLOC_CAM_PSRAM=(1<<1),
    CAM_FRAME_SIZE_96X96=(1<<2)
};
enum {
    CAM_NO_CHANGE = -3,
    CAM_LOWEST,
    CAM_LOW,
    CAM_MEDIUM,
    CAM_HIGH,
    CAM_HIGHEST
};
// optionally implemented by user: notify when transfer complete
extern IRAM_ATTR void lcd_on_flush_complete() __attribute__((weak));
extern void lcd_initialize(size_t max_transfer_size);
extern void lcd_rotation(int rotation);
extern void lcd_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const void* bitmap);
extern int lcd_wait_flush(uint32_t timeout);
extern void lcd_deinitialize();

extern void led_initialize();
extern void led_enable(int enabled);
extern void led_deinitialize();

extern void camera_initialize(int flags);
extern void camera_levels(int brightness, int contrast,
                   int saturation, int sharpness);
extern void camera_rotation(int rotation);
extern const void* camera_frame_buffer();
extern void camera_deinitialize();
#endif // FREENOVE_S3_DEVKIT_H