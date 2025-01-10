#ifndef FREENOVE_S3_DEVKIT_H
#define FREENOVE_S3_DEVKIT_H
#include <stdint.h>
#include <stddef.h>
#include "esp_attr.h"
enum {
    CAM_DEFAULT = 0,
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
enum {
    TOUCH_THRESH_DEFAULT = 32
};
enum {
    AUDIO_44_1K_STEREO=0,
    AUDIO_44_1K_MONO,
    AUDIO_22K_STEREO,
    AUDIO_22K_MONO,
    AUDIO_11K_STEREO,
    AUDIO_11K_MONO,
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

extern void neopixel_initialize();
extern void neopixel_color(uint8_t r, uint8_t g, uint8_t b);
extern void neopixel_deinitialize();

extern void touch_initialize(int threshhold);
extern void touch_rotation(int rotation);
extern int touch_xy(uint16_t* out_x, uint16_t* out_y);
extern int touch_xy2(uint16_t* out_x, uint16_t* out_y);
extern void touch_deinitialize();

extern const size_t audio_max_samples;
extern void audio_initialize(int format);
extern void audio_deinitialize();
extern size_t audio_write_int16(const int16_t* samples, size_t sample_count);
extern size_t audio_write_float(const float* samples, size_t sample_count, float vel);

#endif // FREENOVE_S3_DEVKIT_H