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
enum {
    //PROX_SENS_SAMPLEAVG_MASK = ~0b11100000,
    PROX_SENS_SAMPLEAVG_1 = 0x00,
    PROX_SENS_SAMPLEAVG_2 = 0x20,
    PROX_SENS_SAMPLEAVG_4 = 0x40,
    PROX_SENS_SAMPLEAVG_8 = 0x60,
    PROX_SENS_SAMPLEAVG_16 = 0x80,
    PROX_SENS_SAMPLEAVG_32 = 0xA0,
    PROX_SENS_SAMPLEAVG_DEFAULT=PROX_SENS_SAMPLEAVG_4
};
enum {
    //PROX_SENS_ADCRANGE_MASK = 0x9F,
    PROX_SENS_ADCRANGE_2048 = 0x00,
    PROX_SENS_ADCRANGE_4096 = 0x20,
    PROX_SENS_ADCRANGE_8192 = 0x40,
    PROX_SENS_ADCRANGE_16384 = 0x60,
    PROX_SENS_ADCRANGE_DEFAULT = PROX_SENS_ADCRANGE_4096
};
enum {
    //PROX_SENS_SAMPLERATE_MASK = 0xE3,
    PROX_SENS_SAMPLERATE_50 = 0x00,
    PROX_SENS_SAMPLERATE_100 = 0x04,
    PROX_SENS_SAMPLERATE_200 = 0x08,
    PROX_SENS_SAMPLERATE_400 = 0x0C,
    PROX_SENS_SAMPLERATE_800 = 0x10,
    PROX_SENS_SAMPLERATE_1000 = 0x14,
    PROX_SENS_SAMPLERATE_1600 = 0x18,
    PROX_SENS_SAMPLERATE_3200 = 0x1C,
    PROX_SENS_SAMPLERATE_DEFAULT = PROX_SENS_SAMPLERATE_400
};
enum {
    //PROX_SENS_PULSEWIDTH_MASK = 0xFC,
    PROX_SENS_PULSEWIDTH_69 = 0x00,
    PROX_SENS_PULSEWIDTH_118 = 0x01,
    PROX_SENS_PULSEWIDTH_215 = 0x02,
    PROX_SENS_PULSEWIDTH_411 = 0x03,
    PROX_SENS_PULSEWIDTH_DEFAULT = PROX_SENS_PULSEWIDTH_411
};
enum {
    // Multi-LED Mode configuration (pg 22)
    //PROX_SENS_SLOT1_MASK = 0xF8,
    PROX_SENS_SLOT2_MASK = 0x8F,
    PROX_SENS_SLOT3_MASK = 0xF8,
    PROX_SENS_SLOT4_MASK = 0x8F
};
enum {
    PROX_SENS_SLOT_NONE = 0x00,
    PROX_SENS_SLOT_RED_LED = 0x01,
    PROX_SENS_SLOT_IR_LED = 0x02,
    PROX_SENS_SLOT_GREEN_LED = 0x03,
    PROX_SENS_SLOT_NONE_PILOT = 0x04,
    PROX_SENS_SLOT_RED_PILOT = 0x05,
    PROX_SENS_SLOT_IR_PILOT = 0x06,
    PROX_SENS_SLOT_GREEN_PILOT = 0x07
};
enum {
    PROX_SENS_MODE_REDONLY = 	0x02,
    PROX_SENS_MODE_REDIRONLY = 	0x03,
    PROX_SENS_MODE_MULTILED = 	0x07,
    PROX_SENS_MODE_DEFAULT = PROX_SENS_MODE_MULTILED
};
enum {
    PROX_SENS_THRESH_NO_CHANGE = -1
};
enum {
    PROX_SENS_TIMEOUT_DEFAULT = 250
};
enum {
    // 0.4mA - Presence detection of ~4 inch
    PROX_SENS_AMP_0_4MA = 0x02,
    // 6.4mA - Presence detection of ~8 inch
    PROX_SENS_AMP_6_4MA = 0x1F,
    // 25.4mA - Presence detection of ~8 inch
    PROX_SENS_AMP_25_4MA = 0x7F,
    // 50.0mA - Presence detection of ~12 inch
    PROX_SENS_AMP_50MA = 0xFF,
    // Default is 6.4mA
    PROX_SENS_AMP_DEFAULT = PROX_SENS_AMP_6_4MA
};
// optionally implemented by user: notify when transfer complete
extern IRAM_ATTR void lcd_on_flush_complete(void) __attribute__((weak));
extern void lcd_initialize(size_t max_transfer_size);
extern void lcd_rotation(int rotation);
extern void lcd_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2, const void* bitmap);
extern int lcd_wait_flush(uint32_t timeout);
extern void lcd_deinitialize(void);

extern void led_initialize(void);
extern void led_enable(int enabled);
extern void led_deinitialize(void);

extern void camera_initialize(int flags);
extern void camera_levels(int brightness, int contrast,
                   int saturation, int sharpness);
extern void camera_rotation(int rotation);
extern const void* camera_frame_buffer(void);
extern void camera_deinitialize(void);

extern void neopixel_initialize(void);
extern void neopixel_color(uint8_t r, uint8_t g, uint8_t b);
extern void neopixel_deinitialize(void);

extern void touch_initialize(int threshhold);
extern void touch_rotation(int rotation);
extern int touch_xy(uint16_t* out_x, uint16_t* out_y);
extern int touch_xy2(uint16_t* out_x, uint16_t* out_y);
extern void touch_deinitialize(void);

extern const size_t audio_max_samples;
extern void audio_initialize(int format);
extern void audio_deinitialize(void);
extern size_t audio_write_int16(const int16_t* samples, size_t sample_count);
extern size_t audio_write_float(const float* samples, size_t sample_count, float vel);

extern void prox_sensor_initialize(void);
extern void prox_sensor_deinitialize(void);
extern void prox_sensor_configure(uint8_t powerLevel, uint8_t sampleAverage, uint8_t mode ,
           uint8_t sampleRate, uint8_t pulseWidth , uint8_t adcRange);
extern int prox_sensor_read_raw(uint32_t* out_red, uint32_t* out_ir, uint32_t* out_green, uint32_t timeout);
extern void prox_sensor_pulse_amp_threshold(int16_t red, int16_t ir, int16_t green, int16_t prox,int16_t thresh);
extern void prox_sensor_configure2(uint8_t powerLevel, uint8_t sampleAverage , uint8_t ledMode ,
           int sampleRate , int pulseWidth , int adcRange);
#endif // FREENOVE_S3_DEVKIT_H