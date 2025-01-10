#include "freenove_s3_devkit.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/gpio.h>
#include <driver/i2s_std.h>
#include <driver/spi_master.h>
#include <esp_camera.h>
#include <hal/gpio_ll.h>
// TODO: Update the following when the camera component is updated
#include <driver/i2c.h>
#include <memory.h>
#define LCD_DC 0
#define LCD_CS 47
#define LCD_MOSI 20
#define LCD_CLK 21
#define LCD_SPEED (80 * 1000 * 1000)

#define LED 2

#define CAM_SIOD 4
#define CAM_SIOC 5
#define CAM_VSYNC 6
#define CAM_HREF 7
#define CAM_XCLK 15
#define CAM_Y9 16
#define CAM_Y8 17
#define CAM_Y7 18
#define CAM_Y6 10
#define CAM_Y5 10
#define CAM_Y4 8
#define CAM_Y3 9
#define CAM_Y2 11
#define CAM_PCLK 13
#define CAM_PWDN -1
#define CAM_RST -1
#define CAM_SPEED (20 * 1000 * 1000)

#define NEOPIXEL 48

#define I2C_SCL 1
#define I2C_SDA 2

#define DC_C GPIO.out_w1tc = (1 << LCD_DC);
#define DC_D GPIO.out_w1ts = (1 << LCD_DC);

#define LED_OFF GPIO.out_w1tc = (1 << LED);
#define LED_ON GPIO.out_w1ts = (1 << LED);

static spi_device_handle_t lcd_spi_handle = NULL;
static spi_transaction_t lcd_trans[14];
static size_t lcd_trans_index = 0;
static int lcd_rot = 0;
static volatile int lcd_flushing = 0;

static void lcd_command(uint8_t cmd, const uint8_t* args, size_t len) {
    spi_transaction_t* tx = &lcd_trans[lcd_trans_index++];
    if (lcd_trans_index > 13) lcd_trans_index = 0;
    tx->length = 8;
    tx->tx_data[0] = cmd;
    tx->user = (void*)0;
    tx->flags = SPI_TRANS_USE_TXDATA;
    spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY);
    if (len && args) {
        tx = &lcd_trans[lcd_trans_index++];
        if (lcd_trans_index > 13) lcd_trans_index = 0;
        tx->length = 8 * len;
        if (len <= 4) {
            memcpy(tx->tx_data, args, len);
            tx->flags = SPI_TRANS_USE_TXDATA;
        } else {
            tx->tx_buffer = args;
            tx->flags = 0;
        }
        tx->user = (void*)1;
        spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY);
    }
}
IRAM_ATTR static void lcd_spi_pre_cb(spi_transaction_t* trans) {
    if (((int)trans->user) == 0) {
        DC_C;
    }
}
IRAM_ATTR static void lcd_spi_post_cb(spi_transaction_t* trans) {
    lcd_flushing = 0;
    if (((int)trans->user) == 0) {
        DC_D;
    } else {
        if (((int)trans->user) == 2) {
            lcd_on_flush_complete();
        }
    }
}
static void lcd_st7789_init() {
    lcd_command(0x01, NULL, 0);      // reset
    vTaskDelay(pdMS_TO_TICKS(120));  // Wait for reset to complete
    lcd_command(0x11, NULL, 0);      // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x13, NULL, 0);  // Normal display mode on
    static const uint8_t params1 = 0x08;
    lcd_command(0x36, &params1, 1);
    static const uint8_t params2[] = {0x0A, 0xB2};
    lcd_command(0xB6, params2, 2);
    static const uint8_t params3[] = {0x00, 0xE0};
    lcd_command(0xB0, params3, 2);
    static const uint8_t params4 = 0x55;
    lcd_command(0x3A, &params4, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    static const uint8_t params5[] = {0x0C, 0xC, 0x00, 0x33, 0x33};
    lcd_command(0xB2, params5, 5);
    static const uint8_t params6 = 0x35;
    lcd_command(0xB7, &params6, 1);  // Voltages: VGH / VGL
    static const uint8_t params7 = 0x28;
    lcd_command(0xBB, &params7, 1);
    static const uint8_t params8 = 0x0C;
    lcd_command(0xC0, &params8, 1);
    static const uint8_t params9[] = {0x01, 0xFF};
    lcd_command(0xC2, params9, 2);
    static const uint8_t params10 = 0x10;
    lcd_command(0xC3, &params10, 1);  // voltage VRHS
    static const uint8_t params11 = 0x20;
    lcd_command(0xC4, &params11, 1);
    static const uint8_t params12 = 0x0F;
    lcd_command(0xC6, &params12, 1);
    static const uint8_t params13[] = {0xA4, 0xA1};
    lcd_command(0xD0, params13, 2);
    static const uint8_t params14[] = {0xD0, 0x00, 0x02, 0x07, 0x0A,
                                       0x28, 0x32, 0x44, 0x42, 0x06,
                                       0x0E, 0x12, 0x14, 0x17};
    lcd_command(0xE0, params14, 14);
    static const uint8_t params15[] = {0xD0, 0x00, 0x02, 0x07, 0x0A,
                                       0x28, 0x31, 0x54, 0x47, 0x0E,
                                       0x1C, 0x17, 0x1B, 0x1E};
    lcd_command(0xE1, params15, 14);
    lcd_command(0x21, NULL, 0);
    static const uint8_t params16[] = {0x00, 0x00, 0x00, 0xEF};
    lcd_command(0x2A, params16, 4);  // Column address set
    static const uint8_t params17[] = {0x00, 0x00, 0x01, 0x3F};
    lcd_command(0x2B, params17, 4);  // Row address set
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x29, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x20, NULL, 0);
}
static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint8_t args[4];
    args[0] = (x1 >> 8);
    args[1] = (x1 & 0xFF);
    args[2] = (x2 >> 8);
    args[3] = (x2 & 0xFF);
    lcd_command(0x2A, args, 4);
    args[0] = (y1 >> 8);
    args[1] = (y1 & 0xFF);
    args[2] = (y2 >> 8);
    args[3] = (y2 & 0xFF);
    lcd_command(0x2B, args, 4);
}

static void lcd_write_bitmap(const void* data_in, uint32_t len) {
    if (len) {
        spi_transaction_t* tx = &lcd_trans[lcd_trans_index++];
        if (lcd_trans_index > 13) lcd_trans_index = 0;
        tx->user = (void*)0;
        tx->flags = SPI_TRANS_USE_TXDATA;
        tx->tx_data[0] = 0x2C;  // RAMWR
        tx->length = 8;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY));

        tx = &lcd_trans[lcd_trans_index++];
        if (lcd_trans_index > 13) lcd_trans_index = 0;
        tx->flags = 0;
        tx->length = 8 * (len * 2);
        tx->tx_buffer = data_in;
        tx->user = (void*)2;
        lcd_flushing = 1;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY));
    } else {
        lcd_on_flush_complete();
    }
}

int lcd_wait_flush(uint32_t timeout) {
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t total_ms = 0;
    while ((timeout == 0 || total_ms <= timeout) && lcd_flushing > 0) {
        taskYIELD();
        uint32_t new_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        total_ms += (new_ms - ms);
        ms = new_ms;
    }
    return timeout == 0 || total_ms < timeout;
}
void lcd_on_flush_complete() {}
void lcd_rotation(int rotation) {
    uint8_t param;
    switch (rotation & 3) {
        case 1:
            param = (0x40 | 0x20 | 0x08);
            break;
        case 2:
            param = (0x40 | 0x80 | 0x08);
            break;
        case 3:
            param = (0x20 | 0x80 | 0x08);
            break;
        default:  // case 0:
            param = (0x08);
            break;
    };
    lcd_command(0x36, &param, 1);
    lcd_rot = rotation;
}

void lcd_initialize(size_t max_transfer_size) {
    if (lcd_spi_handle != NULL) {
        return;
    }
    memset(lcd_trans, 0, sizeof(spi_transaction_t) * 14);
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (((unsigned long long)1) << LCD_DC);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gpio_conf);
    gpio_set_direction((gpio_num_t)LCD_CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LCD_CS, 0);

    // configure the SPI bus
    const spi_host_device_t host = SPI3_HOST;
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = LCD_CLK;
    buscfg.mosi_io_num = LCD_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;
    // declare enough space for the transfer buffers + 8 bytes SPI DMA overhead
    buscfg.max_transfer_sz = max_transfer_size + 8;
    // Initialize the SPI bus on HSPI (SPI3)
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev_cfg;
    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.dummy_bits = 0;
    dev_cfg.queue_size = 14;
    dev_cfg.flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX;
    dev_cfg.spics_io_num = LCD_CS;
    dev_cfg.pre_cb = lcd_spi_pre_cb;
    dev_cfg.post_cb = lcd_spi_post_cb;
    dev_cfg.clock_speed_hz = LCD_SPEED;
    dev_cfg.cs_ena_posttrans = 1;
    dev_cfg.cs_ena_pretrans = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev_cfg, &lcd_spi_handle));
    ESP_ERROR_CHECK(spi_device_acquire_bus(lcd_spi_handle, portMAX_DELAY));
    // if we don't configure GPIO 0 after we init the SPI it stays high for some
    // reason
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = (1ULL << LCD_DC);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)LCD_DC, 0);

    lcd_st7789_init();
}
void lcd_deinitialize() {
    if (lcd_spi_handle == NULL) {
        return;
    }
    spi_device_release_bus(lcd_spi_handle);
    ESP_ERROR_CHECK(spi_bus_remove_device(lcd_spi_handle));
    ESP_ERROR_CHECK(spi_bus_free(SPI3_HOST));
    lcd_spi_handle = NULL;
}
void lcd_flush(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2,
               const void* bitmap) {
    lcd_set_window(x1, y1, x2, y2);
    int w = x2 - x1 + 1, h = y2 - y1 + 1;
    lcd_write_bitmap(bitmap, w * h);
}
static int led_initialized = 0;
void led_enable(int value) {
    if(!led_initialized) {
        return;
    }
    if(value==0) {
        LED_OFF;
    } else {
        LED_ON;
    }
}
static int i2c_initialized=0;
void led_initialize() {
    if(led_initialized || i2c_initialized) {
        return;
    }
    gpio_set_direction((gpio_num_t)LED, GPIO_MODE_OUTPUT);
    LED_OFF;
    led_initialized = true;
}
void led_deinitialize() { LED_OFF; led_initialized=false; }

static void* camera_fb = NULL;
static int camera_rot = 0;
static int camera_initialized = 0;
static int camera_flags = 0;
static camera_fb_t* camera_current_fb = NULL;
void camera_rotation(int rotation) { camera_rot = rotation & 3; }
static void camera_copy_rotate(const void* bitmap, int rows, int cols) {
    // allocating space for the new rotated image
    const uint16_t* original = (const uint16_t*)bitmap;
    uint16_t* out = (uint16_t*)camera_fb;
    size_t count;
    switch (camera_rot) {
        case 1:
            // rotate 90
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(cols - x - 1) * rows + y] = *(original++);
                }
            }
            break;
        case 2:
            // rotate 180
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(rows - y - 1) * cols + (cols - x - 1)] = *(original++);
                }
            }
            break;
        case 3:
            // rotate 270
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[x * rows + (rows - y - 1)] = *(original++);
                }
            }
            break;
        default:  // case 0:
            count = rows * cols;
            while (count--) {
                *(out++) = *(original++);
            }
            break;
    }
}
const void* camera_frame_buffer() {
    if (!camera_initialized) {
        return NULL;
    }
    camera_current_fb = esp_camera_fb_get();
    if (camera_current_fb != NULL) {
        camera_copy_rotate(camera_current_fb->buf,
                           (camera_flags & CAM_FRAME_SIZE_96X96) ? 96 : 240,
                           (camera_flags & CAM_FRAME_SIZE_96X96) ? 96 : 240);
        esp_camera_fb_return(camera_current_fb);
        return camera_fb;
    }
    esp_camera_fb_return(camera_current_fb);
    return NULL;
}

void camera_initialize(int flags) {
    if (camera_initialized) {
        return;
    }
    camera_flags = flags;
    camera_config_t config;
    memset(&config, 0, sizeof(config));
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = CAM_Y2;
    config.pin_d1 = CAM_Y3;
    config.pin_d2 = CAM_Y4;
    config.pin_d3 = CAM_Y5;
    config.pin_d4 = CAM_Y6;
    config.pin_d5 = CAM_Y7;
    config.pin_d6 = CAM_Y8;
    config.pin_d7 = CAM_Y9;
    config.pin_xclk = CAM_XCLK;
    config.pin_pclk = CAM_PCLK;
    config.pin_vsync = CAM_VSYNC;
    config.pin_href = CAM_HREF;
    config.pin_sccb_sda = CAM_SIOD;
    config.pin_sccb_scl = CAM_SIOC;
    config.pin_pwdn = CAM_PWDN;
    config.pin_reset = CAM_RST;
    config.xclk_freq_hz = CAM_SPEED;
    config.frame_size = 0 != (flags & CAM_FRAME_SIZE_96X96) ? FRAMESIZE_96X96
                                                            : FRAMESIZE_240X240;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = 0 != (flags & CAM_ALLOC_CAM_PSRAM) ? CAMERA_FB_IN_PSRAM
                                                            : CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 1;
    config.fb_count = CAMERA_FB_IN_PSRAM ? 6 : 2;
    ESP_ERROR_CHECK(esp_camera_init(&config));
    sensor_t* s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    s->set_vflip(s, 0);       // flip it back
    s->set_hmirror(s, 1);     // horizontal mirror image
    s->set_brightness(s, 0);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
    camera_initialized = 1;
    const size_t camera_size =
        (flags & CAM_FRAME_SIZE_96X96) ? 96 * 96 * 2 : 240 * 240 * 2;
    camera_fb = heap_caps_malloc(camera_size, (flags & CAM_ALLOC_FB_PSRAM)
                                                  ? MALLOC_CAP_SPIRAM
                                                  : MALLOC_CAP_DEFAULT);
    if (camera_fb == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
}
void camera_levels(int brightness, int contrast, int saturation,
                   int sharpness) {
    sensor_t* s = esp_camera_sensor_get();
    if (brightness != CAM_NO_CHANGE) {
        s->set_brightness(s, (int)brightness);
    }
    if (contrast != CAM_NO_CHANGE) {
        s->set_contrast(s, (int)contrast);
    }
    if (saturation != CAM_NO_CHANGE) {
        s->set_saturation(s, (int)saturation);
    }
    if (sharpness != CAM_NO_CHANGE) {
        s->set_sharpness(s, (int)sharpness);
    }
}
void camera_deinitialize() {
    if (!camera_initialized) {
        return;
    }
    camera_current_fb = NULL;
    camera_initialized = 0;
    esp_camera_deinit();
    if (camera_fb != NULL) {
        free(camera_fb);
        camera_fb = NULL;
    }
}

static i2s_chan_handle_t neopixel_handle=NULL;
static uint8_t neopixel_out_buffer[12] = {0};
static uint8_t neopixel_zero_buffer[48] = {0};

static const uint16_t neopixel_bit_patterns[4] = {0x88, 0x8e, 0xe8, 0xee};

void neopixel_color(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t red = r, green = g, blue = b;

    neopixel_out_buffer[0] = neopixel_bit_patterns[green >> 6 & 0x03];
    neopixel_out_buffer[1] = neopixel_bit_patterns[green >> 4 & 0x03];
    neopixel_out_buffer[2] = neopixel_bit_patterns[green >> 2 & 0x03];
    neopixel_out_buffer[3] = neopixel_bit_patterns[green & 0x03];

    neopixel_out_buffer[4] = neopixel_bit_patterns[red >> 6 & 0x03];
    neopixel_out_buffer[5] = neopixel_bit_patterns[red >> 4 & 0x03];
    neopixel_out_buffer[6] = neopixel_bit_patterns[red >> 2 & 0x03];
    neopixel_out_buffer[7] = neopixel_bit_patterns[red & 0x03];

    neopixel_out_buffer[8] = neopixel_bit_patterns[blue >> 6 & 0x03];
    neopixel_out_buffer[9] = neopixel_bit_patterns[blue >> 4 & 0x03];
    neopixel_out_buffer[10] = neopixel_bit_patterns[blue >> 2 & 0x03];
    neopixel_out_buffer[11] = neopixel_bit_patterns[blue & 0x03];
    i2s_channel_enable(neopixel_handle);
    i2s_channel_write(neopixel_handle, neopixel_out_buffer, sizeof(neopixel_out_buffer), NULL, portMAX_DELAY);
    i2s_channel_write(neopixel_handle, neopixel_zero_buffer, sizeof(neopixel_zero_buffer), NULL, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(10));
    i2s_channel_disable(neopixel_handle);
}

void neopixel_initialize() {
    if(neopixel_handle!=NULL) {
        return;
    }

    i2s_chan_config_t chan_cfg;
    memset(&chan_cfg,0,sizeof(chan_cfg));
    chan_cfg.id = I2S_NUM_1;
    chan_cfg.role = I2S_ROLE_MASTER;
    chan_cfg.dma_desc_num = 4;
    chan_cfg.dma_frame_num = sizeof(neopixel_out_buffer);
    chan_cfg.auto_clear = 0;
    chan_cfg.intr_priority = 0;

    i2s_new_channel(&chan_cfg, &neopixel_handle, NULL);


    i2s_std_config_t std_cfg;
    memset(&std_cfg,0,sizeof(std_cfg));
    std_cfg.clk_cfg.sample_rate_hz = 93650;
    std_cfg.clk_cfg.clk_src = I2S_CLK_SRC_PLL_160M;
    std_cfg.clk_cfg.mclk_multiple = 128;
    std_cfg.gpio_cfg.bclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.ws = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.din = I2S_GPIO_UNUSED;
    std_cfg.gpio_cfg.dout = NEOPIXEL;
    std_cfg.slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.slot_bit_width = 0;
    std_cfg.slot_cfg.slot_mode = I2S_SLOT_MODE_STEREO;
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH;
    std_cfg.slot_cfg.ws_width = I2S_SLOT_BIT_WIDTH_16BIT;
    std_cfg.slot_cfg.ws_pol = 0;
    std_cfg.slot_cfg.bit_shift = 0;
    std_cfg.slot_cfg.left_align = 1;
    std_cfg.slot_cfg.big_endian = 0;
    std_cfg.slot_cfg.bit_order_lsb = 0;
    i2s_channel_init_std_mode(neopixel_handle, &std_cfg);
}

void neopixel_deinitialize() {
    if(neopixel_handle==NULL) {
        return;
    }
    i2s_channel_disable(neopixel_handle);
    i2s_del_channel(neopixel_handle);
    neopixel_handle = NULL;
};

static void i2c_initialize() {
    if(led_initialized||i2c_initialized) {
        return;
    }
    i2c_config_t config;
    memset(&config,0,sizeof(config));
    config.master.clk_speed = 100*1000;
    config.mode = I2C_MODE_MASTER;
    config.scl_io_num = I2C_SCL;
    config.sda_io_num = I2C_SDA;
    config.scl_pullup_en = 1;
    config.sda_pullup_en = 1;
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0,&config));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0,I2C_MODE_MASTER,0,0,0));
    i2c_initialized = 1;
}
// static void i2c_deinitialize() {
//     if(!i2c_initialized) {
//         return;
//     }
//     ESP_ERROR_CHECK(i2c_driver_delete(I2C_NUM_0));
//     i2c_initialized = 0;
// }

static int touch_rot = 0;
static int touch_initialized = 0;
static uint32_t touch_timestamp = 0;
static size_t touch_count = 0;
static uint16_t touch_x_data[2], touch_y_data[2], touch_id_data[2];
static int touch_write_reg(int r, int value) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(ESP_OK!=i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_write_byte(cmd, 0x38 << 1 | I2C_MASTER_WRITE, I2C_MASTER_ACK)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    uint8_t data[2];
    data[0]=r;
    data[1]=value;
    if(ESP_OK!=i2c_master_write(cmd,data,sizeof(data),I2C_MASTER_ACK)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_cmd_begin(I2C_NUM_0,cmd,portMAX_DELAY)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    i2c_cmd_link_delete(cmd);
    return 1;
}
static int touch_read_all() {
    static const uint8_t ACK_CHECK_EN = 0x1;
    uint8_t i2cdat[16];
        // Read data
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if(ESP_OK!=i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_write_byte(cmd, (0x38<<1), ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_write_byte(cmd, 0, ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS( 1000))) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    i2c_cmd_link_delete(cmd);
    
    cmd = i2c_cmd_link_create();
    if(ESP_OK!=i2c_master_start(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_write_byte(cmd, (0x38<<1)|1, ACK_CHECK_EN)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_read(cmd, i2cdat, sizeof(i2cdat),  I2C_MASTER_LAST_NACK)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_stop(cmd)) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    if(ESP_OK!=i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(1000))) {
        i2c_cmd_link_delete(cmd);
        return 0;
    }
    i2c_cmd_link_delete(cmd);

    touch_count = i2cdat[0x02];
    if (touch_count > 2) {
        touch_count = 0;
    }

    for (uint8_t i = 0; i < 2; i++) {
        touch_x_data[i] = i2cdat[0x03 + i * 6] & 0x0F;
        touch_x_data[i] <<= 8;
        touch_x_data[i] |= i2cdat[0x04 + i * 6];
        touch_y_data[i] = i2cdat[0x05 + i * 6] & 0x0F;
        touch_y_data[i] <<= 8;
        touch_y_data[i] |= i2cdat[0x06 + i * 6];
        touch_id_data[i] = i2cdat[0x05 + i * 6] >> 4;
    }
    return 1;
}
void touch_initialize(int threshhold) {
    if(touch_initialized) {
        return;
    }
    i2c_initialize();
    if(0==touch_write_reg(0x80, threshhold)) {
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    
    touch_count = 0;
    touch_initialized = 1;
}
void touch_deinitialize() {
    if(!touch_initialized) {
        return;
    }
    // nothing for now
    touch_initialized = 0;
}
void touch_rotation(int rotation) {
    touch_rot = rotation&3;
}
static int touch_update() {
    if(!touch_initialized) {
        return 0;
    }
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    if(ms>touch_timestamp+13) {
        if(!touch_read_all()) {
            return 0;
        }
        touch_timestamp = ms;
    }
    return 1;
}
static void touch_translate(uint16_t* x, uint16_t* y) {
    uint16_t tmp;
    switch (touch_rot) {
        case 1:
            tmp = *x;
            *x = *y;
            *y = 240 - tmp - 1;
            break;
        case 2:
            *x = 240 - *x - 1;
            *y = 320 - *y - 1;
            break;
        case 3:
            tmp = *x;
            *x = 320 - *y - 1;
            *y = tmp;
        default:
            break;
    }
}
static int touch_read_point(size_t n, uint16_t* out_x, uint16_t* out_y) {
    if (touch_count == 0 || n >= touch_count) {
        if (out_x != NULL) {
            *out_x = 0;
        }
        if (out_y != NULL) {
            *out_y = 0;
        }
        return 0;
    }
    uint16_t x = touch_x_data[n];
    uint16_t y = touch_y_data[n];
    if (x >= 240) {
        x = 240 - 1;
    }
    if (y >= 320) {
        y = 320 - 1;
    }
    touch_translate(&x, &y);
    if (out_x != NULL) {
        *out_x = x;
    }
    if (out_y != NULL) {
        *out_y = y;
    }
    return 1;
}
int touch_xy(uint16_t* out_x, uint16_t* out_y) {
    if(!touch_update()) {
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    return touch_read_point(0,out_x,out_y);
}
int touch_xy2(uint16_t* out_x, uint16_t* out_y) {
    if(!touch_update()) {
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    return touch_read_point(1,out_x,out_y);
}
