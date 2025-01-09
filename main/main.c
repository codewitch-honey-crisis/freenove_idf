#include <stdio.h>
#include <memory.h>
#include <esp_camera.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/gpio_ll.h>

#define PIN_DC 0
#define PIN_CS 47
#define PIN_MOSI 20
#define PIN_CLK 21
#define LCD_SPEED (80*1000*1000)

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

#define LED 2

#define DC_C GPIO.out_w1tc = (1 << PIN_DC);
#define DC_D GPIO.out_w1ts = (1 << PIN_DC);

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

static TaskHandle_t camera_task_handle;  // camera thread task handle
static uint8_t* camera_fb = NULL;
static SemaphoreHandle_t camera_fb_lock = NULL;
static uint8_t camera_rot = 0;
void camera_on_frame();

void camera_copy_rotate(void* bitmap, int rows, int cols) {
    // allocating space for the new rotated image
    const uint16_t* original = (const uint16_t*)bitmap;
    uint16_t* out = (uint16_t*)camera_fb;
    switch (camera_rot) {
        case 1: {
            // rotate 90
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(cols - x - 1) * rows + y] = *(original++);
                }
            }
            break;
        }
        case 2: {
            // rotate 180
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[(rows - y - 1) * cols + (cols - x - 1)] = *(original++);
                }
            }
            break;
        }
        case 3: {
            // rotate 270
            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    out[x * rows + (rows - y - 1)] = *(original++);
                }
            }
            break;
        }
        default:  // case 0:
            memcpy(camera_fb, bitmap, rows * cols * 2);
            break;
    }
}
// camera thread
void camera_task(void* pvParameters) {
    camera_fb_t* fb_buf = NULL;
    while (true) {
        fb_buf = esp_camera_fb_get();
        if (fb_buf != NULL && camera_fb != NULL && fb_buf->buf!=NULL) {
            if (pdTRUE == xSemaphoreTake(camera_fb_lock, 50  )) {
                if (fb_buf != NULL && camera_fb != NULL && fb_buf->buf!=NULL) {
                    memcpy(camera_fb, fb_buf->buf,fb_buf->width*fb_buf->height*2);
                    //camera_copy_rotate(fb_buf->buf, fb_buf->width, fb_buf->height);
                }
                xSemaphoreGive(camera_fb_lock);
            } 
        }
        esp_camera_fb_return(fb_buf);
        
           
    }
}
const void* camera_lock_frame_buffer(bool wait) {
    if (camera_fb != NULL &&
        pdTRUE == xSemaphoreTake(camera_fb_lock, wait ? portMAX_DELAY : 0)) {
        return camera_fb;
    }
    return NULL;
}
void camera_rotation(uint8_t rotation) {
    if (camera_fb == NULL) {
        camera_rot = rotation & 3;
        return;
    }
    xSemaphoreTake(camera_fb_lock, portMAX_DELAY);
    camera_rot = rotation & 3;
    xSemaphoreGive(camera_fb_lock);
}
void camera_unlock_frame_buffer() { xSemaphoreGive(camera_fb_lock); }
void camera_initialize(int flags) {
    if (camera_fb_lock != NULL) {
        return;
    }
    camera_fb_lock = xSemaphoreCreateMutex();
    if (camera_fb_lock == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    camera_fb = (uint8_t*)heap_caps_malloc(
        (CAM_FRAME_SIZE_96X96)?2*96*96:
        2 * 240 * 240, 0 != (flags & CAM_ALLOC_FB_PSRAM) ? MALLOC_CAP_SPIRAM
                                                         : MALLOC_CAP_DMA);
    if (camera_fb == NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
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
    config.xclk_freq_hz = 20 * 1000000;
    config.frame_size = 0 != (flags & CAM_FRAME_SIZE_96X96) ? FRAMESIZE_96X96
                                                            : FRAMESIZE_240X240;
    config.pixel_format = PIXFORMAT_RGB565;
    config.grab_mode = CAMERA_GRAB_LATEST;
    config.fb_location = 0 != (flags & CAM_ALLOC_CAM_PSRAM) ? CAMERA_FB_IN_PSRAM
                                                            : CAMERA_FB_IN_DRAM;
    config.jpeg_quality = 10;
    config.fb_count = CAMERA_FB_IN_PSRAM?6:2;
    ESP_ERROR_CHECK(esp_camera_init(&config));
    sensor_t* s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    s->set_vflip(s, 0);  // flip it back
    // s->set_contrast(s,0);
    s->set_hmirror(s, 1);     // horizontal mirror image
    s->set_brightness(s, 0);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
    xTaskCreate(camera_task, "camera_task", 8 * 1024, NULL, 1,
                &camera_task_handle);
}
void camera_levels(int brightness, int contrast,
                   int saturation, int sharpness) {
    if (camera_fb == NULL) {
        return;
    }
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
    if (camera_task_handle != NULL) {
        vTaskDelete(camera_task_handle);
        camera_task_handle = NULL;
    }
    if (camera_fb != NULL) {
        free(camera_fb);
        camera_fb = NULL;
    }
    if (camera_fb_lock != NULL) {
        vSemaphoreDelete(camera_fb_lock);
    }
    esp_camera_deinit();
}

static volatile int lcd_flushing = 0;
static spi_device_handle_t lcd_spi_handle = NULL;
static IRAM_ATTR void lcd_on_flush_complete() {
    //gpio_set_level(LED,0);
    lcd_flushing = 0;
}
static void lcd_command(uint8_t cmd, const uint8_t* args,
                        size_t len) {
    spi_transaction_t tx;
    memset(&tx, 0, sizeof(tx));
    tx.length = 8;
    tx.tx_buffer = &cmd;
    tx.user = 0;
    spi_device_queue_trans(lcd_spi_handle, &tx,portMAX_DELAY);
    if (len && args) {   
        tx.length = 8 * len;
        tx.tx_buffer = args;
        tx.user = (void*)1;
        spi_device_queue_trans(lcd_spi_handle, &tx,portMAX_DELAY);
    }
}
IRAM_ATTR void lcd_spi_pre_cb(spi_transaction_t* trans) {
    if (((int)trans->user) == 0) {
        DC_C;
        //}
    } else {
        DC_D;
    }
}
IRAM_ATTR void lcd_spi_post_cb(spi_transaction_t* trans) {
    if (((int)trans->user) == 0) {
        DC_D;
    } else {

        if (((int)trans->user) == 2) {
            
            lcd_on_flush_complete();
        }
    }
}
static void lcd_st7789_init() {
    lcd_command(0x01,NULL,0);  // reset
    vTaskDelay(pdMS_TO_TICKS(120));  // Wait for reset to complete
    lcd_command(0x11,NULL,0);  // Sleep out
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x13,NULL,0);  // Normal display mode on
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
    lcd_command(0x21,NULL,0);
    static const uint8_t params16[] = {0x00, 0x00, 0x00, 0xEF};
    lcd_command(0x2A, params16, 4);  // Column address set
    static const uint8_t params17[] = {0x00, 0x00, 0x01, 0x3F};
    lcd_command(0x2B, params17, 4);  // Row address set
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x29,NULL,0);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_command(0x20,NULL,0);
}
static void lcd_set_window(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
    uint8_t args[4];
    args[1] = (x1 & 0xFF);
    args[0] = (x1 >> 8);
    args[3] = (x2 & 0xFF);
    args[2] = (x2 >> 8);
    lcd_command(0x2A, args, 4);
    args[1] = (y1 & 0xFF);
    args[0] = (y1 >> 8);
    args[3] = (y2 & 0xFF);
    args[2] = (y2 >> 8);
    lcd_command(0x2B, args, 4);
    lcd_command(0x2c,NULL,0);
}
static void lcd_write_bitmap(const void* data_in, uint32_t len) {
    if (len) {
        spi_transaction_t tx;
        memset(&tx, 0, sizeof(tx));
        tx.length = 8 * (len*2);
        tx.tx_buffer = data_in;
        tx.user = (void*)2;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, &tx, portMAX_DELAY));
    } else {
        lcd_on_flush_complete();
    }
}
static const bool big_cam = false;
void app_main(void)
{
    gpio_config_t gpio_conf;
    gpio_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_conf.mode = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = (((unsigned long long)1) << PIN_DC);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&gpio_conf);
    gpio_set_direction((gpio_num_t)PIN_CS, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)PIN_CS, 0);

    gpio_set_direction((gpio_num_t)LED, GPIO_MODE_OUTPUT);
    gpio_set_level((gpio_num_t)LED, 0);
    // configure the SPI bus
    const spi_host_device_t host = SPI3_HOST;
    spi_bus_config_t buscfg;
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.sclk_io_num = PIN_CLK;
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = -1;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;    
    // declare enough space for the transfer buffers + 8 bytes SPI DMA overhead
    buscfg.max_transfer_sz = 32768 + 8;
    // Initialize the SPI bus on HSPI (SPI3)
    ESP_ERROR_CHECK(spi_bus_initialize(host, &buscfg, SPI_DMA_CH_AUTO));
    spi_device_interface_config_t dev_cfg;
    memset(&dev_cfg, 0, sizeof(dev_cfg));
    dev_cfg.dummy_bits = 0;
    dev_cfg.queue_size = 14;
    dev_cfg.flags = SPI_DEVICE_NO_DUMMY | SPI_DEVICE_HALFDUPLEX;
    dev_cfg.spics_io_num = PIN_CS;
    dev_cfg.pre_cb = lcd_spi_pre_cb;
    dev_cfg.post_cb = lcd_spi_post_cb;
    dev_cfg.clock_speed_hz = LCD_SPEED;
    dev_cfg.cs_ena_posttrans = 1;
    dev_cfg.cs_ena_pretrans = 1;
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &dev_cfg, &lcd_spi_handle));
    // if we don't configure GPIO 0 after we init the SPI it stays high for some reason
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = (1ULL << PIN_DC);
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_set_level((gpio_num_t)PIN_DC, 0);
    
    lcd_st7789_init();
    
    camera_initialize(big_cam?0:CAM_FRAME_SIZE_96X96);
    uint32_t ms =pdTICKS_TO_MS(xTaskGetTickCount());
    while(true) {
        camera_on_frame();
        if(pdTICKS_TO_MS(xTaskGetTickCount())>=ms+200) {
            ms =pdTICKS_TO_MS(xTaskGetTickCount());
            vTaskDelay(5);
        }
    }
}
void camera_on_frame() {
    const void* bmp=camera_lock_frame_buffer(true);
    if(big_cam) {
        static const size_t size = 240*48;
        for(int y=0;y<240;y+=48) {
            
            lcd_flushing = 1;
            lcd_set_window(0,y,239,y+47);
            lcd_write_bitmap(((const uint16_t*)bmp)+(y*240),size);
            uint32_t ms =pdTICKS_TO_MS(xTaskGetTickCount());
            while(lcd_flushing) {
                portYIELD();
                if(pdTICKS_TO_MS(xTaskGetTickCount())>=ms+200) {
                    ms =pdTICKS_TO_MS(xTaskGetTickCount());
                    vTaskDelay(5);
                }
            }
        }
        camera_unlock_frame_buffer();
    } else {
        static const size_t size = 96*96;        
        lcd_flushing = 1;
        gpio_set_level(LED,1);
        lcd_set_window(0,0,95,95);
        lcd_write_bitmap(bmp,size);
        uint32_t yield_ms =pdTICKS_TO_MS(xTaskGetTickCount());
        uint32_t total_ms=0;
        while(total_ms<1000 && lcd_flushing) {
            portYIELD();
            uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
            if(ms>=yield_ms+200) {
                total_ms+=(ms-yield_ms);
                yield_ms =pdTICKS_TO_MS(xTaskGetTickCount());
                vTaskDelay(5);
            }
        }   
        if(total_ms>=1000) {
            puts("FLUSH TIMEOUT");
        } 
        //uint32_t yield_ms=0,ms = pdTICKS_TO_MS(xTaskGetTickCount());
        // if(ms>=yield_ms+200) {
        //     yield_ms =pdTICKS_TO_MS(xTaskGetTickCount());
        //     vTaskDelay(5);
        // }
        camera_unlock_frame_buffer();
    }
    

}