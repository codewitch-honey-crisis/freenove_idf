#include <stdio.h>
#include <memory.h>
#include <esp_camera.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/gpio_ll.h>
#include <esp_task_wdt.h>
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

void camera_on_frame();

// void camera_copy_rotate(void* bitmap, int rows, int cols) {
//     // allocating space for the new rotated image
//     const uint16_t* original = (const uint16_t*)bitmap;
//     uint16_t* out = (uint16_t*)camera_fb;
//     size_t count;
//     switch (camera_rot) {
//         case 1: 
//             // rotate 90
//             for (int y = 0; y < rows; ++y) {
//                 for (int x = 0; x < cols; ++x) {
//                     out[(cols - x - 1) * rows + y] = *(original++);
//                 }
//             }
//             break;
//         case 2: 
//             // rotate 180
//             for (int y = 0; y < rows; ++y) {
//                 for (int x = 0; x < cols; ++x) {
//                     out[(rows - y - 1) * cols + (cols - x - 1)] = *(original++);
//                 }
//             }
//             break;
//         case 3: 
//             // rotate 270
//             for (int y = 0; y < rows; ++y) {
//                 for (int x = 0; x < cols; ++x) {
//                     out[x * rows + (rows - y - 1)] = *(original++);
//                 }
//             }
//             break;
//         default:  // case 0:
//             count = rows*cols;
//             while(count--) {
//                 *(out++)=*(original++);
//             }
//             break;
//     }
// }

static bool camera_initialized = false;
static camera_fb_t* camera_current_fb = NULL;
const void* camera_lock_frame_buffer() {
    if(!camera_initialized) {
        return NULL;
    }
    camera_current_fb=esp_camera_fb_get();
    if(camera_current_fb!=NULL) {
        return camera_current_fb->buf;
    }
    return NULL;
}

void camera_unlock_frame_buffer() { 
    if(!camera_initialized) {
        return;
    }
    esp_camera_fb_return(camera_current_fb);
    camera_current_fb = NULL;
}
void camera_initialize(int flags) {
    if(camera_initialized) {
        return;
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
    config.jpeg_quality = 1;
    config.fb_count = CAMERA_FB_IN_PSRAM?6:2;
    ESP_ERROR_CHECK(esp_camera_init(&config));
    sensor_t* s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    s->set_vflip(s, 0);  // flip it back
    s->set_hmirror(s, 1);     // horizontal mirror image
    s->set_brightness(s, 0);  // up the brightness just a bit
    s->set_saturation(s, 0);  // lower the saturation
    camera_initialized = true;
}
void camera_levels(int brightness, int contrast,
                   int saturation, int sharpness) {
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
    if(!camera_initialized) {
        return;
    }
    camera_current_fb = NULL;
    camera_initialized=false;
    esp_camera_deinit();
}
static const size_t lcd_transfer_buffer_size = 32760;//240*32*2;
static void* lcd_transfer_buffer1=NULL;
static void* lcd_transfer_buffer2=NULL;
static volatile int lcd_dma_sel = 0;
static volatile int lcd_flushing = 0;
static spi_device_handle_t lcd_spi_handle = NULL;
static void* lcd_transfer_buffer() {
    return lcd_dma_sel?lcd_transfer_buffer1:lcd_transfer_buffer2;    
}

static bool lcd_wait_dma(uint32_t timeout) { 
    uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
    uint32_t total_ms = 0;
    while ((timeout==0 || total_ms<=timeout ) && lcd_flushing > 0) {
        taskYIELD();
        uint32_t new_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        total_ms += (new_ms - ms);
        ms=new_ms;
    }
    return timeout==0 || total_ms<timeout;
}
static void lcd_switch_buffers() {
    if(lcd_dma_sel==0) {
        lcd_dma_sel = 1;
    } else {
        lcd_dma_sel = 0;
    }
}
static IRAM_ATTR void lcd_on_flush_complete() {
    
}
static spi_transaction_t lcd_trans[14];
static size_t lcd_trans_index = 0;
static void lcd_command(uint8_t cmd, const uint8_t* args,
                        size_t len) {
    spi_transaction_t* tx=&lcd_trans[lcd_trans_index++]; if(lcd_trans_index>13) lcd_trans_index=0;
    tx->length = 8;
    tx->tx_data[0] = cmd;
    tx->user = (void*)0;
    tx->flags = SPI_TRANS_USE_TXDATA;
    spi_device_queue_trans(lcd_spi_handle, tx,portMAX_DELAY);
    if (len && args) {   
        tx=&lcd_trans[lcd_trans_index++]; if(lcd_trans_index>13) lcd_trans_index=0;
        tx->length = 8 * len;
        if(len<=4) {
            memcpy(tx->tx_data,args,len);
            tx->flags = SPI_TRANS_USE_TXDATA;
        } else {
            tx->tx_buffer = args;
            tx->flags = 0;
        }
        tx->user = (void*)1;
        spi_device_queue_trans(lcd_spi_handle, tx,portMAX_DELAY);
    }
}
IRAM_ATTR void lcd_spi_pre_cb(spi_transaction_t* trans) {
    if (((int)trans->user) == 0) {
        DC_C;
    } 
}
IRAM_ATTR void lcd_spi_post_cb(spi_transaction_t* trans) {
    
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
        spi_transaction_t* tx=&lcd_trans[lcd_trans_index++]; if(lcd_trans_index>13) lcd_trans_index=0;
        tx->user = (void*)0;
        tx->flags =  SPI_TRANS_USE_TXDATA;
        tx->tx_data[0]=0x2C; // RAMWR
        tx->length = 8;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY));
        
        tx=&lcd_trans[lcd_trans_index++]; if(lcd_trans_index>13) lcd_trans_index=0;
        tx->flags = 0;
        tx->length = 8 * (len*2);
        tx->tx_buffer = data_in;
        tx->user = (void*)2;
        lcd_flushing = 1;
        ESP_ERROR_CHECK(
            spi_device_queue_trans(lcd_spi_handle, tx, portMAX_DELAY));
    } else {
        lcd_on_flush_complete();
    }
}
static void lcd_clear_screen(uint16_t color) {
    uint16_t* p1 = lcd_transfer_buffer1;
    uint16_t* p2 = lcd_transfer_buffer2;
    size_t pixels = lcd_transfer_buffer_size/sizeof(uint16_t);
    while(pixels--) {
        *(p1++)=color;
        *(p2++)=color;
    }
    pixels = lcd_transfer_buffer_size/sizeof(uint16_t);
    for(int y = 0;y<320;y+=32) {
        lcd_set_window(0,y,319,y+31);
        lcd_wait_dma(0);
        lcd_write_bitmap(lcd_transfer_buffer(),pixels);
        lcd_switch_buffers();
    }
}
static const bool big_cam =true;
void app_main(void)
{
    memset(lcd_trans,0,sizeof(spi_transaction_t)*14);
    lcd_transfer_buffer1 = heap_caps_malloc(lcd_transfer_buffer_size,MALLOC_CAP_DMA);
    lcd_transfer_buffer2 = heap_caps_malloc(lcd_transfer_buffer_size,MALLOC_CAP_DMA);
    if(lcd_transfer_buffer1==NULL || lcd_transfer_buffer2==NULL) {
        puts("Could not allocate transfer buffers");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
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

    uint32_t total_ms = 0;
    int frames = 0;
    uint32_t ts_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    camera_initialize(big_cam?CAM_ALLOC_CAM_PSRAM|CAM_ALLOC_FB_PSRAM:CAM_FRAME_SIZE_96X96);
    lcd_clear_screen(0);
    esp_task_wdt_config_t wdt_config;
    memset(&wdt_config,0,sizeof(wdt_config));
    wdt_config.timeout_ms = 30*1000;
    wdt_config.trigger_panic = 0;
    ESP_ERROR_CHECK(esp_task_wdt_reconfigure(&wdt_config));
    while(true) {
        uint32_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        camera_on_frame();
        ++frames;
        uint32_t end_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        total_ms+=(end_ms-start_ms);
        if(end_ms>ts_ms+1000) {
            ts_ms = end_ms;
            if(frames>0) {
                printf("FPS: %d, avg ms: %0.2f\n",frames,(float)total_ms/(float)frames);
            }
            total_ms=0;
            frames = 0;
        }
    }
}
void camera_on_frame() {
    if(big_cam) {
        static const size_t size = 240*24;
        const uint8_t* bmp=(const uint8_t*)camera_lock_frame_buffer();
        if(bmp!=NULL) {
            for(int y=0;y<240;y+=24) {
                uint8_t* data = lcd_transfer_buffer();
                memcpy(data,bmp+(y*240*2),size*2);
                if(lcd_wait_dma(400)) {
                    lcd_switch_buffers();
                    lcd_set_window(0,y,239,y+23);
                    lcd_write_bitmap(data,size);
                } else {
                    puts("LCD flush timeout");
                    vTaskDelay(1);
                }
                
                
            }
        }
        camera_unlock_frame_buffer();

    } else {
        static const size_t size = 96*96;
        const uint8_t* bmp=(const uint8_t*)camera_lock_frame_buffer();
        if(bmp!=NULL) {
            uint8_t* data = lcd_transfer_buffer();
            memcpy(data,bmp,size*2);
            if(lcd_wait_dma(400)) {
                lcd_switch_buffers();
                lcd_set_window(0,0,95,95);
                lcd_write_bitmap(data,size);
            } else {
                puts("LCD flush timeout");
                vTaskDelay(1);
            }
        } 
        camera_unlock_frame_buffer();
        
    }
    

}