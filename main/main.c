#include <stdio.h>
#include <memory.h>
#include <esp_camera.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <hal/gpio_ll.h>
#include <esp_task_wdt.h>
#include "freenove_s3_devkit.h"

void camera_on_frame();

//static const size_t lcd_transfer_buffer_size = 32760;//240*32*2;
static void* lcd_transfer_buffer1=NULL;
static void* lcd_transfer_buffer2=NULL;
static volatile int lcd_dma_sel = 0;
static volatile int lcd_flushing = 0;

static void* lcd_transfer_buffer() {
    return lcd_dma_sel?lcd_transfer_buffer1:lcd_transfer_buffer2;    
}


static void lcd_switch_buffers() {
    if(lcd_dma_sel==0) {
        lcd_dma_sel = 1;
    } else {
        lcd_dma_sel = 0;
    }
}

static void lcd_clear_screen(uint16_t color) {
    uint16_t* p1 = lcd_transfer_buffer1;
    uint16_t* p2 = lcd_transfer_buffer2;
    size_t pixels = 240*32;
    while(pixels--) {
        *(p1++)=color;
        *(p2++)=color;
    }
    for(int y = 0;y<320;y+=32) {
        lcd_wait_flush(0);
        lcd_flush(0,y,239,y+31,lcd_transfer_buffer());
        lcd_switch_buffers();
    }
}

static const bool big_cam =true;
void app_main(void)
{
    static const size_t max_size = 320*24*2;
    lcd_initialize(max_size);
    
    lcd_transfer_buffer1 = heap_caps_malloc(max_size,MALLOC_CAP_DMA);
    lcd_transfer_buffer2 = heap_caps_malloc(max_size,MALLOC_CAP_DMA);
    if(lcd_transfer_buffer1==NULL || lcd_transfer_buffer2==NULL) {
        puts("Could not allocate transfer buffers");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    
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
    camera_rotation(3);
    lcd_rotation(3);
    printf("Free SRAM: %0.2fKB, free PSRAM: %0.2fMB\n",heap_caps_get_free_size(MALLOC_CAP_INTERNAL)/1024.f,heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/1024.f/1024.f);
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
        const uint8_t* bmp=(const uint8_t*)camera_frame_buffer();
        for(int y=0;y<240;y+=24) {
            uint8_t* data = lcd_transfer_buffer();
            memcpy(data,((const uint8_t*)bmp)+(y*240*2),size*2);
            if(lcd_wait_flush(400)) {
                lcd_switch_buffers();
                lcd_flush(0,y,239,y+23,data);
            } else {
                puts("LCD flush timeout");
                vTaskDelay(1);
            }   
        }
    } else {
        static const size_t size = 96*96;
        const uint8_t* bmp=(const uint8_t*)camera_frame_buffer();
        
        uint8_t* data = lcd_transfer_buffer();
        memcpy(data,bmp,size*2);
        if(lcd_wait_flush(400)) {
            lcd_switch_buffers();
            lcd_flush(0,0,95,95,data);
        } else {
            puts("LCD flush timeout");
            vTaskDelay(1);
        }        
    }
    

}