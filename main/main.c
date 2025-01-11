#include <stdio.h>
#include <memory.h>
#include <math.h>
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

static SemaphoreHandle_t audio_sync=NULL;
static float audio_freq = 1000;
static long samplesTaken = 0; //Counter for calculating the Hz or read rate
static long unblockedValue; //Average IR at power up
static long startTime; //Used to calculate measurement rate


void next_waveform(int shape,float frequency, float amplitude, float* phase,float* samples, size_t sample_count) {
    static const float pi = 3.141592653589793238462643f; 
    const float delta = (pi*2.f)*frequency/44100.f;
    float f;
    float* out = samples;
    float p = *phase;
    for(int i = 0;i<sample_count;++i) {
        switch(shape) {
            case 1: // square:
                f=(p>pi)*2-1.f;
                break;
            case 2: // triangle:
                f=(p-pi)/pi;
                break;
            case 3: // sawtooth:
                f=(p-pi)/pi;
                break;
            default: // 0 = sine:
                f=sinf(p);
                break;
            
        }
        f*=amplitude;
        if(f<-1) f=-1;
        if(f>1) f=1;
        *(out++) = f;
        *(out++) = f;
        p+=delta;
        if(p>(pi*2.f)) {
            p-=(pi*2.f);
        }
        *phase = p;
    }
}
static const bool big_cam =true;

static void audio_task(void* arg) {
    static float samples[1024]={};
    uint32_t wdt_ts = 0;
    uint32_t aud_ts = 0;
    float phase = 0;
    float freq = 100.f;
    float freq_delta = 20.f;
    while(1) {
        uint32_t ms = pdTICKS_TO_MS(xTaskGetTickCount());
        if(ms>=wdt_ts+200) {
            wdt_ts=ms;
            vTaskDelay(1);
        }
        if(ms>=aud_ts+10) {
            
            aud_ts = ms;
            float f;
            xSemaphoreTake(audio_sync,portMAX_DELAY);
            f=audio_freq;
            xSemaphoreGive(audio_sync);
            next_waveform(0,f,.007f,&phase,samples,audio_max_samples>>1);
            audio_write_float(samples,audio_max_samples,1);
        }
    }
}
void app_main(void)
{
    static const size_t max_size =big_cam? 240*32*2:96*96*2;
    lcd_initialize(max_size);
    lcd_transfer_buffer1 = heap_caps_malloc(max_size,MALLOC_CAP_DMA);
    lcd_transfer_buffer2 = heap_caps_malloc(max_size,MALLOC_CAP_DMA);
    if(lcd_transfer_buffer1==NULL || lcd_transfer_buffer2==NULL) {
        puts("Could not allocate transfer buffers");
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    touch_initialize(TOUCH_THRESH_DEFAULT);
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
    touch_rotation(3);
    neopixel_initialize();

    //Setup to sense up to 18 inches, max LED brightness
    prox_sensor_initialize();
    prox_sensor_configure(PROX_SENS_AMP_DEFAULT,PROX_SENS_SAMPLEAVG_DEFAULT,PROX_SENS_MODE_DEFAULT,PROX_SENS_SAMPLERATE_DEFAULT,PROX_SENS_PULSEWIDTH_DEFAULT, PROX_SENS_ADCRANGE_DEFAULT);
    //prox_sensor_pulse_amp_threshold(0,PROX_SENS_THRESH_NO_CHANGE,0,PROX_SENS_THRESH_NO_CHANGE,PROX_SENS_THRESH_NO_CHANGE);
     //Take an average of IR readings at power up
    unblockedValue = 0;
    for (uint8_t x = 0 ; x < 32 ; ++x)
    {
        uint8_t ir;
        if(0==prox_sensor_read_raw(NULL,&ir,NULL,250)) {
            puts("TIMEOUT");
        }
        unblockedValue += ir;
    }
    unblockedValue /= 32;

    startTime = pdTICKS_TO_MS(xTaskGetTickCount());
    audio_sync = xSemaphoreCreateMutex();
    audio_initialize(AUDIO_44_1K_STEREO);
    TaskHandle_t audio_handle;
    xTaskCreatePinnedToCore(audio_task,"audio_task",8192,NULL,10,&audio_handle,1-xTaskGetAffinity(xTaskGetCurrentTaskHandle()));
    //led_initialize(); // conflicts with touch/i2c
    printf("Free SRAM: %0.2fKB, free PSRAM: %0.2fMB\n",heap_caps_get_free_size(MALLOC_CAP_INTERNAL)/1024.f,heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/1024.f/1024.f);
    int col = 0;
    
    while(true) {
        uint32_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        camera_on_frame();
        uint16_t x,y;
        if(touch_xy(&x,&y)) {
            printf("touch: (%d, %d)\n",x,y);
        }
        uint32_t ir;
        prox_sensor_read_raw(NULL,&ir,NULL,250);
    
        long currentDelta = ir - unblockedValue;
        if(currentDelta>500) {
            currentDelta=500;
        } else if(currentDelta<0) {
            currentDelta = 0;
        }
        xSemaphoreTake(audio_sync,50);
        audio_freq = 1000-currentDelta;
        xSemaphoreGive(audio_sync);

        ++frames;
        uint32_t end_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        
        total_ms+=(end_ms-start_ms);
        if(end_ms>ts_ms+1000) {
            ts_ms = end_ms;
            if(++col == 4) {
                col = 0;
            }
            neopixel_color(255*(col==1),255*(col==2),255*(col==3));
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
