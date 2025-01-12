// turn off the audio
#define SILENCE
#include <stdio.h>
#include <memory.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp_task_wdt.h"
#include "freenove_s3_devkit.h"
#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#define TSF_IMPLEMENTATION
#include "tsf.h"
#define TML_IMPLEMENTATION
#include "tml.h"
struct tsf_allocator tsf_alloc;
struct tsf* tsf_handle;
struct tml_message* tml_messages;
struct tml_message* tml_message_cursor;
static void* ps_malloc(size_t size) {
    return heap_caps_malloc(size,MALLOC_CAP_SPIRAM);
}
static void* ps_realloc(void* ptr,size_t size) {
    return heap_caps_realloc(ptr, size,MALLOC_CAP_SPIRAM);
}
static float* audio_output_buffer;
void audio_task(void* arg) {
    uint64_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    uint64_t wdt_ms = start_ms;
    while(true) {
        uint64_t ms = pdTICKS_TO_MS(xTaskGetTickCount());

        while(tml_message_cursor && tml_message_cursor->time<=(ms-start_ms)) {
            if(ms>wdt_ms+150) {
                wdt_ms = ms;
                vTaskDelay(pdMS_TO_TICKS(1));
                ++ms;
            }
            switch (tml_message_cursor->type)
			{
				case TML_PROGRAM_CHANGE: //channel program (preset) change (special handling for 10th MIDI channel with drums)
					tsf_channel_set_presetnumber(tsf_handle, tml_message_cursor->channel, tml_message_cursor->program, (tml_message_cursor->channel == 9));
					break;
				case TML_NOTE_ON: //play a note
					tsf_channel_note_on(tsf_handle, tml_message_cursor->channel, tml_message_cursor->key, tml_message_cursor->velocity / 127.0f);
					break;
				case TML_NOTE_OFF: //stop a note
					tsf_channel_note_off(tsf_handle, tml_message_cursor->channel, tml_message_cursor->key);
					break;
				case TML_PITCH_BEND: //pitch wheel modification
					tsf_channel_set_pitchwheel(tsf_handle, tml_message_cursor->channel, tml_message_cursor->pitch_bend);
					break;
				case TML_CONTROL_CHANGE: //MIDI controller messages
					tsf_channel_midi_control(tsf_handle, tml_message_cursor->channel, tml_message_cursor->control, tml_message_cursor->control_value);
					break;
			}
            tml_message_cursor = tml_message_cursor->next;
        }
        tsf_render_float(tsf_handle, (float*)audio_output_buffer,AUDIO_MAX_SAMPLES>>1, 0);
        audio_write_float(audio_output_buffer,AUDIO_MAX_SAMPLES,0.05);
        if(tml_message_cursor==NULL) {
            start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
            tml_message_cursor = tml_messages;
        }
    }
    // tsf_note_off_all(tsf_handle);
    // audio_deinitialize();
    // vTaskDelete(NULL);
}

void camera_on_frame();

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
static uint32_t prox_average; //Average IR at power up


static const bool big_cam =true;

void app_main(void)
{
    if(sd_initialize("/sdcard",2,16384,SD_FREQ_DEFAULT,SD_FLAGS_DEFAULT)) {    
        sdmmc_card_print_info(stdout,sd_card());
        tsf_alloc.alloc = ps_malloc;
        tsf_alloc.realloc = ps_realloc;
        tsf_alloc.free = free;
        tsf_handle = tsf_load_filename("/sdcard/1mgm.sf2",&tsf_alloc);
        if(tsf_handle==NULL) {
            puts("Unable to load soundfont");
            ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
        }
        tml_messages = tml_load_filename("/sdcard/furelise.mid");
        if(tml_messages==NULL) {
            puts("Unable to load midi");
            ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
        }
        tml_message_cursor = tml_messages;
        //Initialize preset on special 10th MIDI channel to use percussion sound bank (128) if available
        tsf_channel_set_bank_preset(tsf_handle, 9, 128, 0);
        // Set the SoundFont rendering output mode
        tsf_set_output(tsf_handle, TSF_STEREO_INTERLEAVED, 44100, 0.0f);
        tsf_set_max_voices(tsf_handle,4);
        sd_deinitialize();
    } else {
        puts("This demo requires a prepared SD card");
        //ESP_ERROR_CHECK(ESP_ERR_INVALID_STATE);
        while(1) vTaskDelay(1);
    }
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
    camera_rotation(3);
    lcd_rotation(3);
    touch_rotation(3);
    neopixel_initialize();
    //Setup to sense up to 18 inches, max LED brightness
    prox_sensor_initialize();
    prox_sensor_configure(PROX_SENS_AMP_50MA,PROX_SENS_SAMPLEAVG_4,PROX_SENS_MODE_REDIRONLY,PROX_SENS_SAMPLERATE_400,PROX_SENS_PULSEWIDTH_411, PROX_SENS_ADCRANGE_2048);
    
    
    //Take an average of IR readings at power up
    prox_average = 0;
    int avg_div = 0;
    for(int j = 0;j<10;++j) {
        for (uint8_t x = 0 ; x < 32 ; ++x)
        {
            uint32_t ir;
            if(0!=prox_sensor_read_raw(NULL,&ir,NULL,250)) {
                prox_average += ir;
                ++avg_div;
            }
            vTaskDelay(1);
        }
        if(avg_div>=32) {
            break;
        }
    }
    if(avg_div == 0 ){
        puts("Retry count for prox sensor exceeded");
        ESP_ERROR_CHECK(ESP_ERR_INVALID_RESPONSE);
    }
    prox_average /= avg_div;

    audio_sync = xSemaphoreCreateMutex();
    if(audio_sync==NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    audio_output_buffer=(float*)malloc(AUDIO_MAX_SAMPLES*sizeof(float));
    if(audio_output_buffer==NULL) {
        ESP_ERROR_CHECK(ESP_ERR_NO_MEM);
    }
    memset(audio_output_buffer,0,AUDIO_MAX_SAMPLES*sizeof(float));
    audio_initialize(AUDIO_44_1K_STEREO);
    TaskHandle_t audio_handle;
    xTaskCreatePinnedToCore(audio_task,"audio_task",8192,NULL,10,&audio_handle,1-xTaskGetAffinity(xTaskGetCurrentTaskHandle()));
    //led_initialize(); // conflicts with touch/i2c
    printf("Free SRAM: %0.2fKB, free PSRAM: %0.2fMB\n",heap_caps_get_free_size(MALLOC_CAP_INTERNAL)/1024.f,heap_caps_get_free_size(MALLOC_CAP_SPIRAM)/1024.f/1024.f);
    int col = 0;
    uint32_t wdt_ts =  pdTICKS_TO_MS(xTaskGetTickCount());
    while(true) {
        uint32_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        camera_on_frame();
        uint16_t x,y;
        if(touch_xy(&x,&y)) {
            printf("touch: (%d, %d)\n",x,y);
        }
        uint32_t ir;
        prox_sensor_read_raw(NULL,&ir,NULL,250);
        long currentDelta = ir - prox_average;
        if(currentDelta>500) {
            currentDelta=500;
        } else if(currentDelta<0) {
            currentDelta = 0;
        }
        
        // xSemaphoreTake(audio_sync,50);
        // audio_freq = 1000-currentDelta;
        // xSemaphoreGive(audio_sync);

        ++frames;
        uint32_t end_ms = pdTICKS_TO_MS(xTaskGetTickCount());
        if(end_ms>wdt_ts+150) {
            wdt_ts = end_ms;
            vTaskDelay(1);
        }
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
