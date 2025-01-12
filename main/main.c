#include "esp_heap_caps.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "freenove_s3_devkit.h"
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
static float audio_output_buffer[AUDIO_MAX_SAMPLES];
void audio_task(void* arg) {
    uint64_t start_ms = pdTICKS_TO_MS(xTaskGetTickCount());
    uint64_t wdt_ms = start_ms;
    while(tml_message_cursor) {
        uint64_t ms = pdTICKS_TO_MS(xTaskGetTickCount());

        while(tml_message_cursor && tml_message_cursor->time<=(ms-start_ms)) {
            // if(ms>wdt_ms+150) {
            //     wdt_ms = ms;
            //     vTaskDelay(pdMS_TO_TICKS(1));
            //     //++ms;
            // }
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
        tsf_render_float(tsf_handle, (float*)audio_output_buffer,AUDIO_MAX_SAMPLES, 0);
        audio_write_float(audio_output_buffer,AUDIO_MAX_SAMPLES,0.0125);
    
    }
    tsf_note_off_all(tsf_handle);
    audio_deinitialize();
    vTaskDelete(NULL);
}
void app_main() {
    if(!sd_initialize("/sdcard",2,16384,SD_FREQ_DEFAULT,SD_FLAGS_DEFAULT)) {
        puts("Unable to mount SD");
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    audio_initialize(AUDIO_11K_MONO);
    memset(audio_output_buffer,0,sizeof(float)*AUDIO_MAX_SAMPLES);
    tsf_alloc.alloc = ps_malloc;
    tsf_alloc.realloc = ps_realloc;
    tsf_alloc.free = free;
    tsf_handle = tsf_load_filename("/sdcard/1mgm.sf2",&tsf_alloc);
    if(tsf_handle==NULL) {
        puts("Unable to load soundfont");
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    tml_messages = tml_load_filename("/sdcard/warm.mid");
    if(tml_messages==NULL) {
        puts("Unable to load midi");
        ESP_ERROR_CHECK(ESP_ERR_NOT_FOUND);
    }
    tml_message_cursor = tml_messages;
    //Initialize preset on special 10th MIDI channel to use percussion sound bank (128) if available
	tsf_channel_set_bank_preset(tsf_handle, 9, 128, 0);
	// Set the SoundFont rendering output mode
	tsf_set_output(tsf_handle, TSF_MONO, 11025, 0.0f);
    tsf_set_max_voices(tsf_handle,4);
    TaskHandle_t audio_handle;
    xTaskCreatePinnedToCore(audio_task,"audio_task", 8192,NULL,10,&audio_handle,xTaskGetCoreID(xTaskGetCurrentTaskHandle()));
}