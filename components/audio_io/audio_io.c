#include "audio_io.h"

#include "esp_log.h"

static const char *TAG = "audio_io";

esp_err_t audio_io_init(const board_pins_t *pins)
{
    ESP_LOGI(TAG, "Speaker BCLK=%d WS=%d DOUT=%d",
             pins->gpio_speaker_bclk, pins->gpio_speaker_ws, pins->gpio_speaker_dout);
    ESP_LOGI(TAG, "Mic BCLK=%d WS=%d DIN=%d",
             pins->gpio_mic_bclk, pins->gpio_mic_ws, pins->gpio_mic_din);
    ESP_LOGI(TAG, "Audio module skeleton ready, I2S bring-up next step.");
    return ESP_OK;
}
