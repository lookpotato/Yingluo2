#include "audio_io.h"

#include <stdint.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "es8388_hw.h"
#include "xl9555.h"

static const char *TAG = "audio_io";

#define AUDIO_SAMPLE_RATE_HZ 44100
#define VOICE_BASE_HZ        125
#define VOICE_AMPLITUDE      15000
#define VOICE_CLIP_FRAMES    (AUDIO_SAMPLE_RATE_HZ * 6 / 5)
#define FRAMES_PER_WRITE     256
#define PHASE_INC(hz)        ((uint32_t)(((uint64_t)(hz) << 32) / AUDIO_SAMPLE_RATE_HZ))

static i2c_obj_t *s_i2c;
static i2s_chan_handle_t s_tx;
static volatile bool s_playing;
static volatile bool s_ready;
static volatile uint32_t s_play_cmd_count;

static int32_t voice_envelope_q15(uint32_t clip_frame)
{
    const uint32_t attack = AUDIO_SAMPLE_RATE_HZ / 40;
    const uint32_t release_start = VOICE_CLIP_FRAMES - AUDIO_SAMPLE_RATE_HZ / 12;

    if (clip_frame < attack) {
        return (int32_t)((clip_frame * 32767U) / attack);
    }
    if (clip_frame > release_start) {
        return (int32_t)(((VOICE_CLIP_FRAMES - clip_frame) * 32767U) / (VOICE_CLIP_FRAMES - release_start));
    }
    return 32767;
}

static int32_t osc_triangle(uint32_t phase)
{
    uint32_t x = phase >> 16;
    int32_t tri = (x < 32768U) ? (int32_t)x : (int32_t)(65535U - x);
    return (tri * 2) - 32768;
}

static void select_voice_partials(uint32_t clip_frame,
                                  uint32_t *p1, uint32_t *p2, uint32_t *p3,
                                  int32_t *g1, int32_t *g2, int32_t *g3)
{
    uint32_t third = VOICE_CLIP_FRAMES / 3;

    if (clip_frame < third) {
        *p1 = PHASE_INC(125);
        *p2 = PHASE_INC(750);    // "a"
        *p3 = PHASE_INC(1125);
        *g1 = 11000;
        *g2 = 9500;
        *g3 = 6500;
    } else if (clip_frame < third * 2) {
        *p1 = PHASE_INC(120);
        *p2 = PHASE_INC(600);    // "o"
        *p3 = PHASE_INC(850);
        *g1 = 12000;
        *g2 = 7800;
        *g3 = 5200;
    } else {
        *p1 = PHASE_INC(130);
        *p2 = PHASE_INC(300);    // "i"
        *p3 = PHASE_INC(2300);
        *g1 = 9000;
        *g2 = 4200;
        *g3 = 9000;
    }
}

static void fill_voice_stereo(int16_t *stereo, size_t frame_count,
                              uint32_t *phase1, uint32_t *phase2, uint32_t *phase3,
                              uint32_t *sample_index)
{
    for (size_t i = 0; i < frame_count; i++) {
        uint32_t clip_frame = *sample_index % VOICE_CLIP_FRAMES;
        uint32_t inc1, inc2, inc3;
        int32_t gain1, gain2, gain3;
        select_voice_partials(clip_frame, &inc1, &inc2, &inc3, &gain1, &gain2, &gain3);

        *phase1 += inc1;
        *phase2 += inc2;
        *phase3 += inc3;

        int32_t sample =
            (osc_triangle(*phase1) * gain1 +
             osc_triangle(*phase2) * gain2 +
             osc_triangle(*phase3) * gain3) >> 15;
        sample = (sample * VOICE_AMPLITUDE) >> 15;
        sample = (sample * voice_envelope_q15(clip_frame)) >> 15;

        if (sample > 32767) {
            sample = 32767;
        } else if (sample < -32768) {
            sample = -32768;
        }

        int16_t v = (int16_t)sample;
        stereo[i * 2] = v;
        stereo[i * 2 + 1] = v;
        (*sample_index)++;
    }
}

static void audio_task(void *arg)
{
    int16_t buf[FRAMES_PER_WRITE * 2];
    uint32_t phase1 = 0;
    uint32_t phase2 = 0;
    uint32_t phase3 = 0;
    uint32_t sample_index = 0;
    bool last_play = false;
    uint32_t write_count = 0;
    uint32_t short_write_count = 0;

    ESP_LOGI(TAG, "AUDIO_DBG task started, frames_per_write=%d bytes_per_write=%d",
             FRAMES_PER_WRITE, (int)sizeof(buf));

    for (;;) {
        bool play = s_playing;
        if (play != last_play) {
            ESP_LOGI(TAG, "AUDIO_DBG task sees play=%d ready=%d tx=%p",
                     play, s_ready, (void *)s_tx);
            last_play = play;
        }

        if (play) {
            fill_voice_stereo(buf, FRAMES_PER_WRITE, &phase1, &phase2, &phase3, &sample_index);
        } else {
            memset(buf, 0, sizeof(buf));
            phase1 = 0;
            phase2 = 0;
            phase3 = 0;
            sample_index = 0;
        }

        size_t written = 0;
        esp_err_t e = i2s_channel_write(s_tx, buf, sizeof(buf), &written, portMAX_DELAY);
        if (e != ESP_OK) {
            ESP_LOGW(TAG, "AUDIO_DBG i2s_channel_write err=%s play=%d requested=%d written=%u",
                     esp_err_to_name(e), play, (int)sizeof(buf), (unsigned)written);
            continue;
        }

        write_count++;
        if (written != sizeof(buf)) {
            short_write_count++;
            ESP_LOGW(TAG, "AUDIO_DBG short I2S write #%u requested=%d written=%u play=%d",
                     (unsigned)short_write_count, (int)sizeof(buf), (unsigned)written, play);
        }

        if (play && (write_count % 200U) == 0U) {
            ESP_LOGI(TAG, "AUDIO_DBG I2S active writes=%u last_written=%u voice_sample=%u",
                     (unsigned)write_count, (unsigned)written, (unsigned)sample_index);
        }
    }
}

esp_err_t audio_io_init(const board_pins_t *pins, i2c_obj_t *i2c)
{
    if (!pins || !i2c) {
        return ESP_ERR_INVALID_ARG;
    }
    s_i2c = i2c;

    ESP_LOGI(TAG, "AUDIO_DBG init begin: pins=%p i2c=%p", (void *)pins, (void *)i2c);
    ESP_LOGI(TAG, "AUDIO_DBG board pins: MCLK=%d BCLK=%d WS=%d DOUT=%d",
             pins->gpio_speaker_mclk, pins->gpio_speaker_bclk,
             pins->gpio_speaker_ws, pins->gpio_speaker_dout);
    ESP_LOGI(TAG, "AUDIO_DBG format: sample_rate=%d bits=16 channels=2 voice_base=%dHz amplitude=%d clip_ms=%d",
             AUDIO_SAMPLE_RATE_HZ, VOICE_BASE_HZ, VOICE_AMPLITUDE,
             (VOICE_CLIP_FRAMES * 1000) / AUDIO_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "Preparing speaker path: ES8388 + I2S 44.1kHz/16-bit/stereo");

    esp_err_t err = es8388_hw_init_for_speaker(i2c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ES8388 init failed, abort audio init: %s", esp_err_to_name(err));
        s_ready = false;
        return err;
    }
    ESP_LOGI(TAG, "AUDIO_DBG ES8388 init OK, addr=0x%02x", es8388_hw_get_addr());

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_LOGI(TAG, "AUDIO_DBG creating I2S channel: port=I2S_NUM_0 role=master");
    err = i2s_new_channel(&chan_cfg, &s_tx, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "AUDIO_DBG i2s_new_channel OK tx=%p", (void *)s_tx);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)pins->gpio_speaker_mclk,
            .bclk = (gpio_num_t)pins->gpio_speaker_bclk,
            .ws = (gpio_num_t)pins->gpio_speaker_ws,
            .dout = (gpio_num_t)pins->gpio_speaker_dout,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    err = i2s_channel_init_std_mode(s_tx, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode: %s", esp_err_to_name(err));
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_ready = false;
        return err;
    }
    ESP_LOGI(TAG, "AUDIO_DBG i2s_channel_init_std_mode OK");

    err = i2s_channel_enable(s_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_ready = false;
        return err;
    }
    ESP_LOGI(TAG, "AUDIO_DBG i2s_channel_enable OK");

    s_playing = false;
    if (s_i2c) {
        esp_err_t mute_err = es8388_hw_mute(s_i2c, true);
        ESP_LOGI(TAG, "AUDIO_DBG initial mute result=%s", esp_err_to_name(mute_err));
    }

    BaseType_t ok = xTaskCreatePinnedToCore(audio_task, "audio_io", 4096, NULL, 5, NULL, tskNO_AFFINITY);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Create audio task failed");
        i2s_channel_disable(s_tx);
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_ready = false;
        return ESP_FAIL;
    }

    s_ready = true;
    ESP_LOGI(TAG, "Audio ready: ES8388=0x%02x MCLK=%d BCLK=%d WS=%d DOUT=%d, synthetic voice test",
             es8388_hw_get_addr(), pins->gpio_speaker_mclk, pins->gpio_speaker_bclk,
             pins->gpio_speaker_ws, pins->gpio_speaker_dout);
    return ESP_OK;
}

void audio_io_set_playing(bool play)
{
    s_play_cmd_count++;
    ESP_LOGI(TAG, "AUDIO_DBG set_playing cmd=%u play=%d ready=%d tx=%p i2c=%p",
             (unsigned)s_play_cmd_count, play, s_ready, (void *)s_tx, (void *)s_i2c);

    if (play) {
        uint16_t io_state = xl9555_pin_write(SPK_EN_IO, 0);
        ESP_LOGI(TAG, "AUDIO_DBG amp enable requested: SPK_EN_IO=0 xl9555_state=0x%04x", io_state);
        ESP_LOGI(TAG, "SPK_EN_IO=0, speaker amp enabled");
    } else {
        uint16_t io_state = xl9555_pin_write(SPK_EN_IO, 1);
        ESP_LOGI(TAG, "AUDIO_DBG amp disable requested: SPK_EN_IO=1 xl9555_state=0x%04x", io_state);
        ESP_LOGI(TAG, "SPK_EN_IO=1, speaker amp disabled");
    }

    s_playing = play;

    if (s_i2c) {
        esp_err_t err = es8388_hw_mute(s_i2c, !play);
        ESP_LOGI(TAG, "AUDIO_DBG codec mute request mute=%d result=%s",
                 !play, esp_err_to_name(err));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ES8388 mute=%s failed: %s",
                     play ? "off" : "on", esp_err_to_name(err));
        }
    }

    if (!s_ready) {
        ESP_LOGW(TAG, "audio_io is not ready, only recording play state=%d", play);
    }
}

bool audio_io_is_ready(void)
{
    return s_ready;
}
