#include "audio_io.h"

#include <stdint.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "es8388_hw.h"
#include "xl9555.h"

static const char *TAG = "audio_io";

#define AUDIO_SAMPLE_RATE_HZ 16000
#define MIC_SAMPLE_RATE_HZ   16000
#define VOICE_BASE_HZ        125
#define VOICE_AMPLITUDE      15000
#define VOICE_CLIP_FRAMES    (AUDIO_SAMPLE_RATE_HZ * 6 / 5)
#define FRAMES_PER_WRITE     256
#define I2S_DMA_DESC_NUM     12
#define I2S_DMA_FRAME_NUM    256
#define I2S_BG_WRITE_TIMEOUT_MS 200
#define I2S_STREAM_WRITE_TIMEOUT_MS 3000
#define PHASE_INC(hz)        ((uint32_t)(((uint64_t)(hz) << 32) / AUDIO_SAMPLE_RATE_HZ))

static i2c_obj_t *s_i2c;
static i2s_chan_handle_t s_tx;
static i2s_chan_handle_t s_rx;
static SemaphoreHandle_t s_tx_mutex;
static volatile bool s_playing;
static volatile bool s_ready;
static volatile bool s_mic_ready;
static volatile bool s_tx_enabled;
static volatile uint32_t s_stream_write_count;
static volatile size_t s_stream_last_written;
static volatile uint32_t s_play_cmd_count;
static bool s_stream_old_playing;
static bool s_stream_active;

void audio_io_log_state(const char *tag)
{
    ESP_LOGI(TAG,
             "AUDIO_STATE[%s]: ready=%d mic=%d tx=%p rx=%p tx_enabled=%d playing=%d stream=%d "
             "stream_writes=%u last_written=%u stack=%u heap_free=%u largest_internal=%u largest_spiram=%u",
             tag ? tag : "-",
             s_ready, s_mic_ready, (void *)s_tx, (void *)s_rx, s_tx_enabled,
             s_playing, s_stream_active, (unsigned)s_stream_write_count,
             (unsigned)s_stream_last_written,
             (unsigned)uxTaskGetStackHighWaterMark(NULL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
}

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
        if (s_stream_active) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

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

        if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(20)) != pdTRUE) {
            continue;
        }

        size_t written = 0;
        esp_err_t e = i2s_channel_write(s_tx, buf, sizeof(buf), &written,
                                        pdMS_TO_TICKS(I2S_BG_WRITE_TIMEOUT_MS));
        xSemaphoreGive(s_tx_mutex);
        if (e != ESP_OK) {
            ESP_LOGW(TAG,
                     "AUDIO_DBG bg i2s_channel_write err=%s play=%d requested=%d written=%u "
                     "tx_enabled=%d stream=%d",
                     esp_err_to_name(e), play, (int)sizeof(buf), (unsigned)written,
                     s_tx_enabled, s_stream_active);
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
    ESP_LOGI(TAG, "AUDIO_DBG mic pins: BCLK=%d WS=%d DIN=%d sample_rate=%d",
             pins->gpio_mic_bclk, pins->gpio_mic_ws, pins->gpio_mic_din,
             MIC_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "AUDIO_DBG format: sample_rate=%d bits=16 channels=2 voice_base=%dHz amplitude=%d clip_ms=%d",
             AUDIO_SAMPLE_RATE_HZ, VOICE_BASE_HZ, VOICE_AMPLITUDE,
             (VOICE_CLIP_FRAMES * 1000) / AUDIO_SAMPLE_RATE_HZ);
    ESP_LOGI(TAG, "Preparing ES8388 path: I2S 16kHz/16-bit/stereo TX+RX");

    esp_err_t err = es8388_hw_init_for_speaker(i2c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ES8388 init failed, abort audio init: %s", esp_err_to_name(err));
        s_ready = false;
        return err;
    }
    ESP_LOGI(TAG, "AUDIO_DBG ES8388 init OK, addr=0x%02x", es8388_hw_get_addr());

    s_tx_mutex = xSemaphoreCreateMutex();
    if (s_tx_mutex == NULL) {
        ESP_LOGE(TAG, "Create tx mutex failed");
        return ESP_ERR_NO_MEM;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num = I2S_DMA_DESC_NUM;
    chan_cfg.dma_frame_num = I2S_DMA_FRAME_NUM;
    ESP_LOGI(TAG, "AUDIO_DBG creating I2S channels: port=I2S_NUM_0 role=master dma_desc=%d dma_frame=%d",
             I2S_DMA_DESC_NUM, I2S_DMA_FRAME_NUM);
    err = i2s_new_channel(&chan_cfg, &s_tx, &s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "AUDIO_DBG DMA alloc/channel create OK tx=%p rx=%p desc=%d frame=%d",
             (void *)s_tx, (void *)s_rx, I2S_DMA_DESC_NUM, I2S_DMA_FRAME_NUM);
    ESP_LOGI(TAG, "AUDIO_DBG i2s_new_channel OK tx=%p", (void *)s_tx);

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)pins->gpio_speaker_mclk,
            .bclk = (gpio_num_t)pins->gpio_speaker_bclk,
            .ws = (gpio_num_t)pins->gpio_speaker_ws,
            .dout = (gpio_num_t)pins->gpio_speaker_dout,
            .din = (gpio_num_t)pins->gpio_mic_din,
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
    ESP_LOGI(TAG, "AUDIO_DBG tx i2s_channel_init_std_mode OK");

    err = i2s_channel_enable(s_tx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_ready = false;
        return err;
    }
    s_tx_enabled = true;
    ESP_LOGI(TAG, "AUDIO_DBG i2s_channel_enable OK");
    err = i2s_channel_init_std_mode(s_rx, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mic i2s_channel_init_std_mode: %s", esp_err_to_name(err));
        i2s_channel_disable(s_tx);
        s_tx_enabled = false;
        i2s_del_channel(s_tx);
        s_tx = NULL;
        i2s_del_channel(s_rx);
        s_rx = NULL;
        s_ready = false;
        s_mic_ready = false;
        return err;
    }

    err = i2s_channel_enable(s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "mic i2s_channel_enable: %s", esp_err_to_name(err));
        i2s_channel_disable(s_tx);
        s_tx_enabled = false;
        i2s_del_channel(s_tx);
        s_tx = NULL;
        i2s_del_channel(s_rx);
        s_rx = NULL;
        s_ready = false;
        s_mic_ready = false;
        return err;
    }
    s_mic_ready = true;
    ESP_LOGI(TAG, "Mic ready: I2S_NUM_0 MCLK=%d BCLK=%d WS=%d DOUT=%d DIN=%d 16kHz/16-bit/stereo slots",
             pins->gpio_speaker_mclk, pins->gpio_speaker_bclk, pins->gpio_speaker_ws,
             pins->gpio_speaker_dout, pins->gpio_mic_din);

    s_playing = false;
    if (s_i2c) {
        esp_err_t mute_err = es8388_hw_mute(s_i2c, true);
        ESP_LOGI(TAG, "AUDIO_DBG initial mute result=%s", esp_err_to_name(mute_err));
    }

    BaseType_t ok = xTaskCreatePinnedToCore(audio_task, "audio_io", 8192, NULL, 5, NULL, tskNO_AFFINITY);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "Create audio task failed");
        i2s_channel_disable(s_tx);
        s_tx_enabled = false;
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

bool audio_io_mic_is_ready(void)
{
    return s_mic_ready;
}

esp_err_t mic_init(void)
{
    static i2c_obj_t mic_i2c;
    static bool initialized;

    if (initialized) {
        return ESP_OK;
    }

    board_pins_t pins = board_config_get_default();
    mic_i2c = iic_init(I2C_NUM_0);
    esp_err_t err = audio_io_init(&pins, &mic_i2c);
    if (err == ESP_OK) {
        initialized = true;
    }
    return err;
}

size_t mic_read_pcm(uint8_t *buf, size_t len)
{
    if (!buf || len < sizeof(int16_t)) {
        return 0;
    }

    size_t samples_read = 0;
    esp_err_t err = audio_io_read_mic_mono16((int16_t *)buf, len / sizeof(int16_t),
                                             &samples_read, portMAX_DELAY);
    if (err != ESP_OK) {
        return 0;
    }
    return samples_read * sizeof(int16_t);
}

esp_err_t audio_io_read_mic_mono16(int16_t *samples, size_t sample_count,
                                   size_t *samples_read, TickType_t timeout_ticks)
{
    if (!samples || sample_count == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_rx || !s_mic_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    size_t total_samples = 0;
    esp_err_t err = ESP_OK;
    int16_t stereo[FRAMES_PER_WRITE * 2];

    while (total_samples < sample_count) {
        size_t frames = sample_count - total_samples;
        if (frames > FRAMES_PER_WRITE) {
            frames = FRAMES_PER_WRITE;
        }

        size_t bytes_read = 0;
        err = i2s_channel_read(s_rx, stereo, frames * 2 * sizeof(int16_t),
                               &bytes_read, timeout_ticks);
        if (err != ESP_OK) {
            break;
        }

        size_t frames_read = bytes_read / (2 * sizeof(int16_t));
        for (size_t i = 0; i < frames_read; i++) {
            samples[total_samples + i] = stereo[i * 2];
        }
        total_samples += frames_read;

        if (frames_read == 0 || frames_read < frames) {
            break;
        }
    }

    if (samples_read) {
        *samples_read = total_samples;
    }
    return err;
}

esp_err_t audio_io_playback_begin(void)
{
    if (!s_tx || !s_ready || !s_tx_mutex) {
        return ESP_ERR_INVALID_STATE;
    }
    if (s_stream_active) {
        return ESP_ERR_INVALID_STATE;
    }

    s_stream_old_playing = s_playing;
    s_playing = false;
    s_stream_active = true;
    s_stream_write_count = 0;
    s_stream_last_written = 0;
    audio_io_log_state("playback_begin_before_mutex");

    uint16_t io_state = xl9555_pin_write(SPK_EN_IO, 0);
    ESP_LOGI(TAG, "PCM stream playback start: SPK_EN state=0x%04x", io_state);
    if (s_i2c) {
        es8388_hw_mute(s_i2c, false);
    }

    if (xSemaphoreTake(s_tx_mutex, pdMS_TO_TICKS(3000)) != pdTRUE) {
        s_stream_active = false;
        if (s_i2c) {
            es8388_hw_mute(s_i2c, !s_stream_old_playing);
        }
        if (!s_stream_old_playing) {
            xl9555_pin_write(SPK_EN_IO, 1);
        }
        s_playing = s_stream_old_playing;
        audio_io_log_state("playback_begin_mutex_timeout");
        return ESP_ERR_TIMEOUT;
    }

    audio_io_log_state("playback_begin_ready");
    return ESP_OK;
}

esp_err_t audio_io_playback_write_pcm(const int16_t *pcm, size_t frame_count,
                                      int sample_rate_hz, int channel_count)
{
    if (!pcm || frame_count == 0 || sample_rate_hz <= 0 ||
        (channel_count != 1 && channel_count != 2)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_stream_active) {
        return ESP_ERR_INVALID_STATE;
    }

    int16_t out[FRAMES_PER_WRITE * 2];
    uint64_t out_frames_total = ((uint64_t)frame_count * AUDIO_SAMPLE_RATE_HZ) /
                                (uint32_t)sample_rate_hz;
    if (out_frames_total == 0) {
        out_frames_total = 1;
    }

    esp_err_t ret = ESP_OK;
    uint64_t out_pos = 0;
    while (out_pos < out_frames_total) {
        size_t todo = (size_t)(out_frames_total - out_pos);
        if (todo > FRAMES_PER_WRITE) {
            todo = FRAMES_PER_WRITE;
        }

        for (size_t i = 0; i < todo; i++) {
            uint64_t src_frame = ((out_pos + i) * (uint32_t)sample_rate_hz) /
                                 AUDIO_SAMPLE_RATE_HZ;
            if (src_frame >= frame_count) {
                src_frame = frame_count - 1;
            }

            int32_t mono;
            if (channel_count == 1) {
                mono = pcm[src_frame];
            } else {
                mono = ((int32_t)pcm[src_frame * 2] + (int32_t)pcm[src_frame * 2 + 1]) / 2;
            }
            out[i * 2] = (int16_t)mono;
            out[i * 2 + 1] = (int16_t)mono;
        }

        size_t written = 0;
        ret = i2s_channel_write(s_tx, out, todo * 2 * sizeof(int16_t),
                                &written, pdMS_TO_TICKS(I2S_STREAM_WRITE_TIMEOUT_MS));
        s_stream_write_count++;
        s_stream_last_written = written;
        if (ret != ESP_OK) {
            ESP_LOGW(TAG,
                     "PCM playback write failed: %s requested=%u written=%u tx_enabled=%d "
                     "stream=%d write_count=%u",
                     esp_err_to_name(ret), (unsigned)(todo * 2 * sizeof(int16_t)),
                     (unsigned)written, s_tx_enabled, s_stream_active,
                     (unsigned)s_stream_write_count);
            audio_io_log_state("playback_write_failed");
            break;
        }
        if (written != todo * 2 * sizeof(int16_t)) {
            ESP_LOGW(TAG, "PCM playback short write requested=%u written=%u tx_enabled=%d",
                     (unsigned)(todo * 2 * sizeof(int16_t)), (unsigned)written,
                     s_tx_enabled);
        }
        out_pos += todo;
    }

    return ret;
}

void audio_io_playback_end(void)
{
    if (!s_stream_active) {
        return;
    }

    s_stream_active = false;
    xSemaphoreGive(s_tx_mutex);

    if (s_i2c) {
        es8388_hw_mute(s_i2c, !s_stream_old_playing);
    }
    if (!s_stream_old_playing) {
        xl9555_pin_write(SPK_EN_IO, 1);
    }
    s_playing = s_stream_old_playing;
    audio_io_log_state("playback_end");
    ESP_LOGI(TAG, "PCM stream playback done");
}

esp_err_t audio_io_play_pcm(const int16_t *pcm, size_t frame_count,
                            int sample_rate_hz, int channel_count)
{
    esp_err_t ret = audio_io_playback_begin();
    if (ret != ESP_OK) {
        return ret;
    }
    ret = audio_io_playback_write_pcm(pcm, frame_count, sample_rate_hz, channel_count);
    audio_io_playback_end();
    ESP_LOGI(TAG, "PCM playback done: %s", esp_err_to_name(ret));
    return ret;
}
