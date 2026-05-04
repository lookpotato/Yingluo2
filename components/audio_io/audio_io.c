#include "audio_io.h"

#include <math.h>
#include <string.h>

#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "es8388_hw.h"
#include "xl9555.h"

static const char *TAG = "audio_io";

#define AUDIO_SAMPLE_RATE_HZ 44100
#define TONE_HZ              440
#define TONE_AMPLITUDE       18000.0f
#define FRAMES_PER_WRITE     256

static i2c_obj_t *s_i2c;
static i2s_chan_handle_t s_tx;
static volatile bool s_playing;
static volatile bool s_ready;
static volatile uint32_t s_play_cmd_count;

static void fill_tone_stereo(int16_t *stereo, size_t frame_count, float *phase_rad)
{
    const float delta = 2.0f * (float)M_PI * (float)TONE_HZ / (float)AUDIO_SAMPLE_RATE_HZ;
    const float amp = TONE_AMPLITUDE;

    for (size_t i = 0; i < frame_count; i++) {
        float s = sinf(*phase_rad) * amp;
        *phase_rad += delta;
        if (*phase_rad > 2.0f * (float)M_PI) {
            *phase_rad -= 2.0f * (float)M_PI;
        }
        int16_t v = (int16_t)s;
        stereo[i * 2] = v;
        stereo[i * 2 + 1] = v;
    }
}

static void audio_task(void *arg)
{
    int16_t buf[FRAMES_PER_WRITE * 2];
    float phase = 0.0f;
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
            fill_tone_stereo(buf, FRAMES_PER_WRITE, &phase);
        } else {
            memset(buf, 0, sizeof(buf));
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
            ESP_LOGI(TAG, "AUDIO_DBG I2S active writes=%u last_written=%u phase=%.3f",
                     (unsigned)write_count, (unsigned)written, phase);
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
    ESP_LOGI(TAG, "AUDIO_DBG format: sample_rate=%d bits=16 channels=2 tone=%dHz amplitude=%.0f",
             AUDIO_SAMPLE_RATE_HZ, TONE_HZ, TONE_AMPLITUDE);
    ESP_LOGI(TAG, "准备初始化扬声器链路: ES8388 + I2S 44.1kHz/16-bit/stereo");

    esp_err_t err = es8388_hw_init_for_speaker(i2c);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ES8388 初始化失败，中止音频初始化: %s", esp_err_to_name(err));
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
        ESP_LOGE(TAG, "创建 audio 任务失败");
        i2s_channel_disable(s_tx);
        i2s_del_channel(s_tx);
        s_tx = NULL;
        s_ready = false;
        return ESP_FAIL;
    }

    s_ready = true;
    ESP_LOGI(TAG, "音频就绪: ES8388=0x%02x MCLK=%d BCLK=%d WS=%d DOUT=%d, %dHz 正弦试音",
             es8388_hw_get_addr(), pins->gpio_speaker_mclk, pins->gpio_speaker_bclk,
             pins->gpio_speaker_ws, pins->gpio_speaker_dout, TONE_HZ);
    return ESP_OK;
}

void audio_io_set_playing(bool play)
{
    s_play_cmd_count++;
    ESP_LOGI(TAG, "AUDIO_DBG set_playing cmd=%u play=%d ready=%d tx=%p i2c=%p",
             (unsigned)s_play_cmd_count, play, s_ready, (void *)s_tx, (void *)s_i2c);

    if (play) {
        uint16_t io_state = xl9555_pin_write(SPK_EN_IO, 0);   // 低电平打开功放
        ESP_LOGI(TAG, "AUDIO_DBG amp enable requested: SPK_EN_IO=0 xl9555_state=0x%04x", io_state);
        ESP_LOGI(TAG, "SPK_EN_IO=0，功放使能");
    } else {
        uint16_t io_state = xl9555_pin_write(SPK_EN_IO, 1);   // 高电平关闭功放
        ESP_LOGI(TAG, "AUDIO_DBG amp disable requested: SPK_EN_IO=1 xl9555_state=0x%04x", io_state);
        ESP_LOGI(TAG, "SPK_EN_IO=1，功放关闭");
    }

    s_playing = play;

    if (s_i2c) {
        esp_err_t err = es8388_hw_mute(s_i2c, !play);
        ESP_LOGI(TAG, "AUDIO_DBG codec mute request mute=%d result=%s",
                 !play, esp_err_to_name(err));
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "ES8388 mute=%s 失败: %s",
                     play ? "off" : "on", esp_err_to_name(err));
        }
    }

    if (!s_ready) {
        ESP_LOGW(TAG, "audio_io 尚未就绪，当前只记录播放状态 play=%d", play);
    }
}

bool audio_io_is_ready(void)
{
    return s_ready;
}
