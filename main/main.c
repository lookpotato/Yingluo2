#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_netif.h"
#include "esp_psram.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#include "minimp3.h"

#include "audio_io.h"
#include "board_config.h"
#include "iic.h"
#include "led_group.h"
#include "servo.h"
#include "ws2812b.h"
#include "xl9555.h"

#define WIFI_STA_SSID "ChinaNet-S1z8"
#define WIFI_STA_PASSWORD "2g7g2j28"
#define WIFI_MAXIMUM_RETRY 20

#define VOICE_SERVER_BASE_URL "http://101.34.244.21:8000"
#define VOICE_CHAT_URL VOICE_SERVER_BASE_URL "/api/voice/chat"
#define VOICE_DEVICE_ID "esp32s3-001"
#define VOICE_SAMPLE_RATE_HZ 16000
#define VOICE_MAX_RECORD_MS 2000
#define VOICE_RECORD_CHUNK_SAMPLES 512
#define VOICE_HTTP_TIMEOUT_MS 90000
#define VOICE_MIN_RECORD_MS 1000
#define VOICE_MP3_STREAM_BUFFER_SIZE 16384
#define VOICE_HTTP_READ_CHUNK_SIZE 2048

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define WS2812B_GPIO GPIO_NUM_6
#define WS2812B_LED_COUNT 1
#define WS2812B_TEST_BRIGHTNESS 96

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_num;
static volatile bool s_voice_task_running;

static void voice_log_mem(const char *stage)
{
    printf("VOICE_MEM[%s]: free_heap=%u largest_internal=%u largest_spiram=%u stack_free=%u\n",
           stage ? stage : "-",
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT),
           (unsigned)uxTaskGetStackHighWaterMark(NULL));
    audio_io_log_state(stage);
}

static void *voice_malloc_spiram(size_t size, const char *label)
{
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!ptr) {
        printf("VOICE: allocate %s failed (%u bytes), free_heap=%u largest_internal=%u largest_spiram=%u\n",
               label ? label : "buffer", (unsigned)size,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    }
    return ptr;
}

static void voice_debug_dump_bytes(const char *label, const uint8_t *data, size_t len)
{
    size_t dump_len = len < 16 ? len : 16;

    printf("VOICE_DBG: %s len=%u first%u=", label, (unsigned)len, (unsigned)dump_len);
    for (size_t i = 0; i < dump_len; i++) {
        printf("%02x", data[i]);
        if (i + 1 < dump_len) {
            printf(" ");
        }
    }
    printf("\n");
}

static void voice_debug_pcm_stats(const int16_t *samples, size_t sample_count,
                                  size_t total_samples)
{
    if (!samples || sample_count == 0) {
        return;
    }

    int16_t min_sample = samples[0];
    int16_t max_sample = samples[0];
    uint64_t abs_sum = 0;
    for (size_t i = 0; i < sample_count; i++) {
        int16_t sample = samples[i];
        int32_t abs_sample = sample < 0 ? -(int32_t)sample : sample;
        if (sample < min_sample) {
            min_sample = sample;
        }
        if (sample > max_sample) {
            max_sample = sample;
        }
        abs_sum += (uint32_t)abs_sample;
    }

    size_t print_count = sample_count < 8 ? sample_count : 8;
    printf("VOICE_DBG: mic chunk total_samples=%u chunk=%u min=%d max=%d avg_abs=%u first%u=",
           (unsigned)total_samples, (unsigned)sample_count, min_sample, max_sample,
           (unsigned)(abs_sum / sample_count), (unsigned)print_count);
    for (size_t i = 0; i < print_count; i++) {
        printf("%d", samples[i]);
        if (i + 1 < print_count) {
            printf(",");
        }
    }
    printf("\n");
}

static void wav_write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
}

static void wav_write_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xff);
    p[1] = (uint8_t)((v >> 8) & 0xff);
    p[2] = (uint8_t)((v >> 16) & 0xff);
    p[3] = (uint8_t)((v >> 24) & 0xff);
}

static void wav_write_header(uint8_t *wav, uint32_t pcm_bytes)
{
    memcpy(wav + 0, "RIFF", 4);
    wav_write_u32_le(wav + 4, pcm_bytes + 36);
    memcpy(wav + 8, "WAVEfmt ", 8);
    wav_write_u32_le(wav + 16, 16);
    wav_write_u16_le(wav + 20, 1);
    wav_write_u16_le(wav + 22, 1);
    wav_write_u32_le(wav + 24, VOICE_SAMPLE_RATE_HZ);
    wav_write_u32_le(wav + 28, VOICE_SAMPLE_RATE_HZ * 2);
    wav_write_u16_le(wav + 32, 2);
    wav_write_u16_le(wav + 34, 16);
    memcpy(wav + 36, "data", 4);
    wav_write_u32_le(wav + 40, pcm_bytes);
}

static uint8_t *voice_alloc_wav_buffer(size_t *max_samples_out)
{
    size_t max_samples = (VOICE_SAMPLE_RATE_HZ * VOICE_MAX_RECORD_MS) / 1000;
    const size_t min_samples = (VOICE_SAMPLE_RATE_HZ * VOICE_MIN_RECORD_MS) / 1000;

    while (max_samples >= min_samples) {
        size_t bytes = 44 + max_samples * sizeof(int16_t);
        uint8_t *wav = voice_malloc_spiram(bytes, "wav buffer");
        if (wav) {
            *max_samples_out = max_samples;
            return wav;
        }

        printf("VOICE: allocate wav buffer failed (%u bytes), free_heap=%u largest_internal=%u largest_spiram=%u\n",
               (unsigned)bytes,
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        max_samples /= 2;
    }

    return NULL;
}

static uint8_t *voice_record_wav_until_stop(size_t *wav_len)
{
    voice_log_mem("record_before");
    size_t max_samples = 0;
    uint8_t *wav = voice_alloc_wav_buffer(&max_samples);
    if (!wav) {
        printf("VOICE: allocate wav buffer failed; cannot record\n");
        return NULL;
    }

    printf("VOICE: recording started, press KEY2 to stop and upload, max=%d ms\n",
           (int)((max_samples * 1000) / VOICE_SAMPLE_RATE_HZ));
    size_t sample_count = 0;
    size_t last_debug_sample_count = 0;
    int last_stop = KEY2;
    TickType_t start = xTaskGetTickCount();

    while (sample_count < max_samples) {
        size_t got = 0;
        size_t room = max_samples - sample_count;
        if (room > VOICE_RECORD_CHUNK_SAMPLES) {
            room = VOICE_RECORD_CHUNK_SAMPLES;
        }

        int16_t *pcm = (int16_t *)(wav + 44);
        esp_err_t err = audio_io_read_mic_mono16(&pcm[sample_count], room, &got,
                                                 pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            printf("VOICE: mic read failed: %s\n", esp_err_to_name(err));
            break;
        }
        sample_count += got;
        if (got > 0 && sample_count - last_debug_sample_count >= VOICE_SAMPLE_RATE_HZ) {
            voice_debug_pcm_stats(&pcm[sample_count - got], got, sample_count);
            last_debug_sample_count = sample_count;
        }

        int cur_stop = KEY2;
        if (last_stop == 1 && cur_stop == 0) {
            printf("VOICE: stop key pressed\n");
            break;
        }
        last_stop = cur_stop;
    }

    uint32_t duration_ms = (uint32_t)((xTaskGetTickCount() - start) * portTICK_PERIOD_MS);
    printf("VOICE: recording finished, samples=%u duration=%u ms\n",
           (unsigned)sample_count, (unsigned)duration_ms);

    if (sample_count < VOICE_SAMPLE_RATE_HZ / 5) {
        printf("VOICE: recording too short, skip upload\n");
        free(wav);
        return NULL;
    }

    size_t total_len = 44 + sample_count * sizeof(int16_t);
    int16_t *pcm = (int16_t *)(wav + 44);
    wav_write_header(wav, (uint32_t)(sample_count * sizeof(int16_t)));
    voice_debug_pcm_stats(pcm, sample_count < VOICE_SAMPLE_RATE_HZ ? sample_count : VOICE_SAMPLE_RATE_HZ,
                          sample_count);
    *wav_len = total_len;
    printf("VOICE_DBG: wav built bytes=%u pcm_bytes=%u sample_rate=%d bits=16 channels=1\n",
           (unsigned)total_len, (unsigned)(sample_count * sizeof(int16_t)),
           VOICE_SAMPLE_RATE_HZ);
    voice_debug_dump_bytes("wav header", wav, total_len < 44 ? total_len : 44);
    if (total_len > 44) {
        voice_debug_dump_bytes("wav pcm data", wav + 44, total_len - 44);
    }
    return wav;
}

static int http_write_all(esp_http_client_handle_t client, const char *data, int len)
{
    int sent = 0;
    while (sent < len) {
        int ret = esp_http_client_write(client, data + sent, len - sent);
        if (ret <= 0) {
            return ret;
        }
        sent += ret;
    }
    return sent;
}

static char *http_read_text_response(esp_http_client_handle_t client, int max_len)
{
    char *buf = calloc(1, max_len + 1);
    if (!buf) {
        return NULL;
    }

    int total = 0;
    while (total < max_len) {
        int ret = esp_http_client_read(client, buf + total, max_len - total);
        if (ret <= 0) {
            break;
        }
        total += ret;
    }
    buf[total] = '\0';
    return buf;
}

static char *voice_upload_wav(const uint8_t *wav, size_t wav_len)
{
    voice_log_mem("upload_before");
    static const char *boundary = "----esp32s3-voice-boundary";
    char head[512];
    char tail[128];
    int head_len = snprintf(head, sizeof(head),
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"device_id\"\r\n\r\n"
        "%s\r\n"
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"format\"\r\n\r\n"
        "wav\r\n"
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"sample_rate\"\r\n\r\n"
        "%d\r\n"
        "--%s\r\n"
        "Content-Disposition: form-data; name=\"audio\"; filename=\"record.wav\"\r\n"
        "Content-Type: audio/wav\r\n\r\n",
        boundary, VOICE_DEVICE_ID, boundary, boundary, VOICE_SAMPLE_RATE_HZ, boundary);
    int tail_len = snprintf(tail, sizeof(tail), "\r\n--%s--\r\n", boundary);

    if (head_len <= 0 || head_len >= (int)sizeof(head) ||
        tail_len <= 0 || tail_len >= (int)sizeof(tail)) {
        printf("VOICE: multipart header build failed\n");
        return NULL;
    }

    esp_http_client_config_t config = {
        .url = VOICE_CHAT_URL,
        .timeout_ms = VOICE_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        printf("VOICE: http client init failed\n");
        return NULL;
    }

    char content_type[96];
    snprintf(content_type, sizeof(content_type), "multipart/form-data; boundary=%s", boundary);
    esp_http_client_set_method(client, HTTP_METHOD_POST);
    esp_http_client_set_header(client, "Content-Type", content_type);

    int content_len = head_len + (int)wav_len + tail_len;
    printf("VOICE_DBG: upload multipart head=%d wav=%u tail=%d total=%d url=%s\n",
           head_len, (unsigned)wav_len, tail_len, content_len, VOICE_CHAT_URL);
    voice_debug_dump_bytes("upload wav", wav, wav_len);
    esp_err_t err = esp_http_client_open(client, content_len);
    if (err != ESP_OK) {
        printf("VOICE: POST open failed: %s\n", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return NULL;
    }

    int ok = http_write_all(client, head, head_len);
    if (ok > 0) {
        ok = http_write_all(client, (const char *)wav, (int)wav_len);
    }
    if (ok > 0) {
        ok = http_write_all(client, tail, tail_len);
    }
    if (ok <= 0) {
        printf("VOICE: POST write failed\n");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    int fetch_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    printf("VOICE: POST status=%d content_len=%d\n", status, fetch_len);
    char *response = http_read_text_response(client, 4096);
    if (response) {
        printf("VOICE_DBG: POST response bytes=%u\n", (unsigned)strlen(response));
    }
    esp_http_client_close(client);
    esp_http_client_cleanup(client);

    if (status != 200) {
        printf("VOICE: POST failed, status=%d body=%s\n", status, response ? response : "");
        free(response);
        return NULL;
    }
    return response;
}

static bool json_ok_true(const char *json)
{
    const char *p = strstr(json, "\"ok\"");
    if (!p) {
        return false;
    }
    p = strchr(p, ':');
    if (!p) {
        return false;
    }
    do {
        p++;
    } while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n');
    return strncmp(p, "true", 4) == 0;
}

static bool json_get_string(const char *json, const char *key, char *out, size_t out_len)
{
    char pattern[48];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);
    const char *p = strstr(json, pattern);
    if (!p) {
        return false;
    }
    p = strchr(p, ':');
    if (!p) {
        return false;
    }
    p = strchr(p, '"');
    if (!p) {
        return false;
    }
    p++;

    size_t n = 0;
    while (*p && *p != '"' && n + 1 < out_len) {
        if (*p == '\\' && p[1]) {
            p++;
        }
        out[n++] = *p++;
    }
    out[n] = '\0';
    return n > 0;
}

static esp_err_t voice_stream_mp3_url(const char *url)
{
    voice_log_mem("download_before");
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = VOICE_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        printf("VOICE: GET client init failed\n");
        return ESP_ERR_NO_MEM;
    }

    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        printf("VOICE: GET open failed: %s\n", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return err;
    }

    int content_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    printf("VOICE: GET status=%d content_len=%d\n", status, content_len);
    if (status != 200) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_FAIL;
    }

    voice_log_mem("playback_before");
    esp_err_t stream_err = audio_io_playback_begin();
    if (stream_err != ESP_OK) {
        printf("VOICE: playback begin failed: %s\n", esp_err_to_name(stream_err));
        voice_log_mem("playback_begin_failed");
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return stream_err;
    }

    mp3dec_t dec;
    mp3dec_init(&dec);

    int16_t *pcm = heap_caps_malloc(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!pcm) {
        printf("VOICE: allocate mp3 pcm buffer failed (%u bytes), free_heap=%u largest_internal=%u largest_spiram=%u\n",
               (unsigned)(MINIMP3_MAX_SAMPLES_PER_FRAME * sizeof(int16_t)),
               (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
               (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
        audio_io_playback_end();
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    uint8_t *mp3_buf = voice_malloc_spiram(VOICE_MP3_STREAM_BUFFER_SIZE, "mp3 stream buffer");
    if (!mp3_buf) {
        free(pcm);
        audio_io_playback_end();
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return ESP_ERR_NO_MEM;
    }

    bool eof = false;
    size_t have = 0;
    size_t total_read = 0;
    int frame_index = 0;
    esp_err_t play_err = ESP_OK;
    while (!eof || have > 0) {
        while (!eof && have <= VOICE_MP3_STREAM_BUFFER_SIZE - VOICE_HTTP_READ_CHUNK_SIZE) {
            int ret = esp_http_client_read(client, (char *)mp3_buf + have,
                                           VOICE_HTTP_READ_CHUNK_SIZE);
            if (ret < 0) {
                printf("VOICE: GET read failed ret=%d\n", ret);
                play_err = ESP_FAIL;
                eof = true;
                break;
            }
            if (ret == 0) {
                eof = true;
                break;
            }
            if (total_read == 0) {
                voice_debug_dump_bytes("mp3 stream first bytes", mp3_buf, (size_t)ret);
            }
            have += (size_t)ret;
            total_read += (size_t)ret;
            printf("VOICE_DBG: GET stream chunk=%d total=%u buffered=%u\n",
                   ret, (unsigned)total_read, (unsigned)have);
        }

        if (have == 0) {
            continue;
        }

        mp3dec_frame_info_t info = {0};
        int samples = mp3dec_decode_frame(&dec, mp3_buf, (int)have, pcm, &info);
        if (info.frame_bytes == 0) {
            if (eof) {
                break;
            }
            if (have == VOICE_MP3_STREAM_BUFFER_SIZE) {
                memmove(mp3_buf, mp3_buf + 1, have - 1);
                have--;
            }
            continue;
        }

        size_t used = (size_t)info.frame_bytes;
        if (used > have) {
            used = have;
        }
        memmove(mp3_buf, mp3_buf + used, have - used);
        have -= used;

        if (samples > 0 && info.hz > 0 && info.channels > 0) {
            if (frame_index == 0) {
                printf("VOICE_DBG: mp3 first frame bytes=%d samples=%d hz=%d channels=%d layer=%d bitrate=%d\n",
                       info.frame_bytes, samples, info.hz, info.channels,
                       info.layer, info.bitrate_kbps);
                voice_debug_pcm_stats(pcm, (size_t)samples, (size_t)samples);
            }
            esp_err_t err = audio_io_playback_write_pcm(pcm, (size_t)samples,
                                                        info.hz, info.channels);
            if (err != ESP_OK) {
                printf("VOICE: pcm play failed at frame %d: %s\n",
                       frame_index, esp_err_to_name(err));
                voice_log_mem("playback_failed");
                play_err = err;
                break;
            }
            frame_index++;
        }
    }
    free(mp3_buf);
    free(pcm);
    audio_io_playback_end();
    printf("VOICE_DBG: mp3 playback stack free after=%u\n",
           (unsigned)uxTaskGetStackHighWaterMark(NULL));
    printf("VOICE: mp3 stream playback frames=%d bytes=%u result=%s\n",
           frame_index, (unsigned)total_read, esp_err_to_name(play_err));
    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    return play_err;
}

static void voice_chat_once(void)
{
    if (!audio_io_mic_is_ready()) {
        printf("VOICE: mic is not ready, cannot record\n");
        return;
    }
    if (!audio_io_is_ready()) {
        printf("VOICE: speaker is not ready, cannot play reply\n");
        return;
    }

    size_t wav_len = 0;
    uint8_t *wav = voice_record_wav_until_stop(&wav_len);
    if (!wav) {
        return;
    }

    printf("VOICE: uploading wav, bytes=%u\n", (unsigned)wav_len);
    char *json = voice_upload_wav(wav, wav_len);
    free(wav);
    if (!json) {
        return;
    }

    printf("VOICE: response=%s\n", json);
    if (!json_ok_true(json)) {
        printf("VOICE: server ok is not true, skip playback\n");
        free(json);
        return;
    }

    char reply_text[512];
    if (json_get_string(json, "reply_text", reply_text, sizeof(reply_text))) {
        printf("VOICE: reply_text=%s\n", reply_text);
    }

    char audio_url[256];
    if (!json_get_string(json, "audio_url", audio_url, sizeof(audio_url))) {
        printf("VOICE: audio_url missing, skip playback\n");
        free(json);
        return;
    }
    free(json);

    char full_url[384];
    snprintf(full_url, sizeof(full_url), "%s%s", VOICE_SERVER_BASE_URL, audio_url);
    printf("VOICE: streaming reply mp3: %s\n", full_url);
    esp_err_t play_err = voice_stream_mp3_url(full_url);
    if (play_err != ESP_OK) {
        printf("VOICE: mp3 stream playback failed: %s\n", esp_err_to_name(play_err));
    }
}

static void voice_chat_task(void *arg)
{
    (void)arg;
    printf("VOICE_DBG: voice task started stack_free=%u\n",
           (unsigned)uxTaskGetStackHighWaterMark(NULL));
    audio_io_set_playing(false);
    voice_chat_once();
    printf("VOICE: ready, press KEY1 again for next recording\n");
    s_voice_task_running = false;
    vTaskDelete(NULL);
}

static void ws2812b_gpio_probe(gpio_num_t gpio_num)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    vTaskDelay(pdMS_TO_TICKS(100));

    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ret = gpio_config(&io_conf);
    vTaskDelay(pdMS_TO_TICKS(100));

    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        return;
    }

    gpio_set_level(gpio_num, 1);
    vTaskDelay(pdMS_TO_TICKS(300));

    gpio_set_level(gpio_num, 0);
    vTaskDelay(pdMS_TO_TICKS(300));

    gpio_config_t idle_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&idle_conf);
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_data;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        printf("wifi connecting to %s\n", WIFI_STA_SSID);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        printf("wifi connected to ap, waiting for ip\n");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_wifi_retry_num < WIFI_MAXIMUM_RETRY) {
            s_wifi_retry_num++;
            printf("wifi disconnected, retry %d/%d\n", s_wifi_retry_num, WIFI_MAXIMUM_RETRY);
            esp_wifi_connect();
        } else {
            printf("wifi failed after %d retries\n", WIFI_MAXIMUM_RETRY);
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    }
}

static void ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data)
{
    (void)arg;
    (void)event_base;
    (void)event_id;

    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    s_wifi_retry_num = 0;
    printf("wifi got ip: " IPSTR "\n", IP2STR(&event->ip_info.ip));
    xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
}

static void wifi_init_storage(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

static void wifi_sta_init(void)
{
    wifi_init_storage();

    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(s_wifi_event_group == NULL ? ESP_ERR_NO_MEM : ESP_OK);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &ip_event_handler, NULL));

    wifi_config_t wifi_config = { 0 };
    snprintf((char *)wifi_config.sta.ssid, sizeof(wifi_config.sta.ssid), "%s", WIFI_STA_SSID);
    snprintf((char *)wifi_config.sta.password, sizeof(wifi_config.sta.password), "%s", WIFI_STA_PASSWORD);
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    printf("wifi init done\n");
}

void app_main(void)
{
    board_pins_t pins = board_config_get_default();

    printf("BanZiXueXi firmware start (ESP32-S3)\n");
#if CONFIG_SPIRAM
    size_t psram_size = esp_psram_get_size();
    printf("Found %uMB PSRAM, SPI RAM enabled, largest_spiram=%u\n",
           (unsigned)(psram_size / (1024 * 1024)),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
#else
    printf("VOICE: PSRAM is disabled in sdkconfig\n");
#endif
    voice_log_mem("boot");
    printf("AUDIO_DBG board pins: MCLK=%d BCLK=%d WS=%d DOUT=%d\n",
           pins.gpio_speaker_mclk, pins.gpio_speaker_bclk,
           pins.gpio_speaker_ws, pins.gpio_speaker_dout);
    printf("VOICE controls: KEY1 starts recording, KEY2 stops recording and sends to server\n");

    wifi_sta_init();
    EventBits_t wifi_bits = xEventGroupWaitBits(s_wifi_event_group,
                                                WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                                pdFALSE, pdFALSE,
                                                pdMS_TO_TICKS(30000));
    if (wifi_bits & WIFI_CONNECTED_BIT) {
        printf("wifi ready for voice chat\n");
    } else {
        printf("wifi is not connected yet; voice upload will fail until network is ready\n");
    }

    i2c_obj_t i2c = iic_init(I2C_NUM_0);
    xl9555_init(i2c);

    esp_err_t audio_ret = audio_io_init(&pins, &i2c);
    if (audio_ret != ESP_OK) {
        printf("audio_io_init failed: %s\n", esp_err_to_name(audio_ret));
    }
    led_group_init(&pins);
    servo_init(&pins);
    ws2812b_gpio_probe(WS2812B_GPIO);
    esp_err_t ws2812_ret = ws2812b_init(WS2812B_GPIO, WS2812B_LED_COUNT);
    if (ws2812_ret == ESP_OK) {
        ws2812b_fill(0, WS2812B_TEST_BRIGHTNESS, 0);
    }

    int angle_s3 = 0;
    servo_set_angle(SERVO_3, (uint8_t)angle_s3);

    vTaskDelay(pdMS_TO_TICKS(200));

    int last_key1 = 1;
    int last_key1_log = last_key1;
    int last_key0 = 0;
    int last_key2 = 1;
    int last_key2_log = last_key2;
    TickType_t last_ws2812_tick = xTaskGetTickCount();
    uint8_t ws2812_step = 0;
    const ws2812b_color_t ws2812_demo_colors[] = {
        { .red = WS2812B_TEST_BRIGHTNESS, .green = 0, .blue = 0 },
        { .red = 0, .green = WS2812B_TEST_BRIGHTNESS, .blue = 0 },
        { .red = 0, .green = 0, .blue = WS2812B_TEST_BRIGHTNESS },
        { .red = WS2812B_TEST_BRIGHTNESS, .green = WS2812B_TEST_BRIGHTNESS, .blue = WS2812B_TEST_BRIGHTNESS },
    };

    while (1) {
        int cur1 = KEY1;
        if (cur1 != last_key1_log) {
            printf("VOICE: KEY1 level changed: %d -> %d\n", last_key1_log, cur1);
            last_key1_log = cur1;
        }
        if (last_key1 == 1 && cur1 == 0) {
            printf("VOICE: KEY1 falling edge, start voice chat\n");
            if (s_voice_task_running) {
                printf("VOICE: voice task already running, ignore KEY1\n");
            } else {
                s_voice_task_running = true;
                BaseType_t ok = xTaskCreatePinnedToCore(voice_chat_task, "voice_chat",
                                                        12288, NULL, 5, NULL,
                                                        tskNO_AFFINITY);
                if (ok != pdPASS) {
                    s_voice_task_running = false;
                    printf("VOICE: create voice task failed\n");
                }
            }
        }
        last_key1 = cur1;

        int cur0 = KEY0;
        if (last_key0 == 0 && cur0 == 1) {
            angle_s3 += 15;
            if (angle_s3 > 180) {
                angle_s3 = 180;
            }
            servo_set_angle(SERVO_3, (uint8_t)angle_s3);
            printf("SERVO_3 (GPIO5) angle = %d\n", angle_s3);
        }
        last_key0 = cur0;

        int cur2 = KEY2;
        if (cur2 != last_key2_log) {
            printf("VOICE: KEY2 level changed: %d -> %d\n", last_key2_log, cur2);
            last_key2_log = cur2;
        }
        if (last_key2 == 1 && cur2 == 0) {
            printf("VOICE: KEY2 falling edge (only stops upload while recording)\n");
        }
        last_key2 = cur2;

        if (ws2812_ret == ESP_OK &&
            xTaskGetTickCount() - last_ws2812_tick >= pdMS_TO_TICKS(500)) {
            const ws2812b_color_t color = ws2812_demo_colors[ws2812_step];
            ws2812b_fill(color.red, color.green, color.blue);
            ws2812_step = (ws2812_step + 1) %
                          (sizeof(ws2812_demo_colors) / sizeof(ws2812_demo_colors[0]));
            last_ws2812_tick = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
