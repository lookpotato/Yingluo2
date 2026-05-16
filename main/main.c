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
#define VOICE_MAX_RECORD_MS 30000
#define VOICE_RECORD_CHUNK_SAMPLES 512
#define VOICE_HTTP_TIMEOUT_MS 90000

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define WS2812B_GPIO GPIO_NUM_6
#define WS2812B_LED_COUNT 1
#define WS2812B_TEST_BRIGHTNESS 96

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_num;

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

static uint8_t *voice_record_wav_until_stop(size_t *wav_len)
{
    const size_t max_samples = (VOICE_SAMPLE_RATE_HZ * VOICE_MAX_RECORD_MS) / 1000;
    int16_t *pcm = heap_caps_malloc(max_samples * sizeof(int16_t), MALLOC_CAP_8BIT);
    if (!pcm) {
        printf("VOICE: allocate pcm buffer failed (%u bytes)\n",
               (unsigned)(max_samples * sizeof(int16_t)));
        return NULL;
    }

    printf("VOICE: recording started, press KEY2 to stop and upload, max=%d ms\n",
           VOICE_MAX_RECORD_MS);
    size_t sample_count = 0;
    int last_stop = KEY2;
    TickType_t start = xTaskGetTickCount();

    while (sample_count < max_samples) {
        size_t got = 0;
        size_t room = max_samples - sample_count;
        if (room > VOICE_RECORD_CHUNK_SAMPLES) {
            room = VOICE_RECORD_CHUNK_SAMPLES;
        }

        esp_err_t err = audio_io_read_mic_mono16(&pcm[sample_count], room, &got,
                                                 pdMS_TO_TICKS(1000));
        if (err != ESP_OK) {
            printf("VOICE: mic read failed: %s\n", esp_err_to_name(err));
            break;
        }
        sample_count += got;

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
        free(pcm);
        return NULL;
    }

    size_t total_len = 44 + sample_count * sizeof(int16_t);
    uint8_t *wav = heap_caps_malloc(total_len, MALLOC_CAP_8BIT);
    if (!wav) {
        printf("VOICE: allocate wav buffer failed (%u bytes)\n", (unsigned)total_len);
        free(pcm);
        return NULL;
    }

    wav_write_header(wav, (uint32_t)(sample_count * sizeof(int16_t)));
    memcpy(wav + 44, pcm, sample_count * sizeof(int16_t));
    free(pcm);
    *wav_len = total_len;
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

static uint8_t *http_download_binary(const char *url, size_t *out_len)
{
    esp_http_client_config_t config = {
        .url = url,
        .timeout_ms = VOICE_HTTP_TIMEOUT_MS,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (!client) {
        return NULL;
    }

    esp_http_client_set_method(client, HTTP_METHOD_GET);
    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        printf("VOICE: GET open failed: %s\n", esp_err_to_name(err));
        esp_http_client_cleanup(client);
        return NULL;
    }

    int content_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    printf("VOICE: GET status=%d content_len=%d\n", status, content_len);
    if (status != 200) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    size_t cap = content_len > 0 ? (size_t)content_len : 8192;
    uint8_t *buf = malloc(cap);
    if (!buf) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
        return NULL;
    }

    size_t total = 0;
    while (1) {
        if (total == cap) {
            size_t new_cap = cap * 2;
            uint8_t *new_buf = realloc(buf, new_cap);
            if (!new_buf) {
                free(buf);
                esp_http_client_close(client);
                esp_http_client_cleanup(client);
                return NULL;
            }
            buf = new_buf;
            cap = new_cap;
        }

        int ret = esp_http_client_read(client, (char *)buf + total, cap - total);
        if (ret <= 0) {
            break;
        }
        total += ret;
    }

    esp_http_client_close(client);
    esp_http_client_cleanup(client);
    *out_len = total;
    return buf;
}

static void voice_play_mp3(const uint8_t *mp3, size_t mp3_len)
{
    mp3dec_t dec;
    mp3dec_init(&dec);

    esp_err_t stream_err = audio_io_playback_begin();
    if (stream_err != ESP_OK) {
        printf("VOICE: playback begin failed: %s\n", esp_err_to_name(stream_err));
        return;
    }

    size_t offset = 0;
    int frame_index = 0;
    while (offset < mp3_len) {
        mp3dec_frame_info_t info = {0};
        int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
        int samples = mp3dec_decode_frame(&dec, mp3 + offset, (int)(mp3_len - offset),
                                          pcm, &info);
        if (info.frame_bytes == 0) {
            offset++;
            continue;
        }
        offset += info.frame_bytes;
        if (samples > 0 && info.hz > 0 && info.channels > 0) {
            esp_err_t err = audio_io_playback_write_pcm(pcm, (size_t)samples,
                                                        info.hz, info.channels);
            if (err != ESP_OK) {
                printf("VOICE: pcm play failed at frame %d: %s\n",
                       frame_index, esp_err_to_name(err));
                break;
            }
            frame_index++;
        }
    }
    audio_io_playback_end();
    printf("VOICE: mp3 playback frames=%d bytes=%u\n", frame_index, (unsigned)mp3_len);
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
    printf("VOICE: downloading reply mp3: %s\n", full_url);

    size_t mp3_len = 0;
    uint8_t *mp3 = http_download_binary(full_url, &mp3_len);
    if (!mp3 || mp3_len == 0) {
        printf("VOICE: mp3 download failed or empty\n");
        free(mp3);
        return;
    }

    voice_play_mp3(mp3, mp3_len);
    free(mp3);
}

static void ws2812b_gpio_probe(gpio_num_t gpio_num)
{
    printf("WS2812B_PROBE start: temporarily testing GPIO%d pad level\n", gpio_num);

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t ret = gpio_config(&io_conf);
    printf("WS2812B_PROBE input pull-up config(GPIO%d) = %s\n", gpio_num, esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("WS2812B_PROBE GPIO%d input with internal pull-up, readback=%d\n",
           gpio_num, gpio_get_level(gpio_num));

    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ret = gpio_config(&io_conf);
    printf("WS2812B_PROBE input pull-down config(GPIO%d) = %s\n", gpio_num, esp_err_to_name(ret));
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("WS2812B_PROBE GPIO%d input with internal pull-down, readback=%d\n",
           gpio_num, gpio_get_level(gpio_num));

    io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ret = gpio_config(&io_conf);
    printf("WS2812B_PROBE input/output config(GPIO%d) = %s\n", gpio_num, esp_err_to_name(ret));
    if (ret != ESP_OK) {
        return;
    }

    ret = gpio_set_level(gpio_num, 1);
    vTaskDelay(pdMS_TO_TICKS(300));
    int high_level = gpio_get_level(gpio_num);
    printf("WS2812B_PROBE GPIO%d forced HIGH ret=%s, readback=%d\n",
           gpio_num, esp_err_to_name(ret), high_level);

    ret = gpio_set_level(gpio_num, 0);
    vTaskDelay(pdMS_TO_TICKS(300));
    int low_level = gpio_get_level(gpio_num);
    printf("WS2812B_PROBE GPIO%d forced LOW ret=%s, readback=%d\n",
           gpio_num, esp_err_to_name(ret), low_level);

    if (high_level != 1) {
        printf("WS2812B_PROBE WARNING: GPIO%d did not read HIGH. Disconnect WS2812B DIN from GPIO%d and reboot to test whether the external wire/light is pulling it low.\n",
               gpio_num, gpio_num);
    } else if (low_level != 0) {
        printf("WS2812B_PROBE WARNING: GPIO%d did not read LOW. Check whether it is pulled high externally.\n",
               gpio_num);
    } else {
        printf("WS2812B_PROBE GPIO%d basic output test passed.\n", gpio_num);
    }

    gpio_config_t idle_conf = {
        .pin_bit_mask = 1ULL << gpio_num,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&idle_conf);
    printf("WS2812B_PROBE done: if readback changed but LED stays dark, check DIN direction, GND, 5V, or 3.3V data level compatibility\n");
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
    printf("WS2812B wiring expected: ESP32-S3 GPIO%d -> WS2812B DIN, VDD -> 5V, GND common\n", WS2812B_GPIO);
    printf("WS2812B warning: GPIO%d also maps to LCD_B7 / OV_D2 on this board\n", WS2812B_GPIO);
    ws2812b_gpio_probe(WS2812B_GPIO);
    esp_err_t ws2812_ret = ws2812b_init(WS2812B_GPIO, WS2812B_LED_COUNT);
    if (ws2812_ret != ESP_OK) {
        printf("WS2812B init on GPIO%d failed: %s\n", WS2812B_GPIO, esp_err_to_name(ws2812_ret));
    } else {
        printf("WS2812B ready on GPIO%d, LED count=%d\n", WS2812B_GPIO, WS2812B_LED_COUNT);
        printf("NOTE: GPIO6 is also routed as LCD_B7 / OV_D2 on this board; do not use RGB LCD or camera on the same IO at the same time.\n");
        esp_err_t fill_ret = ws2812b_fill(0, WS2812B_TEST_BRIGHTNESS, 0);
        printf("WS2812B_TEST initial green send ret=%s\n", esp_err_to_name(fill_ret));
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
            audio_io_set_playing(false);
            voice_chat_once();
            printf("VOICE: ready, press KEY1 again for next recording\n");
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
        last_key2 = cur2;

        if (ws2812_ret == ESP_OK &&
            xTaskGetTickCount() - last_ws2812_tick >= pdMS_TO_TICKS(500)) {
            const ws2812b_color_t color = ws2812_demo_colors[ws2812_step];
            esp_err_t fill_ret = ws2812b_fill(color.red, color.green, color.blue);
            printf("WS2812B_TEST send step=%u RGB=(%u,%u,%u), ret=%s\n",
                   ws2812_step, color.red, color.green, color.blue, esp_err_to_name(fill_ret));
            ws2812_step = (ws2812_step + 1) %
                          (sizeof(ws2812_demo_colors) / sizeof(ws2812_demo_colors[0]));
            last_ws2812_tick = xTaskGetTickCount();
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
