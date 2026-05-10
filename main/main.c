#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"

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

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

#define WS2812B_GPIO GPIO_NUM_6
#define WS2812B_LED_COUNT 1
#define WS2812B_TEST_BRIGHTNESS 96

static EventGroupHandle_t s_wifi_event_group;
static int s_wifi_retry_num;

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
    printf("AUDIO_DBG controls: KEY2 toggles speaker test, SPK_EN_IO active level is low in current code\n");

    wifi_sta_init();

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

    int angle_s1 = 0;
    int angle_s3 = 0;
    servo_set_angle(SERVO_1, (uint8_t)angle_s1);
    servo_set_angle(SERVO_3, (uint8_t)angle_s3);

    vTaskDelay(pdMS_TO_TICKS(200));

    int last_key1 = 0;
    int last_key0 = 0;
    int last_key2 = 1;
    int last_key2_log = last_key2;
    bool speaker_on = false;
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
        if (last_key1 == 0 && cur1 == 1) {
            angle_s1 += 15;
            if (angle_s1 > 180) {
                angle_s1 = 180;
            }
            servo_set_angle(SERVO_1, (uint8_t)angle_s1);
            printf("SERVO_1 angle = %d\n", angle_s1);
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
            printf("AUDIO_DBG KEY2 level changed: %d -> %d\n", last_key2_log, cur2);
            last_key2_log = cur2;
        }
        if (last_key2 == 1 && cur2 == 0) {
            speaker_on = !speaker_on;
            printf("AUDIO_DBG KEY2 falling edge, speaker_on=%d\n", speaker_on);
            audio_io_set_playing(speaker_on);
            printf("AUDIO_DBG speaker test %s (audio ready=%d)\n",
                   speaker_on ? "on" : "off", audio_io_is_ready());
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
