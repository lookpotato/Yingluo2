#include <stdbool.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#include "audio_io.h"
#include "board_config.h"
#include "iic.h"
#include "led_group.h"
#include "servo.h"
#include "xl9555.h"

void app_main(void)
{
    board_pins_t pins = board_config_get_default();

    printf("BanZiXueXi firmware start (ESP32-S3)\n");
    printf("AUDIO_DBG board pins: MCLK=%d BCLK=%d WS=%d DOUT=%d\n",
           pins.gpio_speaker_mclk, pins.gpio_speaker_bclk,
           pins.gpio_speaker_ws, pins.gpio_speaker_dout);
    printf("AUDIO_DBG controls: KEY2 toggles speaker test, SPK_EN_IO active level is low in current code\n");

    i2c_obj_t i2c = iic_init(I2C_NUM_0);
    xl9555_init(i2c);

    esp_err_t audio_ret = audio_io_init(&pins, &i2c);
    if (audio_ret != ESP_OK) {
        printf("audio_io_init failed: %s\n", esp_err_to_name(audio_ret));
    }
    led_group_init(&pins);
    servo_init(&pins);

    int angle_s1 = 0;
    int angle_s3 = 0;
    servo_set_angle(SERVO_1, (uint8_t)angle_s1);
    servo_set_angle(SERVO_3, (uint8_t)angle_s3);

    // 上电稳定一下
    vTaskDelay(pdMS_TO_TICKS(200));

    int last_key1 = 0;
    int last_key0 = 0;
    int last_key2 = 1;
    int last_key2_log = last_key2;
    bool speaker_on = false;

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

        /* KEY2：未按下为 1，按下为 0；在按下沿切换喇叭试音 */
        int cur2 = KEY2;
        if (cur2 != last_key2_log) {
            printf("AUDIO_DBG KEY2 level changed: %d -> %d\n", last_key2_log, cur2);
            last_key2_log = cur2;
        }
        if (last_key2 == 1 && cur2 == 0) {
            speaker_on = !speaker_on;
            printf("AUDIO_DBG KEY2 falling edge, speaker_on=%d\n", speaker_on);
            audio_io_set_playing(speaker_on);
            printf("扬声器试音 %s (audio ready=%d)\n", speaker_on ? "开" : "关", audio_io_is_ready());
        }
        last_key2 = cur2;

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
