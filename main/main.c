#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    i2c_obj_t i2c = iic_init(I2C_NUM_0);
    xl9555_init(i2c);

    audio_io_init(&pins);
    led_group_init(&pins);
    servo_init(&pins);

    int angle = 0;
    servo_set_angle(SERVO_1, (uint8_t)angle);

    /* 上电瞬间 I2C/XL9555 读数不可靠，易把输入全读成 0 → 误判 KEY1 一直按下，角度被连续加满 */
    vTaskDelay(pdMS_TO_TICKS(300));

    /* 连续约 250ms 读到 KEY1 松开(1)，才认为总线可信；否则先不武装，等真正松开 */
    int key1_armed = 0;
    {
        int stable_ticks = 0;
        for (int t = 0; t < 400; t++) {
            if (KEY1 == 1) {
                if (++stable_ticks >= 25) {
                    key1_armed = 1;
                    break;
                }
            } else {
                stable_ticks = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    /* KEY1 见 xl9555.h；按下为 0。每完整“按下再松开”只加 15° */

    while (1) {
        if (KEY1 == 0) {
            if (key1_armed) {
                vTaskDelay(pdMS_TO_TICKS(25));
                if (KEY1 == 0) {
                    angle += 15;
                    if (angle > 180) {
                        angle = 180;
                    }
                    servo_set_angle(SERVO_1, (uint8_t)angle);
                    key1_armed = 0;
                }
            }
        } else {
            key1_armed = 1;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
