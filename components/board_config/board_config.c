#include "board_config.h"

board_pins_t board_config_get_default(void)
{
    board_pins_t pins = {
        /* 扬声器 I2S → ES8388（与 IIC0 的 41/42 分离，避免冲突） */
        .gpio_speaker_mclk = 3,
        .gpio_speaker_bclk = 46,
        .gpio_speaker_ws = 9,
        .gpio_speaker_dout = 10,

        .gpio_mic_bclk = 47,
        .gpio_mic_ws = 48,
        .gpio_mic_din = 39,

        .gpio_servo_1 = 4,
        .gpio_servo_2 = 8,
        .gpio_servo_3 = 5,

        /* GPIO5 已接第三路舵机；若 LED A 仍接在旧引脚请改此处 */
        .gpio_led_group_a = 15,
        .gpio_led_group_b = 6,
    };

    return pins;
}
