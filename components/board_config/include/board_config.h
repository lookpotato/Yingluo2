#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

typedef struct {
    int gpio_speaker_mclk;
    int gpio_speaker_bclk;
    int gpio_speaker_ws;
    int gpio_speaker_dout;

    int gpio_mic_bclk;
    int gpio_mic_ws;
    int gpio_mic_din;

    int gpio_servo_1;
    int gpio_servo_2;
    int gpio_servo_3;

    int gpio_led_group_a;
    int gpio_led_group_b;
} board_pins_t;

board_pins_t board_config_get_default(void);

#endif
