#include "servo.h"

#include <stdbool.h>

#include "driver/ledc.h"

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define SERVO_FREQ_HZ 50
#define SERVO_PERIOD_US 20000
#define SERVO_DUTY_BITS LEDC_TIMER_14_BIT

static bool s_ready = false;

static void servo_config_channel(ledc_channel_t channel, int gpio)
{
    ledc_channel_config_t cfg = {
        .gpio_num = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&cfg);
}

void servo_init(const board_pins_t *pins)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = SERVO_DUTY_BITS,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    servo_config_channel(LEDC_CHANNEL_0, pins->gpio_servo_1);
    servo_config_channel(LEDC_CHANNEL_1, pins->gpio_servo_2);

    s_ready = true;
}

void servo_set_angle(servo_id_t id, uint8_t angle)
{
    if (!s_ready) {
        return;
    }

    if (angle > 180) {
        angle = 180;
    }

    uint32_t max_duty = (1U << 14) - 1U;
    uint32_t pulse_us = SERVO_MIN_US + ((SERVO_MAX_US - SERVO_MIN_US) * angle) / 180U;
    uint32_t duty = (pulse_us * max_duty) / SERVO_PERIOD_US;
    ledc_channel_t channel = (id == SERVO_1) ? LEDC_CHANNEL_0 : LEDC_CHANNEL_1;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, channel, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, channel);
}
