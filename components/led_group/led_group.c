#include "led_group.h"

#include "driver/gpio.h"

static int s_led_gpio_a = -1;
static int s_led_gpio_b = -1;

void led_group_init(const board_pins_t *pins)
{
    s_led_gpio_a = pins->gpio_led_group_a;
    s_led_gpio_b = pins->gpio_led_group_b;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << s_led_gpio_a) | (1ULL << s_led_gpio_b),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    gpio_set_level(s_led_gpio_a, 0);
    gpio_set_level(s_led_gpio_b, 0);
}

void led_group_set(led_group_id_t id, bool on)
{
    int gpio = (id == LED_GROUP_A) ? s_led_gpio_a : s_led_gpio_b;
    if (gpio < 0) {
        return;
    }
    gpio_set_level(gpio, on ? 1 : 0);
}
