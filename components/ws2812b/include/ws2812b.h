#ifndef WS2812B_H
#define WS2812B_H

#include <stdint.h>

#include "esp_err.h"
#include "hal/gpio_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef WS2812B_DEFAULT_GPIO
#define WS2812B_DEFAULT_GPIO GPIO_NUM_6
#endif

#ifndef WS2812B_DEFAULT_LED_COUNT
#define WS2812B_DEFAULT_LED_COUNT 1
#endif

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} ws2812b_color_t;

esp_err_t ws2812b_init(gpio_num_t gpio_num, uint32_t led_count);
esp_err_t ws2812b_set_pixel(uint32_t index, uint8_t red, uint8_t green, uint8_t blue);
esp_err_t ws2812b_fill(uint8_t red, uint8_t green, uint8_t blue);
esp_err_t ws2812b_refresh(void);
esp_err_t ws2812b_clear(void);

#ifdef __cplusplus
}
#endif

#endif
