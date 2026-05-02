#ifndef LED_GROUP_H
#define LED_GROUP_H

#include <stdbool.h>

#include "board_config.h"

typedef enum {
    LED_GROUP_A = 0,
    LED_GROUP_B = 1,
} led_group_id_t;

void led_group_init(const board_pins_t *pins);
void led_group_set(led_group_id_t id, bool on);

#endif
