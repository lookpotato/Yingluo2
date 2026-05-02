#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

#include "board_config.h"

typedef enum {
    SERVO_1 = 0,
    SERVO_2 = 1,
} servo_id_t;

void servo_init(const board_pins_t *pins);
void servo_set_angle(servo_id_t id, uint8_t angle);

#endif
