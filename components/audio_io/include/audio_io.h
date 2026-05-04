#ifndef AUDIO_IO_H
#define AUDIO_IO_H

#include <stdbool.h>

#include "esp_err.h"

#include "board_config.h"
#include "iic.h"

esp_err_t audio_io_init(const board_pins_t *pins, i2c_obj_t *i2c);
void audio_io_set_playing(bool play);
bool audio_io_is_ready(void);

#endif
