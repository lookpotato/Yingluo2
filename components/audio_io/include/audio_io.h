#ifndef AUDIO_IO_H
#define AUDIO_IO_H

#include "esp_err.h"

#include "board_config.h"

esp_err_t audio_io_init(const board_pins_t *pins);

#endif
