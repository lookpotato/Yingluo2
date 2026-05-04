#ifndef ES8388_HW_H
#define ES8388_HW_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "iic.h"

#define ES8388_I2C_ADDR_PRIMARY   0x10
#define ES8388_I2C_ADDR_FALLBACK  0x11

esp_err_t es8388_hw_init_for_speaker(i2c_obj_t *i2c);
esp_err_t es8388_hw_mute(i2c_obj_t *i2c, bool mute);
uint8_t es8388_hw_get_addr(void);

#endif
