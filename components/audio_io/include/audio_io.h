#ifndef AUDIO_IO_H
#define AUDIO_IO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#include "board_config.h"
#include "iic.h"

esp_err_t audio_io_init(const board_pins_t *pins, i2c_obj_t *i2c);
void audio_io_set_playing(bool play);
bool audio_io_is_ready(void);
bool audio_io_mic_is_ready(void);
void audio_io_log_state(const char *tag);
esp_err_t audio_io_read_mic_mono16(int16_t *samples, size_t sample_count,
                                   size_t *samples_read, TickType_t timeout_ticks);
esp_err_t mic_init(void);
size_t mic_read_pcm(uint8_t *buf, size_t len);
esp_err_t audio_io_play_pcm(const int16_t *pcm, size_t frame_count,
                            int sample_rate_hz, int channel_count);
esp_err_t audio_io_playback_begin(void);
esp_err_t audio_io_playback_write_pcm(const int16_t *pcm, size_t frame_count,
                                      int sample_rate_hz, int channel_count);
void audio_io_playback_end(void);

#endif
