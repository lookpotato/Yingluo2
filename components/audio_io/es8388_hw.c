#include "es8388_hw.h"

#include "esp_log.h"
#include "iic.h"

static const char *TAG = "es8388_hw";
static uint8_t s_es8388_addr = ES8388_I2C_ADDR_PRIMARY;

/* 寄存器号与 Espressif esp_codec_dev / ES8388 数据手册一致 */
#define ES8388_CONTROL1        0x00
#define ES8388_CONTROL2        0x01
#define ES8388_CHIPPOWER       0x02
#define ES8388_ADCPOWER        0x03
#define ES8388_DACPOWER        0x04
#define ES8388_MASTERMODE      0x08
#define ES8388_ADCCONTROL1     0x09
#define ES8388_ADCCONTROL2     0x0a
#define ES8388_ADCCONTROL3     0x0b
#define ES8388_ADCCONTROL4     0x0c
#define ES8388_ADCCONTROL5     0x0d
#define ES8388_ADCCONTROL8     0x10
#define ES8388_ADCCONTROL9     0x11
#define ES8388_DACCONTROL1     0x17
#define ES8388_DACCONTROL2     0x18
#define ES8388_DACCONTROL3     0x19
#define ES8388_DACCONTROL4     0x1a
#define ES8388_DACCONTROL5     0x1b
#define ES8388_DACCONTROL16    0x26
#define ES8388_DACCONTROL17    0x27
#define ES8388_DACCONTROL20    0x2a
#define ES8388_DACCONTROL21    0x2b
#define ES8388_DACCONTROL23    0x2d
#define ES8388_DACCONTROL24    0x2e
#define ES8388_DACCONTROL25    0x2f
#define ES8388_DACCONTROL26    0x30
#define ES8388_DACCONTROL27    0x31

#define DAC_OUT_ALL            0x3c

static esp_err_t wr(i2c_obj_t *i2c, uint8_t reg, uint8_t val)
{
    i2c_buf_t bufs[2] = {
        {.len = 1, .buf = &reg},
        {.len = 1, .buf = &val},
    };
    return i2c_transfer(i2c, s_es8388_addr, 2, bufs, I2C_FLAG_STOP);
}

static esp_err_t rd(i2c_obj_t *i2c, uint8_t reg, uint8_t *val);

static esp_err_t wseq(i2c_obj_t *i2c, uint8_t reg, uint8_t val)
{
    esp_err_t e = wr(i2c, reg, val);
    if (e != ESP_OK) {
        ESP_LOGE(TAG, "ES8388_DBG write failed addr=0x%02x reg=0x%02x val=0x%02x err=%s",
                 s_es8388_addr, reg, val, esp_err_to_name(e));
        ESP_LOGE(TAG, "I2C 0x%02x 写寄存器 0x%02x=0x%02x 失败: %s",
                 s_es8388_addr, reg, val, esp_err_to_name(e));
        return e;
    }

    uint8_t rb = 0;
    e = rd(i2c, reg, &rb);
    if (e == ESP_OK) {
        ESP_LOGI(TAG, "ES8388_DBG write/read addr=0x%02x reg=0x%02x wrote=0x%02x read=0x%02x%s",
                 s_es8388_addr, reg, val, rb, (rb == val) ? "" : " MISMATCH");
    } else {
        ESP_LOGW(TAG, "ES8388_DBG write OK but readback failed addr=0x%02x reg=0x%02x val=0x%02x err=%s",
                 s_es8388_addr, reg, val, esp_err_to_name(e));
    }
    return ESP_OK;
}

static esp_err_t rd(i2c_obj_t *i2c, uint8_t reg, uint8_t *val)
{
    i2c_buf_t bufs[2] = {
        {.len = 1, .buf = &reg},
        {.len = 1, .buf = val},
    };
    return i2c_transfer(i2c, s_es8388_addr, 2, bufs,
                        I2C_FLAG_WRITE | I2C_FLAG_READ | I2C_FLAG_STOP);
}

esp_err_t es8388_hw_mute(i2c_obj_t *i2c, bool mute)
{
    uint8_t r = 0;
    esp_err_t e = rd(i2c, ES8388_DACCONTROL3, &r);
    if (e != ESP_OK) {
        ESP_LOGW(TAG, "ES8388_DBG mute read failed addr=0x%02x reg=0x%02x err=%s",
                 s_es8388_addr, ES8388_DACCONTROL3, esp_err_to_name(e));
        ESP_LOGW(TAG, "I2C 0x%02x 读取 mute 寄存器失败: %s", s_es8388_addr, esp_err_to_name(e));
        return e;
    }
    uint8_t before = r;
    uint8_t after = (uint8_t)((r & (uint8_t)~0x04) | (mute ? 0x04U : 0U));
    ESP_LOGI(TAG, "ES8388_DBG mute request mute=%d reg_before=0x%02x reg_after=0x%02x",
             mute, before, after);
    e = wr(i2c, ES8388_DACCONTROL3, after);
    if (e == ESP_OK) {
        uint8_t rb = 0;
        esp_err_t rb_err = rd(i2c, ES8388_DACCONTROL3, &rb);
        if (rb_err == ESP_OK) {
            ESP_LOGI(TAG, "ES8388_DBG mute readback reg=0x%02x", rb);
        } else {
            ESP_LOGW(TAG, "ES8388_DBG mute readback failed: %s", esp_err_to_name(rb_err));
        }
        ESP_LOGI(TAG, "ES8388 mute=%s (I2C 0x%02x)", mute ? "on" : "off", s_es8388_addr);
    } else {
        ESP_LOGW(TAG, "ES8388_DBG mute write failed: %s", esp_err_to_name(e));
    }
    return e;
}

esp_err_t es8388_hw_init_for_speaker(i2c_obj_t *i2c)
{
    esp_err_t err;

    /* 以下序列参考 Espressif esp-adf es8388_codec open/enable（DAC → LOUT 到后级 MD8002A） */
#define W(R, V)                                                                                      \
    do {                                                                                           \
        err = wseq(i2c, (R), (V));                                                                  \
        if (err != ESP_OK) {                                                                       \
            last_err = err;                                                                        \
            goto try_next_addr;                                                                    \
        }                                                                                          \
    } while (0)

    const uint8_t addrs[] = {ES8388_I2C_ADDR_PRIMARY, ES8388_I2C_ADDR_FALLBACK};
    esp_err_t last_err = ESP_FAIL;

    for (size_t i = 0; i < sizeof(addrs) / sizeof(addrs[0]); i++) {
        s_es8388_addr = addrs[i];
        ESP_LOGI(TAG, "ES8388_DBG init attempt index=%u addr=0x%02x",
                 (unsigned)i, s_es8388_addr);
        ESP_LOGI(TAG, "尝试初始化 ES8388 (I2C 7-bit 地址 0x%02x)", s_es8388_addr);

        W(ES8388_DACCONTROL3, 0x04);
        W(ES8388_CONTROL2, 0x50);
        W(ES8388_CHIPPOWER, 0x00);
        W(0x35, 0xa0);
        W(0x37, 0xd0);
        W(0x39, 0xd0);
        W(ES8388_MASTERMODE, 0x00); /* I2S slave */

        W(ES8388_DACPOWER, 0xc0);
        W(ES8388_CONTROL1, 0x12);
        W(ES8388_DACCONTROL1, 0x18); /* 16-bit I2S */
        W(ES8388_DACCONTROL2, 0x02);
        W(ES8388_DACCONTROL16, 0x00);
W(ES8388_DACCONTROL17, 0xB8);  // 0x27 L mixer
W(ES8388_DACCONTROL20, 0xB8);  // 0x2A R mixer
        W(ES8388_DACCONTROL21, 0x80);
        W(ES8388_DACCONTROL23, 0x00);
        W(ES8388_DACCONTROL4, 0x00);
        W(ES8388_DACCONTROL5, 0x00);
        W(ES8388_DACCONTROL24, 0x1e);
        W(ES8388_DACCONTROL25, 0x1e);
W(ES8388_DACCONTROL26, 0x14);
W(ES8388_DACCONTROL27, 0x14);
        W(ES8388_DACPOWER, DAC_OUT_ALL);

        W(ES8388_ADCPOWER, 0xff);
        W(ES8388_ADCCONTROL1, 0xbb);
        W(ES8388_ADCCONTROL2, 0x00);
        W(ES8388_ADCCONTROL3, 0x02);
        W(ES8388_ADCCONTROL4, 0x0c);
        W(ES8388_ADCCONTROL5, 0x02);
        W(ES8388_ADCCONTROL8, 0x00);
        W(ES8388_ADCCONTROL9, 0x00);
        W(ES8388_ADCPOWER, 0x09);

        W(ES8388_DACCONTROL21, 0x80);
        W(ES8388_CHIPPOWER, 0xf0);
        W(ES8388_CHIPPOWER, 0x00);
        W(ES8388_DACPOWER, 0x3c);

        err = es8388_hw_mute(i2c, false);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "ES8388 初始化完成 (I2C 7-bit 地址 0x%02x)", s_es8388_addr);
            return ESP_OK;
        }
        last_err = err;

try_next_addr:
        ESP_LOGW(TAG, "ES8388 地址 0x%02x 初始化未成功: %s",
                 s_es8388_addr, esp_err_to_name(last_err));
    }
#undef W

    ESP_LOGE(TAG, "ES8388 初始化失败，已尝试 0x%02x 和 0x%02x",
             ES8388_I2C_ADDR_PRIMARY, ES8388_I2C_ADDR_FALLBACK);
    return last_err;
}

uint8_t es8388_hw_get_addr(void)
{
    return s_es8388_addr;
}
