// SPDX-License-Identifier: MIT
#include "bosch_aux.h"

#if __has_include("bmi270.h")

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "tab5_bus.h"
#include "bmi270.h"
#include "bmm150.h"

static const char *TAG = "bosch_aux";

static struct bmi2_dev  s_bmi;
static struct bmm150_dev s_bmm;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_addr;

// --- interface callbacks -------------------------------------------------

static BMI2_INTF_RETURN_TYPE bmi_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr)
{
    return (tab5_reg_read(s_dev, reg, data, len) == ESP_OK) ? BMI2_OK : -1;
}

static BMI2_INTF_RETURN_TYPE bmi_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr)
{
    uint8_t buf[64];
    if (len + 1 > sizeof(buf)) return -1;
    buf[0] = reg;
    memcpy(buf + 1, data, len);
    return (i2c_master_transmit(s_dev, buf, len + 1, pdMS_TO_TICKS(200)) == ESP_OK) ? BMI2_OK : -1;
}

static void bmi_delay_us(uint32_t period, void *ptr)
{
    // Bosch asks for microseconds; FreeRTOS ticks are 1 ms. Busy-wait the
    // short ones rather than rounding them all up to a tick, since the config
    // upload issues thousands of them.
    if (period < 2000) {
        int64_t end = esp_timer_get_time() + period;
        while (esp_timer_get_time() < end) { }
    } else {
        vTaskDelay(pdMS_TO_TICKS(period / 1000));
    }
}

static BMM150_INTF_RET_TYPE aux_read(uint8_t reg, uint8_t *data, uint32_t len, void *ptr)
{
    return bmi2_read_aux_man_mode(reg, data, len, &s_bmi);
}

static BMM150_INTF_RET_TYPE aux_write(uint8_t reg, const uint8_t *data, uint32_t len, void *ptr)
{
    return bmi2_write_aux_man_mode(reg, data, len, &s_bmi);
}

// --- bring-up ------------------------------------------------------------

esp_err_t bosch_aux_bmm150_start(uint8_t bmi_addr)
{
    s_addr = bmi_addr;
    if (tab5_bus_add_device(bmi_addr, &s_dev) != ESP_OK) return ESP_FAIL;

    s_bmi.chip_id         = bmi_addr;
    s_bmi.read            = bmi_read;
    s_bmi.write           = bmi_write;
    s_bmi.delay_us        = bmi_delay_us;
    s_bmi.intf            = BMI2_I2C_INTF;
    s_bmi.intf_ptr        = &s_addr;
    s_bmi.read_write_len  = 32;
    s_bmi.config_file_ptr = NULL;   // use the image built into bmi270.c

    int8_t rslt = bmi270_init(&s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "bmi270_init failed: %d", rslt);
        return ESP_FAIL;
    }

    // Internal pull-ups on the aux bus. No external resistors on the M135.
    uint8_t pupsel = BMI2_ASDA_PUPSEL_2K;
    rslt = bmi2_set_regs(BMI2_AUX_IF_TRIM, &pupsel, 1, &s_bmi);
    if (rslt != BMI2_OK) ESP_LOGW(TAG, "AUX_IF_TRIM write failed: %d", rslt);

    struct bmi2_sens_config cfg = { .type = BMI2_AUX };
    rslt = bmi270_get_sensor_config(&cfg, 1, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "aux get_sensor_config failed: %d", rslt);
        return ESP_FAIL;
    }
    cfg.cfg.aux.odr             = BMI2_AUX_ODR_100HZ;
    cfg.cfg.aux.aux_en          = BMI2_ENABLE;
    cfg.cfg.aux.i2c_device_addr = BMM150_DEFAULT_I2C_ADDRESS;
    cfg.cfg.aux.fcu_write_en    = BMI2_ENABLE;
    cfg.cfg.aux.man_rd_burst    = BMI2_AUX_READ_LEN_3;
    cfg.cfg.aux.read_addr       = BMM150_REG_DATA_X_LSB;
    cfg.cfg.aux.manual_en       = BMI2_ENABLE;

    rslt = bmi270_set_sensor_config(&cfg, 1, &s_bmi);
    if (rslt != BMI2_OK) {
        ESP_LOGE(TAG, "aux set_sensor_config failed: %d", rslt);
        return ESP_FAIL;
    }

    uint8_t sens[3] = { BMI2_ACCEL, BMI2_GYRO, BMI2_AUX };
    rslt = bmi270_sensor_enable(sens, 3, &s_bmi);
    if (rslt != BMI2_OK) ESP_LOGW(TAG, "sensor_enable: %d", rslt);

    s_bmm.read     = aux_read;
    s_bmm.write    = aux_write;
    s_bmm.delay_us = bmi_delay_us;
    s_bmm.intf     = BMM150_I2C_INTF;
    s_bmm.intf_ptr = &s_addr;
    s_bmm.chip_id  = BMM150_DEFAULT_I2C_ADDRESS;

    rslt = bmm150_init(&s_bmm);
    if (rslt != BMM150_OK) {
        ESP_LOGE(TAG, "bmm150_init failed: %d (chip id read 0x%02X)", rslt, s_bmm.chip_id);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "bmm150_init ok, chip ID 0x%02X", s_bmm.chip_id);

    struct bmm150_settings set = { 0 };
    set.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&set, &s_bmm);
    if (rslt != BMM150_OK) ESP_LOGW(TAG, "set_op_mode: %d", rslt);

    set.preset_mode = BMM150_PRESETMODE_REGULAR;
    rslt = bmm150_set_presetmode(&set, &s_bmm);
    if (rslt != BMM150_OK) ESP_LOGW(TAG, "set_presetmode: %d", rslt);

    return ESP_OK;
}

esp_err_t bosch_aux_bmm150_read_ut(float *x, float *y, float *z)
{
    struct bmm150_mag_data d;
    if (bmm150_read_mag_data(&d, &s_bmm) != BMM150_OK) return ESP_FAIL;

    // BMM150_USE_FLOATING_POINT is not defined by default, so the driver
    // returns fixed-point microtesla in int16_t. No further scaling needed.
    *x = (float)d.x;
    *y = (float)d.y;
    *z = (float)d.z;
    return ESP_OK;
}

#endif  // __has_include("bmi270.h")
