// SPDX-License-Identifier: MIT
// BMP280 on the M135, at 0x76 on the Tab5's internal I2C bus.

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tab5_bus.h"
#include "baro_example.h"

static const char *TAG = "tab5_baro";

#define BMP280_REG_CALIB     0x88
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7

#define BMP280_CHIP_ID       0x58
#define BME280_CHIP_ID       0x60

// Sea-level reference. Altitude output is only as good as this number; the
// 1013.25 default is a standard atmosphere, not your local pressure, so treat
// the altitude as relative unless you feed it a real QNH.
#define SEA_LEVEL_HPA        1013.25

static i2c_master_dev_handle_t s_dev;

static struct {
    uint16_t t1; int16_t t2, t3;
    uint16_t p1; int16_t p2, p3, p4, p5, p6, p7, p8, p9;
} s_cal;

static int32_t s_t_fine;

static double compensate_temp(int32_t adc)
{
    double v1 = (((double)adc) / 16384.0 - ((double)s_cal.t1) / 1024.0) * ((double)s_cal.t2);
    double v2 = ((((double)adc) / 131072.0 - ((double)s_cal.t1) / 8192.0) *
                 (((double)adc) / 131072.0 - ((double)s_cal.t1) / 8192.0)) * ((double)s_cal.t3);
    s_t_fine = (int32_t)(v1 + v2);
    return (v1 + v2) / 5120.0;
}

static double compensate_press(int32_t adc)
{
    double v1 = ((double)s_t_fine / 2.0) - 64000.0;
    double v2 = v1 * v1 * ((double)s_cal.p6) / 32768.0;
    v2 = v2 + v1 * ((double)s_cal.p5) * 2.0;
    v2 = (v2 / 4.0) + (((double)s_cal.p4) * 65536.0);
    v1 = (((double)s_cal.p3) * v1 * v1 / 524288.0 + ((double)s_cal.p2) * v1) / 524288.0;
    v1 = (1.0 + v1 / 32768.0) * ((double)s_cal.p1);
    if (v1 == 0.0) return 0.0;   // divide by zero at the extremes of calibration

    double p = 1048576.0 - (double)adc;
    p = (p - (v2 / 4096.0)) * 6250.0 / v1;
    v1 = ((double)s_cal.p9) * p * p / 2147483648.0;
    v2 = p * ((double)s_cal.p8) / 32768.0;
    return p + (v1 + v2 + ((double)s_cal.p7)) / 16.0;   // Pa
}

static void baro_task(void *arg)
{
    for (;;) {
        uint8_t raw[6];
        if (tab5_reg_read(s_dev, BMP280_REG_PRESS_MSB, raw, sizeof(raw)) != ESP_OK) {
            ESP_LOGW(TAG, "read failed");
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        // 20-bit samples, left-aligned across three bytes.
        int32_t adc_p = ((int32_t)raw[0] << 12) | ((int32_t)raw[1] << 4) | (raw[2] >> 4);
        int32_t adc_t = ((int32_t)raw[3] << 12) | ((int32_t)raw[4] << 4) | (raw[5] >> 4);

        double temp = compensate_temp(adc_t);   // must run first; sets t_fine
        double pa   = compensate_press(adc_p);
        double hpa  = pa / 100.0;
        double alt  = 44330.0 * (1.0 - pow(hpa / SEA_LEVEL_HPA, 0.1903));

        ESP_LOGI(TAG, "%.2f hPa, %.2f C, %.1f m (vs %.2f hPa reference)",
                 hpa, temp, alt, SEA_LEVEL_HPA);

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

esp_err_t baro_example_start(void)
{
    esp_err_t err = tab5_bus_add_device(M135_BMP280_ADDR, &s_dev);
    if (err != ESP_OK) return err;

    uint8_t id = 0;
    err = tab5_reg_read(s_dev, BMP280_REG_ID, &id, 1);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "no answer at 0x%02X: %s", M135_BMP280_ADDR, esp_err_to_name(err));
        return err;
    }
    if (id == BME280_CHIP_ID) {
        ESP_LOGW(TAG, "chip ID 0x60 is a BME280, not the BMP280 this expects");
    } else if (id != BMP280_CHIP_ID) {
        ESP_LOGE(TAG, "unexpected chip ID 0x%02X at 0x%02X", id, M135_BMP280_ADDR);
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "BMP280 at 0x%02X, chip ID 0x%02X", M135_BMP280_ADDR, id);

    tab5_reg_write8(s_dev, BMP280_REG_RESET, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t c[24];
    err = tab5_reg_read(s_dev, BMP280_REG_CALIB, c, sizeof(c));
    if (err != ESP_OK) return err;

    // Little-endian, and the T2/T3/P2..P9 words are signed. Reading them all
    // as unsigned gives plausible-looking pressure that drifts badly with
    // temperature, which is a slow thing to notice.
    s_cal.t1 = (uint16_t)(c[1]  << 8 | c[0]);
    s_cal.t2 = (int16_t) (c[3]  << 8 | c[2]);
    s_cal.t3 = (int16_t) (c[5]  << 8 | c[4]);
    s_cal.p1 = (uint16_t)(c[7]  << 8 | c[6]);
    s_cal.p2 = (int16_t) (c[9]  << 8 | c[8]);
    s_cal.p3 = (int16_t) (c[11] << 8 | c[10]);
    s_cal.p4 = (int16_t) (c[13] << 8 | c[12]);
    s_cal.p5 = (int16_t) (c[15] << 8 | c[14]);
    s_cal.p6 = (int16_t) (c[17] << 8 | c[16]);
    s_cal.p7 = (int16_t) (c[19] << 8 | c[18]);
    s_cal.p8 = (int16_t) (c[21] << 8 | c[20]);
    s_cal.p9 = (int16_t) (c[23] << 8 | c[22]);

    // config: standby 500 ms, IIR filter x16 — the filter is what makes the
    // altitude readable rather than a few metres of noise.
    tab5_reg_write8(s_dev, BMP280_REG_CONFIG, (0x04 << 5) | (0x04 << 2));
    // ctrl_meas: temp x2, pressure x16, normal mode.
    tab5_reg_write8(s_dev, BMP280_REG_CTRL_MEAS, (0x02 << 5) | (0x05 << 2) | 0x03);

    // The first conversion after a mode change takes ~100 ms at x16
    // oversampling, and until it lands the data registers still hold the reset
    // default of 0x800000. That decodes to roughly 650 hPa and 3500 m, which
    // reads as a broken sensor rather than as a sensor that has not finished
    // its first sample. Throw one reading away.
    vTaskDelay(pdMS_TO_TICKS(250));
    uint8_t discard[6];
    tab5_reg_read(s_dev, BMP280_REG_PRESS_MSB, discard, sizeof(discard));

    xTaskCreate(baro_task, "baro", 4096, NULL, 4, NULL);
    return ESP_OK;
}
