// SPDX-License-Identifier: MIT

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tab5_bus.h"

static const char *TAG = "tab5_bus";

// PI4IOE5V6408 register map. Same chip, same trap, as the USB-A example:
// OUT_SET decides the level, OUT_H_IM decides whether the pin drives at all.
#define PI4IO_REG_IO_DIR    0x03
#define PI4IO_REG_OUT_SET   0x05
#define PI4IO_REG_OUT_H_IM  0x07
#define PI4IO_REG_PULL_SEL  0x0D
#define PI4IO_REG_PULL_EN   0x0B

static i2c_master_bus_handle_t s_bus;

esp_err_t tab5_bus_init(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port = TAB5_I2C_PORT,
        .sda_io_num = TAB5_I2C_SDA,
        .scl_io_num = TAB5_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&cfg, &s_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c bus init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "internal I2C up on SDA=G%d SCL=G%d @ %d Hz",
             TAB5_I2C_SDA, TAB5_I2C_SCL, TAB5_I2C_HZ);
    return ESP_OK;
}

i2c_master_bus_handle_t tab5_bus_handle(void)
{
    return s_bus;
}

esp_err_t tab5_bus_add_device(uint8_t addr, i2c_master_dev_handle_t *out)
{
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = TAB5_I2C_HZ,
    };
    return i2c_master_bus_add_device(s_bus, &dc, out);
}

esp_err_t tab5_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, buf, len, pdMS_TO_TICKS(100));
}

esp_err_t tab5_reg_write8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t b[2] = { reg, val };
    return i2c_master_transmit(dev, b, sizeof(b), pdMS_TO_TICKS(100));
}

esp_err_t tab5_ext5v_enable(void)
{
    i2c_master_dev_handle_t exp;
    esp_err_t err = tab5_bus_add_device(TAB5_EXP1_ADDR, &exp);
    if (err != ESP_OK) return err;

    const uint8_t mask = 1u << TAB5_EXP1_EXT5V_PIN;
    uint8_t v;

    // Read-modify-write all three, in this order. Skipping OUT_H_IM leaves the
    // pin parked in high impedance and the rail stays off with no error.
    err = tab5_reg_read(exp, PI4IO_REG_IO_DIR, &v, 1);
    if (err != ESP_OK) goto out;
    err = tab5_reg_write8(exp, PI4IO_REG_IO_DIR, v | mask);      // output
    if (err != ESP_OK) goto out;

    err = tab5_reg_read(exp, PI4IO_REG_OUT_SET, &v, 1);
    if (err != ESP_OK) goto out;
    err = tab5_reg_write8(exp, PI4IO_REG_OUT_SET, v | mask);     // high
    if (err != ESP_OK) goto out;

    err = tab5_reg_read(exp, PI4IO_REG_OUT_H_IM, &v, 1);
    if (err != ESP_OK) goto out;
    err = tab5_reg_write8(exp, PI4IO_REG_OUT_H_IM, v & ~mask);   // actually drive it
    if (err != ESP_OK) goto out;

    ESP_LOGI(TAG, "M5-Bus 5V on (expander 0x%02X, P%d)", TAB5_EXP1_ADDR, TAB5_EXP1_EXT5V_PIN);

    // The NEO-M9N holds reset for a moment and the BMI270 wants a settled
    // rail before its config upload. Cheaper to wait here than to debug there.
    vTaskDelay(pdMS_TO_TICKS(250));

out:
    if (err != ESP_OK) ESP_LOGE(TAG, "EXT5V enable failed: %s", esp_err_to_name(err));
    i2c_master_bus_rm_device(exp);
    return err;
}

static const char *known_device(uint8_t addr)
{
    switch (addr) {
    case 0x10: return "ES8388 codec (Tab5)";
    case 0x32: return "RX8130CE RTC (Tab5)";
    case 0x40: return "ES7210 mic ADC (Tab5)";
    case 0x41: return "INA226 power monitor (Tab5)";
    case 0x43: return "PI4IOE5V6408 #1 (Tab5)";
    case 0x44: return "PI4IOE5V6408 #2 (Tab5)";
    case 0x14: return "GT911 touch (Tab5, pre-Oct-2025 panels)";
    case 0x55: return "ST7123/ST7121 touch (Tab5)";
    case 0x68: return "BMI270 (Tab5 onboard IMU)";
    case 0x69: return "BMI270 (M135 module IMU)";
    case 0x76: return "BMP280 (M135 barometer)";
    case 0x42: return "NEO-M9N DDC port (if strapped to the bus)";
    case 0x50: case 0x51: return "EEPROM (M135)";
    default:   return NULL;
    }
}

void tab5_bus_scan(void)
{
    ESP_LOGI(TAG, "scanning internal I2C bus");
    int found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_bus, addr, 50) != ESP_OK) continue;
        const char *name = known_device(addr);
        ESP_LOGI(TAG, "  0x%02X  %s", addr, name ? name : "unknown");
        found++;
    }
    ESP_LOGI(TAG, "%d device%s", found, found == 1 ? "" : "s");
    ESP_LOGI(TAG, "  (BMM150 is behind the BMI270's aux bus and never appears here)");
}
