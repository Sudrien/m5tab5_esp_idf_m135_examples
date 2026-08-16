// SPDX-License-Identifier: MIT
// Tab5 internal I2C bus + M5-Bus 5V rail control.

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

// Tab5 internal I2C. Everything on the board and everything on the M5-Bus
// (bus pins 17/18) shares this one bus.
#define TAB5_I2C_PORT   I2C_NUM_0
#define TAB5_I2C_SDA    GPIO_NUM_31
#define TAB5_I2C_SCL    GPIO_NUM_32
#define TAB5_I2C_HZ     400000

// PI4IOE5V6408 expanders. #1 carries EXT5V_EN, which is what feeds the
// M5-Bus 5V pin the M135 runs from.
#define TAB5_EXP1_ADDR  0x43
#define TAB5_EXP2_ADDR  0x44
#define TAB5_EXP1_EXT5V_PIN 2

// M135 addresses, as seen from the Tab5 side of the bus.
#define M135_BMI270_ADDR 0x69   // note: NOT 0x68, that is the Tab5's own IMU
#define M135_BMP280_ADDR 0x76
#define M135_BMM150_AUX_ADDR 0x10  // behind the BMI270, not on this bus

esp_err_t tab5_bus_init(void);
i2c_master_bus_handle_t tab5_bus_handle(void);

// Turns on EXT_5V_BUS. Without this the M5-Bus 5V pin is dead and the M135
// never powers up; see README.
esp_err_t tab5_ext5v_enable(void);

// Adds a device to the shared bus at 7-bit `addr`.
esp_err_t tab5_bus_add_device(uint8_t addr, i2c_master_dev_handle_t *out);

// Convenience register access, 8-bit register addressing.
esp_err_t tab5_reg_read(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *buf, size_t len);
esp_err_t tab5_reg_write8(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val);

// Probes every address and logs what answered, annotating the ones we know.
void tab5_bus_scan(void);
