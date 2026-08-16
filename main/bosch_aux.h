// SPDX-License-Identifier: MIT
//
// Thin glue over Bosch's BMI270 + BMM150 reference drivers, used as the
// fallback when this project's own aux sequence fails. Compiled only when
// components/bosch_sensortec/ exists; see tools/fetch_bosch_drivers.sh.
#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

// Initialises the BMI270 at `bmi_addr` through Bosch's API, configures its
// aux master, and brings the BMM150 to normal mode.
esp_err_t bosch_aux_bmm150_start(uint8_t bmi_addr);

// Reads the compensated magnetic field in microtesla. Bosch's driver applies
// the factory trim values held in the BMM150's NVM, which is why these are
// real units rather than raw counts.
esp_err_t bosch_aux_bmm150_read_ut(float *x, float *y, float *z);
