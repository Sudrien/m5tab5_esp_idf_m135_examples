// SPDX-License-Identifier: MIT
#pragma once
#include "esp_err.h"
// Brings up the M135's BMI270 at 0x69 and the BMM150 behind it, and starts
// a task that reports both alongside the Tab5's own BMI270 at 0x68.
esp_err_t imu_example_start(void);
