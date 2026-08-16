// SPDX-License-Identifier: MIT
//
// M5Stack Tab5 + Module GNSS (M135), plain ESP-IDF. No M5Unified, no BSP.
// The panel is never touched; everything goes to the console over USB-C.

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "tab5_bus.h"
#include "gnss_example.h"
#include "imu_example.h"
#include "baro_example.h"

static const char *TAG = "tab5_m135";

void app_main(void)
{
    ESP_LOGI(TAG, "Tab5 + M135 GNSS module");

    ESP_ERROR_CHECK(tab5_bus_init());

    // Before anything else. The M5-Bus 5V pin is off at boot and the module is
    // simply not there until this runs.
    ESP_ERROR_CHECK(tab5_ext5v_enable());

    tab5_bus_scan();

    if (gnss_example_start() != ESP_OK) ESP_LOGE(TAG, "GNSS did not start");
    if (baro_example_start() != ESP_OK) ESP_LOGE(TAG, "barometer did not start");
    if (imu_example_start()  != ESP_OK) ESP_LOGE(TAG, "IMU did not start");
}
