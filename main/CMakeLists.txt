idf_component_register(
    SRCS
        "main.c"
        "tab5_bus.c"
        "gnss_example.c"
        "imu_example.c"
        "baro_example.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_driver_i2c esp_driver_uart esp_driver_gpio
)
