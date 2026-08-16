idf_component_register(
    SRCS
        "main.c"
        "tab5_bus.c"
        "gnss_example.c"
        "imu_example.c"
        "baro_example.c"
        "bosch_aux.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_driver_i2c esp_driver_uart esp_driver_gpio
    # Present only after ./tools/fetch_bosch_drivers.sh has run; bosch_aux.c
    # guards on __has_include so the build works either way.
    PRIV_REQUIRES bosch_sensortec
)
