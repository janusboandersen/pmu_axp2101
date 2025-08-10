idf_component_register(
    SRCS
        "src/pmu.c"
        "src/axp2101.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_common      # uses esp_err.h in public headers
        esp_driver_i2c  # same
        esp_driver_gpio # For the board-aware header-only lib.
    PRIV_REQUIRES
        freertos        # freertos only used in impl.
)
