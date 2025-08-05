idf_component_register(
    SRCS
        "src/pmu_axp2101.c"
        "src/pmu_regulators.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        esp_common      # uses esp_err.h in public headers
    PRIV_REQUIRES
        esp_driver_i2c
        freertos
)
