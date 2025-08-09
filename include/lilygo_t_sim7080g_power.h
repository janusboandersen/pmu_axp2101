/*
 * Power controls for the LilyGo T-SIM7080G v1.0 board.
 * Convenience functions.
 * Header-only implementation. 
 * 
 * Janus, August 2025.
 */

#pragma once

#include "pmu.h"
#include "axp2101.h"

// Enable power
esp_err_t board_enable_modem_power()
{
    // Enable communication with PMU
    pmu_init(pmu_axp2101_i2c_mst_config, pmu_axp2101_dev_cfg);

    // Modem SOC
    pmu_regulator_cfg_t dc3 = { 
        .output = PMU_OUTPUT_ON,
        .dvm = PMU_DVM_ON,
        .level_mv = 3000,
        .regulator = &PMU_DC3
    };

    // Level shifter for UART Tx Rx to modem IC
    pmu_regulator_cfg_t bldo1 = { 
        .output = PMU_OUTPUT_ON,
        .dvm = PMU_DVM_OFF,
        .level_mv = 3300,
        .regulator = &PMU_BLDO1
    };

    // GNSS antenna
    pmu_regulator_cfg_t bldo2 = { 
        .output = PMU_OUTPUT_ON,
        .dvm = PMU_DVM_OFF,
        .level_mv = 3300,
        .regulator = &PMU_BLDO2
    };

    // Power up - if any of these fail, the board isn't working properly - stop!
    ESP_ERROR_CHECK(pmu_set_regulator_state(&dc3));
    ESP_ERROR_CHECK(pmu_set_regulator_state(&bldo1));
    ESP_ERROR_CHECK(pmu_set_regulator_state(&bldo2));

    return ESP_OK;
}

