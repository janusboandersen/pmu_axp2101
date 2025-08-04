/*
 * Minimal implementation of the AXP2101 PMU/PMIC.
 * Targeted at ESP32S3, and ESP-IDF framework ver. >= v5.5(NEW I2C API).
 * 
 * The available regulators are defined in pmu_regulators.h / .c.
 * 
 * Configured for the LilyGo-T-SIM7080G v1.0 board.
 
 * Version: See idf_component.yml
 * Janus Andersen, August 2025.
 */

#pragma once

#include <stdbool.h>
#include "esp_err.h"
#include "pmu_regulators.h"

// I2C configuration
#define PMU_I2C_PORT    I2C_NUM_0
#define PMU_SDA         GPIO_NUM_15
#define PMU_SCL         GPIO_NUM_7
#define PMU_FREQ_HZ     100000          // 100 kHz, standard mode, lower than typical for LilyGo and ESP32, but board has 10K pullups (slow)
#define PMU_I2C_ADDR    AXP2101_SLAVE_ADDRESS

// Initialize and deinitialize the bus and device for AXP2101 comms
esp_err_t pmu_init(void);
esp_err_t pmu_deinit(void);

// Get and set mode and voltage of regulators
esp_err_t pmu_get_regulator_state(pmu_regulator_cfg_t*);
esp_err_t pmu_set_regulator_state(pmu_regulator_cfg_t*);

// Enable power to the modem on DC/DC3 and level shifter on BLDO1.
//esp_err_t pmu_enable_modem_power(void);

// Disable power to the modem and level shifter.
//esp_err_t pmu_disable_modem_power(void);

// Enable power to the GNSS antenna on BLDO2
//esp_err_t pmu_enable_gnss_ant_power(void);

// Disable power to the GNSS antenna
//esp_err_t pmu_disable_gnss_ant_power(void);
