/*
 * pmu.h / .c - core functionality to control a PMU (PMIC) over I2C.
 * For XPowers PMU, minimal implementation.
 * ESP32-S3 target, ESP-IDF framework ver. >= v5.5 (NEW I2C API).
 * 
 * The PMU itself is defined in axp2101.h / .c.
 *
 * Configuration types: 
 * - pmu_regulator_ctrl_t: Specifies the HOW; Register addresses, bitfields and conversion functions for a specific regulator inside a PMU.
 * - pmu_regulator_cfg_t: Specifies the WHAT; Settings like on/switches and output levels for the regulator.
 * 
 * Structure:
 * - pmu_init sets up R/W accessible device, and the mutex objects to serialize access.
 * - publically available functions are threadsafe.
 * - internal functions marked _nolock do not take a mutex, and are not threadsafe.
 * 
 * Version: See idf_component.yml
 * Janus Andersen, August 2025.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

// Types to manage regulators
// Register and conversion definition
typedef struct pmu_regulator_ctrl_t {
    uint8_t volt_reg_addr;
    uint8_t dvm_bit_mask;                   // 0 if no dvm
    uint8_t level_bit_mask;
    uint8_t onoff_reg_addr;
    uint8_t onoff_bit_mask;
    uint16_t (*code_to_mv)(uint8_t);        // converts from code (8-bit) to level in mV (16-bit)
    bool (*is_valid_level)(uint16_t);       // checks if a requested level (uint16_t) is valid for the regulator
    uint8_t (*mv_to_code)(uint16_t);        // converts from level in mV (16-bit) to code (8-bit)
} pmu_regulator_ctrl_t;

typedef enum pmu_output_mode_t {
    PMU_OUTPUT_OFF,
    PMU_OUTPUT_ON
} pmu_output_mode_t;

typedef enum pmu_dvm_mode_t {
    PMU_DVM_OFF,
    PMU_DVM_ON
} pmu_dvm_mode_t;

typedef struct pmu_regulator_cfg_t {
    pmu_output_mode_t output;               // on (true), off (false)
    pmu_dvm_mode_t dvm;                     // on (true), off (false)
    uint16_t level_mv;                      // voltage level in mV
    pmu_regulator_ctrl_t *regulator;        // knows where to get/set
} pmu_regulator_cfg_t;


// Initialize and deinitialize the bus and device for PMU comms
esp_err_t pmu_init(i2c_master_bus_config_t, i2c_device_config_t);
esp_err_t pmu_deinit(void);

// Get and set mode and voltage of regulators
esp_err_t pmu_get_regulator_state(pmu_regulator_cfg_t*);
esp_err_t pmu_set_regulator_state(pmu_regulator_cfg_t*);

// Get voltage level and output mode
uint16_t pmu_get_regulator_level(pmu_regulator_ctrl_t);
bool pmu_get_regulator_is_on(pmu_regulator_ctrl_t);
