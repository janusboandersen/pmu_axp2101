/**
 * pmu_axp2101/include/pmu_regulators.h + .c
 * 
 * Declares and defines types and functions for using the AXP2101 regulators.
 * 
 * A pmu_regulator_cfg_t object is used to get/set modes for a regulator.
 * It holds the configuration that was read from / to be written to the PMU,
 * as well as a pointer to the regulator's pmu_regulator_ctrl_t object.
 * The pmu_regulator_ctrl_t object contains register addresses, bitfields and conversion functions
 * that correspond to metadata for the particular regulator from the AXP2101 datasheet.
 * 
 * Janus, August 2025.
 * 
 */

#pragma once

#include <stdbool.h>
#include "AXP2101Constants.h"
#include "pmu_utils.h"

// Types to manage regulators
// Register and conversion definition
typedef struct pmu_regulator_ctrl_t {
    uint8_t volt_reg_addr;
    uint8_t dvm_bit_mask;           // 0 if no dvm
    uint8_t level_bit_mask;
    uint8_t onoff_reg_addr;
    uint8_t onoff_bit_mask;
    uint16_t (*code_to_mv)(uint8_t); // converts from code (8-bit) to level in mV (16-bit)
    bool (*is_valid_level)(uint16_t);   // checks if a requested level (uint16_t) is valid for the regulator
    uint8_t (*mv_to_code)(uint16_t); // converts from level in mV (16-bit) to code (8-bit)
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
    uint16_t level_mv;                  // voltage level in mV
    pmu_regulator_ctrl_t *regulator;        // knows where to get/set
} pmu_regulator_cfg_t;


// DC/DC 3 is a buck converter (0.5-3.4V, max. 2A)
extern pmu_regulator_ctrl_t PMU_DC3;
uint16_t pmu_dc3_code_to_mv(uint8_t);
bool pmu_dc3_is_valid_level(uint16_t);
uint8_t pmu_dc3_mv_to_code(uint16_t);

// BLDO1 is an LDO (0.5V-3.5V, max. 300mA)
extern pmu_regulator_ctrl_t PMU_BLDO1;
uint16_t pmu_bldo1_code_to_mv(uint8_t);
bool pmu_bldo1_is_valid_level(uint16_t);
uint8_t pmu_bldo1_mv_to_code(uint16_t);
