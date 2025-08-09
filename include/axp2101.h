/**
 * axp2101.h / .c - specifics for controlling the AXP2101 PMU (PMIC) from XPowers.
 * Configured for the LilyGo-T-SIM7080G v1.0 board.
 * 
 * Control registers in the AXP2101 are all 8 bits (expect for possible the ADC registers).
 * The datasheet for the PMU is available at 
 * https://files.waveshare.com/wiki/common/X-power-AXP2101_SWcharge_V1.0.pdf
 * 
 * This file only has prototypes for the regulators that are in use.
 * Add more prototypes (and implement them in the .c file), if you need additional regulators.
 * 
 * Janus, August 2025.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c_master.h"

#include "pmu.h"
#include "pmu_utils.h"

#include "axp2101_constants.h"

// I2C configuration
extern i2c_master_bus_config_t pmu_axp2101_i2c_mst_config;
extern i2c_device_config_t pmu_axp2101_dev_cfg;

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

// BLDO2 is an LDO (0.5V-3.5V, max. 300mA)
extern pmu_regulator_ctrl_t PMU_BLDO2;
uint16_t pmu_bldo2_code_to_mv(uint8_t);
bool pmu_bldo2_is_valid_level(uint16_t);
uint8_t pmu_bldo2_mv_to_code(uint16_t);