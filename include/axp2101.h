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

// TODO: Get these with Kconfig
//#define PMU_I2C_PORT    I2C_NUM_0
//#define PMU_SDA         GPIO_NUM_15
//#define PMU_SCL         GPIO_NUM_7
//#define PMU_FREQ_HZ     100000          // 100 kHz, standard mode, lower than typical for LilyGo and ESP32, but board has 10K pullups (slow)
//#define PMU_I2C_ADDR    AXP2101_SLAVE_ADDRESS

// Enable power to the modem on DC/DC3 and level shifter on BLDO1.
//esp_err_t pmu_enable_modem_power(void);

// Disable power to the modem and level shifter.
//esp_err_t pmu_disable_modem_power(void);

// Enable power to the GNSS antenna on BLDO2
//esp_err_t pmu_enable_gnss_ant_power(void);

// Disable power to the GNSS antenna
//esp_err_t pmu_disable_gnss_ant_power(void);

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