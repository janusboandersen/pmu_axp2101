/**
 * Defines the regulators in the AXP2101 in terms of
 * their registers, bitfields, and conversion functions.
 * 
 * XPOWERS_... macros correspond to the datasheet, but are taken from the XPowers implementation by Lewis X. He.
 * 
 * Janus, August 2025.
 * 
 */

#include <stdint.h>
#include "pmu_regulators.h"


/* ****************
 *      DC/DC 3
 * ****************/

pmu_regulator_ctrl_t PMU_DC3 = {
    .volt_reg_addr = XPOWERS_AXP2101_DC_VOL2_CTRL,           // reg 0x84
    .dvm_bit_mask = _BIT(7),
    .level_bit_mask = MASK_BITS_6_0,                         // bits 6:0
    .onoff_reg_addr = XPOWERS_AXP2101_DC_ONOFF_DVM_CTRL,     // reg 0x80
    .onoff_bit_mask = _BIT(2),
    .code_to_mv = pmu_dc3_code_to_mv,
    .is_valid_level = pmu_dc3_is_valid_level,
    .mv_to_code = pmu_dc3_mv_to_code
};

uint16_t pmu_dc3_code_to_mv(uint8_t code)
{
    // assumes valid code
    if ( code >= XPOWERS_AXP2101_DCDC3_VOL_STEPS3_BASE ) {
        // Last 19 steps (88-106): 1.6-3.4V -> 100mV steps, 19 steps
        return XPOWERS_AXP2101_DCDC3_VOL3_MIN + 
            ((uint16_t)code - XPOWERS_AXP2101_DCDC3_VOL_STEPS3_BASE) * XPOWERS_AXP2101_DCDC3_VOL_STEPS3;
    }
    else if ( code >= XPOWERS_AXP2101_DCDC3_VOL_STEPS2_BASE ) {
        // Intermediate 17 steps (71-87): 1.22-1.54V -> 20mV steps
        return XPOWERS_AXP2101_DCDC3_VOL2_MIN + 
            ((uint16_t)code - XPOWERS_AXP2101_DCDC3_VOL_STEPS2_BASE) * XPOWERS_AXP2101_DCDC3_VOL_STEPS2;
    } else { // if ( code >= XPOWERS_AXP2101_DCDC3_VOL_STEPS1_BASE )
        // First 71 steps (0-70): 0.5-1.2V -> 10mV steps
        return XPOWERS_AXP2101_DCDC3_VOL1_MIN + 
            ((uint16_t)code) * XPOWERS_AXP2101_DCDC3_VOL_STEPS1;
    }
}

bool pmu_dc3_is_valid_level(uint16_t voltage_mv) {
    if (voltage_mv < XPOWERS_AXP2101_DCDC3_VOL_MIN || voltage_mv > XPOWERS_AXP2101_DCDC3_VOL_MAX) {
        return false;
    }
    return true;
}

uint8_t pmu_dc3_mv_to_code(uint16_t voltage_mv)
{
    // TODO: Should we check for overflow before narrowing cast?
    if (voltage_mv >= XPOWERS_AXP2101_DCDC3_VOL3_MIN) {
        // 1.6V-3.4V
        return (uint8_t)(
            (voltage_mv - XPOWERS_AXP2101_DCDC3_VOL3_MIN)/XPOWERS_AXP2101_DCDC3_VOL_STEPS3 + XPOWERS_AXP2101_DCDC3_VOL_STEPS3_BASE);
    } else if (voltage_mv >= XPOWERS_AXP2101_DCDC3_VOL2_MIN) {
        // 1.22V-1.54V
        return (uint8_t)(
            (voltage_mv - XPOWERS_AXP2101_DCDC3_VOL2_MIN)/XPOWERS_AXP2101_DCDC3_VOL_STEPS2 + XPOWERS_AXP2101_DCDC3_VOL_STEPS2_BASE);
    } else { // if (voltage_mv >= XPOWERS_AXP2101_DCDC3_VOL1_MIN) {
        // 0.5V-1.2V
        return (uint8_t)(
            (voltage_mv - XPOWERS_AXP2101_DCDC3_VOL1_MIN)/XPOWERS_AXP2101_DCDC3_VOL_STEPS1 + XPOWERS_AXP2101_DCDC3_VOL_STEPS1_BASE);
    }
}


/* ****************
 *      BLDO1
 * ****************/

pmu_regulator_ctrl_t PMU_BLDO1 = {
    .volt_reg_addr = XPOWERS_AXP2101_LDO_VOL4_CTRL,         // reg 0x96
    .dvm_bit_mask = 0,                                      // no DVM
    .level_bit_mask = MASK_BITS_4_0,                        // bits 4:0
    .onoff_reg_addr = XPOWERS_AXP2101_LDO_ONOFF_CTRL0,      // reg 0x90
    .onoff_bit_mask = _BIT(4),                              // typo in the datasheet
    .code_to_mv = pmu_bldo1_code_to_mv,
    .is_valid_level = pmu_bldo1_is_valid_level,
    .mv_to_code = pmu_bldo1_mv_to_code
};

uint16_t pmu_bldo1_code_to_mv(uint8_t code) {
    return ((uint16_t)code) * XPOWERS_AXP2101_BLDO1_VOL_STEPS + XPOWERS_AXP2101_BLDO1_VOL_MIN;
}

bool pmu_bldo1_is_valid_level(uint16_t voltage_mv) {
    if (voltage_mv < XPOWERS_AXP2101_BLDO1_VOL_MIN || voltage_mv > XPOWERS_AXP2101_BLDO1_VOL_MAX) {
        return false;
    }
    return true;
}

uint8_t pmu_bldo1_mv_to_code(uint16_t voltage_mv) {
    return (uint8_t)(
        (voltage_mv - XPOWERS_AXP2101_BLDO1_VOL_MIN) / XPOWERS_AXP2101_BLDO1_VOL_STEPS);
}
