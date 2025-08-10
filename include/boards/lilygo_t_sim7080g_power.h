/*
 * boards/lilygo_t_sim7080g_power.h.
 *
 * Power controls for the peripherals on the LilyGo T-SIM7080G v1.0 board.
 * Board-specific convenience functions.
 * Header-only implementation. 
 * 
 * Janus, August 2025.
 */

#pragma once

#include "driver/gpio.h"
#include "esp_log.h"

#include "pmu.h"
#include "axp2101.h"
#define TAG                             "LILYGO-T-SIM7080G"
#define BOARD_MODEM_PWRKEY_PIN          GPIO_NUM_41
#define BOARD_MODEM_DTR_PIN             GPIO_NUM_42

// Helpers
esp_err_t board_modem_init_dtr(void);
esp_err_t board_modem_dtr_high(void);
esp_err_t board_modem_init_pwrkey(void);
esp_err_t board_modem_pwr_key_pull_low(uint16_t duration_ms);
esp_err_t board_modem_pwr_on(void);
esp_err_t board_modem_pwr_off(void);

// Enable power
esp_err_t board_modem_power_up()
{
    // The board implements an auto-on for the modem via cap C36.
    // PWRKEY is pulled low while the cap is charging.
    // It is not recommended to power cycle the modem via VBAT due to risk of FS corruption,
    // use the PWRKEY instead.

    // Init PWRKEY and DTR GPIO
    board_modem_init_pwrkey();
    board_modem_init_dtr();

    // Enable communication with PMU
    pmu_init(pmu_axp2101_i2c_mst_config, pmu_axp2101_dev_cfg);

    // Modem SOC is supplied 2.7-4.8V
    // Datasheet: Typical is 3.8V, with 500 mA peak current draw (PMU must be able to supply up to 2.0A).
    pmu_regulator_cfg_t dc3_up = { 
        .output = PMU_OUTPUT_ON,
        .dvm = PMU_DVM_ON,
        .level_mv = 3000,
        .regulator = &PMU_DC3
    };

    // Level shifter for UART Tx Rx to modem IC
    // Modem IO and UART is 1.8V, and ESP32-S3 is 3.3V
    pmu_regulator_cfg_t bldo1 = { 
        .output = PMU_OUTPUT_ON,
        .dvm = PMU_DVM_OFF,
        .level_mv = 3300,
        .regulator = &PMU_BLDO1
    };

    // GNSS active antenna
    pmu_regulator_cfg_t bldo2 = { 
        .output = PMU_OUTPUT_ON,
        .dvm = PMU_DVM_OFF,
        .level_mv = 3300,
        .regulator = &PMU_BLDO2
    };
    
    // Power up - if any of these fail, the board isn't working properly - stop!
    ESP_ERROR_CHECK(pmu_set_regulator_state(&bldo1));   // Level shifter
    ESP_ERROR_CHECK(pmu_set_regulator_state(&bldo2));   // GNSS antenna
    ESP_ERROR_CHECK(pmu_set_regulator_state(&dc3_up));  // Modem

    // Assert DTR high - don't go to sleep.
    //board_modem_dtr_high();

    return ESP_OK;
}

// initialize the gpio for signalling to modem DTR
esp_err_t board_modem_init_dtr(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_MODEM_DTR_PIN), // 64-bit shift supported at compile time
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&io_conf);
}


// initialize the gpio for signalling to modem PWRKEY
esp_err_t board_modem_init_pwrkey(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BOARD_MODEM_PWRKEY_PIN), // 64-bit shift supported at compile time
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    return gpio_config(&io_conf);
    gpio_set_level(BOARD_MODEM_PWRKEY_PIN, 0);
}


esp_err_t board_modem_dtr_high(void) {
    return gpio_set_level(BOARD_MODEM_DTR_PIN, 1);   // keep modem awake
}


// Pull the PWRKEY pin on the modem to low for duration_ms. This is used for power on, off, and reset.
esp_err_t board_modem_pwr_key_pull_low(uint16_t duration_ms)
{
    // PWRKEY is high at 1.5V (internal pull-up).
    // Drive low (<0.4V) to signal:
    // 1 sec < t < 12 sec to turn on (if off).
    // t > 12.5 sec to reset (if on).
    // t > 1.2 sec to turn off (if on).
    // ESP GPIO pin 41 connects via inverter to modem PWRKEY (pin 39).
    // Set GPIO41 high -> pulls PWRKEY low.
    // Set GPIO41 low -> let's PWRKEY go high.

    // Ensure clean transition edge
    gpio_set_level(BOARD_MODEM_PWRKEY_PIN, 0);
    vTaskDelay(100);

    // Pull PWRKEY low (via inverter and level-shift)
    gpio_set_level(BOARD_MODEM_PWRKEY_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));

    // Release
    gpio_set_level(BOARD_MODEM_PWRKEY_PIN, 0);

    return ESP_OK;
}

esp_err_t board_modem_pwr_on(void) {
    return board_modem_pwr_key_pull_low(1000);
}

esp_err_t board_modem_pwr_off(void) {
    return board_modem_pwr_key_pull_low(1200);
}