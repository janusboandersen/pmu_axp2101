/*
 * The datasheet for the PMU is available at 
 * https://files.waveshare.com/wiki/common/X-power-AXP2101_SWcharge_V1.0.pdf
 * 
 * Control registers in the AXP2101 are all 8 bits (expect for possible the ADC registers).
 * 
 * This library assumes that we own the I2C device. 
 * That may not always be true in practice, so be careful!
 * - See init_master(), which is being used by pmu_init() and vice versa in pmu_deinit().
 * 
 * Janus Andersen, August 2025.
 * 
 */
#include <assert.h>

#include "pmu_axp2101.h"
//#include "AXP2101Constants.h"

#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_check.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "PMU_AXP2101"

// handles for bus and PMU
i2c_master_bus_handle_t master_handle = NULL;
i2c_master_dev_handle_t pmu_handle = NULL;


// bool valid_single_bit_mask(uint8_t bit_mask)
// {
//     // iff bit_mask is power of 2, then only 1 bit is active
//     // check if power of 2 by x & (x-1) == 0
//     return ( bit_mask != 0 && (bit_mask & (bit_mask-1)) == 0 );
// }

/* ***********************
 * PROTECT PMU WITH MUTEX 
 * ***********************/

static SemaphoreHandle_t s_pmu_mtx;
static volatile bool s_pmu_shutting_down = false;

void pmu_init_mutex(void) {
    s_pmu_mtx = xSemaphoreCreateMutex();
    assert(s_pmu_mtx);                      // Critical fail, probably out of mem.
}

void pmu_deinit_mutex(void) {
    vSemaphoreDelete(s_pmu_mtx);            // Do not delete a semaphore that has tasks blocked on it
    s_pmu_mtx = NULL;
}

// Serialize access to the PMU
// Blocks for to_ticks, timeout after
static inline bool pmu_lock(TickType_t to_ticks)
{
    if (s_pmu_shutting_down) { return false; }              // stop new entrants here
    return (xSemaphoreTake(s_pmu_mtx, to_ticks) == pdTRUE);
}

// Release serialized access
static inline void pmu_unlock(void)
{
    xSemaphoreGive(s_pmu_mtx);
}

// Acquire mutex or timeout with error
static inline esp_err_t pmu_lock_or_timeout(TickType_t to_ticks)
{
    return pmu_lock(to_ticks) ? ESP_OK : ESP_ERR_TIMEOUT;
}

/* ********************************
 * INIT AND DEINIT OF I2C RESOURCE
 * ********************************/

// set up eps32s3 i2c as master of the bus
esp_err_t static init_master(void)
{
    //TODO: Check if the port is already in existence, if so, just return handle to existing bus.
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = PMU_I2C_PORT,
        .scl_io_num = PMU_SCL,
        .sda_io_num = PMU_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,  // board has 10K pull-ups
    };

    return i2c_new_master_bus(&i2c_mst_config, &master_handle);
}

// set up the PMU as slave on the bus
esp_err_t static init_slave(void)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = PMU_I2C_ADDR,
        .scl_speed_hz = PMU_FREQ_HZ,
    };

    return i2c_master_bus_add_device(master_handle, &dev_cfg, &pmu_handle);
}

// Init master and slave so we can talk to the PMU. 
// They will be kept alive throughout the duration of the program.
// TODO: Do nothing if already set up, make thread safe
esp_err_t pmu_init(void)
{
    if (!s_pmu_mtx) { pmu_init_mutex(); }
    s_pmu_shutting_down = false;            // reset (it might've been on from a prev. shutdown)

    // TODO: If the bus master already exists, use that instead of a new
    ESP_LOGD(TAG, "Initializing the I2C bus master.");
    esp_err_t err = init_master();
    if (err != ESP_OK) {
        pmu_deinit_mutex();
        ESP_LOGE(TAG, "I2C bus master init, failed: %s", esp_err_to_name(err));
        return err;
    }
    
    ESP_LOGD(TAG, "Initializing PMU (AXP2101) on the I2C bus.");
    err = init_slave();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C PMU init. failed: %s", esp_err_to_name(err));
        i2c_del_master_bus(master_handle);
        master_handle = NULL;
        pmu_deinit_mutex();
        return err;
    }
    return ESP_OK;
}

// Release resources used to communicate with PMU
esp_err_t pmu_deinit(void) {

    if (s_pmu_mtx)
    {
        // Stop new entrants from acq mutex and trying to use PMU
        s_pmu_shutting_down = true; 
        
        // Ensure the mutex is free by taking it. Release it as per FreeRTOS spec. before deleting it.
        if (xSemaphoreTake(s_pmu_mtx, pdMS_TO_TICKS(1000)) != pdTRUE)
        {
            // someone is blocked for a long time..
            ESP_LOGE(TAG, "PMU mutex busy during deinit. Aborting shutdown.");
            return ESP_ERR_TIMEOUT;     // STOP, not safe to continue.
        }
        xSemaphoreGive(s_pmu_mtx);      // we got it, release and then safe to delete
        pmu_deinit_mutex();             // delete
    }

    if (pmu_handle)
    {
        ESP_RETURN_ON_ERROR(i2c_master_bus_rm_device(pmu_handle), TAG, "rm device");
        pmu_handle = NULL;
    }

    // TODO: Don't delete the master, if it already existed at init time. Someone else owns it.
    if (master_handle)
    {
        ESP_RETURN_ON_ERROR(i2c_del_master_bus(master_handle), TAG, "del master");
        master_handle = NULL;
    }
    
    return ESP_OK;
}


/* ********************************************
 * READ-WRITE PMU REGISTERS INTERNAL -- NOLOCK
 * DO NOT EXPOSE IN THE PUBLIC API
 * ********************************************/

// Read a whole 8-bit register from the PMU.
// This is a leaf utility function. Any error is propagated, so caller can decide action.
esp_err_t pmu_read_register_nolock(uint8_t reg_addr, uint8_t *dest) {

    ESP_LOGD(TAG, "Reading (nolock) reg_addr=0x%02x on PMU (AXP2101) over I2C.", reg_addr);

    // Guard against uninit'ed bus/device or destination
    ESP_RETURN_ON_FALSE(master_handle && pmu_handle, ESP_ERR_INVALID_STATE, TAG, "I2C bus/dev not initialized.");
    ESP_RETURN_ON_FALSE(dest, ESP_ERR_INVALID_ARG, TAG, "Pointer dest is NULL.");

    esp_err_t err = i2c_master_transmit_receive(
        pmu_handle,
        &reg_addr,          // write_buf = tell slave which register address to fetch
        sizeof(uint8_t),    // write_len = number of bytes to write to slave
        dest,               // read_buf = where to place the value fetched
        sizeof(uint8_t),    // read_len = number of bytes fetched
        1000                // timeout ms
    );

    return err;
}

// Write a whole 8-bit register from the PMU.
// This is a leaf utility function. Any error is propagated, so caller can decide action.
esp_err_t pmu_write_register_nolock(uint8_t reg_addr, uint8_t data) {

    ESP_LOGD(TAG, "Writing (nolock) reg_addr=0x%02x on PMU (AXP2101) over I2C.", reg_addr);

    // Guard against uninit'ed bus/device
    ESP_RETURN_ON_FALSE(master_handle && pmu_handle, ESP_ERR_INVALID_STATE, TAG, "I2C bus/dev not initialized.");

    // set up address + byte to write
    uint8_t wr_buf[2] = { reg_addr, data };
    esp_err_t err = i2c_master_transmit(pmu_handle, wr_buf, sizeof(wr_buf), 1000);

    return err;
}


/* *****************************************
 * READ-WRITE PMU BITS IN REGISTER - LOCKED
 * OK TO EXPOSE IN THE PUBLIC API
 * *****************************************/

// Read the value of a single bit from register address.
// Utility function. Any error is propagated, so caller can decide action.
esp_err_t pmu_read_register_bits(uint8_t reg_addr, uint8_t bit_mask, uint8_t *dest) {

    ESP_LOGD(TAG, "Read (lock) reg_addr=0x%02x on PMU (AXP2101) over I2C.", reg_addr);

    esp_err_t err = pmu_lock_or_timeout(pdMS_TO_TICKS(50));
    if (err != ESP_OK) { return err; }

        uint8_t data = 0;
        err = pmu_read_register_nolock(reg_addr, &data);

    pmu_unlock();

    // only update dest if read was OK
    if (err == ESP_OK) { *dest = (data & bit_mask); }

    return err;
}

// Write the data into the register, masked with bit_mask.
// Propages error.
esp_err_t pmu_write_register_bits(uint8_t reg_addr, uint8_t bit_mask, uint8_t data)
{
    ESP_LOGD(TAG, "Read-modify-write (lock) reg_addr=0x%02x on PMU (AXP2101) over I2C.", reg_addr);

    esp_err_t err = pmu_lock_or_timeout(pdMS_TO_TICKS(50));
    if (err != ESP_OK) { return err; }
    
        uint8_t reg_val = 0;

        // Read old register value
        err = pmu_read_register_nolock(reg_addr, &reg_val);
    
        // Compute updated bits
        //                clear bits to ovwrite + bits we want to write
        uint8_t upd_val = (reg_val & ~bit_mask) | (data & bit_mask);

        // Write back new register value
        pmu_write_register_nolock(reg_addr, upd_val);
    
    pmu_unlock();

    return err;
}


/* **************************
 * Regulator Voltage Control 
 * For DC/DC and LDOs
 * **************************/

// Get the state from a regulator: Output on/off, voltage level in mV, DVM on/off.
esp_err_t pmu_get_regulator_state(PmuRegulatorConfig *cfg)
{
    // Fetch voltage control register
    uint8_t volt_reg_val = 0;
    uint8_t err = pmu_read_register_bits(cfg->regulator->volt_reg_addr, 
                                         cfg->regulator->dvm_bit_mask | cfg->regulator->level_bit_mask, 
                                         &volt_reg_val);
    // Not OK
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Can't read config of regulator (reg_addr=0x%02X).", esp_err_to_name(err), cfg->regulator->volt_reg_addr);
        return err;
    }

    // Fetch OnOff control register
    uint8_t onoff_reg_val = 0;
    err = pmu_read_register_bits(cfg->regulator->onoff_reg_addr, 
                                 cfg->regulator->onoff_bit_mask, 
                                 &onoff_reg_val);
    // Not OK
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Can't read config of regulator (reg_addr=0x%02X).", esp_err_to_name(err), cfg->regulator->onoff_reg_addr);
        return err;
    }

    ESP_LOGD(TAG, "reg_val=0x%02X in reg_addr=0x%02X.", volt_reg_val, cfg->regulator->volt_reg_addr);
    ESP_LOGD(TAG, "dvm_bit=0x%02X", (volt_reg_val & cfg->regulator->dvm_bit_mask) ? 1u : 0u);
    ESP_LOGD(TAG, "level_bits=0x%02X", (volt_reg_val & cfg->regulator->level_bit_mask));
    ESP_LOGD(TAG, "reg_val=0x%02X in reg_addr=0x%02X.", onoff_reg_val, cfg->regulator->onoff_reg_addr);
    ESP_LOGD(TAG, "onoff_bit=0x%02X", (onoff_reg_val & cfg->regulator->onoff_bit_mask) ? 1u : 0u); 

    // OK: Convert dvm and output mode bits, then compute voltage from level code bits, manual p. 52
    cfg->dvm = (volt_reg_val & cfg->regulator->dvm_bit_mask) ? PMU_DVM_ON : PMU_DVM_OFF; 
    cfg->output = (onoff_reg_val & cfg->regulator->onoff_bit_mask) ? PMU_OUTPUT_ON : PMU_OUTPUT_OFF;
    uint8_t code = volt_reg_val & cfg->regulator->level_bit_mask;
    cfg->level_mv = cfg->regulator->code_to_mv(code);
    return ESP_OK;
}

// Set the state of a regulator: Output on/off, voltage in mV, DVM on/off.
esp_err_t pmu_set_regulator_state(PmuRegulatorConfig *cfg) 
{
    // Involves read-modify-write (RMW) to two different registers
    // while holding mutex to avoid race conditions on the regulator.
    // So before taking mutex, check that input is valid and pre-compute voltage code.
    ESP_RETURN_ON_FALSE(
        cfg->regulator->is_valid_level(cfg->level_mv), ESP_ERR_INVALID_ARG, TAG,
        "Voltage level_mv=%u invalid for this regulator", cfg->level_mv);
    
    uint8_t level_code = cfg->regulator->mv_to_code(cfg->level_mv) & cfg->regulator->level_bit_mask;
    ESP_LOGD(TAG, "Set regulator voltage=%u mV -> code=0x%02X.", cfg->level_mv, level_code);

    // Acquire lock
    //esp_err_t err = ESP_OK;
    esp_err_t err = pmu_lock_or_timeout(pdMS_TO_TICKS(50));
    if (err != ESP_OK) { return err; }

    // RMW voltage ctrl register
    uint8_t volt_reg_val = 0;
    pmu_read_register_nolock(cfg->regulator->volt_reg_addr, &volt_reg_val);
    ESP_LOGD(TAG, "Read reg_addr=0x%02X -> val=0x%02X.", cfg->regulator->volt_reg_addr, volt_reg_val);

    volt_reg_val &= ~cfg->regulator->level_bit_mask;    // clear bits for voltage level code
    volt_reg_val &= ~cfg->regulator->dvm_bit_mask;      // clear bits for DVM mode (DVM off)
    volt_reg_val |= level_code;                         // set level
    if (cfg->dvm == PMU_DVM_ON) {
        volt_reg_val |= cfg->regulator->dvm_bit_mask;   // set DVM on
    }

    // write back
    err = pmu_write_register_nolock(cfg->regulator->volt_reg_addr, volt_reg_val);
    if (err != ESP_OK) {
        pmu_unlock();
        return err;
    }
    ESP_LOGD(TAG, "Set reg_addr=0x%02X -> val=0x%02X.", cfg->regulator->volt_reg_addr, volt_reg_val);

    // RMW OnOff ctrl register
    uint8_t onoff_reg_val = 0;
    pmu_read_register_nolock(cfg->regulator->onoff_reg_addr, &onoff_reg_val);
    ESP_LOGD(TAG, "Read reg_addr=0x%02X -> val=0x%02X.", cfg->regulator->onoff_reg_addr, onoff_reg_val);

    onoff_reg_val &= ~cfg->regulator->onoff_bit_mask;   // clear bits for output mode (output off)
    if (cfg->output == PMU_OUTPUT_ON) {
        onoff_reg_val |= cfg->regulator->onoff_bit_mask;
    }

    // write back
    err = pmu_write_register_nolock(cfg->regulator->onoff_reg_addr, onoff_reg_val);
    if (err != ESP_OK) {
        pmu_unlock();
        return err;
    }
    ESP_LOGD(TAG, "Set reg_addr=0x%02X -> val=0x%02X.", cfg->regulator->onoff_reg_addr, onoff_reg_val);

    // Release lock
    pmu_unlock();
    return ESP_OK;
}

 /* ************************
 * LED Control 
 * Blinks to signal state
 * ************************/