#ifndef __BH1750_H__
#define __BH1750_H__

#include <stdint.h>
#include <stdbool.h>
#include "hal/i2c_types.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BH1750_ADDR_LO 0x23 //!< I2C address when ADDR pin floating/low
#define BH1750_ADDR_HI 0x5c //!< I2C address when ADDR pin high

/**
 * Measurement mode
 */
typedef enum
{
    BH1750_MODE_ONE_TIME = 0, //!< One time measurement
    BH1750_MODE_CONTINUOUS    //!< Continuous measurement
} bh1750_mode_t;

/**
 * Measurement resolution
 */
typedef enum
{
    BH1750_RES_LOW = 0,  //!< 4 lx resolution, measurement time is usually 16 ms
    BH1750_RES_HIGH,     //!< 1 lx resolution, measurement time is usually 120 ms
    BH1750_RES_HIGH2     //!< 0.5 lx resolution, measurement time is usually 120 ms
} bh1750_resolution_t;

typedef struct i2c_lowlevel_s
{
   /* If bus == NULL, port, pin_sda, and pin_scl will be used to 
      initialize the I2C bus, otherwise it's assumed that a previous
      call was made to i2c_new_master_bus, and the resulting handle
      is assigned to this 'bus' variable.  */
   i2c_master_bus_handle_t *bus;
   /* If bus != NULL, the following variables will be used to
      initialize the I2C bus. */
   i2c_port_t port;
   int pin_sda;
   int pin_scl;
} i2c_lowlevel_config;

typedef void *bh1750_t;

/**
 * @brief Initialize device descriptor
 * @param config ESP-IDF I2C configuration structure
 * @param i2c_address I2C slave address of BH1750 device (BH1750_ADDR_LO or BH1750_ADDR_HI)
 * @param mode Measurement mode
 * @param resolution Measurement resolution
 * @return bh1750_t on success, NULL on failure
 */
bh1750_t bh1750_init(i2c_lowlevel_config *config, uint8_t i2c_address, bh1750_mode_t mode, bh1750_resolution_t resolution);

/**
 * @brief Free device descriptor
 * @param bh1750 obtained from a successful bh1750_init() call
 * @return true on success, false on failure
 */
bool bh1750_free(bh1750_t bh1750);

/**
 * @brief Power down device
 * @param bh1750 obtained from a successful bh1750_init() call
 * @return true on success, false on failure
 */
bool bh1750_power_down(bh1750_t bh1750);

/**
 * @brief Power on device
 * @param bh1750 obtained from a successful bh1750_init() call
 * @return true on success, false on failure
 */
bool bh1750_power_on(bh1750_t bh1750);

/**
 * @brief Set measurement time
 * @param bh1750 obtained from a successful bh1750_init() call
 * @param time Measurement time (see datasheet)
 * @return true on success, false on failure
 */
bool bh1750_set_measurement_time(bh1750_t bh1750, uint8_t time);

/**
 * @brief Read LUX value from the device
 * @param bh1750 obtained from a successful bh1750_init() call
 * @param[out] lux_level read value in lux units
 * @return true on success, false on failure
 */
bool bh1750_read(bh1750_t bh1750, uint16_t *lux_level);

#ifdef __cplusplus
}
#endif

#endif /* __BH1750_H__ */