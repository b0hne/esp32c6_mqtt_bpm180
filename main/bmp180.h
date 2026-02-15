#pragma once
#include <stdbool.h>
#include <stdint.h>

/**
 * Initialize I2C and read BMP180 calibration constants once.
 * Call this once after boot.
 */
void bmp180_init(void);

/**
 * Read temperature and pressure.
 * @param temp_c        output temperature in °C (may be NULL)
 * @param pressure_pa   output pressure in Pa (may be NULL)
 * @return true on success
 */
bool bmp180_read(float *temp_c, int32_t *pressure_pa);
