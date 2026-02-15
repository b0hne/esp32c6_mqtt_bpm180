#include "bmp180.h"

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"


/**
 * BMP180 I2C address is 0x77.
 * Uses the legacy ESP-IDF I2C driver (driver/i2c.h).
 */
#define BMP180_ADDR 0x77

#define I2C_PORT I2C_NUM_0
#define I2C_SDA  GPIO_NUM_0
#define I2C_SCL  GPIO_NUM_1

// 0..3 (higher = slower, more resolution)
// #define BMP180_OSS 3
#define BMP180_OSS 3

typedef struct {
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1, B2, MB, MC, MD;
    bool     ok;
} bmp180_cal_t;

static bmp180_cal_t g_cal = {0};

/**
 * @brief Initialize I2C master for BMP180 communication.
 *
 * Configures the selected I2C port with defined SDA/SCL pins
 * and 100 kHz clock speed. Must be called before any sensor access.
 *
 * @return void
 */
static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
}

/**
 * @brief Read bytes from a BMP180 register.
 *
 * Performs a write-then-read I2C transaction.
 *
 * @param addr   I2C device address
 * @param reg    Register address to read from
 * @param data   Output buffer
 * @param len    Number of bytes to read
 *
 * @return ESP_OK on success, otherwise error code
 */
static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, data, len, pdMS_TO_TICKS(200));
}

/**
 * @brief Write one byte to a BMP180 register.
 *
 * @param addr   I2C device address
 * @param reg    Register address
 * @param val    Value to write
 *
 * @return ESP_OK on success, otherwise error code
 */
static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(I2C_PORT, addr, buf, sizeof(buf), pdMS_TO_TICKS(200));
}

/**
 * @brief Read an unsigned 16-bit value from BMP180.
 *
 * @param reg  Register address (MSB first)
 *
 * @return 16-bit unsigned value
 */
static uint16_t read_u16(uint8_t reg)
{
    uint8_t b[2];
    ESP_ERROR_CHECK(i2c_read_reg(BMP180_ADDR, reg, b, 2));
    return (uint16_t)((b[0] << 8) | b[1]);
}

/**
 * @brief Read a signed 16-bit value from BMP180.
 *
 * @param reg  Register address (MSB first)
 *
 * @return 16-bit signed value
 */
static int16_t read_s16(uint8_t reg)
{
    return (int16_t)read_u16(reg);
}

/**
 * @brief Read calibration constants from BMP180 EEPROM.
 *
 * Loads all calibration coefficients into internal storage.
 * Must be executed once during initialization.
 *
 * @return void
 */
static void bmp180_read_calibration(void)
{
    g_cal.AC1 = read_s16(0xAA);
    g_cal.AC2 = read_s16(0xAC);
    g_cal.AC3 = read_s16(0xAE);
    g_cal.AC4 = read_u16(0xB0);
    g_cal.AC5 = read_u16(0xB2);
    g_cal.AC6 = read_u16(0xB4);
    g_cal.B1  = read_s16(0xB6);
    g_cal.B2  = read_s16(0xB8);
    g_cal.MB  = read_s16(0xBA);
    g_cal.MC  = read_s16(0xBC);
    g_cal.MD  = read_s16(0xBE);
    g_cal.ok = true;
}

/**
 * @brief Read uncompensated temperature (UT).
 *
 * Starts a temperature conversion and reads the raw value.
 *
 * @return Raw uncompensated temperature value
 */
static int32_t bmp180_read_ut(void)
{
    ESP_ERROR_CHECK(i2c_write_reg(BMP180_ADDR, 0xF4, 0x2E));
vTaskDelay(pdMS_TO_TICKS(80));   // was 26/35/etc
    uint8_t raw[2];
    ESP_ERROR_CHECK(i2c_read_reg(BMP180_ADDR, 0xF6, raw, 2));
    return (int32_t)((raw[0] << 8) | raw[1]);
}

/**
 * @brief Read uncompensated pressure (UP).
 *
 * Starts a pressure conversion using configured oversampling
 * and reads the raw pressure value.
 *
 * @return Raw uncompensated pressure value
 */
static int32_t bmp180_read_up(void)
{
    // start pressure conversion
    ESP_ERROR_CHECK(i2c_write_reg(BMP180_ADDR, 0xF4,
                                  (uint8_t)(0x34 + (BMP180_OSS << 6))));

    // wait until conversion finished (bit 5 == 0)
    uint8_t status = 0;
    for (int i = 0; i < 50; i++) {   // max ~50ms safety window
        ESP_ERROR_CHECK(i2c_read_reg(BMP180_ADDR, 0xF4, &status, 1));

        if ((status & 0x20) == 0)    // bit 5 cleared → ready
            break;

        vTaskDelay(pdMS_TO_TICKS(2));
    }

    uint8_t raw[3];
    ESP_ERROR_CHECK(i2c_read_reg(BMP180_ADDR, 0xF6, raw, 3));

    return ((((int32_t)raw[0] << 16) |
             ((int32_t)raw[1] << 8)  |
              (int32_t)raw[2]) >> (8 - BMP180_OSS));
}

/**
 * @brief Read compensated temperature and pressure.
 *
 * Applies Bosch datasheet compensation formulas using
 * previously loaded calibration data.
 *
 * @param temp_c        Pointer to store temperature in °C (may be NULL)
 * @param pressure_pa   Pointer to store pressure in Pa (may be NULL)
 *
 * @return true if calibration data is valid and read succeeded,
 *         false otherwise
 */
bool bmp180_read(float *temp_c, int32_t *pressure_pa)
{
    if (!g_cal.ok) return false;

    int32_t UT = bmp180_read_ut();

    int32_t X1 = ((UT - (int32_t)g_cal.AC6) * (int32_t)g_cal.AC5) >> 15;
    int32_t X2 = ((int32_t)g_cal.MC << 11) / (X1 + (int32_t)g_cal.MD);
    int32_t B5 = X1 + X2;

    if (temp_c) {
        int32_t T = (B5 + 8) >> 4; // 0.1°C
        *temp_c = (float)T / 10.0f;
    }

    int32_t UP = bmp180_read_up();

    int32_t B6 = B5 - 4000;
    X1 = ((int32_t)g_cal.B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)g_cal.AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)g_cal.AC1 * 4 + X3) << BMP180_OSS) + 2) >> 2;

    X1 = ((int32_t)g_cal.AC3 * B6) >> 13;
    X2 = ((int32_t)g_cal.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    uint32_t B4 = ((uint32_t)g_cal.AC4 * (uint32_t)(X3 + 32768)) >> 15;

    uint32_t B7 = (uint32_t)(UP - B3) * (uint32_t)(50000U >> BMP180_OSS);

    int32_t p;
    if (B7 < 0x80000000U) p = (int32_t)((B7 * 2) / B4);
    else                  p = (int32_t)((B7 / B4) * 2);

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);

    if (pressure_pa) *pressure_pa = p;
    return true;
}

/**
 * @brief Initialize BMP180 sensor.
 *
 * Initializes I2C interface and reads calibration constants.
 * Must be called once before bmp180_read().
 *
 * @return void
 */
void bmp180_init(void)
{
    i2c_master_init();
    bmp180_read_calibration();
}
