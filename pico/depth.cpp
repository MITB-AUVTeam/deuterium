#include "depth.hpp"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#define I2C_ADDR 0x76

const uint8_t MS5837_RESET = 0x1E;
const uint8_t MS5837_ADC_READ = 0x00;
const uint8_t MS5837_PROM_READ = 0xA0;
const uint8_t MS5837_CONVERT_D1_8192 = 0x4A;
const uint8_t MS5837_CONVERT_D2_8192 = 0x5A;

const float Pa = 100.0f;
const float bar = 0.001f;
const float mbar = 1.0f;

const uint8_t MS5837_30BA = 0;
const uint8_t MS5837_02BA = 1;
const uint8_t MS5837_UNRECOGNISED = 255;

const uint16_t MS5837_02BA_MAX_SENSITIVITY = 49000;
const uint16_t MS5837_02BA_30BA_SEPARATION = 37000;
const uint16_t MS5837_30BA_MIN_SENSITIVITY = 26000;

uint32_t D1_pres = 0, D2_temp = 0;
int32_t P, TEMP;
uint8_t _model;

uint16_t C[8];

float fluidDensity = 1029;

uint8_t crc4(uint16_t n_prom[]) {
    uint16_t n_rem = 0;

    n_prom[0] = ((n_prom[0]) & 0x0FFF);
    n_prom[7] = 0;

    for (uint8_t i = 0; i < 16; i++) {
        if (i % 2 == 1) {
            n_rem ^= (uint16_t)((n_prom[i >> 1]) & 0x00FF);
        }
        else {
            n_rem ^= (uint16_t)(n_prom[i >> 1] >> 8);
        }
        for (uint8_t n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000) {
                n_rem = (n_rem << 1) ^ 0x3000;
            }
            else {
                n_rem = (n_rem << 1);
            }
        }
    }
    return (n_rem >> 12) & 0xF;
}

bool init() {

    bool crc_check = 0;

    i2c_init(MS5837_PORT, 400 * 1000);
    gpio_set_function(MS5837_SDA, GPIO_FUNC_I2C);
    gpio_set_function(MS5837_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(MS5837_SDA);
    gpio_pull_up(MS5837_SCL);

    do {
        uint8_t cmd = MS5837_RESET;
        i2c_write_blocking(MS5837_PORT, I2C_ADDR, &cmd, 1, false);
        sleep_ms(500);

        for (uint8_t i = 0; i < 7; i++) {
            uint8_t reg = MS5837_PROM_READ + i * 2;
            i2c_write_blocking(MS5837_PORT, I2C_ADDR, &reg, 1, true);
            uint8_t buffer[2];
            i2c_read_blocking(MS5837_PORT, I2C_ADDR, buffer, 2, false);

            C[i] = (buffer[0] << 8) | buffer[1];
        }
        uint8_t crc_read = C[0] >> 12;
        uint8_t crc_calculated = crc4(&C[0]);

        crc_check = crc_calculated == crc_read;

    } while (!crc_check);

    if (C[1] < MS5837_30BA_MIN_SENSITIVITY || C[1] > MS5837_02BA_MAX_SENSITIVITY)
    {
        _model = MS5837_UNRECOGNISED;
    }
    else if (C[1] > MS5837_02BA_30BA_SEPARATION)
    {
        _model = MS5837_02BA;
    }
    else
    {
        _model = MS5837_30BA;
    }
    return true;
}