#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 28
#define I2C_SCL 29

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

// context: https://github.com/ArduPilot/ardupilot/pull/29122#issuecomment-2877269114
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

    i2c_init(I2C_PORT, 400 * 1000);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    uint8_t cmd = MS5837_RESET;
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &cmd, 1, false);
    sleep_ms(500);

    for (uint8_t i = 0; i < 7; i++) {
        uint8_t reg = MS5837_PROM_READ + i * 2;
        i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, true);
        uint8_t buffer[2];
        i2c_read_blocking(I2C_PORT, I2C_ADDR, buffer, 2, false);

        C[i] = (buffer[0] << 8) | buffer[1];
    }

    uint8_t crcRead = C[0] >> 12;
    uint8_t crcCalculated = crc4(&C[0]);

    if (crcCalculated != crcRead) {
        return false;
    }
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
void calculate() {
    // Given C1-C6 and D1, D2, calculated TEMP and P
    // Do conversion first and then second order temp compensation

    int32_t dT = 0;
    int64_t SENS = 0;
    int64_t OFF = 0;
    int32_t SENSi = 0;
    int32_t OFFi = 0;
    int32_t Ti = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    // Terms called
    dT = D2_temp - uint32_t(C[5]) * 256l;
    if (_model == MS5837_02BA) {
        SENS = int64_t(C[1]) * 65536l + (int64_t(C[3]) * dT) / 128l;
        OFF = int64_t(C[2]) * 131072l + (int64_t(C[4]) * dT) / 64l;
        P = (D1_pres * SENS / (2097152l) - OFF) / (32768l);
    }
    else {
        SENS = int64_t(C[1]) * 32768l + (int64_t(C[3]) * dT) / 256l;
        OFF = int64_t(C[2]) * 65536l + (int64_t(C[4]) * dT) / 128l;
        P = (D1_pres * SENS / (2097152l) - OFF) / (8192l);
    }

    // Temp conversion
    TEMP = 2000l + int64_t(dT) * C[6] / 8388608LL;

    //Second order compensation
    if (_model == MS5837_02BA) {
        if ((TEMP / 100) < 20) {         //Low temp
            Ti = (11 * int64_t(dT) * int64_t(dT)) / (34359738368LL);
            OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
        }
    }
    else {
        if ((TEMP / 100) < 20) {         //Low temp
            Ti = (3 * int64_t(dT) * int64_t(dT)) / (8589934592LL);
            OFFi = (3 * (TEMP - 2000) * (TEMP - 2000)) / 2;
            SENSi = (5 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            if ((TEMP / 100) < -15) {    //Very low temp
                OFFi = OFFi + 7 * (TEMP + 1500l) * (TEMP + 1500l);
                SENSi = SENSi + 4 * (TEMP + 1500l) * (TEMP + 1500l);
            }
        }
        else if ((TEMP / 100) >= 20) {    //High temp
            Ti = 2 * (dT * dT) / (137438953472LL);
            OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
            SENSi = 0;
        }
    }

    OFF2 = OFF - OFFi;           //Calculate pressure and temp second order
    SENS2 = SENS - SENSi;

    TEMP = (TEMP - Ti);

    if (_model == MS5837_02BA) {
        P = (((D1_pres * SENS2) / 2097152l - OFF2) / 32768l);
    }
    else {
        P = (((D1_pres * SENS2) / 2097152l - OFF2) / 8192l);
    }
}
float pressure() {
    if (_model == MS5837_02BA) {
        return P / 100.0f;
    }
    else {
        return P / 10.0f;
    }
}

float temperature() {
    return TEMP / 100.0f;
}

float depth() {
    return (pressure(Pa) - 101300) / (fluidDensity * 9.80665);
}

void read() {

    uint8_t reg;
    uint8_t buffer[3];

    reg = MS5837_CONVERT_D1_8192;
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, false);
    sleep_ms(20);

    reg = MS5837_ADC_READ;
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, false);

    i2c_read_blocking(I2C_PORT, I2C_ADDR, buffer, 3, false);
    D1_pres = buffer[0] << 16 | buffer[1] << 8 | buffer[2];



    reg = MS5837_CONVERT_D2_8192;
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, false);
    sleep_ms(20);

    reg = MS5837_ADC_READ;
    i2c_write_blocking(I2C_PORT, I2C_ADDR, &reg, 1, false);

    i2c_read_blocking(I2C_PORT, I2C_ADDR, buffer, 3, false);
    D2_temp = buffer[0] << 16 | buffer[1] << 8 | buffer[2];

    calculate();
}


int main() {

    stdio_init_all();
    sleep_ms(3000);
    printf("Hello, world!\n");
    bool test = init();
    printf("%d", test);
    sleep_ms(3000);

    for (;;) {
        read();
        float p = pressure();
        float t = temperature();
        printf("%f      %f\n", p, t);
        sleep_ms(100);
    }

}