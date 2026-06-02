#include "imu.hpp"


extern State state;

static float roll0 = 0.0f;
static float pitch0 = 0.0f;
static float yaw0 = 0.0f;

static const float alpha = 0.1;
static float wx_filt_last = 0;
static float wy_filt_last = 0;
static float wz_filt_last = 0;


float wrapAngle(float a) {
    while (a > M_PI) a -= 2 * M_PI;
    while (a < -M_PI) a += 2 * M_PI;
    return a;
}

uint8_t buffer[6];

void imu::init() {

    i2c_init(BNO055_PORT, 400 * 1000);
    gpio_set_function(BNO055_SDA, GPIO_FUNC_I2C);
    gpio_set_function(BNO055_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(BNO055_SDA);
    gpio_pull_up(BNO055_SCL);

    uint8_t reg = 0x00;
    uint8_t chipID = 0;

    while (chipID != 0xA0) {
#if DEBUG_MODE
        printf("BNO055 not connected\n");
#endif
        i2c_write_blocking(BNO055_PORT, BNO055_ADDR, &reg, 1, false);
        i2c_read_blocking(BNO055_PORT, BNO055_ADDR, &chipID, 1, false);
        sleep_ms(500);
    }

#if DEBUG_MODE
    printf("BNO055 connected\n");
#endif
    sleep_ms(1000);

    uint8_t data[2];

    data[0] = 0x3D;     // OPR_MODE
    data[1] = 0x00;     // CONFIG mode
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(50);

    data[0] = 0x3F;     // SYS_TRIGGER
    data[1] = 0x40;     // internal oscillator
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(50);

    data[0] = 0x3B;     // UNIT_SEL
    data[1] = 0x06;     // gyro in rad/s
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(50);

    data[0] = 0x3D;     // OPR_MODE
    data[1] = 0x08;     // IMU
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, data, 2, false);
    sleep_ms(500);

    // read roll and pitch for lock values
    uint8_t reg_euler = 0x1A;

    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, &reg_euler, 1, false);
    i2c_read_blocking(BNO055_PORT, BNO055_ADDR, buffer, 6, false);
    int16_t raw_roll0 = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_pitch0 = (int16_t)((buffer[5] << 8) | buffer[4]);
    int16_t raw_yaw0 = (int16_t)((buffer[1] << 8) | buffer[0]);
    roll0 = (raw_roll0 / 900.0f);
    pitch0 = (raw_pitch0 / 900.0f);

#if DEBUG_MODE
    printf("Roll, Pitch, Yaw locked\n");
#endif
}

void imu::ask_euler() {
    uint8_t reg = 0x1A;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, &reg, 1, false);
}

void imu::read_euler() {
    i2c_read_blocking(BNO055_PORT, BNO055_ADDR, buffer, 6, false);

    int16_t raw_roll = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_pitch = (int16_t)((buffer[5] << 8) | buffer[4]);

    state.roll = wrapAngle(raw_roll / 900.0f - roll0);
    state.pitch = wrapAngle(raw_pitch / 900.0f - pitch0);
}

void imu::ask_gyro() {
    uint8_t reg = 0x14;
    i2c_write_blocking(BNO055_PORT, BNO055_ADDR, &reg, 1, false);
}

void imu::read_gyro() {
    i2c_read_blocking(BNO055_PORT, BNO055_ADDR, buffer, 6, false);

    int16_t raw_wx = (int16_t)((buffer[1] << 8) | buffer[0]);
    int16_t raw_wy = (int16_t)((buffer[3] << 8) | buffer[2]);
    int16_t raw_wz = (int16_t)((buffer[5] << 8) | buffer[4]);

    state.wx = raw_wx / 900.0f;
    state.wy = raw_wy / 900.0f;
    state.wz = raw_wz / 900.0f;
}