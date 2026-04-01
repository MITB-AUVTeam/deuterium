#pragma once

//stablization loop speed in milli seconds
#define STB_LOOP_MS 10

//time out for navigation comms to be sent by raspi mpu in micro seconds
#define NAV_TIME_OUT_US 500000

//BNO055
#define BNO055_PORT i2c1
#define BNO055_ADDR 0x28
#define BNO055_SDA  26
#define BNO055_SCL  27

//MS5837
#define MS5837_PORT i2c0
#define MS5837_ADDR 0x76
#define MS5837_SDA 28
#define MS5837_SCL 29

//PIO of ESC
#define PIO_VB 5
#define PIO_VR 6
#define PIO_VL 7
#define PIO_HR 8
#define PIO_HL 9

//RASPI UART 
#define RASPI_TX 24
#define RASPI_RX 25
#define RASPI_BAUDRATE  115200
#define RASPI_UARTID  uart1
#define RASPI_SOF0 0xAA
#define RASPI_SOF1 0x55