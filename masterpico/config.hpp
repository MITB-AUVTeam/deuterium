#pragma once

//PICO uses Serial1
#define PICO_TXPIN   18
#define PICO_RXPIN   19
#define PICO_BAUDRATE  115200

//RP uses Serial2
#define RP_TXPIN   17
#define RP_RXPIN   16
#define RP_BAUDRATE  115200

//BNO055
#define BNO055_SDA  21
#define BNO055_SCL  22
#define BNO055_BAUDRATE 10000