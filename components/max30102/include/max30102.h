#pragma once
#include "driver/i2c.h"
#define MAX30102_I2C_ADDR           0x57
#define MAX30102_INT_STATUS_1       0x00
#define MAX30102_FIFO_WR_PTR        0x04
#define MAX30102_FIFO_RD_PTR        0x06
#define MAX30102_FIFO_DATA          0x07
#define MAX30102_FIFO_CONFIG        0x08
#define MAX30102_MODE_CONFIG        0x09
#define MAX30102_SPO2_CONFIG        0x0A
#define MAX30102_LED_PULSE_AMP_1    0x0C
#define MAX30102_LED_PULSE_AMP_2    0x0D
#define MAX30102_PART_ID            0xFE
typedef struct {
    uint32_t ir;
    uint32_t red;
} max30102_data_t;
esp_err_t max30102_init(i2c_port_t i2c_bus_num);
esp_err_t max30102_read_fifo(i2c_port_t i2c_bus_num, max30102_data_t *data);