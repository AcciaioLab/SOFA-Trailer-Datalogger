#ifndef I2C_H
#define I2C_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

static char *I2C_DRIVER_TAG = "SOFA_DL_DRV_I2C";

extern i2c_master_bus_config_t i2c_master_config;
extern i2c_master_bus_handle_t i2c_bus_handle;
extern i2c_device_config_t i2c_accel_cfg;
extern i2c_master_dev_handle_t i2c_accel_handle;

/* 
 * I2C / Accelerometer constants
 */
// Pins
#define I2C_MASTER_SDA_GPIO         1                          // GPIO number used for I2C master clock
#define I2C_MASTER_SCL_GPIO         2                          // GPIO number used for I2C master data

// I2C Master
#define I2C_MASTER_NUM              0                           // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0                           // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0                           // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// I2C Accelerometer
#define ACCEL_ADDR                  0x6A                       // Accelerometer I2C address

typedef struct {
    uint8_t tx;     // Register address
    uint8_t rx;     // Received data
    esp_err_t err;
} i2cRead1Reg;

typedef struct {
    uint8_t tx[2];  // Register address, data to write
    uint8_t rx;     // Received data
    esp_err_t err;
} i2cWrite1Reg;

typedef struct {
    uint8_t reg;
    size_t length;
    uint8_t m[6]; 
    esp_err_t err;
} i2cReadIMUReg;

// Function Prototypes
int i2cBusStart(void);
int i2cBusEnd(void);
int i2cTransmitReg(i2cWrite1Reg *data);
int i2cReadReg(i2cRead1Reg *data);
int i2cReadDataReg(i2cReadIMUReg *data);

#endif