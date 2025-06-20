#include "i2c.h"

/* 
 * I2C / Accelerometer data
 */
i2c_master_bus_config_t i2c_master_config = 
{
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_MASTER_NUM,
    .scl_io_num = I2C_MASTER_SCL_GPIO,
    .sda_io_num = I2C_MASTER_SDA_GPIO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t i2c_bus_handle;

i2c_device_config_t i2c_accel_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ACCEL_ADDR,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t i2c_accel_handle;

void imuWhoami(void)
{
    // Create the data for this read.
    // TX1 - Who Am I
    i2cRead1Reg whoami = {
        .tx = 0x0F,     // Who Am I register
        .rx = 0x00,     // Expecting 0x6C
        .success = 0x6C,
        .err = ESP_FAIL
    };  

    // Read the WHO_AM_I (0x0F) register to see if we can see the accelerometer.
    // WHO_AM_I returns 0x6C.
    while (whoami.rx != 0x6C)
    {
        whoami.err = i2c_master_transmit_receive(i2c_accel_handle, &whoami.tx, sizeof(whoami.tx), &whoami.rx, 1, -1);
        ESP_LOGI(I2C_DRIVER_TAG, "Connecting to accelerometer on I2C bus at address 0x6A.");
    }

    // Log that we saw the accelerometer
    ESP_LOGI(I2C_DRIVER_TAG, "I2C initialized successfully. WHO AM I = 0x%X", whoami.rx);
    
    // Reset in case we need to use it again
    whoami.rx = 0x00;
}

void imuConfig(void)
{
    // Configure the accelerometer before reading data.

    // Config 1
    // CTRL1_XL 0x10 Accelerometer control register 1 (R/W)
    // 104Hz high-performance mode, +/- 8g. 
    i2cWrite1Reg config1 = {
        .tx = {0x10, 0x4C},
        .rx = 0x00,
        .err = ESP_FAIL
    };
    
    i2cTransmitReg(&config1);

    // Config 2
    // CTRL2_G 0x11 Gyroscope control register 2 (R/W)
    // 104Hz high-performance mode, +/- 1000dps
    i2cWrite1Reg config2 = {
        .tx = {0x11, 0x48},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    i2cTransmitReg(&config2);
}

int i2cTransmitReg(i2cWrite1Reg *data)
{
    // This function bundles writing data to a register over i2c and then reading that same register
    // to make sure it wrote.
    i2c_master_transmit(i2c_accel_handle, data->tx, 2, -1);

    // Check the value we sent wrote to register 0x10
    data->err = i2c_master_transmit_receive(i2c_accel_handle, &data->tx[0], sizeof(data->tx[0]), &data->rx, 1, -1);
    if (data->err == ESP_OK)
    {
        if (data->rx != data->tx[1])
        {
            ESP_LOGI(I2C_DRIVER_TAG, "Couldn't write to register 0x%X.", data->tx[0]);
            return -1;
        }
        ESP_LOGI(I2C_DRIVER_TAG, "Wrote 0x%X to register 0x%X.", data->rx, data->tx[0]);
    }
    return 0;
}