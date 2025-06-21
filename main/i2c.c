#include "i2c.h"

/* 
 * I2C / Accelerometer data
 */
i2c_master_bus_config_t i2cMasterConfig = 
{
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_MASTER_NUM,
    .scl_io_num = I2C_MASTER_SCL_GPIO,
    .sda_io_num = I2C_MASTER_SDA_GPIO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t i2cBusHandle;

i2c_device_config_t i2c_accel_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ACCEL_ADDR,
    .scl_speed_hz = 100000,
};

i2c_master_dev_handle_t i2cIMUHandle;

int i2cBusStart(void)
{
    // Add the bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2cMasterConfig, &i2cBusHandle));
    // Add the device on the bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2cBusHandle, &i2c_accel_cfg, &i2cIMUHandle));

    return 0;
}

int i2cBusEnd(void)
{
    // Remove the device
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(i2cIMUHandle));
    // Uninstall the bus
    ESP_ERROR_CHECK(i2c_del_master_bus(i2cBusHandle));

    return 0;
}

int i2cTransmitReg(i2cWrite1Reg *data)
{
    // This function bundles writing data to a register over i2c and then reading that same register
    // to make sure it wrote.
    i2c_master_transmit(i2cIMUHandle, data->tx, 2, -1);

    // Check the value we sent wrote to register 0x10
    data->err = i2c_master_transmit_receive(i2cIMUHandle, &data->tx[0], sizeof(data->tx[0]), &data->rx, 1, -1);
    if (data->err == ESP_OK)
    {
        if (data->rx != data->tx[1])
        {
            ESP_LOGI(I2C_DRIVER_TAG, "Couldn't write to register %#04X.", data->tx[0]);
            return -1;
        }
        ESP_LOGI(I2C_DRIVER_TAG, "Wrote %#04X to register %#04X.", data->rx, data->tx[0]);
    }
    return 0;
}

int i2cReadReg(i2cRead1Reg *data)
{
    data->err = i2c_master_transmit_receive(i2cIMUHandle, &data->tx, sizeof(data->tx), &data->rx, 1, -1);
    if (data->err == ESP_ERR_TIMEOUT)
    {
        ESP_LOGI(I2C_DRIVER_TAG, "Couldn't read register %#04X.", data->tx);
        return -1;
    }
    
    return 0;
}

int i2cReadDataReg(i2cReadIMUReg *data)
{
    data->err = i2c_master_transmit_receive(i2cIMUHandle, &data->reg, sizeof(data->reg), data->m, data->length, -1);
    if (data->err == ESP_ERR_TIMEOUT)
    {
        return -1;
    }

    return 0;
}