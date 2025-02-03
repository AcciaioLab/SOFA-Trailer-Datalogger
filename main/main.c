/*  
 * SOFA TRAILER DATALOGGER
 * 
 * Connects and reads accelerometer data from LSM6DSOX via I2C.
 * Responds to CANBus requests and sends to Teletonika.
 *
 * Based on twai_network_example_slave and twai_network_example_master for CAN bus comms
 * Based on i2c_basic_example for I2C comms
 * 
 * 
 */

#include <stdio.h>
#include <stdlib.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"

/*
* Definitions
*/ 

// Logging Tags
static const char *I2C_ACCEL_TAG = "Axle_1_L_I2C";
static const char *CAN_ACCEL_TAG = "Axle_1_L_CAN";

/* 
 * I2C / Accelerometer constants
 */
// I2C Master
#define I2C_MASTER_SCL_IO           12                          // GPIO number used for I2C master clock
#define I2C_MASTER_SDA_IO           11                          // GPIO number used for I2C master data
#define I2C_MASTER_NUM              0                           // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0                           // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0                           // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// I2C Accelerometer
#define ACCEL_ADDR                  0x6A                       // Accelerometer I2C address

// Accelerometer data conversion (for diagnostic)
static const float ACCEL_SCALE = 0.000244; // +/-8 LSB value

/*
 * CAN BUS constants
 */
#define DATA_PERIOD_MS                  500
#define ITER_DELAY_MS                   1000
#define RX_TASK_PRIO                    8       // Receiving task priority
#define TX_TASK_PRIO                    9       // Sending task priority
#define CTRL_TSK_PRIO                   10      // Control task priority
#define TX_GPIO_NUM                     13
#define RX_GPIO_NUM                     14

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

/* 
 * I2C / Accelerometer data
 */
static i2c_master_bus_config_t i2c_master_config = 
{
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_MASTER_NUM,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

static i2c_master_bus_handle_t i2c_bus_handle;

static i2c_device_config_t i2c_accel_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ACCEL_ADDR,
    .scl_speed_hz = 100000,
};

static i2c_master_dev_handle_t i2c_accel_handle;

typedef struct {
    uint8_t reg;
    size_t length;
    uint8_t a[6];
    int16_t a_X, a_Y, a_Z;
    float A_X, A_Y, A_Z;
    esp_err_t err;
} AccelData;

typedef struct {
    uint8_t tx[2];  // Register address, data to write
    uint8_t rx;     // Received data
    esp_err_t err;
} i2c_1_reg_rw;

typedef struct {
    uint8_t tx;     // Register address
    uint8_t rx;     // Received data
    uint8_t success; // If the response is known to compare
    esp_err_t err;
} i2c_1_reg_r;

// 
AccelData acceldata = {
    .reg = 0x28,    // accel x axis reg, y follows 0x2A, z follows 0x2C
    .length = 6,    // read all accel data in 1 go
    .err = ESP_FAIL
};

// TX1 - Who Am I
i2c_1_reg_r tx1_whoami = {
    .tx = 0x0F,     // Who Am I register
    .rx = 0x00,     // Expecting 0x6C
    .success = 0x6C,
    .err = ESP_FAIL
};

// TX2 - Config 1
i2c_1_reg_rw tx1_config1 = {
    .tx = {0x10, 0x4C},
    .rx = 0x00,
    .err = ESP_FAIL
};

/* 
 * CAN BUS data
 */

static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

static twai_message_t data_message = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 0,                // Not single shot
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = ID_SLAVE_DATA,
    .data_length_code = 6,
    .data = {0x00, 0x00, 0x00, 0x00, 0, 0, 0, 0x00},
};

// Function prototypes
// Keeping everything in here and optimise code after it works
void accel_tx_whoami(void);
void accel_tx_config1(void);
void accel_tx_selftest(void);

void taskAccelI2C(void *pvParameters)
{
    // Log the accelerometer task has started.
    ESP_LOGI(I2C_ACCEL_TAG, "I2C task created.");

    // uint8_t i2c_tx_data[3] = {0, 0, 0};
    // uint8_t i2c_rx_data = 0;
    esp_err_t i2c_err;

    // Add the bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_config, &i2c_bus_handle));
    // Add the device on the bus
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &i2c_accel_cfg, &i2c_accel_handle));

    // Read the WHO_AM_I (0x0F) register to see if we can see the accelerometer.
    accel_tx_whoami();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Configure the accelerometer before reading data.
    accel_tx_config1();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Perform self test
    accel_tx_selftest();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Loop to read in accelerometer data
    while(1)
    {
        // ESP_LOGI(I2C_ACCEL_TAG, "In transmit loop");

        i2c_err = i2c_master_transmit_receive(i2c_accel_handle, &acceldata.reg, sizeof(acceldata.reg), acceldata.a, acceldata.length, -1);
        if (i2c_err == ESP_OK)
        {
            acceldata.a_X = ((int16_t)acceldata.a[1] << 8) + acceldata.a[0];
            acceldata.A_X = ((float)acceldata.a_X) * ACCEL_SCALE;
            // DEBUG: X axis data
            // ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer X axis reg %#04X %#04X = %#06X = %d = %.6f", acceldata.a[0], acceldata.a[1], acceldata.a_X, acceldata.A_X);

            acceldata.a_Y = ((int16_t)acceldata.a[3] << 8) + acceldata.a[2];
            acceldata.A_Y = ((float)acceldata.a_Y) * ACCEL_SCALE;
            // DEBUG: Y axis data
            // ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer Y axis reg %#04X %#04X = %#06X = %d = %.6f", acceldata.a_y[2], acceldata.a_y[3], acceldata.a_Y, acceldata.A_Y);

            acceldata.a_Z = ((int16_t)acceldata.a[5] << 8) + acceldata.a[4];
            acceldata.A_Z = ((float)acceldata.a_Z) * ACCEL_SCALE;
            // DEBUG: Z axis data
            // ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer Z axis reg %#04X %#04X = %#06X = %d = %.6f", acceldata.a_z[4], acceldata.a_z[5], acceldata.a_Z, acceldata.A_Z);
        }

        ESP_LOGI(I2C_ACCEL_TAG, "X/Y/Z axis reg %#04X %#04X %#06X | %#04X %#04X %#06X | %#04X %#04X %#06X = %.6f %.6f %.6f", acceldata.a[0], acceldata.a[1], acceldata.a_X, acceldata.a[2], acceldata.a[3], acceldata.a_Y, acceldata.a[4], acceldata.a[5], acceldata.a_Z, acceldata.A_X, acceldata.A_Y, acceldata.A_Z);

        vTaskDelay(pdMS_TO_TICKS(500));

    }

    // Remove the device
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(i2c_accel_handle));
    // Uninstall the bus
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus_handle));
    ESP_LOGI(I2C_ACCEL_TAG, "I2C removed successfully.");

    vTaskDelete(NULL);
}

void taskCANRx(void *pvParameters)
{
    ESP_LOGI(CAN_ACCEL_TAG, "Receive task started.");
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    vTaskDelete(NULL);
}

void taskCANTx(void *pvParameters)
{
    ESP_LOGI(CAN_ACCEL_TAG, "Transmit task started.");
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    vTaskDelete(NULL);
}

void taskCANCtrl(void *pvParameters)
{
    ESP_LOGI(CAN_ACCEL_TAG, "Control task started.");
    //Install TWAI driver, trigger tasks to start
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_ACCEL_TAG, "Driver installed");

    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(CAN_ACCEL_TAG, "Driver uninstalled");
    
    vTaskDelete(NULL);
}

void app_main(void)
{
    // 
    xTaskCreatePinnedToCore(taskAccelI2C, "Accel_Data_I2C", 4096, NULL, 4, NULL, tskNO_AFFINITY);

    // 
    xTaskCreatePinnedToCore(taskCANRx, "CANBus_Rx", 4096, NULL, 1, NULL, tskNO_AFFINITY);

    // 
    xTaskCreatePinnedToCore(taskCANTx, "CANBus_Tx", 4096, NULL, 2, NULL, tskNO_AFFINITY);

    // 
    xTaskCreatePinnedToCore(taskCANCtrl, "CANBus_Ctrl", 4096, NULL, 3, NULL, tskNO_AFFINITY);

    

    // Semaphores


    
    
    // Don't need to start the scheduler - it starts automatically.
    // vTaskStartScheduler();
    
}

void accel_tx_whoami(void)
{
    // Read the WHO_AM_I (0x0F) register to see if we can see the accelerometer.
    // WHO_AM_I returns 0x6C.
    while (tx1_whoami.rx != 0x6C)
    {
        tx1_whoami.err = i2c_master_transmit_receive(i2c_accel_handle, &tx1_whoami.tx, sizeof(tx1_whoami.tx), &tx1_whoami.rx, 1, -1);
        ESP_LOGI(I2C_ACCEL_TAG, "Connecting to accelerometer on I2C bus at address 0x6A.");
    }

    // Log that we saw the accelerometer
    ESP_LOGI(I2C_ACCEL_TAG, "I2C initialized successfully. WHO AM I = 0x%X", tx1_whoami.rx);
    
    // Reset in case we need to use it again
    tx1_whoami.rx = 0x00;

}

void accel_tx_config1(void)
{
    // Configure the accelerometer before reading data.
    // CTRL1_XL 0x10 Accelerometer control register 1 (R/W)
    // 104Hz normal mode, +/- 8g

    i2c_master_transmit(i2c_accel_handle, tx1_config1.tx, 2, -1);

    // Check the value we sent wrote to register 0x10
    tx1_config1.err = i2c_master_transmit_receive(i2c_accel_handle, &tx1_config1.tx[0], sizeof(tx1_config1.tx[0]), &tx1_config1.rx, 1, -1);
    if (tx1_config1.err == ESP_OK)
        {
            if (tx1_config1.rx != tx1_config1.tx[1])
            {
                ESP_LOGI(I2C_ACCEL_TAG, "Couldn't write accelerometer control register 0x10");
            }
            ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer control reg 0x10 wrote 0x%X", tx1_config1.rx);
        }
}

void accel_tx_selftest(void)
{
    // Self test

}