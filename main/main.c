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
static const char *CON_ACCEL_TAG = "Axle_1_L_CON";
static const char *I2C_ACCEL_TAG = "Axle_1_L_I2C";
static const char *CAN_ACCEL_TAG = "Axle_1_L_CAN";

// Tasks
#define TASK_PRIORITY_ACCELI2C      13
#define TASK_PRIORITY_CAN_RX        9
#define TASK_PRIORITY_CAN_TX        11
#define TASK_PRIORITY_CAN_CTRL      12

// Pins
#define I2C_MASTER_SDA_GPIO         1                          // GPIO number used for I2C master clock
#define I2C_MASTER_SCL_GPIO         2                          // GPIO number used for I2C master data
#define CAN_RX_GPIO                 14                          
#define CAN_TX_GPIO                 13

// Device Constants
static const int startupDelay = 1;                             // Delay time at boot to line up with FMC650

// Queues and Semaphores
// static QueueHandle_t CAN_rx_queue;
// static QueueHandle_t CAN_tx_queue;
static QueueHandle_t CAN_control_queue;
static SemaphoreHandle_t SEM_Accel_Data;
static SemaphoreHandle_t SEM_CAN_Control;
static SemaphoreHandle_t SEM_Done;

typedef enum {
    CAN_Init,
    CAN_RX_RTR_LISTEN,
    CAN_RX_RTR_RECEIVED,
    CAN_TX_ACCEL,
    CAN_RX_END,
    CAN_TX_END
} CAN_control_actions;

// typedef enum {
//     CAN_RX_RTR,
//     CAN_RX_END
// } CAN_rx_actions;

// typedef enum {
//     CAN_TX_ACCEL,
//     CAN_TX_END
// } CAN_tx_actions;

/* 
 * I2C / Accelerometer constants
 */
// I2C Master

#define I2C_MASTER_NUM              0                           // I2C port number for master dev
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY // I2C master clock frequency
#define I2C_MASTER_TX_BUF_DISABLE   0                           // I2C master doesn't need buffer
#define I2C_MASTER_RX_BUF_DISABLE   0                           // I2C master doesn't need buffer
#define I2C_MASTER_TIMEOUT_MS       1000

// I2C Accelerometer
#define ACCEL_ADDR                  0x6A                       // Accelerometer I2C address

// Accelerometer data conversion (for diagnostic)
static const float ACCEL_SCALE = 0.000244; // +/-8 LSB value

// Data ready
static const uint8_t XLDA = 0x01;
// static const uint8_t GDA = 0x02;

/*
 * CAN BUS constants
 */
#define DATA_PERIOD_MS                  500
#define ITER_DELAY_MS                   1000

#define ID_MASTER_STOP_CMD              0x0A0
#define ID_MASTER_START_CMD             0x0A1
#define ID_MASTER_PING                  0x0A2
#define ID_SLAVE_STOP_RESP              0x0B0
#define ID_SLAVE_DATA                   0x0B1
#define ID_SLAVE_PING_RESP              0x0B2

#define CAN_ID_ACCEL_RTR                0x0300
#define CAN_ID_ACCEL_RESP               0x0101
#define CAN_ID_TEST                     0x0333

/* 
 * I2C / Accelerometer data
 */
static i2c_master_bus_config_t i2c_master_config = 
{
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_MASTER_NUM,
    .scl_io_num = I2C_MASTER_SCL_GPIO,
    .sda_io_num = I2C_MASTER_SDA_GPIO,
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
    uint8_t tx;     // Register address
    uint8_t rx;     // Received data
    uint8_t success; // If the response is known to compare
    esp_err_t err;
} i2c_1_reg_r;

typedef struct {
    uint8_t tx[2];  // Register address, data to write
    uint8_t rx;     // Received data
    esp_err_t err;
} i2c_1_reg_rw;

/* 
 * CAN BUS data
 */

// static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO, CAN_RX_GPIO, TWAI_MODE_NORMAL);
static const twai_general_config_t g_config = {
    .mode = TWAI_MODE_NORMAL,
    .tx_io = CAN_TX_GPIO,
    .rx_io = CAN_RX_GPIO,
    .clkout_io = TWAI_IO_UNUSED,
    .bus_off_io = TWAI_IO_UNUSED,
    .tx_queue_len = 5,
    .rx_queue_len = 5,
    .alerts_enabled = TWAI_ALERT_NONE,
    .clkout_divider = 0
};
static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

// static twai_message_t CAN_received_message;
static twai_message_t CAN_Data_Accelerometer = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 0,                // Not single shot
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = CAN_ID_ACCEL_RESP,
    .data_length_code = 7,
    .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
};

static twai_message_t CAN_TEST = {
    // Message type and format settings
    .extd = 0,              // Standard Format message (11-bit ID)
    .rtr = 0,               // Send a data frame
    .ss = 0,                // Not single shot
    .self = 0,              // Not a self reception request
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = CAN_ID_TEST,
    .data_length_code = 4,
    .data = {0xDE, 0xAD, 0xBE, 0xEF},
};

// Function prototypes
// Keeping everything in here and optimise code after it works
int i2c_tx_check(i2c_1_reg_rw *data);
esp_err_t accel_read_data(AccelData *data);
int accel_align_data(AccelData *data);
void accel_tx_whoami(void);
void accel_tx_config1(void);
int accel_tx_selftest(void);

void taskAccelI2C(void *pvParameters)
{
    AccelData acceldata = {
        .reg = 0x28,    // accel x axis reg, y follows 0x2A, z follows 0x2C
        .length = 6,    // read all accel data in 1 go
        .err = ESP_FAIL
    };

    esp_err_t i2c_err;

    // Log the accelerometer task has started.
    ESP_LOGI(I2C_ACCEL_TAG, "I2C task created.");

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
    // accel_tx_selftest();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Loop to read in accelerometer data
    while(1)
    {
        // ESP_LOGI(I2C_ACCEL_TAG, "In transmit loop");
        xSemaphoreTake(SEM_Accel_Data, portMAX_DELAY);

        i2c_err = accel_read_data(&acceldata);
       
        if (i2c_err == ESP_OK)
        {
            accel_align_data(&acceldata);
            // DEBUG: X axis data
            // ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer X axis reg %#04X %#04X = %#06X = %.6f", acceldata.a[0], acceldata.a[1], acceldata.a_X, acceldata.A_X);
            // DEBUG: Y axis data
            // ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer Y axis reg %#04X %#04X = %#06X = %.6f", acceldata.a_y[2], acceldata.a_y[3], acceldata.a_Y, acceldata.A_Y);
            // DEBUG: Z axis data
            // ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer Z axis reg %#04X %#04X = %#06X = %.6f", acceldata.a_z[4], acceldata.a_z[5], acceldata.a_Z, acceldata.A_Z);
        }

        // DEBUG: Print out axis data in real number.
        ESP_LOGI(I2C_ACCEL_TAG, "X/Y/Z axis reg %#04X %#04X %#06X | %#04X %#04X %#06X | %#04X %#04X %#06X = %.6f %.6f %.6f", acceldata.a[0], acceldata.a[1], acceldata.a_X, acceldata.a[2], acceldata.a[3], acceldata.a_Y, acceldata.a[4], acceldata.a[5], acceldata.a_Z, acceldata.A_X, acceldata.A_Y, acceldata.A_Z);

        // Move into CAN message.
        CAN_Data_Accelerometer.data[0] = acceldata.a[0];
        CAN_Data_Accelerometer.data[1] = acceldata.a[1];
        CAN_Data_Accelerometer.data[2] = acceldata.a[2];
        CAN_Data_Accelerometer.data[3] = acceldata.a[3];
        CAN_Data_Accelerometer.data[4] = acceldata.a[4];
        CAN_Data_Accelerometer.data[5] = acceldata.a[5];
        CAN_Data_Accelerometer.data[6] = 0xFF;

        xSemaphoreGive(SEM_Accel_Data);
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
    // xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
    ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task started.");

    while (1)
    {
        // Read the queue to see if a receive action is required.
        CAN_control_actions actions;
        xQueueReceive(CAN_control_queue, &actions, portMAX_DELAY);

        // Check what action was required.
        if (actions == CAN_RX_RTR_LISTEN)
        {
            xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
            ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task received action CAN_RX_RTR %d.", actions);
            twai_message_t received_msg;
            while (1)
            {
                // Waiting to receive request
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task waiting to receive request for data.");
                esp_err_t err_rxrtr = twai_receive(&received_msg, portMAX_DELAY);
                // if (err_rxrtr != ESP_OK)
                // {
                //     ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task twai_receive NOT GOOD %d.", err_rxrtr);
                // }
                // if (received_msg.identifier == CAN_ID_ACCEL_RTR)
                // {
                //     ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task CAN_ID_ACCEL_RTR received.");
                // }
                // if (received_msg.rtr)
                // {
                //     // The message was a request for data.
                //     ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task request for data received.");
                    
                //     // Send the message from here, as debug.
                //     // Who knows will probably be final code.
                //     twai_transmit(&CAN_Data_Accelerometer, portMAX_DELAY);
                //     ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task returned data as requested.");

                //     // Health
                //     CAN_Data_Accelerometer.data[6] = 0x00;

                //     // Give the semaphore back and loop around.
                //     xSemaphoreGive(SEM_CAN_Control);
                //     break;
                // }
                
                twai_transmit(&CAN_Data_Accelerometer, portMAX_DELAY);
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task sent accelerometer data.");
            }
        }
        else if (actions == CAN_Init)
        {
            ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task no receive action, current action %d.", actions);
        }
        else
        {
            ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task unknown action %d.", actions);
        }
    }

    xSemaphoreGive(SEM_CAN_Control);
    vTaskDelete(NULL);
}

void taskCANTx(void *pvParameters)
{
    // xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
    ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task started.");

    while (1)
    {
        // Read the queue to see if a receive action is required.
        CAN_control_actions actions;
        xQueueReceive(CAN_control_queue, &actions, portMAX_DELAY);

        // Check what action was required.
        if (actions == CAN_TX_ACCEL)
        {
            // Response to RTR with accelerometer data.
            xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
            ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task received action CAN_TX_ACCEL %d.", actions);

        //     twai_message_t send_msg;

            xSemaphoreGive(SEM_CAN_Control);
            
        }
        // else if (actions == CAN_Init)
        // {
        //     ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task no transmit action, current action %d.", actions);
        // }
        // else if (actions == CAN_RX_RTR_LISTEN)
        // {
        //     // Do nothing
        //     vTaskDelay(pdMS_TO_TICKS(100));
        // }
        // else
        // {
        //     ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task unknown action %d.", actions);
        // }
        
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    xSemaphoreGive(SEM_CAN_Control);
    vTaskDelete(NULL);
}

void taskCANCtrl(void *pvParameters)
{
    // Start when the control semaphore is available.
    xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
    xSemaphoreTake(SEM_Done, portMAX_DELAY);

    ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task started.");

    CAN_control_actions actions = CAN_Init;

    // xSemaphoreGive(SEM_CAN_Control);

    while(1)
    {
        switch (actions)
        {
            case CAN_Init:
                // CAN started, wait for RTR.
                actions = CAN_RX_RTR_LISTEN;

                // Tell RX task to start listening
                xQueueSend(CAN_control_queue, &actions, portMAX_DELAY);
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to CAN_RX_RTR %d.", actions);
                
                xSemaphoreGive(SEM_CAN_Control);
                break;

            case CAN_RX_RTR_LISTEN:
                // ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task waiting for RTR.");
                twai_transmit(&CAN_TEST, portMAX_DELAY);
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task sent CAN_TEST.");

                // //
                // xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);

                // // Got the semaphore from RX
                // actions = CAN_RX_RTR_LISTEN;
                // xQueueSend(CAN_control_queue, &actions, portMAX_DELAY);
                // ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to CAN_RX_RTR %d.", actions);
                // xSemaphoreGive(SEM_CAN_Control);
                break;
            
            case CAN_TX_ACCEL:
                // ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task waiting for TX ACCEL.");
                // xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
                // ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to CAN_TX_ACCEL.");
                // actions = CAN_RX_RTR_LISTEN;
                
                // xQueueSend(CAN_control_queue, &actions, portMAX_DELAY);
                // xSemaphoreGive(SEM_CAN_Control);
                break;

            default:
                xSemaphoreGive(SEM_CAN_Control);
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    // Delete the control task
    // Shouldn't get here
    xSemaphoreGive(SEM_Done);
    vTaskDelete(NULL);
}

void app_main(void)
{
    // Delay the start of the device to line up with the FMC650
    for (int i = 0; i < startupDelay; i++)
    {
        ESP_LOGI(CON_ACCEL_TAG, "Device starting in %d\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Semaphores and queues
    // CAN_rx_queue = xQueueCreate(1, sizeof(CAN_rx_actions));
    // CAN_tx_queue = xQueueCreate(1, sizeof(CAN_tx_actions));
    CAN_control_queue = xQueueCreate(1, sizeof(CAN_control_actions));

    SEM_Accel_Data = xSemaphoreCreateBinary();
    SEM_CAN_Control = xSemaphoreCreateBinary();
    SEM_Done = xSemaphoreCreateBinary();
    
    // Tasks
    xTaskCreatePinnedToCore(taskAccelI2C, "Accel_Data_I2C", 4096, NULL, TASK_PRIORITY_ACCELI2C, NULL, 1); // on its own core
    xTaskCreatePinnedToCore(taskCANRx, "CANBus_Rx", 4096, NULL, TASK_PRIORITY_CAN_RX, NULL, tskNO_AFFINITY);
    // xTaskCreatePinnedToCore(taskCANTx, "CANBus_Tx", 4096, NULL, TASK_PRIORITY_CAN_TX, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskCANCtrl, "CANBus_Ctrl", 4096, NULL, TASK_PRIORITY_CAN_CTRL, NULL, tskNO_AFFINITY);

    //Install TWAI driver, trigger tasks to start
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver installed");
    
    // Start the TWAI driver
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver started.");

    xSemaphoreGive(SEM_Accel_Data);
    xSemaphoreGive(SEM_CAN_Control);
    xSemaphoreGive(SEM_Done);

    vTaskDelay(pdMS_TO_TICKS(5000));

    xSemaphoreTake(SEM_Done, portMAX_DELAY);

    // End TWAI driver, stop
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver stopped.");

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver uninstalled.");

    // Delete semaphores
    vSemaphoreDelete(SEM_Accel_Data);
    vSemaphoreDelete(SEM_CAN_Control);
    vSemaphoreDelete(SEM_Done);
    
    // Don't need to start the scheduler - it starts automatically.
    // vTaskStartScheduler();
    
}

int i2c_tx_check(i2c_1_reg_rw *data)
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
            ESP_LOGI(I2C_ACCEL_TAG, "Couldn't write to register 0x%X.", data->tx[0]);
            return -1;
        }
        ESP_LOGI(I2C_ACCEL_TAG, "Wrote 0x%X to register 0x%X.", data->rx, data->tx[0]);
    }
    return 0;
}

esp_err_t accel_read_data(AccelData *data)
{
    esp_err_t accel_i2c_err;

    i2c_1_reg_r tx_data_ready = {
        .tx = 0x1E,     // STATUS_REG register
        .rx = 0x00,     // No expectation
        .success = 0x00,
        .err = ESP_FAIL
    };  

    // Check accel data is available.
    uint8_t accel_data_avail = 0x00;

    while (accel_data_avail != XLDA)
    {
        tx_data_ready.err = i2c_master_transmit_receive(i2c_accel_handle, &tx_data_ready.tx, sizeof(tx_data_ready.tx), &tx_data_ready.rx, 1, -1);
        accel_data_avail = tx_data_ready.rx & XLDA;
    }
    
    // New accel data is available, read it.
    accel_i2c_err = i2c_master_transmit_receive(i2c_accel_handle, &data->reg, sizeof(data->reg), data->a, data->length, -1);

    return accel_i2c_err;
}

int accel_align_data(AccelData *data)
{
    data->a_X = ((int16_t)data->a[1] << 8) + data->a[0];
    data->A_X = ((float)data->a_X) * ACCEL_SCALE;

    data->a_Y = ((int16_t)data->a[3] << 8) + data->a[2];
    data->A_Y = ((float)data->a_Y) * ACCEL_SCALE;

    data->a_Z = ((int16_t)data->a[5] << 8) + data->a[4];
    data->A_Z = ((float)data->a_Z) * ACCEL_SCALE;

    return 0;
}

void accel_tx_whoami(void)
{
    // Create the data for this read.
    // TX1 - Who Am I
    i2c_1_reg_r tx1_whoami = {
        .tx = 0x0F,     // Who Am I register
        .rx = 0x00,     // Expecting 0x6C
        .success = 0x6C,
        .err = ESP_FAIL
    };  

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
    // TX2 - Config 1
    i2c_1_reg_rw tx1_config1 = {
        .tx = {0x10, 0x4C},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    // CTRL1_XL 0x10 Accelerometer control register 1 (R/W)
    // 104Hz normal mode, +/- 8g
    i2c_tx_check(&tx1_config1);
}

int accel_tx_selftest(void)
{
    // Accelerometer UI self-test mode 1
    // Datasheet section 11.
    esp_err_t i2c_err;

    AccelData selftest_acceldata = {
        .reg = 0x28,    // accel x axis reg, y follows 0x2A, z follows 0x2C
        .length = 6,    // read all accel data in 1 go
        .err = ESP_FAIL
    };

    // float OUTX_NOST, OUTY_NOST, OUTZ_NOST = 0.0f;
    // float OUTX_ST, OUTY_ST, OUTZ_ST = 0.0f;

    // Initialise and turn on sensor
    // CTRL1_XL (0x10) has already been written to
    i2c_1_reg_rw tx_init = {
        .tx = {0x11, 0x00},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    for (int i = 0; i < 9; i++)
    {
        tx_init.tx[0] = 0x11 + i;
        if (tx_init.tx[0] == 0x12)
        {
            tx_init.tx[1] = 0x44;
        }
        else
        {
            tx_init.tx[1] = 0x00;
        }
        
        i2c_tx_check(&tx_init);
    }

    // Power up, wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read accelerometer data once, discard.
    i2c_err = accel_read_data(&selftest_acceldata);
    
    if (i2c_err == ESP_OK)
    {
        // accel_align_data(&selftest_acceldata);
    }



    return 0;
}