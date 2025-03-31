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
#include <math.h>

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

// Board ID
enum boardID
{
    BID_SPECIAL = 0,
    BID_AXLE_1_R = 1,
    BID_SPARE_1 = 2,
    BID_AXLE_2_R = 3,
    BID_AXLE_1_L = 4,
    BID_SPARE_2 = 5,
    BID_AXLE_2_L = 6,
    BID_KING_PIN = 7
};
static enum boardID SOFA_DL_ID; // Given const CAPS because it is set once.

// Logging Tags
static char *CON_ACCEL_TAG = "SOFA_DL_CON";
static char *I2C_ACCEL_TAG = "SOFA_DL_I2C";
static char *CAN_ACCEL_TAG = "SOFA_DL_CAN";

// IDs
uint32_t CAN_ID_Accel = 0xFFFFFFFF;
uint32_t CAN_ID_Gyro = 0xFFFFFFFF;

// Tasks
#define TASK_PRIORITY_ACCELI2C      13
#define TASK_PRIORITY_CAN_RX        9
#define TASK_PRIORITY_CAN_TX        11
#define TASK_PRIORITY_CAN_CTRL      12

// Pins
#define I2C_MASTER_SDA_GPIO         1                          // GPIO number used for I2C master clock
#define I2C_MASTER_SCL_GPIO         2                          // GPIO number used for I2C master data
#define CAN_RX_GPIO                 14                         // GPIO number used for CAN Rx
#define CAN_TX_GPIO                 13                         // GPIO number used for CAN Tx
#define ID_JUMPER_1_GPIO            12                         // GPIO number used for ID Jumper 1
#define ID_JUMPER_2_GPIO            48                         // GPIO number used for ID Jumper 2
#define ID_JUMPER_3_GPIO            47                         // GPIO number used for ID Jumper 3

// Device Constants
static const int STARTUP_DELAY = 3;                             // Delay time at boot to line up with FMC650

// Queues and Semaphores
static QueueHandle_t CAN_rx_queue;
static QueueHandle_t CAN_tx_queue;
static QueueHandle_t CAN_control_queue;
static SemaphoreHandle_t SEM_Accel_Data;
static SemaphoreHandle_t SEM_CAN_Control;
static SemaphoreHandle_t SEM_Done;

typedef enum {
    CAN_INIT,
    CAN_CTRL_IDLE,
    CAN_TX_MSG,
    CAN_RX_RTR_LISTEN,
    CAN_RX_RTR_RECEIVED,
    CAN_RX_END,
    CAN_TX_END
} CAN_control_actions;

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

// Accelerometer data conversion - unused comment out
static const float ACCEL_SCALE_4 = 0.000122;    // +/-4 LSB value
static const float ACCEL_SCALE_8 = 0.000244;    // +/-8 LSB value
// static const float GYRO_SCALE_250 = 0.00875;    // +/- 2500 dps value
static const float GYRO_SCALE_1000 = 0.035;     // +/- 1000 dps value
static const float GYRO_SCALE_2000 = 0.070;     // +/- 2000 dps value

// Self test ranges - unused comment out
static const float A_ST_MIN = 0.050;
static const float A_ST_MAX = 1.700;
static const float G_ST_MIN_2000 = 150;
static const float G_ST_MAX_2000 = 700;

// Data ready
static const uint8_t XLDA = 0x01;
static const uint8_t GDA = 0x02;

/*
 * CAN BUS constants
 */
#define DATA_PERIOD_MS                  500
#define ITER_DELAY_MS                   1000

// #define CAN_ID_ACCEL_RTR                0x0300
// #define CAN_ID_ACCEL_DATA               0xFF0101FF
// #define CAN_ID_GYRO_DATA                0xFF0102FF
uint32_t CAN_ID_ACCEL_DATA = 0xFFFFFFFF;    // Variable to be setup as part of board ID
uint32_t CAN_ID_GYRO_DATA = 0xFFFFFFFF;     // Variable to be setup as part of board ID

#define CAN_ID_TEST                     0xFF0333FF

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
    uint8_t reg;
    size_t length;
    uint8_t g[6];
    int16_t g_X, g_Y, g_Z;
    float G_X, G_Y, G_Z;
    esp_err_t err;
} GyroData;

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

// CAN messages
static twai_message_t CAN_Data_Accelerometer;
static twai_message_t CAN_Data_Gyro;
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
    .data = {0x00, 0x00, 0x00, 0x00},   // This gets reversed in the FMC. 
};

/*  Function prototypes
*   Keeping everything in here and optimise code after it works
*
*/ 

// Configures jumper GPIOs and sets the board ID.
int IDInit();
// Sets the CAN IDs for messages based on board ID.
int setCANID(uint32_t id);
int setCANMessageID(twai_message_t *message, uint32_t id, uint8_t length);
int i2c_tx_check(i2c_1_reg_rw *data);
esp_err_t accel_read_data(AccelData *data);
esp_err_t gyro_read_data(GyroData *data);
int accel_align_data(AccelData *data, float scale);
int gyro_align_data(GyroData *data, float scale);
void accel_tx_whoami(void);
void accel_tx_config1(void);
uint8_t accel_tx_selftest(void);
uint8_t gyro_tx_selftest(void);

void taskAccelI2C(void *pvParameters)
{
    // Create the accel and gyro data structs
    AccelData acceldata = {
        .reg = 0x28,    // accel x axis reg, y follows 0x2A, z follows 0x2C
        .length = 6,    // read all accel data in 1 go
        .err = ESP_FAIL
    };

    GyroData gyrodata = {
        .reg = 0x22,    // gyro x axis reg, y follows 0x24, z follows 0x26
        .length = 6,    // read all gyro data in 1 go
        .err = ESP_FAIL
    };

    uint8_t boardStatus = 0;
    uint8_t ST_accel = 0;
    uint8_t ST_gyro = 0;

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

    // Perform self tests
    ST_accel = accel_tx_selftest();
    vTaskDelay(pdMS_TO_TICKS(100));
    ST_gyro = gyro_tx_selftest();
    vTaskDelay(pdMS_TO_TICKS(100));

    boardStatus = (((uint8_t)SOFA_DL_ID) << 4) | (ST_accel << 3) | (ST_gyro << 2);

    // Configure the accelerometer before reading data.
    accel_tx_config1();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Loop to read in accelerometer data
    while(1)
    {
        xSemaphoreTake(SEM_Accel_Data, portMAX_DELAY);

        // Accelerometer data
        i2c_err = accel_read_data(&acceldata);
       
        if (i2c_err == ESP_OK)
        {
            accel_align_data(&acceldata, ACCEL_SCALE_8);

            // DEBUG: Print out axis data in real number.
            ESP_LOGI(I2C_ACCEL_TAG, "ID: %d - ACCEL X/Y/Z axis reg %#04X %#04X %#06X | %#04X %#04X %#06X | %#04X %#04X %#06X = %.6f %.6f %.6f", SOFA_DL_ID, acceldata.a[0], acceldata.a[1], acceldata.a_X, acceldata.a[2], acceldata.a[3], acceldata.a_Y, acceldata.a[4], acceldata.a[5], acceldata.a_Z, acceldata.A_X, acceldata.A_Y, acceldata.A_Z);

            // Move into CAN message.
            CAN_Data_Accelerometer.data[0] = acceldata.a[0];
            CAN_Data_Accelerometer.data[1] = acceldata.a[1];
            CAN_Data_Accelerometer.data[2] = acceldata.a[2];
            CAN_Data_Accelerometer.data[3] = acceldata.a[3];
            CAN_Data_Accelerometer.data[4] = acceldata.a[4];
            CAN_Data_Accelerometer.data[5] = acceldata.a[5];
            CAN_Data_Accelerometer.data[6] = boardStatus;
        }

        // Gyro data
        i2c_err = gyro_read_data(&gyrodata);
       
        if (i2c_err == ESP_OK)
        {
            gyro_align_data(&gyrodata, GYRO_SCALE_1000);

            // DEBUG: Print out axis data in real number.
            // ESP_LOGI(I2C_ACCEL_TAG, "ID: %d - GYRO X/Y/Z axis reg %#04X %#04X %#06X | %#04X %#04X %#06X | %#04X %#04X %#06X = %.6f %.6f %.6f", SOFA_DL_ID, gyrodata.g[0], gyrodata.g[1], gyrodata.g_X, gyrodata.g[2], gyrodata.g[3], gyrodata.g_Y, gyrodata.g[4], gyrodata.g[5], gyrodata.g_Z, gyrodata.G_X, gyrodata.G_Y, gyrodata.G_Z);

            // Move into CAN message.
            CAN_Data_Gyro.data[0] = gyrodata.g[0];
            CAN_Data_Gyro.data[1] = gyrodata.g[1];
            CAN_Data_Gyro.data[2] = gyrodata.g[2];
            CAN_Data_Gyro.data[3] = gyrodata.g[3];
            CAN_Data_Gyro.data[4] = gyrodata.g[4];
            CAN_Data_Gyro.data[5] = gyrodata.g[5];
            CAN_Data_Gyro.data[6] = boardStatus;

        }

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
    ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task started.");

    while (1)
    {
        // Read the queue to see if a receive action is required.
        CAN_control_actions actions;
        xQueueReceive(CAN_rx_queue, &actions, portMAX_DELAY);

        // Check what action was required.
        if (actions == CAN_RX_RTR_LISTEN)
        {
            // This is template code for if receiving is required.
            // FMC will only listen for messages from SOFA DL.
            xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
            ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task received action CAN_RX_RTR %d.", actions);
            // Give the semaphore back if it has it for some reason.
            // When this task is needed, this will need to be updated.
            xSemaphoreGive(SEM_CAN_Control);

            twai_message_t received_msg;
            
            while (1)
            {
                // Waiting to receive request
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS receive task waiting to receive request for data.");
                // esp_err_t err_rxrtr = twai_receive(&received_msg, portMAX_DELAY);
            }
        }
    }

    xSemaphoreGive(SEM_CAN_Control);
    vTaskDelete(NULL);
}

void taskCANTx(void *pvParameters)
{
    ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task started.");

    while (1)
    {
        // DEBUG counter
        twai_transmit(&CAN_TEST, portMAX_DELAY);
        ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task sent CAN_TEST, %d", CAN_TEST.data[0]);
        CAN_TEST.data[0] = CAN_TEST.data[0] + 1;

        // Read the queue to see if a transmit action is required.
        CAN_control_actions actions;
        xQueueReceive(CAN_tx_queue, &actions, portMAX_DELAY);

        // Check what action was required.
        if (actions == CAN_TX_MSG)
        {
            // Take control and send the data
            xSemaphoreTake(SEM_CAN_Control, portMAX_DELAY);
            ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task received action CAN_TX_MSG %d.", actions);

            twai_transmit(&CAN_Data_Accelerometer, portMAX_DELAY);
            twai_transmit(&CAN_Data_Gyro, portMAX_DELAY);

            ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task sent accelerometer and gyro data.");

            xSemaphoreGive(SEM_CAN_Control);
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    CAN_control_actions actions = CAN_INIT;

    while(1)
    {
        // Run through the CAN control state machine        
        switch (actions)
        {
            case CAN_INIT:
                // CAN started, perform first actions
                actions = CAN_CTRL_IDLE;

                // Move to IDLE
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to CAN_CTRL_IDLE %d.", actions);
                
                break;

            case CAN_CTRL_IDLE:
                // Idle goes straight to TX
                actions = CAN_TX_MSG;

                // Move to TX_MSG
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to CAN_TX_MSG %d.", actions);
                
                break;
            
            case CAN_TX_MSG:
                // Send the data
                xQueueSend(CAN_tx_queue, &actions, portMAX_DELAY);
                xSemaphoreGive(SEM_CAN_Control);
                break;

            case CAN_RX_END:
                // Never get here
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to CAN_RX_END %d.", actions);
                break;

            case CAN_TX_END:
                // Never get here
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to CAN_TX_END %d.", actions);
                break;

            default:
                // Never get here
                ESP_LOGI(CAN_ACCEL_TAG, "CANBUS control task set action to FAULT %d.", actions);
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
    for (int i = STARTUP_DELAY; i > 0; i--)
    {
        ESP_LOGI(CON_ACCEL_TAG, "Device starting in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Configure the GPIOs for ID
    // Read and set this board's ID
    IDInit();
    setCANID((uint32_t)SOFA_DL_ID);
    // Only call after setCANID()
    setCANMessageID(&CAN_Data_Accelerometer, CAN_ID_Accel, 7);
    setCANMessageID(&CAN_Data_Gyro, CAN_ID_Gyro, 7);
    
    //Install TWAI driver, trigger tasks to start
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver installed");
    
    // Start the TWAI driver
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver started.");

    // Semaphores and queues
    CAN_rx_queue = xQueueCreate(1, sizeof(CAN_control_actions));
    CAN_tx_queue = xQueueCreate(1, sizeof(CAN_control_actions));
    CAN_control_queue = xQueueCreate(1, sizeof(CAN_control_actions));

    SEM_Accel_Data = xSemaphoreCreateBinary();
    SEM_CAN_Control = xSemaphoreCreateBinary();
    SEM_Done = xSemaphoreCreateBinary();
    
    // Tasks
    xTaskCreatePinnedToCore(taskAccelI2C, "Accel_Data_I2C", 4096, NULL, TASK_PRIORITY_ACCELI2C, NULL, 1); // on its own core
    xTaskCreatePinnedToCore(taskCANRx, "CANBus_Rx", 4096, NULL, TASK_PRIORITY_CAN_RX, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskCANTx, "CANBus_Tx", 4096, NULL, TASK_PRIORITY_CAN_TX, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskCANCtrl, "CANBus_Ctrl", 4096, NULL, TASK_PRIORITY_CAN_CTRL, NULL, tskNO_AFFINITY);

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
    
    // Don't start the scheduler - it starts automatically.
    // vTaskStartScheduler();
    
}

int IDInit()
{    
    // Create and set the GPIO config structure for the ID jumper inputs
    gpio_config_t input_config = {};
    input_config.intr_type = GPIO_INTR_DISABLE;
    input_config.mode = GPIO_MODE_INPUT;
    input_config.pin_bit_mask = (1ULL << ID_JUMPER_1_GPIO) | (1ULL << ID_JUMPER_2_GPIO) | (1ULL << ID_JUMPER_3_GPIO);
    input_config.pull_down_en = 0;
    input_config.pull_up_en = 1;
    gpio_config(&input_config);

    // Delay before read
    vTaskDelay(pdMS_TO_TICKS(1000));

    int jumper = 0;
    // Read the jumper pins and add to find the ID.
    // As it's pull up inverted with ! so it aligns with jumper sets that bit.
    int jumper1 = !gpio_get_level(ID_JUMPER_1_GPIO);
    int jumper2 = !gpio_get_level(ID_JUMPER_2_GPIO);
    int jumper3 = !gpio_get_level(ID_JUMPER_3_GPIO);
    // Weighted binary from jumpers.
    jumper = (4 * jumper1) + (2 * jumper2) + (1 * jumper3);

    // Based on the jumpers set, set the board ID.
    switch(jumper)
    {
        case 0:
            // SPECIAL
            SOFA_DL_ID = BID_SPECIAL;
            break;

        case 1:
            // Axle 1 Right
            SOFA_DL_ID = BID_AXLE_1_R;
            break;
        
        case 2:
            // SPARE 1
            SOFA_DL_ID = BID_SPARE_1;
            break;
        
        case 3:
            // Axle 2 Right
            SOFA_DL_ID = BID_AXLE_2_R;
            break;

        case 4:
            // Axle 1 Left
            SOFA_DL_ID = BID_AXLE_1_L;
            break;

        case 5:
            // SPARE 2
            SOFA_DL_ID = BID_SPARE_2;
            break;

        case 6:
            // Axle 2 Left
            SOFA_DL_ID = BID_AXLE_2_L;
            break;

        case 7:
            // Kingpin
            SOFA_DL_ID = BID_KING_PIN;
            break;
        
        default:
            // error
            SOFA_DL_ID = BID_SPARE_1;
            break;
    }

    // DEBUG
    ESP_LOGI(CON_ACCEL_TAG, "ID: %d - ID jumper set %d%d%d %d.", SOFA_DL_ID, jumper1, jumper2, jumper3, jumper);

    // TODO: return status
    return 0;
}

int setCANID(uint32_t id)
{
    // Standard message ID length, stored in uint32_t as 0xFF0isnFF, where
    // i = ID
    // s = SET (not used)
    // n = NUM, 1 accel, 2 gyro
    CAN_ID_Accel = (id << 16) | 0xFF0011FF;
    CAN_ID_Gyro = (id << 16) | 0xFF0012FF;

    return 0;
}

int setCANMessageID(twai_message_t *message, uint32_t id, uint8_t length)
{
    // Message type and format settings
    //     .extd = 0,              // Standard Format message (11-bit ID)
    //     .rtr = 0,               // Send a data frame
    //     .ss = 0,                // Not single shot
    //     .self = 0,              // Not a self reception request
    //     .dlc_non_comp = 0,      // DLC is less than 8
    //     // Message ID and payload
    //     .identifier = 0,
    //     .data_length_code = 7,
    //     .data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00},

    message->extd = 0;
    message->rtr = 0;
    message->ss = 0;
    message->self = 0;
    message->dlc_non_comp = 0;
    message->identifier = id;
    message->data_length_code = length;

    ESP_LOGI(CAN_ACCEL_TAG, "CAN message created - ID: %#08lX, DLC: %d, EXTD: %d, RTR: %d, SS: %d, SELF: %d, DLC_NON_COMP: %d.", message->identifier, message->data_length_code, message->extd, message->rtr, message->ss, message->self, message->dlc_non_comp);

    return 0;
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

    i2c_1_reg_r tx_accel_data_ready = {
        .tx = 0x1E,     // STATUS_REG register
        .rx = 0x00,     // No expectation
        .success = 0x00,
        .err = ESP_FAIL
    };  

    // Check accel data is available.
    uint8_t accel_data_avail = 0x00;

    while (accel_data_avail != XLDA)
    {
        tx_accel_data_ready.err = i2c_master_transmit_receive(i2c_accel_handle, &tx_accel_data_ready.tx, sizeof(tx_accel_data_ready.tx), &tx_accel_data_ready.rx, 1, -1);
        accel_data_avail = tx_accel_data_ready.rx & XLDA;
    }
    
    // New accel data is available, read it.
    accel_i2c_err = i2c_master_transmit_receive(i2c_accel_handle, &data->reg, sizeof(data->reg), data->a, data->length, -1);

    return accel_i2c_err;
}

esp_err_t gyro_read_data(GyroData *data)
{
    esp_err_t gyro_i2c_err;

    i2c_1_reg_r tx_gyro_data_ready = {
        .tx = 0x1E,     // STATUS_REG register
        .rx = 0x00,     // No expectation
        .success = 0x00,
        .err = ESP_FAIL
    };  

    // Check accel data is available.
    uint8_t gyro_data_avail = 0x00;

    while (gyro_data_avail != GDA)
    {
        tx_gyro_data_ready.err = i2c_master_transmit_receive(i2c_accel_handle, &tx_gyro_data_ready.tx, sizeof(tx_gyro_data_ready.tx), &tx_gyro_data_ready.rx, 1, -1);
        gyro_data_avail = tx_gyro_data_ready.rx & GDA;
    }
    
    // New accel data is available, read it.
    gyro_i2c_err = i2c_master_transmit_receive(i2c_accel_handle, &data->reg, sizeof(data->reg), data->g, data->length, -1);

    return gyro_i2c_err;
}

int accel_align_data(AccelData *data, float scale)
{
    data->a_X = ((int16_t)data->a[1] << 8) + data->a[0];
    data->A_X = ((float)data->a_X) * scale;

    data->a_Y = ((int16_t)data->a[3] << 8) + data->a[2];
    data->A_Y = ((float)data->a_Y) * scale;

    data->a_Z = ((int16_t)data->a[5] << 8) + data->a[4];
    data->A_Z = ((float)data->a_Z) * scale;

    return 0;
}

int gyro_align_data(GyroData *data, float scale)
{
    data->g_X = ((int16_t)data->g[1] << 8) + data->g[0];
    data->G_X = ((float)data->g_X) * scale;

    data->g_Y = ((int16_t)data->g[3] << 8) + data->g[2];
    data->G_Y = ((float)data->g_Y) * scale;

    data->g_Z = ((int16_t)data->g[5] << 8) + data->g[4];
    data->G_Z = ((float)data->g_Z) * scale;

    return 0;
}

void accel_tx_whoami(void)
{
    // Create the data for this read.
    // TX1 - Who Am I
    i2c_1_reg_r tx_whoami = {
        .tx = 0x0F,     // Who Am I register
        .rx = 0x00,     // Expecting 0x6C
        .success = 0x6C,
        .err = ESP_FAIL
    };  

    // Read the WHO_AM_I (0x0F) register to see if we can see the accelerometer.
    // WHO_AM_I returns 0x6C.
    while (tx_whoami.rx != 0x6C)
    {
        tx_whoami.err = i2c_master_transmit_receive(i2c_accel_handle, &tx_whoami.tx, sizeof(tx_whoami.tx), &tx_whoami.rx, 1, -1);
        ESP_LOGI(I2C_ACCEL_TAG, "Connecting to accelerometer on I2C bus at address 0x6A.");
    }

    // Log that we saw the accelerometer
    ESP_LOGI(I2C_ACCEL_TAG, "I2C initialized successfully. WHO AM I = 0x%X", tx_whoami.rx);
    
    // Reset in case we need to use it again
    tx_whoami.rx = 0x00;
}

void accel_tx_config1(void)
{
    // Configure the accelerometer before reading data.

    // Config 1
    // CTRL1_XL 0x10 Accelerometer control register 1 (R/W)
    // 104Hz high-performance mode, +/- 8g. 
    i2c_1_reg_rw tx_config1 = {
        .tx = {0x10, 0x4C},
        .rx = 0x00,
        .err = ESP_FAIL
    };
    
    i2c_tx_check(&tx_config1);

    // Config 2
    // CTRL2_G 0x11 Gyroscope control register 2 (R/W)
    // 104Hz high-performance mode, +/- 1000dps
    i2c_1_reg_rw tx_config2 = {
        .tx = {0x11, 0x48},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    i2c_tx_check(&tx_config2);
}

uint8_t accel_tx_selftest(void)
{
    // Accelerometer UI self-test mode 1
    // Datasheet section 11.
    ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer self-test started.");
    int result = 0;

    esp_err_t i2c_err;

    AccelData selftest_acceldata = {
        .reg = 0x28,    // accel x axis reg, y follows 0x2A, z follows 0x2C
        .length = 6,    // read all accel data in 1 go
        .err = ESP_FAIL
    };

    float OUTX_NOST = 0.0f;
    float OUTY_NOST = 0.0f;
    float OUTZ_NOST = 0.0f;
    float OUTX_ST = 0.0f;
    float OUTY_ST = 0.0f;
    float OUTZ_ST = 0.0f;

    // Initialise and turn on sensor
    i2c_1_reg_rw tx_st_1 = {
        .tx = {0x10, 0x38},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    for (int i = 0; i < 10; i++)
    {
        tx_st_1.tx[0] = 0x10 + i;
        if (tx_st_1.tx[0] == 0x10)
        {
            tx_st_1.tx[1] = 0x38;
        }
        else if (tx_st_1.tx[0] == 0x12)
        {
            tx_st_1.tx[1] = 0x44;
        }
        else
        {
            tx_st_1.tx[1] = 0x00;
        }
        
        i2c_tx_check(&tx_st_1);
    }

    // Power up, wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read accelerometer data once, discard.
    i2c_err = accel_read_data(&selftest_acceldata);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        i2c_err = accel_read_data(&selftest_acceldata);
        accel_align_data(&selftest_acceldata, ACCEL_SCALE_4);
        // Compute the average of each axis and store in _NOST.
        // Each read add to _NOST, divide by 5 at end.
        OUTX_NOST = OUTX_NOST + selftest_acceldata.A_X;
        OUTY_NOST = OUTY_NOST + selftest_acceldata.A_Y;
        OUTZ_NOST = OUTZ_NOST + selftest_acceldata.A_Z;

        // DEBUG
        // ESP_LOGI(I2C_ACCEL_TAG, "_NOST %d: %.6f \t %.6f \t %.6f", i, selftest_acceldata.A_X, selftest_acceldata.A_Y, selftest_acceldata.A_Z);
        ESP_LOGI(I2C_ACCEL_TAG, "_NOST %d: %.6f \t %.6f \t %.6f", i, OUTX_NOST, OUTY_NOST, OUTZ_NOST);
    }
    // Compute average
    OUTX_NOST = OUTX_NOST / 5;
    OUTY_NOST = OUTY_NOST / 5;
    OUTZ_NOST = OUTZ_NOST / 5;

    // DEBUG
    ESP_LOGI(I2C_ACCEL_TAG, "_NOST AVG: %.6f \t %.6f \t %.6f", OUTX_NOST, OUTY_NOST, OUTZ_NOST);

    // Enable accelerometer self-test
    i2c_1_reg_rw tx_st_2 = {
        .tx = {0x14, 0x01},
        .rx = 0x00,
        .err = ESP_FAIL
    };
    i2c_tx_check(&tx_st_2);

    // Wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read accelerometer data once, discard.
    i2c_err = accel_read_data(&selftest_acceldata);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        i2c_err = accel_read_data(&selftest_acceldata);
        accel_align_data(&selftest_acceldata, ACCEL_SCALE_4);
        // Compute the average of each axis and store in _ST.
        // Each read add to _ST, divide by 5 at end.
        OUTX_ST = OUTX_ST + selftest_acceldata.A_X;
        OUTY_ST = OUTY_ST + selftest_acceldata.A_Y;
        OUTZ_ST = OUTZ_ST + selftest_acceldata.A_Z;

        // DEBUG 
        // ESP_LOGI(I2C_ACCEL_TAG, "_ST %d: %.6f \t %.6f \t %.6f", i, selftest_acceldata.A_X, selftest_acceldata.A_Y, selftest_acceldata.A_Z);
        ESP_LOGI(I2C_ACCEL_TAG, "_ST %d: %.6f \t %.6f \t %.6f", i, OUTX_ST, OUTY_ST, OUTZ_ST);
    }
    // Compute average
    OUTX_ST = OUTX_ST / 5;
    OUTY_ST = OUTY_ST / 5;
    OUTZ_ST = OUTZ_ST / 5;

    // DEBUG
    ESP_LOGI(I2C_ACCEL_TAG, "_ST AVG: %.6f \t %.6f \t %.6f", OUTX_ST, OUTY_ST, OUTZ_ST);

    // Check that each axis is within the self-test range
    float range_ST_X = fabsf(OUTX_ST - OUTX_NOST);
    float range_ST_Y = fabsf(OUTY_ST - OUTY_NOST);
    float range_ST_Z = fabsf(OUTZ_ST - OUTZ_NOST);

    // I'm cheating here because I know the min and max are declared as positive, so leave off the abs.
    // X-axis in range
    if ((range_ST_X >= A_ST_MIN) && (range_ST_X <= A_ST_MAX))
    {
        // Y-axis in range
        if ((range_ST_Y >= A_ST_MIN) && (range_ST_Y <= A_ST_MAX))
        {
            // Z-axis in range
            if ((range_ST_Z >= A_ST_MIN) && (range_ST_Z <= A_ST_MAX))
            {
                result = 1;
            }
        }
    }

    if (result)
    {
        ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer self-test PASSED.");
    }
    else
    {
        ESP_LOGI(I2C_ACCEL_TAG, "Accelerometer self-test FAILED.");
    }

    // Disable self-test
    tx_st_2.tx[0] = 0x14;
    tx_st_2.tx[1] = 0x00;
    i2c_tx_check(&tx_st_2);

    // Disable sensor, ready for config.
    tx_st_1.tx[0] = 0x10;
    tx_st_1.tx[1] = 0x00;
    i2c_tx_check(&tx_st_1);

    return result;
}

uint8_t gyro_tx_selftest(void)
{
    // Gyroscope UI self-test mode 1
    // Datasheet section 11.
    ESP_LOGI(I2C_ACCEL_TAG, "Gyroscope self-test started.");
    int result = 0;

    esp_err_t i2c_err;

    GyroData selftest_gyrodata = {
        .reg = 0x22,    // gyro x axis reg, y follows 0x24, z follows 0x26
        .length = 6,    // read all gyro data in 1 go
        .err = ESP_FAIL
    };

    float OUTX_NOST = 0.0f;
    float OUTY_NOST = 0.0f;
    float OUTZ_NOST = 0.0f;
    float OUTX_ST = 0.0f;
    float OUTY_ST = 0.0f;
    float OUTZ_ST = 0.0f;

    // Initialise and turn on sensor
    i2c_1_reg_rw tx_st_1 = {
        .tx = {0x10, 0x00},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    for (int i = 0; i < 10; i++)
    {
        tx_st_1.tx[0] = 0x10 + i;
        if (tx_st_1.tx[0] == 0x11)
        {
            tx_st_1.tx[1] = 0x5C;
        }
        else if (tx_st_1.tx[0] == 0x12)
        {
            tx_st_1.tx[1] = 0x44;
        }
        else
        {
            tx_st_1.tx[1] = 0x00;
        }
        
        i2c_tx_check(&tx_st_1);
    }

    // Power up, wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read gyro data once, discard.
    i2c_err = gyro_read_data(&selftest_gyrodata);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        i2c_err = gyro_read_data(&selftest_gyrodata);
        gyro_align_data(&selftest_gyrodata, GYRO_SCALE_2000);
        // Compute the average of each axis and store in _NOST.
        // Each read add to _NOST, divide by 5 at end.
        OUTX_NOST = OUTX_NOST + selftest_gyrodata.G_X;
        OUTY_NOST = OUTY_NOST + selftest_gyrodata.G_Y;
        OUTZ_NOST = OUTZ_NOST + selftest_gyrodata.G_Z;

        // DEBUG
        // ESP_LOGI(I2C_ACCEL_TAG, "_NOST %d: %.6f \t %.6f \t %.6f", i, selftest_gyrodata.G_X, selftest_gyrodata.G_Y, selftest_gyrodata.G_Z);
        ESP_LOGI(I2C_ACCEL_TAG, "_NOST %d: %.6f \t %.6f \t %.6f", i, OUTX_NOST, OUTY_NOST, OUTZ_NOST);
    }
    // Compute average
    OUTX_NOST = OUTX_NOST / 5;
    OUTY_NOST = OUTY_NOST / 5;
    OUTZ_NOST = OUTZ_NOST / 5;

    // DEBUG
    ESP_LOGI(I2C_ACCEL_TAG, "_NOST AVG: %.6f \t %.6f \t %.6f", OUTX_NOST, OUTY_NOST, OUTZ_NOST);

    // Enable gyro self-test
    i2c_1_reg_rw tx_st_2 = {
        .tx = {0x14, 0x04},
        .rx = 0x00,
        .err = ESP_FAIL
    };
    i2c_tx_check(&tx_st_2);

    // Wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read accelerometer data once, discard.
    i2c_err = gyro_read_data(&selftest_gyrodata);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        i2c_err = gyro_read_data(&selftest_gyrodata);
        gyro_align_data(&selftest_gyrodata, GYRO_SCALE_2000);
        // Compute the average of each axis and store in _ST.
        // Each read add to _ST, divide by 5 at end.
        OUTX_ST = OUTX_ST + selftest_gyrodata.G_X;
        OUTY_ST = OUTY_ST + selftest_gyrodata.G_Y;
        OUTZ_ST = OUTZ_ST + selftest_gyrodata.G_Z;

        // DEBUG 
        // ESP_LOGI(I2C_ACCEL_TAG, "_ST %d: %.6f \t %.6f \t %.6f", i, selftest_gyrodata.G_X, selftest_gyrodata.G_Y, selftest_gyrodata.G_Z);
        ESP_LOGI(I2C_ACCEL_TAG, "_ST %d: %.6f \t %.6f \t %.6f", i, OUTX_ST, OUTY_ST, OUTZ_ST);
    }
    // Compute average
    OUTX_ST = OUTX_ST / 5;
    OUTY_ST = OUTY_ST / 5;
    OUTZ_ST = OUTZ_ST / 5;

    // DEBUG
    ESP_LOGI(I2C_ACCEL_TAG, "_ST AVG: %.6f \t %.6f \t %.6f", OUTX_ST, OUTY_ST, OUTZ_ST);

    // Check that each axis is within the self-test range
    float range_ST_X = fabsf(OUTX_ST - OUTX_NOST);
    float range_ST_Y = fabsf(OUTY_ST - OUTY_NOST);
    float range_ST_Z = fabsf(OUTZ_ST - OUTZ_NOST);

    // I'm cheating here because I know the min and max are declared as positive, so leave off the abs.
    // X-axis in range
    if ((range_ST_X >= G_ST_MIN_2000) && (range_ST_X <= G_ST_MAX_2000))
    {
        // Y-axis in range
        if ((range_ST_Y >= G_ST_MIN_2000) && (range_ST_Y <= G_ST_MAX_2000))
        {
            // Z-axis in range
            if ((range_ST_Z >= G_ST_MIN_2000) && (range_ST_Z <= G_ST_MAX_2000))
            {
                result = 1;
            }
        }
    }

    if (result)
    {
        ESP_LOGI(I2C_ACCEL_TAG, "Gyroscope self-test PASSED.");
    }
    else
    {
        ESP_LOGI(I2C_ACCEL_TAG, "Gyroscope self-test FAILED.");
    }

    // Disable self-test
    tx_st_2.tx[0] = 0x14;
    tx_st_2.tx[1] = 0x00;
    i2c_tx_check(&tx_st_2);

    // Disable sensor, ready for config.
    tx_st_1.tx[0] = 0x11;
    tx_st_1.tx[1] = 0x00;
    i2c_tx_check(&tx_st_1);

    return result;
}