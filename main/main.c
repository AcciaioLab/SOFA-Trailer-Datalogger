/*  
 * SOFA TRAILER DATALOGGER
 * 
 * Connects and reads accelerometer data from LSM6DSOX via I2C.
 * Sends the accelerometer and gyroscope data to the FMC650 via CANBus.
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

#include "esp_timer.h"

#include "include/i2c.h"
#include "include/sofa.h"

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
static char *SOFA_DL_SYS = "SOFA_DL_SYS";
static char *SOFA_DL_I2C = "SOFA_DL_I2C";
static char *SOFA_DL_IMU = "SOFA_DL_IMU";
static char *CAN_ACCEL_TAG = "SOFA_DL_CAN";

// Board IDs
uint32_t CAN_ID_Accel = 0xFFFFFFFF;
uint32_t CAN_ID_Gyro = 0xFFFFFFFF;

// Tasks
#define TASK_PRIORITY_I2C 5
#define TASK_PRIORITY_IMU 13
#define TASK_PRIORITY_CAN_RX 9
#define TASK_PRIORITY_CAN_TX 11
#define TASK_PRIORITY_CAN_CTRL 12

#define ESP_INTR_FLAG_DEFAULT 0

// Pins
#define CAN_RX_GPIO 14 // GPIO used for CAN Rx
#define CAN_TX_GPIO 13 // GPIO used for CAN Tx
#define GPIO_ID_JMPR_1 12 // GPIO used for ID Jumper 1
#define GPIO_ID_JMPR_2 48 // GPIO used for ID Jumper 2
#define GPIO_ID_JMPR_3 47 // GPIO used for ID Jumper 3
#define GPIO_IMU_XL_INT 6 // GPIO used for IMU interrupt for XL data

// Device Constants
static const int STARTUP_DELAY = 3; // Delay time at boot to line up with FMC650

// Queues and Semaphores
static QueueHandle_t queueIntXL;
static QueueHandle_t queueIMUReadData;

static QueueHandle_t CAN_rx_queue;
static QueueHandle_t CAN_tx_queue;
static QueueHandle_t CAN_control_queue;

static SemaphoreHandle_t semI2C;
static SemaphoreHandle_t SEM_Accel_Data;
static SemaphoreHandle_t SEM_CAN_Control;
static SemaphoreHandle_t SEM_Done;

// Task control enum
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

/*  
*   Function prototypes
*/ 

int configureGPIO(void);
int IDInit(void);
int twaiDriverStart(void);
int twaiDriverEnd(void);
int setCANID(uint32_t id);
int setCANMessageID(twai_message_t *message, uint32_t id, uint8_t length);

static void IRAM_ATTR intHandler(void *arg)
{
    uint32_t gpioNum = (uint32_t) arg;
    xQueueSendFromISR(queueIntXL, &gpioNum, NULL);
}

void taskI2C(void *pvParameters)
{
    // Log the accelerometer task has started.
    ESP_LOGI(SOFA_DL_I2C, "I2C task created.");

    // Pin interrupt was received on
    uint32_t gpioNum;

    // Create the accel and gyro data structs
    IMUReadDataFrame imuReadDataFrame = {
        .xl = {
            .reg = 0x28,    // accel x axis reg, y follows 0x2A, z follows 0x2C
            .length = 6,    // read all accel data in 1 go
            .err = ESP_FAIL
        },
        .gyro = {
            .reg = 0x22,    // gyro x axis reg, y follows 0x24, z follows 0x26
            .length = 6,    // read all gyro data in 1 go
            .err = ESP_FAIL
        }
    };

    // DEBUG - timing
    // uint64_t tStart = esp_timer_get_time();

    while (1)
    {
        if (xQueueReceive(queueIntXL, &gpioNum, portMAX_DELAY))
        {
            // DEBUG - timing
            // uint64_t tEnd = esp_timer_get_time();
            // ESP_LOGI(SOFA_DL_IMU, "Time between interrupt call %.3f ms", ((float)(tEnd-tStart)/1000));
            // tStart = tEnd;
            
            // ESP_LOGI(SOFA_DL_I2C, "INTERRUPT RECEIVED on PIN %lu.....", gpioNum);

            imuReadAData(&imuReadDataFrame.xl, false);
            imuReadGData(&imuReadDataFrame.gyro, true);

            xQueueSend(queueIMUReadData, &imuReadDataFrame, portMAX_DELAY);
        }
    }

    // End i2c
    i2cBusEnd();

    // Log if it ever gets here
    ESP_LOGI(SOFA_DL_I2C, "I2C removed successfully.");

    vTaskDelete(NULL);
    
}

void taskIMU(void *pvParameters)
{
    // Variables for board status reg
    uint8_t boardStatus = 0;
    uint8_t stResultXL = 0;
    uint8_t stResultGyro = 0;

    // Log the accelerometer task has started.
    ESP_LOGI(SOFA_DL_IMU, "IMU task created.");

    // Start i2c
    i2cBusStart();

    // Read the WHO_AM_I (0x0F) register to see if we can see the accelerometer.
    imuWhoAmI();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Disable the IMU interrupt at the start.
    // imuDisableInt();

    // Perform self tests
    stResultXL = imuSelfTestA();
    vTaskDelay(pdMS_TO_TICKS(100));
    stResultGyro = imuSelfTestG();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set the board status based on the self test result.
    boardStatus = (((uint8_t)SOFA_DL_ID) << 4) | (stResultXL << 3) | (stResultGyro << 2);

    // Configure the accelerometer before reading data.
    imuConfig();
    vTaskDelay(pdMS_TO_TICKS(100));

    IMUReadDataFrame bufRead;
    uint16_t bufCount = 0;
    IMUData bufIMURawData[IMU_SAMPLE_PER_SEC];

    // Loop to read in accelerometer data
    // DEBUG - timing
    uint64_t tOverallStart = esp_timer_get_time();
    while(1)
    {
        // ESP_LOGI(SOFA_DL_IMU, "IN IMU TASK...");
        
        // 1 - Receive and format the raw data
        if (xQueueReceive(queueIMUReadData, &bufRead, portMAX_DELAY))
        {
            // DEBUG - timing
            // uint64_t tStart = esp_timer_get_time();

            // ESP_LOGI(SOFA_DL_IMU, "Item %d: XL measurement data received from queue.", bufCount);
            imuFormatData(bufRead.xl.m, &bufIMURawData[bufCount].xl, ACCEL_SCALE_8);
            imuFormatData(bufRead.gyro.m, &bufIMURawData[bufCount].gyro, GYRO_SCALE_1000);
            bufCount++;
            if (bufCount > (IMU_SAMPLE_PER_SEC - 1))
            {
                bufCount = 0;
            }

            // DEBUG - timing
            // uint64_t tEnd = esp_timer_get_time();
            // ESP_LOGI(SOFA_DL_IMU, "Format raw data XL/Gyro took %.3f ms", ((float)(tEnd-tStart)/1000));
        }

        // When at the end of the samples / sec, calc and send
        if (bufCount == (IMU_SAMPLE_PER_SEC - 1))
        {
            ESP_LOGI(SOFA_DL_IMU, "IMU data sample buffer full, calc and send.");

            // 2 - Raw data through Madgwick filter

            // 3 - Remove gravity from acceleration data.

            // 4 - More filtering (OPTIONAL at the moment)
            // Nothing now

            // 5 - Mean of the values
            IMUSendData imuSend; 
            imuMeanData(bufIMURawData, &imuSend);

            // 6 - Send to FMC, package CAN data

            // DEBUG - timing
            uint64_t tOverallEnd = esp_timer_get_time();
            ESP_LOGI(SOFA_DL_IMU, "IMU data cycle took %.3f ms", ((float)(tOverallEnd-tOverallStart)/1000));
            tOverallStart = tOverallEnd;
        }

        /* This is to be updated later
        // Move into CAN message.
        CAN_Data_Accelerometer.data[0] = acceldata.m[0];
        CAN_Data_Accelerometer.data[1] = acceldata.m[1];
        CAN_Data_Accelerometer.data[2] = acceldata.m[2];
        CAN_Data_Accelerometer.data[3] = acceldata.m[3];
        CAN_Data_Accelerometer.data[4] = acceldata.m[4];
        CAN_Data_Accelerometer.data[5] = acceldata.m[5];
        CAN_Data_Accelerometer.data[6] = boardStatus;
        }

        // Move into CAN message.
        CAN_Data_Gyro.data[0] = gyrodata.m[0];
        CAN_Data_Gyro.data[1] = gyrodata.m[1];
        CAN_Data_Gyro.data[2] = gyrodata.m[2];
        CAN_Data_Gyro.data[3] = gyrodata.m[3];
        CAN_Data_Gyro.data[4] = gyrodata.m[4];
        CAN_Data_Gyro.data[5] = gyrodata.m[5];
        CAN_Data_Gyro.data[6] = boardStatus;
        */

    }

    vTaskDelay(pdMS_TO_TICKS(1000));

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

            // twai_message_t received_msg;

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
        ESP_LOGI(CAN_ACCEL_TAG, "CANBUS transmit task sent CAN_TEST, %u", (CAN_TEST.data[1]+CAN_TEST.data[0]));
        if (CAN_TEST.data[0] == 0xFF)
        {
            CAN_TEST.data[1] = CAN_TEST.data[1] + 1;
        }
        CAN_TEST.data[0] = CAN_TEST.data[0] + 1;
        // CAN_TEST.data[0] = 0xDE;
        // CAN_TEST.data[1] = 0xAD;
        // CAN_TEST.data[2] = 0xBE;
        // CAN_TEST.data[3] = 0xEF;

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
        ESP_LOGI(SOFA_DL_SYS, "Device starting in %d", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    // Configure the GPIOs
    configureGPIO();

    // Read and set this board's ID
    IDInit();
    setCANID((uint32_t)SOFA_DL_ID);
    
    // Only call after setCANID()
    setCANMessageID(&CAN_Data_Accelerometer, CAN_ID_Accel, 7);
    setCANMessageID(&CAN_Data_Gyro, CAN_ID_Gyro, 7);
    
    // Start the TWAI driver
    twaiDriverStart();

    // Create queues
    queueIntXL = xQueueCreate(1, sizeof(uint32_t));
    queueIMUReadData = xQueueCreate(1, sizeof(IMUReadDataFrame));

    CAN_rx_queue = xQueueCreate(1, sizeof(CAN_control_actions));
    CAN_tx_queue = xQueueCreate(1, sizeof(CAN_control_actions));
    CAN_control_queue = xQueueCreate(1, sizeof(CAN_control_actions));

    // Create semaphores
    semI2C = xSemaphoreCreateBinary();
    SEM_Accel_Data = xSemaphoreCreateBinary();
    SEM_CAN_Control = xSemaphoreCreateBinary();
    SEM_Done = xSemaphoreCreateBinary();
    
    // Tasks
    xTaskCreatePinnedToCore(taskI2C, "Task_I2C", 4096, NULL, TASK_PRIORITY_I2C, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskIMU, "Task_IMU", 9999, NULL, TASK_PRIORITY_IMU, NULL, tskNO_AFFINITY); // on its own core
    // xTaskCreatePinnedToCore(taskCANRx, "CANBus_Rx", 4096, NULL, TASK_PRIORITY_CAN_RX, NULL, tskNO_AFFINITY);
    // xTaskCreatePinnedToCore(taskCANTx, "CANBus_Tx", 4096, NULL, TASK_PRIORITY_CAN_TX, NULL, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskCANCtrl, "CANBus_Ctrl", 4096, NULL, TASK_PRIORITY_CAN_CTRL, NULL, tskNO_AFFINITY);

    // Install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // Hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_IMU_XL_INT, intHandler, (void*) GPIO_IMU_XL_INT);

    ESP_LOGI(SOFA_DL_SYS, "Minimum free heap size: %"PRIu32" bytes.", esp_get_minimum_free_heap_size());

    xSemaphoreGive(semI2C);
    xSemaphoreGive(SEM_Accel_Data);
    xSemaphoreGive(SEM_CAN_Control);
    xSemaphoreGive(SEM_Done);

    vTaskDelay(pdMS_TO_TICKS(5000));

    xSemaphoreTake(SEM_Done, portMAX_DELAY);

    // Remove the ISR
    gpio_isr_handler_remove(GPIO_IMU_XL_INT);

    // End the TWAI driver
    twaiDriverEnd();

    // Delete semaphores
    vSemaphoreDelete(semI2C);
    vSemaphoreDelete(SEM_Accel_Data);
    vSemaphoreDelete(SEM_CAN_Control);
    vSemaphoreDelete(SEM_Done);
    
    // Don't start the scheduler - it starts automatically.
    // vTaskStartScheduler();
}

// Configure the board GPIO pins
int configureGPIO(void)
{
    // Set the GPIO config ID jumper inputs
    gpio_config_t gpioInputConfig = {};
    gpioInputConfig.intr_type = GPIO_INTR_DISABLE;
    gpioInputConfig.mode = GPIO_MODE_INPUT;
    gpioInputConfig.pin_bit_mask = (1ULL << GPIO_ID_JMPR_1) | (1ULL << GPIO_ID_JMPR_2) | (1ULL << GPIO_ID_JMPR_3);
    gpioInputConfig.pull_down_en = 0;
    gpioInputConfig.pull_up_en = 1;
    gpio_config(&gpioInputConfig);

    // gpio_set_intr_type(GPIO_NUM_6, GPIO_INTR_POSEDGE);

    // Set the GPIO config for IMU interrupt pin
    gpio_config_t gpioIntConfig = {};
    gpioIntConfig.intr_type = GPIO_INTR_POSEDGE;
    gpioIntConfig.mode = GPIO_MODE_INPUT;
    gpioIntConfig.pin_bit_mask = (1ULL << GPIO_IMU_XL_INT);
    gpioIntConfig.pull_down_en = 0;
    gpioIntConfig.pull_up_en = 1;
    gpio_config(&gpioIntConfig);

    // DEBUG - Dump GPIO Config
    // gpio_dump_io_configuration(stdout, (1ULL << GPIO_IMU_XL_INT));
    // gpio_dump_io_configuration(stdout, SOC_GPIO_VALID_GPIO_MASK);
    // vTaskDelay(pdMS_TO_TICKS(1000));

    return 0;
}

// Configures the board ID.
int IDInit(void)
{    
    // Delay before read
    vTaskDelay(pdMS_TO_TICKS(1000));

    int jumper = 0;
    // Read the jumper pins and add to find the ID.
    // As it's pull up inverted with ! so it aligns with jumper sets that bit.
    int jumper1 = !gpio_get_level(GPIO_ID_JMPR_1);
    int jumper2 = !gpio_get_level(GPIO_ID_JMPR_2);
    int jumper3 = !gpio_get_level(GPIO_ID_JMPR_3);
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
    ESP_LOGI(SOFA_DL_SYS, "ID: %d - ID jumper set %d%d%d %d.", SOFA_DL_ID, jumper1, jumper2, jumper3, jumper);

    // TODO: return status
    return 0;
}

// Installs and starts the TWAI driver 
int twaiDriverStart(void)
{
    //Install TWAI driver, trigger tasks to start
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver installed");
    
    // Start the TWAI driver
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver started.");

    return 0;
}

// Stops and uninstalls the TWAI driver 
int twaiDriverEnd(void)
{
    // End TWAI driver, stop
    ESP_ERROR_CHECK(twai_stop());
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver stopped.");

    //Uninstall TWAI driver
    ESP_ERROR_CHECK(twai_driver_uninstall());
    ESP_LOGI(CAN_ACCEL_TAG, "CAN driver uninstalled.");

    return 0;
}

// Sets the CAN IDs for messages based on board ID.
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