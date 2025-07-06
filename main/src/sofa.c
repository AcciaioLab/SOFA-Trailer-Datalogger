#include "../include/sofa.h"

/**
 * @brief Sequence to initialise the IMU for SOFA.
 * @param void
 * @return void
 */
void imuInit(uint8_t *status)
{
    // Self test results
    uint8_t stResultXL = 0;
    uint8_t stResultGyro = 0;

    // Read the WHO_AM_I (0x0F) register to see if we can see the accelerometer.
    imuWhoAmI();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Disable the IMU interrupt at the start.
    imuDisableInt();

    // Perform self tests
    stResultXL = imuSelfTestA();
    vTaskDelay(pdMS_TO_TICKS(100));
    stResultGyro = imuSelfTestG();
    vTaskDelay(pdMS_TO_TICKS(100));

    // Set the board status based on the self test result.
    *status = (((uint8_t)SOFA_DL_ID) << 4) | (stResultXL << 3) | (stResultGyro << 2);

    // Configure the accelerometer before reading data.
    imuConfig();
    vTaskDelay(pdMS_TO_TICKS(100));
}

/**
 * @brief Reads accelerometer data from the IMU. It is pre-configured with the I2C registers.
 * @param data pointer to the i2cReadIMUReg to return the received data.
 * @param check True to wait for the data ready status to check before reading data registers.
 * @return status (int, not used TODO)
 */
int imuReadAData(i2cReadIMUReg *data, bool check)
{
    int err = 0;

    if (check)
    {
        i2cRead1Reg regDataReady = {
            .tx = 0x1E,     // STATUS_REG register
            .rx = 0x00,     // No expectation
            .err = ESP_FAIL
        };  

        // Check accel data is available.
        uint8_t dataAvailable = 0x00;

        // ESP_LOGI(SOFA_FUNC_TAG, "Checking XLDA XL data available...");

        while (dataAvailable != XLDA)
        {
            i2cReadReg(&regDataReady);
            dataAvailable = regDataReady.rx & XLDA;
        }

        // ESP_LOGI(SOFA_FUNC_TAG, "XLDA XL data is available.");
    }
    
    // New accel data is available, read it.
    err = i2cReadDataReg(data);

    return err;
}

/**
 * @brief Reads gyro data from the IMU. It is pre-configured with the I2C registers.
 * @param data pointer to the i2cReadIMUReg to return the received data.
 * @param check True to wait for the data ready status to check before reading data registers.
 * NOTE: This should always be true for gyro.
 * @return status (int, not used TODO)
 */
int imuReadGData(i2cReadIMUReg *data, bool check)
{
    int err = 0;

    if (check)
    {
        i2cRead1Reg regDataReady = {
            .tx = 0x1E,     // STATUS_REG register
            .rx = 0x00,     // No expectation
            .err = ESP_FAIL
        };  

        // Check accel data is available.
        uint8_t dataAvailable = 0x00;

        // ESP_LOGI(SOFA_FUNC_TAG, "Checking GDA gyro data available...");

        while (dataAvailable != GDA)
        {
            i2cReadReg(&regDataReady);
            dataAvailable = regDataReady.rx & GDA;
        }

        // ESP_LOGI(SOFA_FUNC_TAG, "GDA gyro data is available.");
    }

    // New accel data is available, read it.
    err = i2cReadDataReg(data);

    return err;
}

/**
 * @brief Scales the 8bit raw IMU data into a float.
 * @param m[] raw data array, 2 x 8bit unsigned int 
 * @param data pointer to IMUMeasureData struct to save result
 * @param scale scale value depending on accelerometer or gyro 
 * @return int status (TODO)
 */
int imuScaleData(uint8_t m[], IMUMeasureData *data, float scale)
{
    data->mx = ((int16_t)m[1] << 8) + m[0];
    data->mX = ((float)data->mx) * scale;

    data->my = ((int16_t)m[3] << 8) + m[2];
    data->mY = ((float)data->my) * scale;

    data->mz = ((int16_t)m[5] << 8) + m[4];
    data->mZ = ((float)data->mz) * scale;

    // DEBUG
    // ESP_LOGI(SOFA_FUNC_TAG, "X/Y/Z axis reg %#04X %#04X %#04X | %#04X %#04X %#06X | %#04X %#04X %#06X = %.6f %.6f %.6f", m[0], m[1], data->mx, m[2], m[3], data->my, m[4], m[5], data->mz, data->mX, data->mY, data->mZ);

    return 0;
}

/**
 * @brief Applies the Fusion AHRS filter to the raw data as it is received.
 * @param ahrs pointer to Fusion ahrs algorithm structure.
 * @param data pointer to IMUData with the last received IMU data.
 * @return int status (TODO)
 */
int imuFusionAHRS(FusionAhrs *ahrs, IMUData *data, FusionOffset *offset)
{
    // ESP_LOGI(SOFA_FUNC_TAG, "Fusion AHRS filtering raw IMU data.");
    // This follows the Fusion library simple example
    // https://github.com/xioTechnologies/Fusion/blob/main/Examples/Simple/main.c

    FusionVector gyro = {.axis = {
        .x = data->gyro.mX, 
        .y = data->gyro.mY, 
        .z = data->gyro.mZ
    }};
    FusionVector accel = {.axis = {
        .x = data->xl.mX, 
        .y = data->xl.mY, 
        .z = data->xl.mZ
    }};

    gyro = FusionOffsetUpdate(offset, gyro);

    FusionAhrsUpdateNoMagnetometer(ahrs, gyro, accel, IMU_SAMPLE_PERIOD); // I don't know why a const doesn't work but a magic number does.

    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(ahrs));
    const FusionVector earth = FusionAhrsGetEarthAcceleration(ahrs);

    data->xl.mX = earth.axis.x;
    data->xl.mY = earth.axis.y;
    data->xl.mZ = earth.axis.z;

    data->roll = euler.angle.roll;
    data->pitch = euler.angle.pitch;
    data->yaw = euler.angle.yaw;

    // ESP_LOGI(SOFA_FUNC_TAG, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f", euler.angle.roll, euler.angle.pitch, euler.angle.yaw);

    return 0;
}

/**
 * @brief Calculates the mean of the IMU data and saves the result in the send data struct.
 * @param rawData IMUData raw data array 
 * @param data pointer to IMUSendData struct
 * @return int status (TODO)
 */
int imuMeanData(IMUData rawData[], IMUSendData *data)
{
    int i = 0;
    for (i = 0; i < IMU_SAMPLE_RATE; i++)
    {
        data->x = data->x + rawData[i].xl.mX;
        data->y = data->y + rawData[i].xl.mY;
        data->z = data->z + rawData[i].xl.mZ;
        data->roll = data->roll + rawData[i].roll;
        data->pitch = data->pitch + rawData[i].pitch;
        data->yaw = data->yaw + rawData[i].yaw;
        data->zRMS = data->zRMS + (rawData[i].xl.mZ * rawData[i].xl.mZ);
    }

    data->x = data->x / IMU_SAMPLE_RATE;
    data->y = data->y / IMU_SAMPLE_RATE;
    data->z = data->z / IMU_SAMPLE_RATE;
    data->roll = data->roll / IMU_SAMPLE_RATE;
    data->pitch = data->pitch / IMU_SAMPLE_RATE;
    data->yaw = data->yaw / IMU_SAMPLE_RATE;
    data->zRMS = data->zRMS / IMU_SAMPLE_RATE;

    // DEBUG
    ESP_LOGI(SOFA_FUNC_TAG, "AVERAGE: XL X/Y/Z/ZRMS, %.6f, %.6f, %.6f, %.6f, GYRO X/Y/Z, %.6f, %.6f, %.6f,", data->x, data->y, data->z, data->zRMS, data->roll, data->pitch, data->yaw);

    return 0;
}

/**
 * @brief Packs the IMUSendData after all operations to be sent via CANBus (TWAI in ESP32).
 * @param data IMUSendData processed ready to send 
 * @param xlmsg pointer to XL IMUSendData struct
 * @param gyromsg pointer to gyro IMUSendData struct
 * @return int status (TODO)
 */
int imuCreateCANMsg(IMUSendData *data, twai_message_t *xlmsg, twai_message_t *gyromsg, twai_message_t *rmsmsg, uint8_t stat)
{
    // This is awful repetitive code that needs to be split into a few functions. It hurts my eyes.
    uint8_t mxl[7];
    uint8_t mgyro[7];
    uint8_t mzrms[7];
    int16_t xa, ya, za, zrms;
    int16_t xg, yg, zg;

    // Unscale the XL data
    xa = data->x / ACCEL_SCALE_8;
    mxl[0] = (uint8_t)(xa & 0x00FF);
    mxl[1] = (uint8_t)((xa & 0xFF00) >> 8);
    // ESP_LOGI(SOFA_FUNC_TAG, "UNSCALE XL, %.6f, %04X, %02X, %02X,", data->x, xa, mxl[1], mxl[0]);

    ya = data->y / ACCEL_SCALE_8;
    mxl[2] = (uint8_t)(ya & 0x00FF);
    mxl[3] = (uint8_t)((ya & 0xFF00) >> 8);

    za = data->z / ACCEL_SCALE_8;
    mxl[4] = (uint8_t)(za & 0x00FF);
    mxl[5] = (uint8_t)((za & 0xFF00) >> 8);

    zrms = data->zRMS / ACCEL_SCALE_8;
    mzrms[0] = (uint8_t)(zrms & 0x00FF);
    mzrms[1] = (uint8_t)((zrms & 0xFF00) >> 8);

    // Unscale the XL data
    xg = data->roll / EULER_ANGLE_SCALE;
    mgyro[0] = (uint8_t)(xg & 0x00FF);
    mgyro[1] = (uint8_t)((xg & 0xFF00) >> 8);

    yg = data->pitch / EULER_ANGLE_SCALE;
    mgyro[2] = (uint8_t)(yg & 0x00FF);
    mgyro[3] = (uint8_t)((yg & 0xFF00) >> 8);

    zg = data->yaw / EULER_ANGLE_SCALE;
    mgyro[4] = (uint8_t)(zg & 0x00FF);
    mgyro[5] = (uint8_t)((zg & 0xFF00) >> 8);
    // ESP_LOGI(SOFA_FUNC_TAG, "UNSCALE GYRO, %.6f, %04X, %02X, %02X,", data->yaw, zg, mgyro[5], mgyro[4]);

    // Move XL data into CAN message.
    xlmsg->data[0] = mxl[0];
    xlmsg->data[1] = mxl[1];
    xlmsg->data[2] = mxl[2];
    xlmsg->data[3] = mxl[3];
    xlmsg->data[4] = mxl[4];
    xlmsg->data[5] = mxl[5];
    xlmsg->data[6] = stat;

    // Move gyro data into CAN message.
    gyromsg->data[0] = mgyro[0];
    gyromsg->data[1] = mgyro[1];
    gyromsg->data[2] = mgyro[2];
    gyromsg->data[3] = mgyro[3];
    gyromsg->data[4] = mgyro[4];
    gyromsg->data[5] = mgyro[5];
    gyromsg->data[6] = stat;

    // Move rms data into CAN message.
    rmsmsg->data[0] = mzrms[0];
    rmsmsg->data[1] = mzrms[1];
    rmsmsg->data[2] = mzrms[2] = 0;
    rmsmsg->data[3] = mzrms[3] = 0;
    rmsmsg->data[4] = mzrms[4] = 0;
    rmsmsg->data[5] = mzrms[5] = 0;
    rmsmsg->data[6] = stat;

    return 0;
}

void imuWhoAmI(void)
{
    // Create the data for this read.
    // TX1 - Who Am I
    i2cRead1Reg regWhoAmI = {
        .tx = 0x0F,     // Who Am I register
        .rx = 0x00,     // Expecting 0x6C
        .err = ESP_FAIL
    };  

    // Read the WHO_AM_I (0x0F) register to see if we can see the accelerometer.
    // WHO_AM_I returns 0x6C.
    while (regWhoAmI.rx != WHOAMI)
    {
        ESP_LOGI(SOFA_FUNC_TAG, "Connecting to accelerometer on I2C bus at address 0x6A.");
        i2cReadReg(&regWhoAmI);
    }

    // Log that we saw the accelerometer
    ESP_LOGI(SOFA_FUNC_TAG, "I2C initialized successfully. WHO AM I = %#04X", regWhoAmI.rx);
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

    // Config 3
    // INT1_CTRL 0x0D INT1 pin control register (R/W)
    // INT1_DRDY_XL
    i2cWrite1Reg config3 = {
        .tx = {0x0D, 0x01},
        .rx = 0x00,
        .err = ESP_FAIL
    };
    i2cTransmitReg(&config3);

    ESP_LOGI(SOFA_FUNC_TAG, "Wrote config to IMU.");
}

// Disables IMU INT1 pin
void imuDisableInt(void)
{
    // Config 1
    // INT1_CTRL 0x0D INT1 pin control register (R/W)
    // INT1_DRDY_XL
    i2cWrite1Reg config3 = {
        .tx = {0x0D, 0x00},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    i2cTransmitReg(&config3);
}

uint8_t imuSelfTestA(void)
{
    // Accelerometer UI self-test mode 1
    // Datasheet section 11.
    ESP_LOGI(SOFA_FUNC_TAG, "Accelerometer self-test started.");
    
    int result = 0;

    i2cReadIMUReg regSTXL = {
        .reg = 0x28,    // accel x axis reg, y follows 0x2A, z follows 0x2C
        .length = 6,    // read all accel data in 1 go
        .err = ESP_FAIL
    };

    IMUMeasureData dataSTXL;

    float OUTX_NOST = 0.0f;
    float OUTY_NOST = 0.0f;
    float OUTZ_NOST = 0.0f;
    float OUTX_ST = 0.0f;
    float OUTY_ST = 0.0f;
    float OUTZ_ST = 0.0f;

    // Initialise and turn on sensor
    i2cWrite1Reg regST1 = {
        .tx = {0x10, 0x38},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    for (int i = 0; i < 10; i++)
    {
        regST1.tx[0] = 0x10 + i;
        if (regST1.tx[0] == 0x10)
        {
            regST1.tx[1] = 0x38;
        }
        else if (regST1.tx[0] == 0x12)
        {
            regST1.tx[1] = 0x44;
        }
        else
        {
            regST1.tx[1] = 0x00;
        }
        
        i2cTransmitReg(&regST1);
    }

    // Power up, wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read accelerometer data once, discard.
    imuReadAData(&regSTXL, true);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        imuReadAData(&regSTXL, true);
        imuScaleData(regSTXL.m, &dataSTXL, ACCEL_SCALE_4);
        // Compute the average of each axis and store in _NOST.
        // Each read add to _NOST, divide by 5 at end.
        OUTX_NOST = OUTX_NOST + dataSTXL.mX;
        OUTY_NOST = OUTY_NOST + dataSTXL.mY;
        OUTZ_NOST = OUTZ_NOST + dataSTXL.mZ;

        // DEBUG
        ESP_LOGI(SOFA_FUNC_TAG, "_NOST %d: %.6f \t %.6f \t %.6f", i, OUTX_NOST, OUTY_NOST, OUTZ_NOST);
    }
    // Compute average
    OUTX_NOST = OUTX_NOST / 5;
    OUTY_NOST = OUTY_NOST / 5;
    OUTZ_NOST = OUTZ_NOST / 5;

    // DEBUG
    ESP_LOGI(SOFA_FUNC_TAG, "_NOST AVG: %.6f \t %.6f \t %.6f", OUTX_NOST, OUTY_NOST, OUTZ_NOST);

    // Enable accelerometer self-test
    i2cWrite1Reg regST2 = {
        .tx = {0x14, 0x01},
        .rx = 0x00,
        .err = ESP_FAIL
    };
    i2cTransmitReg(&regST2);

    // Wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read accelerometer data once, discard.
    imuReadAData(&regSTXL, true);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        imuReadAData(&regSTXL, true);
        imuScaleData(regSTXL.m, &dataSTXL, ACCEL_SCALE_4);
        // Compute the average of each axis and store in _ST.
        // Each read add to _ST, divide by 5 at end.
        OUTX_ST = OUTX_ST + dataSTXL.mX;
        OUTY_ST = OUTY_ST + dataSTXL.mY;
        OUTZ_ST = OUTZ_ST + dataSTXL.mZ;

        // DEBUG
        ESP_LOGI(SOFA_FUNC_TAG, "_ST %d: %.6f \t %.6f \t %.6f", i, OUTX_ST, OUTY_ST, OUTZ_ST);
    }
    // Compute average
    OUTX_ST = OUTX_ST / 5;
    OUTY_ST = OUTY_ST / 5;
    OUTZ_ST = OUTZ_ST / 5;

    // DEBUG
    ESP_LOGI(SOFA_FUNC_TAG, "_ST AVG: %.6f \t %.6f \t %.6f", OUTX_ST, OUTY_ST, OUTZ_ST);

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
        ESP_LOGI(SOFA_FUNC_TAG, "Accelerometer self-test PASSED.");
    }
    else
    {
        ESP_LOGI(SOFA_FUNC_TAG, "Accelerometer self-test FAILED.");
    }

    // Disable self-test
    regST2.tx[0] = 0x14;
    regST2.tx[1] = 0x00;
    i2cTransmitReg(&regST2);

    // Disable sensor, ready for config.
    regST1.tx[0] = 0x10;
    regST1.tx[1] = 0x00;
    i2cTransmitReg(&regST1);

    return result;
}

uint8_t imuSelfTestG(void)
{
    // Gyroscope UI self-test mode 1
    // Datasheet section 11.
    ESP_LOGI(SOFA_FUNC_TAG, "Gyroscope self-test started.");

    int result = 0;

    i2cReadIMUReg regSTG = {
        .reg = 0x22,    // gyro x axis reg, y follows 0x24, z follows 0x26
        .length = 6,    // read all gyro data in 1 go
        .err = ESP_FAIL
    };

    IMUMeasureData dataSTG;

    float OUTX_NOST = 0.0f;
    float OUTY_NOST = 0.0f;
    float OUTZ_NOST = 0.0f;
    float OUTX_ST = 0.0f;
    float OUTY_ST = 0.0f;
    float OUTZ_ST = 0.0f;

    // Initialise and turn on sensor
    i2cWrite1Reg regST1 = {
        .tx = {0x10, 0x00},
        .rx = 0x00,
        .err = ESP_FAIL
    };

    for (int i = 0; i < 10; i++)
    {
        regST1.tx[0] = 0x10 + i;
        if (regST1.tx[0] == 0x11)
        {
            regST1.tx[1] = 0x5C;
        }
        else if (regST1.tx[0] == 0x12)
        {
            regST1.tx[1] = 0x44;
        }
        else
        {
            regST1.tx[1] = 0x00;
        }
        
        i2cTransmitReg(&regST1);
    }

    // Power up, wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read gyro data once, discard.
    imuReadGData(&regSTG, true);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        imuReadGData(&regSTG, true);
        imuScaleData(regSTG.m, &dataSTG, GYRO_SCALE_2000);
        // Compute the average of each axis and store in _NOST.
        // Each read add to _NOST, divide by 5 at end.
        OUTX_NOST = OUTX_NOST + dataSTG.mX;
        OUTY_NOST = OUTY_NOST + dataSTG.mY;
        OUTZ_NOST = OUTZ_NOST + dataSTG.mZ;

        // DEBUG
        ESP_LOGI(SOFA_FUNC_TAG, "_NOST %d: %.6f \t %.6f \t %.6f", i, OUTX_NOST, OUTY_NOST, OUTZ_NOST);
    }
    // Compute average
    OUTX_NOST = OUTX_NOST / 5;
    OUTY_NOST = OUTY_NOST / 5;
    OUTZ_NOST = OUTZ_NOST / 5;

    // DEBUG
    ESP_LOGI(SOFA_FUNC_TAG, "_NOST AVG: %.6f \t %.6f \t %.6f", OUTX_NOST, OUTY_NOST, OUTZ_NOST);

    // Enable gyro self-test
    i2cWrite1Reg regST2 = {
        .tx = {0x14, 0x04},
        .rx = 0x00,
        .err = ESP_FAIL
    };
    i2cTransmitReg(&regST2);

    // Wait 100ms for stable output
    vTaskDelay(pdMS_TO_TICKS(100));

    // Read accelerometer data once, discard.
    imuReadGData(&regSTG, true);

    for (int i = 0; i < 5; i++)
    {
        // Read the data
        imuReadGData(&regSTG, true);
        imuScaleData(regSTG.m, &dataSTG, GYRO_SCALE_2000);
        // Compute the average of each axis and store in _ST.
        // Each read add to _ST, divide by 5 at end.
        OUTX_ST = OUTX_ST + dataSTG.mX;
        OUTY_ST = OUTY_ST + dataSTG.mY;
        OUTZ_ST = OUTZ_ST + dataSTG.mZ;

        // DEBUG 
        ESP_LOGI(SOFA_FUNC_TAG, "_ST %d: %.6f \t %.6f \t %.6f", i, OUTX_ST, OUTY_ST, OUTZ_ST);
    }
    // Compute average
    OUTX_ST = OUTX_ST / 5;
    OUTY_ST = OUTY_ST / 5;
    OUTZ_ST = OUTZ_ST / 5;

    // DEBUG
    ESP_LOGI(SOFA_FUNC_TAG, "_ST AVG: %.6f \t %.6f \t %.6f", OUTX_ST, OUTY_ST, OUTZ_ST);

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
        ESP_LOGI(SOFA_FUNC_TAG, "Gyroscope self-test PASSED.");
    }
    else
    {
        ESP_LOGI(SOFA_FUNC_TAG, "Gyroscope self-test FAILED.");
    }

    // Disable self-test
    regST2.tx[0] = 0x14;
    regST2.tx[1] = 0x00;
    i2cTransmitReg(&regST2);

    // Disable sensor, ready for config.
    regST1.tx[0] = 0x11;
    regST1.tx[1] = 0x00;
    i2cTransmitReg(&regST1);

    return result;
}