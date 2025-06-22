#include "sofa.h"

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

int imuFormatData(uint8_t m[6], IMUMeasureData *data, float scale)
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

int imuMeanData(IMUData rawData[], IMUSendData *data)
{
    int i = 0;
    for (i = 0; i < IMU_SAMPLE_PER_SEC; i++)
    {
        data->x = data->x + rawData[i].xl.mX;
        data->y = data->y + rawData[i].xl.mY;
        data->z = data->z + rawData[i].xl.mZ;
        data->roll = data->roll + rawData[i].gyro.mX;
        data->pitch = data->pitch + rawData[i].gyro.mY;
        data->yaw = data->yaw + rawData[i].gyro.mZ;
    }

    data->x = data->x / IMU_SAMPLE_PER_SEC;
    data->y = data->y / IMU_SAMPLE_PER_SEC;
    data->z = data->z / IMU_SAMPLE_PER_SEC;
    data->roll = data->roll / IMU_SAMPLE_PER_SEC;
    data->pitch = data->pitch / IMU_SAMPLE_PER_SEC;
    data->yaw = data->yaw / IMU_SAMPLE_PER_SEC;

    // DEBUG
    ESP_LOGI(SOFA_FUNC_TAG, "XL X/Y/Z: %.3f %.3f %.3f GYRO X/Y/Z: %.3f %.3f %.3f", data->x, data->y, data->z, data->roll, data->pitch, data->yaw);

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
        i2cReadReg(&regWhoAmI);
        ESP_LOGI(SOFA_FUNC_TAG, "Connecting to accelerometer on I2C bus at address 0x6A.");
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
        imuFormatData(regSTXL.m, &dataSTXL, ACCEL_SCALE_4);
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
        imuFormatData(regSTXL.m, &dataSTXL, ACCEL_SCALE_4);
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
        imuFormatData(regSTG.m, &dataSTG, GYRO_SCALE_2000);
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
        imuFormatData(regSTG.m, &dataSTG, GYRO_SCALE_2000);
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