#ifndef SOFA_H
#define SOFA_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"

#include "i2c.h"

static char *SOFA_FUNC_TAG = "SOFA_DL_FUNC";

// Data ready
static const uint8_t WHOAMI = 0x6C;
static const uint8_t XLDA = 0x01;
static const uint8_t GDA = 0x02;

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

typedef struct {
    int16_t mx, my, mz; // This is signed because the 2 x 8bit reg store 16 bit number as 2s comp.
    float mX, mY, mZ;
    esp_err_t err;
} IMUMeasureData;

typedef struct {
    IMUMeasureData xl;
    IMUMeasureData gyro;
} IMUData;

// Function Prototypes
void imuWhoAmI(void);
void imuConfig(void);
void imuDisableInt(void);
int imuReadAData(i2cReadIMUReg *data, bool check);
int imuReadGData(i2cReadIMUReg *data, bool check);
int imuFormatData(uint8_t m[6], IMUMeasureData *data, float scale);
uint8_t imuSelfTestA(void);
uint8_t imuSelfTestG(void);

#endif