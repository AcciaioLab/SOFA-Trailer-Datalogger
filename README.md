# SOFA Trailer Datalogger

This project contains the source code to build and flash the SOFA Trailer Datalogger controllers.

Each datalogger sets an ID based on its position on the installed jumpers.
It sets up the accelerometer and polls it for x/y/z data.

## Hardware

ESP32-S3 
CAN
Accelerometer - STM LSM6DSOX

## Software

ESP-IDF v5.4.1 in VSCode, built from template-app sample project for project structure.

### TODO
- [] Madgwick filter
- [] Consolidate self test code into single modular function.
- [] 
- [] 