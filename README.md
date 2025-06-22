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

## Libraries

The following libraries have been used as part of this project. The license is replicated here and included in the source.

### Fusion

The MIT License (MIT)

Copyright (c) 2021 x-io Technologies

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

### TODO
- [] Madgwick filter
- [] Consolidate self test code into single modular function.
- [] 
- [] 