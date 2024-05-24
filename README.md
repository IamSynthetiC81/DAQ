[logo]: https://github.com/IamSynthetiC81/DAQ/blob/main/.pics/logo_fstuc.png "Logo"

![alt text][logo]

# Data Acquisition for FsTUC

## Hardware
This project consists of the following :
- IMU (MPU6050) : Standard Accelerometer and Rate Gyro
- GPS (Ublox M10Q) : Receives UBX-NAV-PVT messages to read position and velocity.
- Wheel Speed Sensor (Standard Hall Encoder)
- SD CARD : Logs the data.

## Description 
All of this runs on an ATMEGA2560 (currenty through the arduino bootloader) at a maximum rate of 1KSPS using the following features.

1. Serial Boot-Screen -> Detects Errors
2. SerialCommander -> Serial-Control during runtime with commands such as `Start`,`Stop`,`EnableSerialoutput` and `getFrequency` (**WIP**).
3. Configurable Timer enforced sampling rate (On Timer2).
4. Counter for Hall encoder (Counter 5).
5. ADC Hardware-Level optimized configuration (Convertions no longer block).

The main idea is to off-load all computations from the controller and focus on increasing the sampling rate. 
Therefore all blocking operations have been removed and all functions have been optimized to use minimal resources and
processing time.

# TO-DO

1. Enable DMP parsing on MPU6050.
2. 8-bit Parallel output option.
3. More sophisticated Error-Handler.
4. Overide Arduinos default serial ISRs 
