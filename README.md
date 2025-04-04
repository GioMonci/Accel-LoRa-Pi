# Accel-LoRa-Pi

## Project Overview

This project enables an ESP32 Feather V2 to collect acceleration data using an Adafruit High Precision 9-DoF IMU FeatherWing (ISM330DHCX + LIS3MDL) and transmit it via LoRa to a remote Raspberry Pi. 
The goal is to establish a low-power, long-range wireless communication system for motion sensing applications.

## Hardware Components

 - ESP32 Feather V2
 - Adafruit High Precision 9-DoF IMU FeatherWing (ISM330DHCX + LIS3MDL)
 - Adafruit LoRa Radio FeatherWing - RFM95W 900 MHz
 - Raspberry Pi

## Improvements Complete
 - Send Acceleration Data to Remote Raspberry Pi using LoRa
 - Merge existing Accel and LoRa code
 - Cleaned up redundant code
 - Sending 48 bytes

## Notes / TO-DO:
  - Make send rates more efficient and stable
