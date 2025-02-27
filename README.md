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
 - Setup Multi-Threading
 - Cleaned up redundant code

## Notes / TO-DO:
  - Try single core to see if EV join is faster.
  - Maybe try rewritting how we send.
  - Check how much we are updating myData[].
  - Is it too fast?
  - Instead of mutlithreading accel and loRa create an algo that does 10 seconds of collection then sends that data.
  - Think about how we are going to implement tensor flow with our exisiting code
