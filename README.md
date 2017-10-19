# README

This is the firmware project of nRF52 series beacon. 

## How to install
1. Install the latest nRF52 SDK
2. Copy the project folder to the 'ble_peripheral' folder in SDK installation directory.
3. Find 'ble_gatt.h' file in '/components/softdevice/s132/headers', and change 'BLE_GATT_ATT_MTU_DEFAULT' to 37