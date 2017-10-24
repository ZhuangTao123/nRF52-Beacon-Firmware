# README

This is the firmware project of nRF52 series beacon. 

## What's NEW
1. Add DFU Service in the application.
2. Add Secure DFU BLE Bootloader.
3. Add static PIN code and secure connection.
4. Application can load UICR values when starting.
5. Application can delete bonding information when disconnecting from central. User should re-enter PIN code everytime you connect to the device.

## How to install
1. Install the latest nRF52 SDK.
2. Copy the project folder to the 'ble_peripheral' folder in SDK installation directory.
3. Find 'ble_gatt.h' file in '/components/softdevice/s132/headers', and change 'BLE_GATT_ATT_MTU_DEFAULT' to 37.
4. Find 'ble_dfu.h' file in your SDK '%INSTALLDIR/components/ble/ble_services/ble_dfu', and change following definition
  **#define BLE_NORDIC_VENDOR_BASE_UUID**
  {{0x95, 0xE2, 0xED, 0xEB, 0x1B, 0xA0, 0x39, 0x8A, 0xDF, 0x4B, 0xD3, 0x8E, 0x00, 0x00, 0xC8, 0xA3}}
5. Backup 'ble_tps.c' and 'ble_tps.h', then replace them by the files in 'components' directory.