# (USB) Serial to HID over GATT Bridge
This project implements a bridge between a USB serial device and a HID over GATT (HoG) device.

It allows you to control multiple devices via HID over GATT, which is a Bluetooth Low Energy (BLE) profile that allows for communication with HID devices such as keyboards, mice, and game controllers. The application act as mouse and keyboard, and forwards HID reports from UART port to specified Bluetooth device. It can be used to control a computer or other devices that support HID over GATT.

## Hardware Requirements
This project is initially designed for nRF52840, but any MCU with BLE support should work by modifying device tree.
- MCU with BLE support (nRF52 series, ESP32 series, etc.), if the MCU comes without USB support, external USB to serial converter is required.
- For USB serial, a MCU with USB support is required (nRF52840, ESP32-S2, etc.), default configuration should work for such devices.
