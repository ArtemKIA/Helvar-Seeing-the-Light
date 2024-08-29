
# Central Bluetooth Device for Light Sensor Data Collection

This repository contains the code for a central Bluetooth device developed as part of a project with Helvar. The device is designed to interact with a custom-built light sensor that advertises its data via Bluetooth Low Energy (BLE).

## Project Overview

The project with Helvar focuses on developing a light sensor which advertise its measurement using BLE technology. The central device, implemented using the Zephyr RTOS, acts as a Bluetooth beacon scanner that specifically targets the BLE-enabled light sensor. The sensor advertises key light metrics, which the central device decrypts and processes.

## Bluetooth Beacon Scanner with AES Decryption

This project is a Bluetooth Low Energy (BLE) beacon scanner implemented using the Zephyr RTOS. The application scans for a specific BLE device, retrieves its advertisement and scan response data, and decrypts the data if it's encrypted. The decrypted data includes sensor information such as azimuth, polar, average lux, and light source type.

## Features

- **Bluetooth Scanning**: Actively scans for a BLE device with a specific MAC address.
- **AES Decryption**: Utilizes TinyCrypt's AES-128 CCM mode to decrypt encrypted advertisement data from the target BLE device.
- **Data Parsing**: Extracts and prints sensor information such as azimuth, polar coordinates, average lux level, and the type of light source (natural or artificial) from the decrypted data.
- **Efficient Memory Handling**: Manages memory buffers to handle advertisement and scan response data efficiently.

## Dependencies

- **Zephyr RTOS**: The project is built on top of the Zephyr real-time operating system.
- **TinyCrypt Library**: Uses the TinyCrypt library for cryptographic operations (AES-128).

## How It Works

1. **Bluetooth Initialization**: The application initializes the Bluetooth subsystem and sets up the parameters for active scanning.
2. **Device Scanning**: The scanner looks for the target BLE device by matching its MAC address.
3. **Data Handling**: Upon detecting the target device, the application captures the advertisement and scan response data.
4. **Decryption and Parsing**: If the data is encrypted, it is decrypted using the AES-128 algorithm. The decrypted data is then parsed to extract sensor information.
5. **Output**: The extracted data, including sensor readings and light source type, is printed to the console.

## Usage

1. Clone this repository and build the project using Zephyr's build tools.
2. Flash the compiled binary onto a compatible Zephyr-supported board.
3. Run the application and observe the console output to see the decrypted and parsed sensor data from the BLE beacon.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
