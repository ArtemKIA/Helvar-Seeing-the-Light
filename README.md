# Seeing the Light - Helvar Sensor Project

## Overview

**Seeing the Light** is a sensor development project conducted by a team of Aalto University students in collaboration with **Helvar Oy**. The goal was to design and prototype a **battery-operated light sensor** capable of:

- Measuring **illuminance** (light intensity).
- Determining the **direction of incoming light**.
- Differentiating between **natural and artificial** light sources.
- Transmitting data via **Bluetooth Low Energy (BLE)**.

This project was carried out as part of **ELEC-D0301 Protopaja** at Aalto University.

## Features

- **Multi-Sensor Array:** Uses multiple light sensors for accurate light detection and direction estimation.
- **Bluetooth Low Energy (BLE):** Data transmission using BLE advertising.
- **Compact & Battery-Powered:** Designed for low-power operation with an estimated **14-hour battery life**.
- **Flexible PCB Design:** Optimized for compact integration.
- **Real-Time Data Processing:** Computes light intensity, color temperature, and light source classification.
- **Encryption:** Optional AES-128 encryption for secure BLE communication.

## Hardware Components

- **Microcontroller:** nRF52 (prototype) → nRF53 (final version, pending full integration).
- **Light Sensors:** LTR-329 / LTR-303 (ambient light sensors).
- **RGB Sensor:** TCS34725 for color detection.
- **I2C Multiplexer:** TCA9548A to manage multiple sensor connections.
- **Battery Management:** Custom battery chamber for easy replacement.
- **Programming Interface:** TC2030-IDC-NL for flashing firmware.

## Software

- **Development Environment:** 
  - Zephyr RTOS
  - Visual Studio Code with Nordic SDK
  - Git for version control

- **Key Functionalities:**
  - I2C communication with light sensors.
  - BLE advertising for real-time data transmission.
  - Data processing for illuminance, color temperature, and light direction.
  - Encryption support for secure BLE communication.

## Future Improvements
- Full **integration of nRF53 microcontroller** with optimized dual-core handling.
- Extended **battery life testing and power optimization**.
- Improved **sensor calibration** for higher accuracy.
- More **efficient BLE transmission** with enhanced encryption.
- Further **real-world testing** in various lighting conditions.

## Team Members

- **Vasilina Toporova** (Project Manager)
- **Artem Kiarkianen**
- **Ben Nguyem**
- **Kirill Levo**
- **Huy Vu**

## Resources

- [GitHub Repository](https://github.com/ArtemKIA/Helvar-Seeing-the-Light)
- [TCS34725 RGB Sensor Datasheet](https://www.alldatasheet.com/datasheet-pdf/view/894928/AMSCO/TCS34725.html)
- [LTR-329ALS Light Sensor Datasheet](https://www.mouser.com/datasheet/2/239/Lite-On_LTR-329ALS-01%20DS_ver1.1-348647.pdf)
- [Nordic nRF5340 DevKit Documentation](https://www.nordicsemi.com/-/media/Software-and-other-downloads/Product-Briefs/nRF5340-DK-PB-10.pdf)
- [Zephyr Project Documentation](https://docs.zephyrproject.org/latest/)

## License

This project is licensed under the **MIT License** – feel free to modify and use it as needed.

---

For questions or contributions, feel free to submit an issue or pull request.
