# Helvar-Seeing-the-Light
 
Project Overview
This project, undertaken for Aalto University in collaboration with Helvar, focuses on the development of a smart IoT-based sensor system designed to monitor lighting conditions within a room. The device utilizes Bluetooth Low Energy (BLE) to transmit sensor data, which is then analyzed to optimize lighting environments. By distinguishing between natural and artificial light sources and calculating the most dominant light source in 3D space, the system aims to enhance energy efficiency and improve lighting control systems.

Features
The system leverages advanced capabilities such as BLE advertising encryption to securely transmit data, ensuring privacy and data integrity. Key functionalities include the ability to measure illuminance levels, identify the light source, and determine the direction from which the light originates. This information allows for accurate adjustments to lighting systems, supporting features like daylight harvesting and constant light control, which are crucial for energy-saving and user comfort.

Implementation Details
Built using Zephyr RTOS, the device integrates various light sensors to gather comprehensive environmental data. Data from the sensors is processed to calculate color temperature and illuminance, and the results are used to classify light sources as natural or artificial. The project also includes a cloud-based tool for reporting and visualization, facilitating real-time monitoring and decision-making. Battery operation and the potential for energy harvesting ensure the device's long-term viability and minimal maintenance.
