

Compass and Light Sensor System

This program is a test implementation designed to interface with LTR329 ambient light sensors and an LIS3MDL magnetometer using the Zephyr RTOS. The primary purpose of the program is to monitor and calculate the direction and intensity of light sources in a room, as well as determine the device's heading based on magnetic field measurements. By integrating these sensor readings, the program can identify the most dominant light source and classify it as either natural or artificial.

How the Program Works

The program begins by initializing the I2C communication and configuring both the LTR329 light sensors and the LIS3MDL magnetometer. Once the sensors are set up, it enters a loop where it continuously reads data from each sensor. Light intensity values are collected from multiple LTR329 sensors positioned to capture different angles, while the magnetometer provides heading information. The collected data is then processed to calculate the azimuth and polar angles, giving the orientation of the light source relative to the device. The program also calculates the average light intensity and prints the device's heading, azimuth, polar angles, and the relative orientation between the magnetic heading and light source.

Limitations

Please note that this is a test program. The magnetometer readings have been found to have an absolute angle error of approximately 30%, which limits the accuracy of the heading and orientation calculations. Due to this significant error, the program is not used for precise orientation detection in practical applications.
