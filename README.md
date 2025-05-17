# GROUP-8_LABORATORY-5

Laboratory Assignment 5: Interfacing with I2C Devices

Learning Objectives:

Understand the I2C communication protocol (Master mode).
Configure an STM32 I2C peripheral.
Communicate with an I2C slave device (like a sensor or display).
Read and write data to device registers.
Process multi-byte sensor readings.

Prerequisites:
Completion of Lab 4.
An I2C sensor or display module (e.g., MPU6050 (accelerometer/gyro), BMP280/BME280 (pressure/temp/humidity), SSD1306 OLED display). Need to know the device's I2C address and register map (usually found in its datasheet).

Create a new project.

Configure an I2C peripheral (e.g., I2C1, I2C2) on your STM32 as Master, with appropriate speed (e.g., 100kHz or 400kHz). Reference the tutorials on DeepBlueEmbedded covering I2C.

Implement code to initialize your specific I2C device. This might involve sending configuration commands to specific registers.

Implement code to read data from your I2C device. For a sensor, this means reading data registers (often multi-byte values). For an OLED display, this would involve sending commands and pixel data. Reference the specific sensor/display tutorials if available on the site.

Process the raw data read from the sensor (e.g., combine bytes, apply scaling factors from the datasheet for temperature, pressure, acceleration, etc.).

Send the processed sensor data (or update the display) periodically. If using a sensor, send the readings via UART (using Lab 3 techniques) to the terminal. If using a display, update the display content.

