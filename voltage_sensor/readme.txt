
# Embedded Sensor Monitoring System (PZEM-004T + LSM6DSL + OLED)

This project reads real-time electrical parameters from a PZEM-004T-100A power meter 
using Modbus RTU (UART) and reads IMU acceleration + temperature data from an LSM6DSL 
sensor using I²C. The processed values are displayed on an SSD1309 OLED screen over SPI.

## Features
• Reads voltage from PZEM-004T using Modbus RTU (9600 baud)  
• Computes CRC16 for Modbus commands  
• Receives 25-byte Modbus response frame  
• Reads X, Y, Z acceleration and temperature from LSM6DSL sensor  
• OLED display output via SPI (SSD1309)  
• FreeRTOS based task scheduling  
• UART, I²C, SPI, and GPIO driver integration  
• Real-time sensor monitoring loop  

## Architecture Overview
1. **UART (Modbus RTU)**  
   - Sends register read commands to PZEM-004T  
   - Receives voltage response frame (function code 0x04)  
   - CRC16 validated communication  

2. **I²C Sensor Interface**  
   - Retrieves IMU acceleration and temperature  
   - Uses LSM6DSL driver  

3. **SPI OLED Graphics**  
   - Displays X, Y, Z axis values  
   - Displays measured voltage  
   - Screen update every cycle  

4. **Main Loop**  
   - Read IMU  
   - Read voltage  
   - Format strings  
   - Update display  
   - Delay  

## Key Files
- **main.c** – Core application logic and sensor loop  
- **i2c_sensor_lsm6dsl.c** – LSM6DSL driver  
- **spi_oled_ssd1309.c** – OLED display driver  
- **uart.c** – UART transmit/receive functions  
- **trace.c** – Debug serial printing  
- **gpio.c** – GPIO configuration  

## Hardware Used
• PZEM-004T-100A Power Meter  
• LSM6DSL 6-axis IMU  
• SSD1309 SPI OLED  
• Embedded SoC running FreeRTOS  

## Protocols Used
• **UART (9600 baud)** – Modbus RTU  
• **I²C** – LSM6DSL  
• **SPI** – OLED screen  
• **GPIO** – Reset/Control pins  

## Example Output
- Acceleration values: `X = -132`, `Y = 98`, `Z = 1024`  
- Voltage: `220.1 V`  

The project demonstrates UART, I²C, and SPI driver integration, Modbus frame handling, 
sensor fusion basics, and real-time display rendering in an embedded system.
