
---

# Exam, 2 led rgb controlling using http requesty and web technologies

## Overview

This project is an embedded application for ESP32 (or similar microcontrollers) that demonstrates:

- **WiFi SoftAP + Station mode** (via `wifi_app`)
- **HTTP server** for web-based interaction
- **RGB LED control** based on temperature readings from an NTC thermistor
- **Potentiometer-based brightness control**
- **UART interface** for dynamic configuration of temperature thresholds and toggling console output
- **Web interface** for user interaction

---

## Features

### 1. **Temperature-Based RGB LED Control**
- Reads temperature from an NTC thermistor every 0.5 seconds.
- The RGB LED color changes according to the temperature, this is controlled via local web, using temperature threshold to update color depending on the user thresholds.
- The temperature thresholds for each color are also configurable at runtime via UART.

### 2. **Brightness Control**
- A potentiometer is used to adjust the brightness of the RGB LED.

### 3. **UART Console**
- UART is used for:
  - Toggling temperature printout to the console.
  - Setting temperature thresholds for each color using the format: `[Color]min|max` (e.g., `[R]40|80` for red between 40°C and 80°C).

### 4. **Web Interface**
- A simple web page (served from `main/webpage/`) allows users to interact with the device.

### 5. **WiFi SoftAP + Station**
- The device can act as both a WiFi access point and a station, allowing configuration and monitoring from any device.

---

## Directory Structure

```

├── CMakeLists.txt
├── partitions_two_ota.csv
├── sdkconfig
├── main/
│   ├── CMakeLists.txt
│   ├── main.c                # Main application logic
│   ├── request/
│   │   ├── http_server.c     # HTTP server implementation
│   │   ├── http_server.h
│   │   ├── tasks_common.h
│   │   ├── wifi_app.c        # WiFi application logic
│   │   ├── wifi_app.h
│   ├── utils/
│   │   ├── adc_utils.c       # ADC utility functions
│   │   ├── adc_utils.h
│   │   ├── io_utils.c        # IO utility functions
│   │   ├── io_utils.h
│   │   ├── tim_ch_duty.c     # PWM and timer utilities
│   │   ├── tim_ch_duty.h
│   ├── webpage/
│   │   ├── app.css
│   │   ├── app.js
│   │   ├── favicon.ico
│   │   ├── index.html
│   │   ├── jquery-3.3.1.min.js
```

---

## How It Works

### Main Application (`main.c`)

- **Initialization**: Sets up mutexes, PWM timers, RGB LEDs, ADC channels, and FreeRTOS queues.
- **Tasks**:
  - `ntc_task`: Reads the NTC thermistor, calculates temperature, and sends it to a queue.
  - `pot_task`: Reads the potentiometer, calculates voltage, and sends it to a queue.
  - `rgb_temp_task`: Receives temperature and potentiometer data, determines the correct LED color and brightness, and updates the RGB LED accordingly.
  - `rgb_pwm_task`: Handles direct RGB value updates (not temperature-based).
  - `uart_rx_task`: Handles UART input for threshold configuration and toggling UART output.

### Temperature Thresholds

- Configurable via UART using the format: `[Color]min|max`
  - Example: `[G]20|40` sets green to 20°C–40°C.
- The thresholds are stored in a global array and used to determine the LED color.

### Web Interface

- Served from `main/webpage/`
- Allows users to view and interact with the device over WiFi.

---


## Author

David Ramírez Betancourth & Santiago Bustamante Montoya

---

**Note:** For detailed hardware connections (GPIOs for LEDs, thermistor, potentiometer), refer to the pin definitions in `main.c` and your board’s schematic.
