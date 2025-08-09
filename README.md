# BMP085 Pressure Sensor Driver for STM32F4

<div align="center">
  <img src="https://img.shields.io/badge/Platform-STM32F4-blue" alt="Platform">
  <img src="https://img.shields.io/badge/Language-C-orange" alt="Language">
  <img src="https://img.shields.io/badge/Interface-I2C-green" alt="Interface">
  <img src="https://img.shields.io/badge/Performance-39Hz-red" alt="Performance">
  <img src="https://img.shields.io/badge/Sensor-BMP085-purple" alt="Sensor">
  <img src="https://img.shields.io/badge/Interrupt-EOC%20Pin-yellow" alt="Interrupt">
</div>

---

## 🌟 Project Overview

A **high-performance C driver** for the Bosch BMP085 pressure sensor, specifically optimized for the ARM STM32F4 microcontroller. This implementation leverages **hardware interrupts** and the sensor's **End of Conversion (EOC) pin** to achieve superior data acquisition rates compared to traditional polling-based approaches.

### 🚀 Key Features
- **⚡ High-Speed Operation**: ~39 Hz data acquisition rate
- **🔌 Hardware Interrupt Driven**: Uses EOC pin for optimal timing
- **📊 Multi-Parameter Sensing**: Temperature, Pressure, and Altitude readings
- **🎯 IoT & Drone Ready**: Designed for real-time applications
- **📚 Well Documented**: Doxygen-compatible code documentation

---

## 🔧 Technical Implementation

### 🎯 Performance Advantage

Most BMP085 drivers use **polling methods** that waste CPU cycles waiting for sensor conversions. This implementation uses the **EOC (End of Conversion) pin** with hardware interrupts to maximize efficiency and data throughput.

```c
// Traditional Polling Approach (Inefficient)
while(!conversion_ready) {
    // CPU wastefully waiting...
}

// This Driver's Interrupt Approach (Efficient)
// CPU continues other tasks until EOC interrupt fires
void EXTI0_IRQHandler(void) {
    // Conversion ready! Process immediately
    bmp085_process_reading();
}
```

### 📈 State Machine Architecture

The driver implements a robust state machine for optimal sensor communication:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   IDLE STATE    │───▶│  REQUEST TEMP    │───▶│  WAIT FOR EOC   │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         ▲                                               │
         │                                               ▼
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│ CALCULATE ALT   │◀───│ REQUEST PRESSURE │◀───│   PROCESS TEMP  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
```

### ⚡ EOC Pin Timing Analysis

The EOC pin behavior ensures optimal conversion timing:

| Phase | EOC State | Duration | Action |
|-------|-----------|----------|--------|
| **Conversion Request** | LOW | ~4.5ms | CPU free for other tasks |
| **Data Ready** | HIGH | Rising Edge | Interrupt triggers data read |
| **Processing** | HIGH | ~100μs | I2C data transfer |

---

## 🛠️ Hardware Requirements

### 📟 Components
- **Bosch BMP085** Pressure Sensor
- **STM32F4** ARM Cortex-M4 Microcontroller (e.g., STM32F4 Discovery Board)

### 🔌 Wiring Connections

| STM32F4 Pin | BMP085 Pin | Function |
|-------------|------------|----------|
| **GND** | GND | Ground |
| **5V** | VIN | Power Supply |
| **PB7** | SDA | I2C Data Line |
| **PB6** | SCL | I2C Clock Line |
| **PD0** | EOC | End of Conversion Interrupt |

> **⚠️ Note**: Pin assignments (PB6, PB7, PD0) are currently fixed in the driver implementation. Future versions may include configurable pin mapping.

---

## 💻 Software Requirements

### 📚 Dependencies
The driver requires the **ARM Standard Peripheral Library** with the following modules:

```c
#include "stm32f4xx.h"         // Core STM32F4 definitions
#include "stm32f4xx_i2c.h"     // I2C peripheral control
#include "stm32f4xx_rcc.h"     // Reset and Clock Control
#include "stm32f4xx_gpio.h"    // GPIO pin management
#include "stm32f4xx_syscfg.h"  // System configuration
#include "stm32f4xx_exti.h"    // External interrupt control
```

---

## 🚀 Usage Guide

### 🔧 Basic Implementation

```c
#include "BMP085.h"
#include "timer.h"

int main(void) {
    // Initialize the BMP085 sensor
    bmp085_init();
    
    // Main application loop
    while(1) {
        // Run sensor state machine
        bmp085_run();
        
        // Access readings (updated at ~39 Hz)
        float temperature = get_temperature();
        float pressure = get_pressure();
        float altitude = get_altitude();
        
        // Your application code here...
    }
}
```

### 📊 Data Output Example

```
Temperature: 23.2°C
Pressure: 1013.25 hPa
Altitude: 152.3 feet (relative to sea level)
Update Rate: 39 Hz
```

---

## 📈 Performance Characteristics

### 🎯 Oversampling Modes

The BMP085 offers multiple oversampling settings that trade accuracy for speed:

| Mode | Samples | Conversion Time | Max Frequency | Accuracy |
|------|---------|-----------------|---------------|----------|
| **Ultra Low Power** | 1 | 4.5ms | ~220 Hz | Standard |
| **Standard** | 2 | 7.5ms | ~130 Hz | Improved |
| **High Resolution** | 4 | 13.5ms | ~75 Hz | High |
| **Ultra High Res** | 8 | 25.5ms | **~39 Hz** | Maximum |

*This driver is optimized for Ultra High Resolution mode for maximum accuracy.*

### ⚡ Performance Benefits

| Feature | Traditional Polling | This Driver (EOC Interrupt) |
|---------|-------------------|---------------------------|
| **CPU Usage** | High (busy waiting) | Low (event-driven) |
| **Response Time** | Variable delay | Immediate (interrupt) |
| **Data Rate** | Limited by polling frequency | Hardware-limited maximum |
| **System Integration** | Blocks other tasks | Non-blocking operation |

---

## 📖 Documentation

### 📚 Code Documentation
- **Doxygen Compatible**: All functions and structures are fully documented
- **Example Code**: Complete working example in `main.c`
- **State Diagrams**: Visual representation of sensor operation

### 🔧 Generating Documentation

```bash
# Use included Doxyfile to generate complete documentation
doxygen Doxyfile

# Generated docs will be in ./html/index.html
```

---

## 🔬 Technical Deep Dive

### 🌡️ Sensor Capabilities

**Temperature Measurement**
- Range: -40°C to +85°C
- Resolution: 0.1°C
- Accuracy: ±2°C

**Pressure Measurement**
- Range: 300-1100 hPa (9000m to -500m altitude)
- Resolution: 0.01 hPa (Ultra High Resolution)
- Accuracy: ±1 hPa

**Altitude Calculation**
- Relative altitude using sea-level reference pressure
- Suitable for drone/aircraft applications
- Configurable reference pressure constant

### 🔧 Advanced Features

**Interrupt-Driven Architecture**
- EOC pin monitoring with EXTI0 interrupt
- State machine-based sensor management
- Non-blocking operation for real-time systems

**I2C Communication**
- Hardware I2C peripheral utilization
- Error handling and timeout protection
- Configurable I2C speed settings

---

## 🎯 Applications

### 🚁 Ideal Use Cases
- **Drone/Quadcopter Altitude Control**
- **IoT Weather Monitoring Stations**
- **Barometric Pressure Logging**
- **Altitude-Based Location Services**
- **Real-Time Environmental Monitoring**

### 🏗️ System Integration
This driver is designed to integrate seamlessly into larger embedded systems without blocking other critical operations.

---

## 📄 Resources & References

### 📋 Datasheets
- [BMP085 Pressure Sensor Datasheet](https://www.sparkfun.com/datasheets/Components/General/BST-BMP085-DS000-05.pdf) (PDF)

### 🔗 Links
- **Source Code**: [GitHub Repository](https://github.com/EasonNYC/BMP085)

---

## 👨‍💻 Author

**Eason Smith** - Embedded Software Engineer  
📧 [Eason@EasonRobotics.com](mailto:Eason@EasonRobotics.com)    
💼 [LinkedIn](https://linkedin.com/in/easonsmith)

### 📅 Development Timeline
**December 2015** - Initial development and optimization

---

## 🔮 Future Enhancements

- [ ] **Configurable Pin Mapping** - Runtime pin assignment
- [ ] **Multiple Sensor Support** - I2C bus sharing
- [ ] **Advanced Filtering** - Digital signal processing
- [ ] **Calibration Features** - Runtime offset adjustment
- [ ] **Power Management** - Sleep/wake functionality

---

## 📜 License

This project is maintained for **educational and reference purposes**. Please contact the author for usage permissions in commercial applications.

---

<div align="center">
  <h3>⚡ Optimized for Performance | 🎯 Designed for Reliability | 🚀 Built for Real-Time ⚡</h3>
</div>
