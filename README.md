# Micromouse Line Follower

 **Project Status: In Development** 

A line-following micromouse robot project combining hardware design (KiCAD PCB layouts) with firmware development. This repository includes both Arduino Nano and ESP32 implementations, with comprehensive testing code and calibration tools.

## Overview

This project aims to build a reliable line-following robot using:
- **QTR-8 sensor array** for line detection
- **TB6612FNG motor driver** for motor control
- **Maze solving algorithm** with path simplification
- **PID control** for stable line following
- Dual MCU support: **ESP32** (primary) and **Arduino Nano** (testing)

The firmware includes motor control, PID tuning, sensor calibration, and a maze-solving algorithm that records and simplifies paths.

## Hardware

### Microcontrollers
- **ESP32 Development Board** (primary target)
- **Arduino Nano** (for testing and prototyping)

### Components
- **Motor Driver**: TB6612FNG
- **Line Sensors**: QTR-8 analog/RC sensor array
- **Motors**: DC motors for wheels
- **Power**: Battery pack
- **Display**: DM-OLED096 (0.96" OLED) - in design
- **Miscellaneous**: Jumper wires, resistors, capacitors

## Project Structure

```
micromouse-line-follower/
 Arduino_Nano/               # Arduino Nano PCB design (KiCAD)
    pcb_v1/                # Version 1 PCB layout
        DM-OLED096-636/    # OLED display footprint & symbol
        ESP32-DEVKIT-V1/   # ESP32 footprint & symbol
        MP1584_buck/       # Buck converter module
        kikad_dsign.*      # KiCAD project files
 ESP32/                      # ESP32 PCB design (KiCAD)
    pcb_v1/                # Version 1 PCB layout
        DM-OLED096-636/    # OLED display footprint & symbol
        ESP32-DEVKIT-V1/   # ESP32 footprint & symbol
        lineMazeSolver.*   # KiCAD project files
 micromouse_v1/             # Main firmware project (PlatformIO)
    micromouse_v1/
        platformio.ini     # PlatformIO configuration
        include/           # Header files
        lib/               # Dependencies (QTRSensors, TB6612FNG)
        src/
           main.cpp       # Main firmware with maze solving & PID
        test/              # Unit tests
 testing_phase_code/        # Standalone test sketches
    ESP32_test_code/       # ESP32-specific tests
       qtrSensor.ino     # QTR sensor test
    Nano_test_code/        # Arduino Nano tests
        Moter_test/        # Motor driver tests
        PID_Bluethooth/    # Bluetooth PID tuning
        plotter/           # Serial plotter data
        QTR_test/          # QTR sensor calibration
 README.md
```

## Firmware Features

### Current Implementation (main.cpp)
- **8-channel QTR sensor** integration with calibration
- **PID-based line following** with configurable gains (Kp, Kd)
- **Maze solving algorithm** with wall detection and path recording
- **Path simplification** to optimize shortest route
- **Motor control** with variable speed and direction
- **Turn decision logic** (left, straight, right)

### Pin Configuration (Arduino Nano)
```
Motor Control:
  - Motor 1: AIN1=4, AIN2=5, PWMA=3
  - Motor 2: BIN1=8, BIN2=9, PWMB=6
Control:
  - Switches: SW1=10, SW2=11, SW3=12
  - LED: 7
```

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (for ESP32 development)
- [Arduino IDE](https://www.arduino.cc/en/software) (for Nano sketches)
- Required libraries:
  - `SparkFun_TB6612` (Motor Driver)
  - `QTRSensors` (Line Sensor Array)

### Building & Uploading (PlatformIO/ESP32)

```powershell
cd micromouse_v1/micromouse_v1
pio run              # Build
pio run -t upload    # Upload to connected ESP32
```

### Testing (Arduino Nano)

1. Open Arduino IDE
2. Load a test sketch: `testing_phase_code/Nano_test_code/<test_folder>/<sketch>.ino`
3. Install dependencies via Library Manager
4. Select board (Arduino Nano) and COM port
5. Upload

### Test Sketches Available

| Sketch | Purpose |
|--------|---------|
| `Moter_test` | Test TB6612 motor driver (forward, backward, turn) |
| `QTR_test` | Read raw QTR sensor values and calibrate |
| `PID_Bluethooth` | Wireless PID tuning via Bluetooth serial |
| `plotter` | Stream sensor data for Serial Plotter visualization |

## Development Workflow

- **Feature branches**: Use `wip/*` prefix for work-in-progress branches
- **Main branch**: Keep stable with tested, documented commits
- **Testing**: Validate on both hardware and in simulation before merging
- **Documentation**: Update this README and code comments when adding features

## PCB Design Status

- **Arduino Nano v1**: In design phase - includes OLED display, buck converter
- **ESP32 v1**: In design phase - primary development board layout

## Troubleshooting

### Sensor Calibration Issues
- Run `QTR_test` to verify raw sensor values
- Check sensor array alignment over the line
- Verify power supply voltage (QTR operates 3.3-5V)

### Motor Control Issues
- Use `Moter_test` to verify TB6612 pin connections
- Check PWM pins are available on your MCU
- Verify motor power supply is adequate

### PID Tuning
- Use `PID_Bluethooth` for real-time parameter adjustment
- Start with low Kp/Kd values and increase gradually
- Monitor motor oscillation and response time

## Contributing

Contributions welcome! Please:
1. Create a feature branch (`wip/feature-name`)
2. Test thoroughly on hardware
3. Submit a pull request with clear description
4. Update documentation as needed

## License

This project is licensed under the terms specified in `LICENSE`.

---

**Last Updated**: January 2026  
**Note**: This project is actively developed. Features and documentation are updated frequently.
