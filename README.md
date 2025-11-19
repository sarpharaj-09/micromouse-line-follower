git # ESP32 Micromouse Line Follower

An advanced line follower robot using ESP32 with PlatformIO in VSCode.

## Features

- **ESP32-based** with WiFi capabilities
- **PID control algorithm** for smooth line following
- **4-sensor array** for better tracking
- **Real-time serial monitoring**
- **Web interface** (planned feature)

## Hardware Requirements

- **ESP32 Dev Board** (ESP32-WROOM-32)
- **IR Line Sensors** (4x TCRT5000 recommended)
- **Motor Driver** (L298N or L293D)
- **DC Motors** with wheels
- **LiPo Battery** (3.7V) or power supply

## Wiring Diagram

| ESP32 GPIO | Component | Pin |
|------------|-----------|-----|
| GPIO34 | Left Sensor | Analog |
| GPIO35 | Right Sensor | Analog |
| GPIO32 | Front Left Sensor | Analog |
| GPIO33 | Front Right Sensor | Analog |
| GPIO25 | Left Motor IN1 | PWM |
| GPIO26 | Left Motor IN2 | PWM |
| GPIO27 | Right Motor IN1 | PWM |
| GPIO14 | Right Motor IN2 | PWM |
| 3.3V | Sensors VCC | - |
| GND | Common Ground | - |

## Setup Instructions

### 1. Software Setup
```bash
git clone https://github.com/your-username/micromouse-line-follower.git
cd micromouse-line-follower
code .  # Opens in VSCode