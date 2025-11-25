# Micromouse Line Follower

🚧 **Project Status: In Development** 🚧

This repository contains the beginnings of a Micromouse line-following robot project. The firmware targets ESP32 as the main development board and includes Arduino Nano test sketches under `Nano_test_code/` that show motor and sensor tests, plus wireless PID tuning examples.

## Overview

The goal is to build a reliable line-following robot using a QTR sensor array, TB6612 motor driver, and an ESP32 (or Arduino Nano for simple tests). The codebase will grow to include motor control, PID tuning, calibration, and eventually navigation and obstacle-handling modules.

## Hardware

- **Primary target**: ESP32 development boards
- **Optional**: Arduino Nano (test sketches)
- **Motor Driver**: TB6612FNG
- **Line Sensors**: QTR analog/RC sensor arrays
- **Other components**: Motors, wheels, battery, jumper wires, and a chassis

## Project Structure
```
micromouse-line-follower/
├── .github/                # GitHub workflows and CI (optional)
├── docs/
│   ├── component_datasheet/
│   │   ├── esp32_datasheet_en.pdf
│   │   ├── L293D.PDF
│   │   └── TB6612FNG.PDF
├── include/                # Header files
├── lib/                    # Project libraries
├── Nano_test_code/         # Arduino Nano / example sketches and tests
│   ├── line_follower/
│   ├── line_follower_github/
│   ├── PID_Bluethooth/
│   ├── Moter_test/
│   └── QTR_test/
├── src/                    # ESP32 firmware (PlatformIO)
│   └── main.cpp
├── test/                   # Unit or hardware test fixtures
├── .gitignore
├── LICENSE
├── platformio.ini          # PlatformIO configuration
└── README.md
```

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) (recommended for ESP32 development)
- Arduino IDE (for the Nano example sketches)
- ESP32 or Arduino Nano board, TB6612 motor driver, QTR8 sensor array, motors and battery

### Quick Start (ESP32 - PlatformIO)

```powershell
cd 'C:\projects\micromouse\micromouse-line-follower'
pio run
pio run -e esp32dev -t upload
```

### Quick Start (Arduino Nano - Arduino IDE)

1. Open Arduino IDE > File > Open > `Nano_test_code/<sketch>/sketch.ino`.
2. Install required libraries in Arduino Library Manager (SparkFun_TB6612, QTRSensors).
3. Select the correct board and port, then upload.

## Tests and Example Sketches (Nano_test_code)

This folder contains simple sketches and tests targeted at Arduino Nano or similar 8-bit boards, useful while developing sensors and motor drivers:

- `line_follower/` — Wireless PID line following sketch that accepts Serial/Bluetooth commands (KP, KI, KD, ILIMIT, MAX, BASE, BLACK/WHITE, STATUS, HELP). Designed for real-time tuning.
- `PID_Bluethooth/` — An advanced Bluetooth serial sketch with state handling (IDLE, CALIBRATING, RUNNING), safe start/stop, and command parsing.
- `line_follower_github/` — A basic line follower sketch for quick functional testing.
- `Moter_test/` — Motor test commands for TB6612 (Forward, Backward, Left, Right, Stop).
- `QTR_test/` — Raw QTR sensor test code printing sensor readings.

For each sketch, the comments at the top list the pin assignments and a summary of commands or behavior.

## Recommended workflow for edits and testing

- Work in smaller branches for major experiments (e.g., `wip/pid-tuning`).
- Keep `main` stable and push incremental changes with descriptive commit messages.
- Use a separate branch for normalization (CRLF/LF) if you want to reformat many files.

## Contributing

Contributions are welcome — please open issues or PRs and use the `wip/*` naming convention for work-in-progress branches.

## License

This project is licensed under the terms specified in `LICENSE`.

---

**Note**: This project is actively developed. The documentation and code will be updated frequently.

