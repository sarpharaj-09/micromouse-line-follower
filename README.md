# Micromouse Line Follower

🚧 **Project Status: In Development** 🚧

This project is currently in its early stages of development.

## Overview

A micromouse robot designed to follow lines autonomously using ESP32/Arduino NANO microcontroller. This project aims to create an intelligent line-following robot capable of navigating predefined paths.

## Hardware

- **Microcontroller**: ESP32/Arduino NANO
- **Motor Driver**: TB6612FNG
- **Display**: L293D-based display system

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
├── Nano_test_code/         # Arduino Nano / test sketches
│   ├── line_follower/
│   ├── line_follower_github/
│   ├── PID_Bluethooth/
│   ├── Moter_test/
│   └── QTR_test/
├── src/
│   └── main.cpp            # Main application code (ESP32)
├── test/                   # Unit or hardware test fixtures
├── .gitignore
├── LICENSE
├── platformio.ini          # PlatformIO configuration
└── README.md
```

## Getting Started

### Prerequisites

- [PlatformIO](https://platformio.org/) IDE or CLI
- ESP32 development board
- Required components (see datasheets in `docs/component_datasheet/`)

### Installation

1. Clone the repository:
```bash
git clone https://github.com/sarpharaj-09/micromouse-line-follower.git
cd micromouse-line-follower
```

2. Open the project in PlatformIO

3. Build the project:
```bash
pio run
```

4. Upload to ESP32:
```bash
pio run --target upload
```

## Documentation

Datasheets for all components are available in the `docs/component_datasheet/` directory:
- ESP32 microcontroller datasheet
- TB6612FNG motor driver datasheet
- L293D motor driver IC datasheet

## Roadmap

- [ ] Basic line detection
- [ ] Motor control implementation
- [ ] PID controller for smooth following
- [ ] Obstacle detection (future enhancement)
- [ ] Speed optimization

## Tests & Example Sketches (Nano_test_code)

I have added a `Nano_test_code/` folder with useful example sketches and test programs for Arduino Nano/ESP32 users. These include hardware tests, QTR sensor tests, and two line follower sketches — one of which supports wireless (serial/Bluetooth) PID tuning.

Folder contents:
- `Nano_test_code/line_follower/` — A line follower sketch with wireless PID tuning support. Commands can be sent over Serial/Bluetooth to set KP, KI, KD, integral limits, speeds, and line color. Example commands: `KP:0.15`, `KI:0.0001`, `KD:5.0`, `ILIMIT:1000`, `MAX:180`, `BASE:150`, `BLACK` / `WHITE`, `STATUS`, `HELP`.
- `Nano_test_code/line_follower_github/` — Original line follower code used for quick testing without wireless tuning.
- `Nano_test_code/PID_Bluethooth/` — A more advanced sketch that implements start/stop states, improved calibration handling, and robust Bluetooth command parsing for PID and system control.
- `Nano_test_code/Moter_test/` — Simple motor test (forward, backward, left, right) using SparkFun TB6612.
- `Nano_test_code/QTR_test/` — QTR-8A sensor test example that prints raw sensor readings.

How to use these sketches:
1. Open the appropriate `.ino` in the Arduino IDE or use PlatformIO in VSCode.
2. Install dependencies:
- SparkFun TB6612 library (SparkFun_TB6612)
- QTRSensors library (Pololu QTRSensors or QTRSensors)
3. Connect your microcontroller and sensors as per the pin definitions in the sketch. The sketches use `INPUT_PULLUP` for button pins (connect button between the pin and GND).
4. Use the Arduino Serial Monitor (or a Bluetooth terminal app for wireless sketches) at `9600` baud to send commands and read feedback.

Notes & Tips:
- The wireless PID sketches can accept very small and very large floating values — they try to parse floats robustly but be careful with extremely large numbers and motor limits.
- If you test on Windows and you see LF/CRLF warnings, consider adding a `.gitattributes` or adjusting `core.autocrlf` (see earlier README sections for guidance).
- The test code is intended for development and testing; use a stable branch for production firmware and keep these sketches in the `Nano_test_code/` tree for experiments.

If you'd like, I can:
- Add short README files inside each `Nano_test_code/*` folder describing exact pinouts and sample commands, or
- Create smaller example sketches focused on specific features (e.g., only PID tuning or a demonstration log script).

## Contributing

As this project is in early development, contributions and suggestions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the terms specified in the LICENSE file.

## Author

*Project started: 2025*

---

**Note**: This is an active development project. Features and documentation will be updated as the project progresses.

