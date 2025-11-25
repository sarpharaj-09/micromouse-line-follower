# Micromouse Firmware

🚧 **Project status:** In development

This folder contains the PlatformIO firmware and project files for the Micromouse line follower. This README documents **only** the files and folders inside this `micromouse/` directory.

## Folder Structure

```
micromouse/
├── .github/                   # GitHub workflows (CI for this folder)
├── .vscode/                   # VS Code/PlatformIO settings
├── .pio/                      # PlatformIO build artifacts (auto-generated)
├── docs/                      # Hardware datasheets and documentation
│   └── component_datasheet/
├── include/                   # Header files and configuration headers
├── lib/                       # Local libraries for this project
├── platformio.ini             # PlatformIO project configuration
├── src/                       # Firmware source code
│   └── main.cpp
├── test/                      # Hardware or unit tests
├── LICENSE
└── README.md                  # This file
```

## Overview

Firmware that reads a QTR sensor array and controls motors through a TB6612 driver using PID corrections. The firmware is designed for line-following robots (micromouse) and supports multiple microcontroller platforms.

## Prerequisites

- [PlatformIO](https://platformio.org/) (required for building and uploading)
- Compatible development board (check `platformio.ini` for supported boards)
- TB6612 motor driver
- QTR sensor array
- Motors and power supply

## Quick Start

From the `micromouse/` folder, build and upload to your board:

```powershell
cd path/to/micromouse-line-follower/micromouse
pio run                     # Build the project
pio run -e <env> -t upload  # Upload to board
```

Replace `<env>` with your specific board environment defined in `platformio.ini` (e.g., `esp32dev`, `nanoatmega328`, etc.).

## Dependencies

Dependencies are declared in `platformio.ini` and will be installed automatically by PlatformIO. Current dependencies include:

- `madhephaestus/ESP32Servo@^0.12.0`

Additional libraries will be listed in `platformio.ini` under `lib_deps`.

## Hardware & Pinouts

⚠️ **Important:** Check the top of `src/main.cpp` for pin definitions used by the firmware (sensors, motors, PWM channels, etc.). Always verify pinouts against your wiring before powering up hardware.

### Safety Notes

- Use a common ground between microcontroller and motor driver
- Keep motor currents within safe limits
- Protect against overcurrent and overheating
- Double-check all connections before applying power

## Configuration

Configuration headers and settings are located in the `include/` directory. Modify these files to customize sensor calibration, PID parameters, and motor control settings.

## Testing

Hardware and unit tests for this project are located in the `test/` directory. Run tests using:

```powershell
pio test
```

## Documentation

Hardware datasheets and component documentation are stored in `docs/component_datasheet/`. Refer to these when working with specific hardware components.

## Development Workflow

- Create feature branches for experiments (e.g., `wip/pid-tuning`, `feature/bluetooth-control`)
- Keep `main` stable with tested, working code
- Use descriptive commit messages
- Test thoroughly before merging to `main`

## Project Scope

**Note:** This README only covers the firmware in the `micromouse/` folder. Example sketches and test code for other platforms are kept in the repository root under `testing_phase_code/` and are intentionally not documented here.

## Contributing

Contributions are welcome! Please:

- Open issues for bugs or feature requests
- Submit pull requests with clear descriptions
- Use `wip/*` branch naming for work-in-progress changes
- Follow existing code style and conventions

## License

See the top-level `LICENSE` file in the repository root for license details.

---


**Note**: This project is actively developed. The documentation and code will be updated frequently.


**For questions or issues specific to this ESP32 firmware, please open an issue in the repository.**


