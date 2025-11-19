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
├── docs/
│   ├── component_datasheet/
│   │   ├── esp32_datasheet_en.pdf
│   │   ├── L293D.PDF
│   │   └── TB6612FNG.PDF
├── include/          # Header files
├── lib/              # Project libraries
├── src/
│   └── main.cpp      # Main application code
├── test/             # Test files
├── .gitignore
├── LICENSE
├── platformio.ini    # PlatformIO configuration
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

## Contributing

As this project is in early development, contributions and suggestions are welcome! Feel free to open issues or submit pull requests.

## License

This project is licensed under the terms specified in the LICENSE file.

## Author

*Project started: 2025*

---

**Note**: This is an active development project. Features and documentation will be updated as the project progresses.