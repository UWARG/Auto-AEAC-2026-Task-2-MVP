# Auto-AEAC-2026-Task-2-MVP

## Groundside Extinguish System

This repository contains the groundside implementation for receiving and displaying extinguish status messages from the airside drone system.

### Quick Start

1. Install dependencies:
   ```bash
   pip install pymavlink
   ```

2. Run the groundside station:
   ```bash
   python -m groundside.main
   ```

### Documentation

**📚 [DOCUMENTATION_INDEX.md](DOCUMENTATION_INDEX.md)** - Complete documentation guide and navigation

**Getting Started:**
- **[GETTING_STARTED.md](GETTING_STARTED.md)** - Step-by-step setup and usage guide ⭐ **START HERE**
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Quick reference card for common tasks

**Problem Solving:**
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions

**Technical Details:**
- **[IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)** - Implementation details and code changes
- **[EXTINGUISH_SYSTEM_DOCUMENTATION.md](EXTINGUISH_SYSTEM_DOCUMENTATION.md)** - Comprehensive technical documentation

### Features

- ✅ Receives extinguish status messages via MAVLink STATUSTEXT
- ✅ Real-time status display with success/failure indicators
- ✅ Mission statistics tracking (successful/failed attempts)
- ✅ Mission summary on exit
- ✅ Target description generation

### Project Structure

```
.
├── util.py                          # Utility classes and constants
├── groundside/
│   ├── __init__.py                  # Package initialization
│   ├── building.py                  # Building geometry class
│   ├── mavlink_comm.py              # MAVLink receiver and parser
│   └── main.py                      # Main control loop
├── IMPLEMENTATION_GUIDE.md          # Quick start guide
└── EXTINGUISH_SYSTEM_DOCUMENTATION.md  # Technical documentation
```

### Message Format

Extinguish status messages follow the format:
```
e_{target_id}_{lat}_{lon}_{alt}_{colour}_{status}
```

Example: `e_0_43.123_-80.456_10.5_RED_SUCCESS`

### Configuration

MAVLink settings can be configured in `util.py`:
- `AIRSIDE_COMPONENT_ID = 191`
- `MAVLINK_TCP_HOST = "127.0.0.1"`
- `MAVLINK_TCP_PORT = 5760`
