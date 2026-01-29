# Quick Reference Card - Extinguish System

## Installation

```bash
pip install pymavlink
```

## Running

```bash
python -m groundside.main
```

## Exit

Press `Ctrl+C` to stop and view mission summary

## Message Format

```
e_{target_id}_{lat}_{lon}_{alt}_{colour}_{status}
```

**Example:**
```
e_0_43.123_-80.456_10.5_RED_SUCCESS
```

## Configuration

Edit `util.py`:

```python
AIRSIDE_COMPONENT_ID = 191          # Component ID filter
MAVLINK_TCP_HOST = "127.0.0.1"      # TCP host
MAVLINK_TCP_PORT = 5760             # TCP port
```

## Message Prefixes

| Prefix | Type | Format |
|--------|------|--------|
| `b_` | Building corner | `b_lat_lon_alt` |
| `t_` | Target detection | `t_lat_lon_alt_colour` |
| `a_` | Acknowledgement | `a_message` |
| `e_` | Extinguish status | `e_target_id_lat_lon_alt_colour_status` |

## Status Values

- `SUCCESS` - Extinguish attempt succeeded
- `FAILED` - Extinguish attempt failed

## Colour Values

- `RED`
- `GREEN`
- `BLUE`
- `WHITE`

## File Structure

```
.
├── util.py                    # Constants and Coordinate class
├── groundside/
│   ├── building.py           # Building geometry
│   ├── mavlink_comm.py       # MAVLink receiver/parser
│   └── main.py               # Main loop
└── [documentation files]
```

## Key Functions

### Parsing (`mavlink_comm.py`)

```python
def _parse_extinguish_status(text: str) -> dict | None:
    """Parse extinguish status message."""
```

### Display (`main.py`)

```python
elif msg_type == 'extinguish_status':
    # Handle extinguish status
    # Update statistics
    # Display status
```

## Statistics Dictionary

```python
extinguish_stats = {
    'total_attempted': 0,
    'successful': 0,
    'failed': 0,
    'targets': []
}
```

## Expected Output

### Status Message
```
================================================================================
EXTINGUISH STATUS ✅: SUCCESS
Target ID: 0
Location: (43.123, -80.456, 10.5)
Colour: RED
Description: ...

Statistics: 1/1 successful
================================================================================
```

### Mission Summary
```
================================================================================
EXTINGUISHING MISSION SUMMARY
================================================================================
Total Targets Attempted: 5
Successful: 4
Failed: 1
Success Rate: 80.0%
================================================================================
```

## Troubleshooting

| Issue | Quick Fix |
|-------|-----------|
| Connection failed | Check airside running, verify network |
| No messages | Check component ID (191), verify format |
| Parsing errors | Verify message format: 6 parts after `e_` |
| Stats not updating | Check message handler is called |

## Testing

```python
from groundside.mavlink_comm import MavlinkReceiver

receiver = MavlinkReceiver.__new__(MavlinkReceiver)
result = receiver._parse_extinguish_status("e_0_43.123_-80.456_10.5_RED_SUCCESS")
```

## Logging Levels

- `DEBUG` - Detailed message info
- `INFO` - Normal operations
- `WARNING` - Invalid formats
- `ERROR` - Parsing failures

## Common Commands

```bash
# Run groundside station
python -m groundside.main

# Test parsing (create test script)
python test_messages.py

# Check Python version
python --version

# Check pymavlink installed
pip show pymavlink

# Test network connectivity (Windows)
Test-NetConnection -ComputerName 127.0.0.1 -Port 5760
```

## Documentation Files

- **[GETTING_STARTED.md](GETTING_STARTED.md)** - Step-by-step setup
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions
- **[IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)** - Implementation details
- **[EXTINGUISH_SYSTEM_DOCUMENTATION.md](EXTINGUISH_SYSTEM_DOCUMENTATION.md)** - Technical reference

## Quick Checklist

- [ ] Dependencies installed (`pymavlink`)
- [ ] MAVLink settings configured (`util.py`)
- [ ] Airside system running
- [ ] Network connectivity verified
- [ ] Component ID matches (191)
- [ ] Message format correct (`e_target_id_lat_lon_alt_colour_status`)

## Support

1. Check [TROUBLESHOOTING.md](TROUBLESHOOTING.md)
2. Review [GETTING_STARTED.md](GETTING_STARTED.md)
3. Consult [EXTINGUISH_SYSTEM_DOCUMENTATION.md](EXTINGUISH_SYSTEM_DOCUMENTATION.md)
