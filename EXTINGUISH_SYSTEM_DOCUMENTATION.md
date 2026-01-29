# Extinguish System Implementation Documentation

## Overview

This document describes the groundside extinguish status tracking and display system implementation. The system receives extinguish status messages from the airside system via MAVLink STATUSTEXT and displays real-time feedback with mission statistics.

## System Architecture

### Components

1. **`util.py`** - Utility classes and constants
   - `Coordinate` dataclass for 3D coordinates
   - MAVLink communication constants

2. **`groundside/building.py`** - Building geometry representation
   - Stores building corners
   - Generates target descriptions

3. **`groundside/mavlink_comm.py`** - MAVLink message receiver and parser
   - Receives STATUSTEXT messages from drone
   - Parses building corners, targets, acknowledgements, and extinguish status
   - Filters messages by component ID

4. **`groundside/main.py`** - Main control loop
   - Processes incoming messages
   - Displays target detections and extinguish status
   - Tracks mission statistics
   - Prints summary on exit

## Message Format

### Extinguish Status Messages

Airside sends extinguish status messages with the prefix `e_` in the following format:

```
e_{target_id}_{lat}_{lon}_{alt}_{colour}_{status}
```

**Format Details:**
- `target_id`: Integer ID of the target (0, 1, 2, ...)
- `lat`: Target latitude (float)
- `lon`: Target longitude (float)
- `alt`: Target altitude in meters (float)
- `colour`: Target color name (RED, GREEN, BLUE, WHITE)
- `status`: Either "SUCCESS" or "FAILED"

**Examples:**
```
e_0_43.123_-80.456_10.5_RED_SUCCESS
e_1_43.125_-80.458_0.0_GREEN_FAILED
e_2_43.130_-80.460_5.2_BLUE_SUCCESS
```

### Message Transmission

- Messages are sent via MAVLink STATUSTEXT
- Component ID filtering: Only messages from `AIRSIDE_COMPONENT_ID = 191` are processed
- Messages are sent after each extinguishing attempt (success or failure)

## Implementation Details

### 1. Message Parsing (`groundside/mavlink_comm.py`)

#### `process_messages()` Method

The method routes messages based on prefix:
- `b_` → Building corner parsing
- `t_` → Target parsing
- `a_` → Acknowledgement parsing
- `e_` → **Extinguish status parsing** (NEW)

#### `_parse_extinguish_status()` Method

**Functionality:**
- Removes `e_` prefix
- Splits message into 6 parts
- Validates format (must have exactly 6 parts)
- Parses numeric values (target_id, lat, lon, alt)
- Extracts colour and status strings
- Creates Coordinate object
- Returns structured dictionary

**Error Handling:**
- Invalid format: Logs warning, returns None
- Invalid numeric values: Logs error, returns None
- Missing fields: Logs error, returns None

**Return Format:**
```python
{
    'type': 'extinguish_status',
    'data': {
        'target_id': int,
        'coordinate': Coordinate,
        'colour': str,
        'status': str  # "SUCCESS" or "FAILED"
    }
}
```

### 2. Status Display and Tracking (`groundside/main.py`)

#### Statistics Tracking

The system maintains statistics in a dictionary:
```python
extinguish_stats = {
    'total_attempted': 0,    # Total extinguishing attempts
    'successful': 0,          # Successful attempts
    'failed': 0,             # Failed attempts
    'targets': []            # List of all extinguished targets
}
```

#### Message Handler

When an extinguish status message is received:

1. **Update Statistics:**
   - Increment `total_attempted`
   - Increment `successful` or `failed` based on status

2. **Store Target Info:**
   - Append target details to `targets` list

3. **Generate Description:**
   - Use `Building.generate_target_description()` for context

4. **Display Status:**
   - Print formatted status message with emoji
   - Show target ID, location, colour, and description
   - Display current statistics

5. **Log Event:**
   - Log extinguish status to file

#### Mission Summary

On program exit (KeyboardInterrupt):
- Print final statistics summary
- Calculate and display success rate
- Show total attempts, successes, and failures

## Usage

### Running the Groundside Station

```bash
python -m groundside.main
```

Or:

```bash
cd groundside
python main.py
```

### Expected Output

#### Individual Extinguish Status Display

When an extinguish status message is received:

```
================================================================================
EXTINGUISH STATUS ✅: SUCCESS
Target ID: 0
Location: (43.123, -80.456, 10.5)
Colour: RED
Description: Target is at (43.123, -80.456, 10.5). Building has 4 corners mapped. The colour is RED

Statistics: 1/1 successful
================================================================================
```

For failed attempts:

```
================================================================================
EXTINGUISH STATUS ❌: FAILED
Target ID: 1
Location: (43.125, -80.458, 0.0)
Colour: GREEN
Description: Target is at (43.125, -80.458, 0.0). Building has 4 corners mapped. The colour is GREEN

Statistics: 1/2 successful
================================================================================
```

#### Mission Summary (on exit)

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

## Dependencies

### Required Python Packages

- `pymavlink` - MAVLink protocol implementation

Install with:
```bash
pip install pymavlink
```

### Standard Library Modules

- `logging` - Logging functionality
- `time` - Time utilities
- `dataclasses` - Coordinate dataclass

## Configuration

### MAVLink Connection Settings (`util.py`)

```python
AIRSIDE_COMPONENT_ID = 191          # Component ID to filter messages
MAVLINK_TCP_HOST = "127.0.0.1"      # TCP host for MAVLink connection
MAVLINK_TCP_PORT = 5760             # TCP port for MAVLink connection
MAVLINK_RECEIVE_TIMEOUT_SEC = 1.0   # Timeout for receiving messages
```

### Logging Configuration (`groundside/main.py`)

```python
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
```

## Testing

### Unit Testing

Test the parsing function with various inputs:

1. **Valid success message:**
   ```python
   text = "e_0_43.123_-80.456_10.5_RED_SUCCESS"
   # Expected: Dictionary with correct target_id, coordinate, colour, and status
   ```

2. **Valid failed message:**
   ```python
   text = "e_1_43.125_-80.458_0.0_GREEN_FAILED"
   # Expected: Dictionary with status="FAILED"
   ```

3. **Invalid format (too few parts):**
   ```python
   text = "e_0_43.123_-80.456"
   # Expected: None, warning logged
   ```

4. **Invalid format (too many parts):**
   ```python
   text = "e_0_43.123_-80.456_10.5_RED_SUCCESS_EXTRA"
   # Expected: None, warning logged
   ```

5. **Invalid numeric values:**
   ```python
   text = "e_abc_43.123_-80.456_10.5_RED_SUCCESS"
   # Expected: None, error logged
   ```

### Integration Testing

1. **Message Flow:**
   - Verify messages from airside are received and parsed correctly
   - Verify statistics are updated correctly
   - Verify display output matches expected format

2. **Statistics Tracking:**
   - Test with multiple success messages
   - Test with multiple failed messages
   - Test with mixed success/failure
   - Verify final summary calculation is correct

3. **Edge Cases:**
   - Empty statistics (no messages received)
   - All successes
   - All failures
   - Rapid message sequence

## Error Handling

### Parsing Errors

- **Invalid format:** Logs warning and returns None (message is ignored)
- **Invalid numeric values:** Logs error and returns None
- **Missing fields:** Logs error and returns None

### Display Errors

- **Missing building data:** Target description will show "Building not fully mapped"
- **Invalid coordinate:** Coordinate will display as-is (may be (0,0,0) if parsing failed)

## Integration Points

### With Existing Code

- Uses existing `Building.generate_target_description()` for context
- Follows same message parsing pattern as building corners and targets
- Uses same logging format and level as other message handlers
- Integrates seamlessly with existing message loop

### With Airside System

- Receives messages sent by airside's `send_extinguish_status_to_ground()` method
- Message format matches exactly what airside sends
- No changes needed to airside code

## Code Quality

### Type Hints

All functions use proper type hints:
- `-> dict | None` for parsing methods
- `-> bool` for connection methods
- `-> str` for description generation

### Docstrings

All methods have comprehensive docstrings explaining:
- Purpose
- Parameters
- Return values
- Behavior

### Logging

Appropriate log levels:
- `INFO` - Normal operations (parsing, status updates)
- `WARNING` - Invalid message formats
- `ERROR` - Parsing failures, connection errors
- `DEBUG` - Detailed message information

### Code Style

- Follows PEP 8 style guidelines
- Consistent naming conventions
- Proper error handling with try/except blocks
- Clear variable names

## File Structure

```
.
├── util.py                          # Utility classes and constants
├── groundside/
│   ├── __init__.py                  # Package initialization
│   ├── building.py                  # Building geometry class
│   ├── mavlink_comm.py              # MAVLink receiver and parser
│   └── main.py                      # Main control loop
└── EXTINGUISH_SYSTEM_DOCUMENTATION.md  # This file
```

## Future Enhancements

Potential improvements:

1. **Enhanced Target Descriptions:**
   - Calculate actual distances to building walls
   - Determine which face of building target is on
   - More detailed spatial descriptions

2. **Statistics Persistence:**
   - Save statistics to file
   - Load previous statistics on startup
   - Historical tracking across missions

3. **Real-time Dashboard:**
   - Web-based status display
   - Live statistics updates
   - Mission progress visualization

4. **Alert System:**
   - Notifications for failed attempts
   - Success rate thresholds
   - Mission completion alerts

## Troubleshooting

### Common Issues

1. **No messages received:**
   - Check MAVLink connection settings
   - Verify airside component ID matches
   - Check network connectivity

2. **Parsing errors:**
   - Verify message format matches specification
   - Check log files for detailed error messages
   - Ensure numeric values are valid floats/integers

3. **Statistics not updating:**
   - Verify message handler is being called
   - Check that extinguish status messages are being parsed correctly
   - Review log output for errors

## Support

For issues or questions:
1. Check log files for error messages
2. Verify message format matches specification
3. Review integration points with airside system
4. Consult this documentation for implementation details
