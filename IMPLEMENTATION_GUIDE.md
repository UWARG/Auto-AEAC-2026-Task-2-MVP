# Implementation Guide - Extinguish System

## Quick Start

This guide provides step-by-step instructions for implementing and using the extinguish status tracking system.

## What Was Implemented

### Files Created

1. **`util.py`** - Core utilities
   - `Coordinate` dataclass for 3D coordinates
   - MAVLink constants (component ID, TCP settings)

2. **`groundside/building.py`** - Building representation
   - Stores building corners
   - Generates target descriptions

3. **`groundside/mavlink_comm.py`** - MAVLink communication
   - Receives and parses STATUSTEXT messages
   - **NEW:** Extinguish status parsing (`_parse_extinguish_status()`)
   - Routes messages by prefix (`b_`, `t_`, `a_`, `e_`)

4. **`groundside/main.py`** - Main application
   - Message processing loop
   - **NEW:** Extinguish status handling
   - **NEW:** Statistics tracking
   - **NEW:** Mission summary on exit

## Key Features

### 1. Extinguish Status Parsing

The system now parses messages with the `e_` prefix:
- Format: `e_target_id_lat_lon_alt_colour_status`
- Validates message format
- Extracts all required fields
- Returns structured data

### 2. Real-time Status Display

When an extinguish status is received:
- Shows success/failure with emoji (✅/❌)
- Displays target ID, location, and colour
- Includes target description for context
- Shows current statistics (successful/total)

### 3. Statistics Tracking

Tracks throughout the mission:
- Total attempts
- Successful attempts
- Failed attempts
- Individual target records

### 4. Mission Summary

On program exit (Ctrl+C):
- Displays final statistics
- Calculates success rate
- Shows complete mission overview

## How to Use

### Step 1: Install Dependencies

```bash
pip install pymavlink
```

**Verify installation:**
```bash
python -c "import pymavlink; print('✓ pymavlink installed')"
```

### Step 2: Configure (if needed)

Check `util.py` for MAVLink settings. Defaults are usually fine:
- Component ID: `191`
- Host: `127.0.0.1`
- Port: `5760`

### Step 3: Run the Groundside Station

```bash
python -m groundside.main
```

**Expected startup:**
```
2024-01-28 19:00:00,000 - INFO - Starting groundside station...
2024-01-28 19:00:00,100 - INFO - Heartbeat received from system 1, component 191
2024-01-28 19:00:00,101 - INFO - Ground station connected to drone
2024-01-28 19:00:00,102 - INFO - Listening for drone messages...
```

### Step 4: Monitor Output

The system will:
1. Connect to MAVLink (retries if connection fails)
2. Listen for messages
3. Display target detections
4. Display extinguish status updates
5. Show statistics after each extinguish attempt
6. Print mission summary on exit (Ctrl+C)

## Message Flow

```
Airside System
    ↓
MAVLink STATUSTEXT (e_target_id_lat_lon_alt_colour_status)
    ↓
MavlinkReceiver.process_messages()
    ↓
_parse_extinguish_status()
    ↓
main.py message handler
    ↓
Update statistics & display
```

## Testing the Implementation

### Manual Testing

1. **Start the groundside station:**
   ```bash
   python -m groundside.main
   ```

2. **Send test messages** (if you have a test harness):
   - Success: `e_0_43.123_-80.456_10.5_RED_SUCCESS`
   - Failure: `e_1_43.125_-80.458_0.0_GREEN_FAILED`

3. **Verify output:**
   - Check that status messages are displayed
   - Verify statistics update correctly
   - Test exit summary (Ctrl+C)

### Unit Testing

Create test file `test_extinguish.py`:

```python
from groundside.mavlink_comm import MavlinkReceiver

def test_parse_extinguish_status():
    receiver = MavlinkReceiver()
    
    # Test valid success message
    text = "e_0_43.123_-80.456_10.5_RED_SUCCESS"
    result = receiver._parse_extinguish_status(text)
    assert result is not None
    assert result['type'] == 'extinguish_status'
    assert result['data']['target_id'] == 0
    assert result['data']['status'] == "SUCCESS"
    
    # Test valid failed message
    text = "e_1_43.125_-80.458_0.0_GREEN_FAILED"
    result = receiver._parse_extinguish_status(text)
    assert result['data']['status'] == "FAILED"
    
    # Test invalid format
    text = "e_0_43.123_-80.456"
    result = receiver._parse_extinguish_status(text)
    assert result is None

if __name__ == "__main__":
    test_parse_extinguish_status()
    print("All tests passed!")
```

## Integration Checklist

- [x] Create `util.py` with Coordinate and constants
- [x] Create `groundside/building.py` with Building class
- [x] Create `groundside/mavlink_comm.py` with message parsing
- [x] Add `_parse_extinguish_status()` method
- [x] Add `e_` prefix routing in `process_messages()`
- [x] Create `groundside/main.py` with main loop
- [x] Add statistics tracking dictionary
- [x] Add extinguish status message handler
- [x] Add mission summary on exit
- [x] Update docstrings
- [x] Create documentation

## Next Steps

1. **Test with Airside System:**
   - Ensure airside sends messages in correct format
   - Verify component ID filtering works
   - Test end-to-end message flow

2. **Enhance Target Descriptions:**
   - Implement detailed spatial calculations
   - Add distance-to-wall calculations
   - Improve building face detection

3. **Add Features (Optional):**
   - Statistics persistence to file
   - Real-time dashboard
   - Alert system for failures

## Troubleshooting

### Issue: No messages received

**Solution:**
- Check MAVLink connection settings in `util.py`
- Verify airside is sending to correct port
- Check component ID matches (should be 191)

### Issue: Parsing errors

**Solution:**
- Verify message format matches specification
- Check log output for detailed errors
- Ensure all numeric values are valid

### Issue: Statistics not updating

**Solution:**
- Verify message handler is called
- Check that `extinguish_status` type is handled
- Review log files for errors

## Code Changes Summary

### `groundside/mavlink_comm.py`

**Added:**
- `e_` prefix check in `process_messages()`
- `_parse_extinguish_status()` method
- Updated docstring to include `extinguish_status` type

**Changed:**
- Message routing logic to include extinguish status

### `groundside/main.py`

**Added:**
- Statistics tracking dictionary
- Extinguish status message handler
- Mission summary on exit
- Updated module docstring

**Changed:**
- KeyboardInterrupt handler to show summary
- Message processing loop to handle extinguish status

## Practical Examples

### Example 1: Testing Message Parsing

Create `test_parsing.py`:

```python
from groundside.mavlink_comm import MavlinkReceiver

# Create receiver instance (without connection)
receiver = MavlinkReceiver.__new__(MavlinkReceiver)

# Test valid messages
test_messages = [
    "e_0_43.123_-80.456_10.5_RED_SUCCESS",
    "e_1_43.125_-80.458_0.0_GREEN_FAILED",
    "e_2_43.130_-80.460_5.2_BLUE_SUCCESS",
]

print("Testing extinguish status parsing...")
for msg in test_messages:
    result = receiver._parse_extinguish_status(msg)
    if result:
        print(f"✅ {msg}")
        print(f"   → Target ID: {result['data']['target_id']}")
        print(f"   → Status: {result['data']['status']}")
    else:
        print(f"❌ {msg} - Failed to parse")
```

### Example 2: Monitoring Statistics

The statistics dictionary is updated in real-time. You can access it:

```python
# In main.py, after receiving extinguish status:
print(f"Current success rate: {extinguish_stats['successful']}/{extinguish_stats['total_attempted']}")
```

### Example 3: Custom Logging

Add custom logging for specific events:

```python
# In main.py, in extinguish status handler:
if status == "FAILED":
    logging.warning(f"Extinguish failed for target {target_id} at {coordinate}")
    # Add custom alert logic here
```

## Support

### Documentation Files

- **[GETTING_STARTED.md](GETTING_STARTED.md)** - Complete setup guide ⭐ **START HERE**
- **[QUICK_REFERENCE.md](QUICK_REFERENCE.md)** - Quick reference card
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - Common issues and solutions
- **[EXTINGUISH_SYSTEM_DOCUMENTATION.md](EXTINGUISH_SYSTEM_DOCUMENTATION.md)** - Technical reference

### Code Files

- `groundside/mavlink_comm.py` - Parsing logic
- `groundside/main.py` - Display and statistics
- `util.py` - Constants and utilities
