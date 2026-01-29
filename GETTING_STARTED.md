# Getting Started - Extinguish System

## Prerequisites

Before you begin, ensure you have:

- Python 3.8 or higher installed
- Access to the airside system or a MAVLink test harness
- Network connectivity to the MAVLink TCP port (default: 127.0.0.1:5760)

## Step-by-Step Setup

### 1. Install Python Dependencies

Open a terminal in the project directory and run:

```bash
pip install pymavlink
```

**Verify installation:**
```bash
python -c "import pymavlink; print('pymavlink installed successfully')"
```

### 2. Configure MAVLink Connection (if needed)

Edit `util.py` if your MAVLink connection uses different settings:

```python
# Default settings (usually fine)
AIRSIDE_COMPONENT_ID = 191          # Component ID to filter messages
MAVLINK_TCP_HOST = "127.0.0.1"      # TCP host for MAVLink connection
MAVLINK_TCP_PORT = 5760             # TCP port for MAVLink connection
MAVLINK_RECEIVE_TIMEOUT_SEC = 1.0   # Timeout for receiving messages
```

**When to change:**
- If airside uses a different component ID (not 191)
- If MAVLink is on a different host/IP address
- If MAVLink uses a different port

### 3. Run the Groundside Station

Start the groundside station:

```bash
python -m groundside.main
```

**Expected startup output:**
```
2024-01-28 19:00:00,000 - INFO - Starting groundside station...
2024-01-28 19:00:00,100 - INFO - Heartbeat received from system 1, component 191
2024-01-28 19:00:00,101 - INFO - Ground station connected to drone
2024-01-28 19:00:00,102 - INFO - Listening for drone messages...
```

**If connection fails:**
- The system will retry automatically every second
- Check that the airside system is running
- Verify network connectivity
- Check firewall settings

### 4. Monitor Output

The system will display:

1. **Building corners** (when received):
   ```
   2024-01-28 19:00:05,123 - INFO - Parsed building corner: (43.123, -80.456, 0.0)
   2024-01-28 19:00:05,124 - INFO - Stored building corner 1: (43.123, -80.456, 0.0)
   ```

2. **Target detections**:
   ```
   ================================================================================
   TARGET DETECTED:
   Target is at (43.123, -80.456, 10.5). Building has 4 corners mapped. The colour is RED
   ================================================================================
   ```

3. **Extinguish status** (main feature):
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

### 5. Exit and View Summary

Press `Ctrl+C` to stop the station and view the mission summary:

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

## Testing Without Airside System

If you want to test the system without the full airside setup, you can create a simple test script:

### Create `test_messages.py`:

```python
"""Simple test script to verify extinguish status parsing."""

from groundside.mavlink_comm import MavlinkReceiver

def test_extinguish_parsing():
    """Test the extinguish status parsing function."""
    receiver = MavlinkReceiver.__new__(MavlinkReceiver)  # Create without connection
    
    # Test cases
    test_cases = [
        ("e_0_43.123_-80.456_10.5_RED_SUCCESS", True, "SUCCESS"),
        ("e_1_43.125_-80.458_0.0_GREEN_FAILED", True, "FAILED"),
        ("e_2_43.130_-80.460_5.2_BLUE_SUCCESS", True, "SUCCESS"),
        ("e_0_43.123_-80.456", False, None),  # Invalid: too few parts
        ("e_0_43.123_-80.456_10.5_RED_SUCCESS_EXTRA", False, None),  # Invalid: too many parts
    ]
    
    print("Testing extinguish status parsing...")
    passed = 0
    failed = 0
    
    for text, should_parse, expected_status in test_cases:
        result = receiver._parse_extinguish_status(text)
        
        if should_parse:
            if result is not None and result['data']['status'] == expected_status:
                print(f"✅ PASS: {text}")
                passed += 1
            else:
                print(f"❌ FAIL: {text} - Expected status {expected_status}, got {result}")
                failed += 1
        else:
            if result is None:
                print(f"✅ PASS: {text} (correctly rejected)")
                passed += 1
            else:
                print(f"❌ FAIL: {text} - Should be rejected but was parsed")
                failed += 1
    
    print(f"\nResults: {passed} passed, {failed} failed")
    return failed == 0

if __name__ == "__main__":
    success = test_extinguish_parsing()
    exit(0 if success else 1)
```

**Run the test:**
```bash
python test_messages.py
```

## Common Workflows

### Workflow 1: Full Mission

1. Start groundside station: `python -m groundside.main`
2. Wait for building corners (4 corners)
3. Wait for target detections
4. Monitor extinguish status messages
5. Press Ctrl+C when mission complete
6. Review mission summary

### Workflow 2: Testing Parsing Only

1. Run test script: `python test_messages.py`
2. Verify all test cases pass
3. Check output for any parsing errors

### Workflow 3: Debugging Connection Issues

1. Check MAVLink settings in `util.py`
2. Verify airside is running
3. Test network connectivity: `telnet 127.0.0.1 5760` (or use `nc`)
4. Check logs for connection errors
5. Verify component ID matches (should be 191)

## Understanding the Output

### Status Messages

- **✅ SUCCESS**: Target was successfully extinguished
- **❌ FAILED**: Extinguish attempt failed

### Statistics Format

```
Statistics: X/Y successful
```

Where:
- `X` = Number of successful attempts
- `Y` = Total number of attempts

### Mission Summary

Shows:
- Total targets attempted
- Number successful
- Number failed
- Success rate percentage

## Next Steps

1. **Integrate with Airside:**
   - Ensure airside sends messages in format: `e_target_id_lat_lon_alt_colour_status`
   - Verify component ID matches (191)
   - Test end-to-end message flow

2. **Customize Display:**
   - Modify output format in `groundside/main.py`
   - Add custom logging
   - Enhance target descriptions

3. **Add Features:**
   - Statistics persistence
   - Real-time dashboard
   - Alert system

## Troubleshooting Quick Reference

| Issue | Solution |
|-------|----------|
| "Failed to connect" | Check airside is running, verify network |
| "No messages received" | Check component ID, verify message format |
| "Parsing errors" | Verify message format matches spec |
| "Statistics not updating" | Check message handler is called |

For detailed troubleshooting, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md)

## Additional Resources

- **[IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md)** - Implementation details
- **[EXTINGUISH_SYSTEM_DOCUMENTATION.md](EXTINGUISH_SYSTEM_DOCUMENTATION.md)** - Technical documentation
- **[README.md](README.md)** - Project overview
