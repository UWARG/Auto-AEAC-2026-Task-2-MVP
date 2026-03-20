# Troubleshooting Guide - Extinguish System

## Common Issues and Solutions

### Connection Issues

#### Problem: "Failed to connect to drone, retrying..."

**Symptoms:**
- Continuous retry messages
- No heartbeat received
- Connection timeout errors

**Possible Causes:**
1. Airside system is not running
2. Wrong TCP host/port configuration
3. Network connectivity issues
4. Firewall blocking connection

**Solutions:**

1. **Verify airside system is running:**
   ```bash
   # Check if airside process is running
   # (method depends on your system)
   ```

2. **Check MAVLink connection settings:**
   - Open `util.py`
   - Verify `MAVLINK_TCP_HOST` and `MAVLINK_TCP_PORT`
   - Default: `127.0.0.1:5760`

3. **Test network connectivity:**
   ```bash
   # On Linux/Mac:
   telnet 127.0.0.1 5760
   # or
   nc -zv 127.0.0.1 5760
   
   # On Windows PowerShell:
   Test-NetConnection -ComputerName 127.0.0.1 -Port 5760
   ```

4. **Check firewall settings:**
   - Ensure port 5760 is not blocked
   - Check both inbound and outbound rules

5. **Verify airside is listening:**
   - Check airside logs
   - Verify airside is configured to use TCP port 5760

#### Problem: "Heartbeat received but no messages"

**Symptoms:**
- Connection successful
- Heartbeat received
- No STATUSTEXT messages processed

**Possible Causes:**
1. Component ID mismatch
2. Messages not being sent
3. Wrong message format

**Solutions:**

1. **Check component ID:**
   - Verify `AIRSIDE_COMPONENT_ID = 191` in `util.py`
   - Check airside logs for actual component ID
   - Ensure airside sends messages with component ID 191

2. **Verify messages are being sent:**
   - Check airside logs for STATUSTEXT messages
   - Use MAVLink inspector/tool to verify messages
   - Check message frequency

3. **Check message format:**
   - Verify messages start with correct prefix (`e_` for extinguish)
   - Ensure format matches: `e_target_id_lat_lon_alt_colour_status`

### Parsing Issues

#### Problem: "Invalid extinguish status format" warnings

**Symptoms:**
- Warning messages in logs
- Messages not being processed
- Statistics not updating

**Possible Causes:**
1. Incorrect message format
2. Missing fields
3. Extra fields
4. Invalid characters

**Solutions:**

1. **Verify message format:**
   ```
   Expected: e_target_id_lat_lon_alt_colour_status
   Example:  e_0_43.123_-80.456_10.5_RED_SUCCESS
   ```

2. **Check message parts:**
   - Must have exactly 6 parts after `e_` prefix
   - Parts separated by single underscore `_`
   - No extra underscores or spaces

3. **Validate data types:**
   - `target_id`: Must be integer (0, 1, 2, ...)
   - `lat`, `lon`, `alt`: Must be valid floats
   - `colour`: Must be string (RED, GREEN, BLUE, WHITE)
   - `status`: Must be "SUCCESS" or "FAILED"

4. **Check for encoding issues:**
   - Ensure UTF-8 encoding
   - No special characters
   - No null bytes

**Example of correct format:**
```
✅ Correct: e_0_43.123_-80.456_10.5_RED_SUCCESS
✅ Correct: e_1_43.125_-80.458_0.0_GREEN_FAILED
❌ Wrong:   e_0_43.123_-80.456_10.5_RED_SUCCESS_EXTRA
❌ Wrong:   e_0_43.123_-80.456
❌ Wrong:   e_0_43.123_-80.456_10.5_RED
```

#### Problem: "Failed to parse extinguish status" errors

**Symptoms:**
- Error messages in logs
- ValueError or IndexError exceptions
- Messages ignored

**Possible Causes:**
1. Invalid numeric values
2. Missing required fields
3. Type conversion errors

**Solutions:**

1. **Check numeric values:**
   ```python
   # target_id must be integer
   "e_0_..."  # ✅ Correct
   "e_abc_..."  # ❌ Wrong
   
   # lat, lon, alt must be floats
   "e_0_43.123_-80.456_10.5_..."  # ✅ Correct
   "e_0_abc_-80.456_10.5_..."  # ❌ Wrong
   ```

2. **Verify all fields present:**
   - Count underscores: should be 5 underscores in message
   - Check each field is non-empty
   - Ensure no missing values

3. **Check coordinate ranges:**
   - Latitude: -90 to 90
   - Longitude: -180 to 180
   - Altitude: Any float value

### Display Issues

#### Problem: Statistics not updating

**Symptoms:**
- Messages parsed successfully
- No statistics displayed
- Statistics stay at 0/0

**Possible Causes:**
1. Message handler not called
2. Wrong message type
3. Statistics dictionary not initialized

**Solutions:**

1. **Check message handler:**
   - Verify `elif msg_type == 'extinguish_status':` is in main loop
   - Check that handler code is executed
   - Add debug prints to verify execution

2. **Verify message type:**
   - Check parsed message has `type == 'extinguish_status'`
   - Verify message routing works correctly

3. **Check statistics initialization:**
   - Ensure `extinguish_stats` dictionary is created
   - Verify it's in the correct scope
   - Check it's not being reset

#### Problem: Mission summary not showing on exit

**Symptoms:**
- Program exits without summary
- No statistics displayed
- KeyboardInterrupt not handled

**Solutions:**

1. **Verify KeyboardInterrupt handler:**
   - Check `except KeyboardInterrupt:` block exists
   - Ensure summary code is in handler
   - Verify handler is not being bypassed

2. **Check exit method:**
   - Use Ctrl+C to exit (not closing terminal)
   - Verify handler catches interrupt
   - Check for exceptions in handler

### Performance Issues

#### Problem: High CPU usage or slow processing

**Symptoms:**
- System lag
- Messages delayed
- High CPU usage

**Possible Causes:**
1. Too many messages
2. Inefficient processing
3. Blocking operations

**Solutions:**

1. **Check message frequency:**
   - Reduce message rate if possible
   - Batch processing if needed

2. **Optimize processing:**
   - Check for blocking operations
   - Verify timeout settings
   - Review logging frequency

3. **Monitor system resources:**
   - Check CPU usage
   - Monitor memory usage
   - Review system logs

## Debugging Tips

### Enable Debug Logging

Modify `groundside/main.py`:

```python
logging.basicConfig(
    level=logging.DEBUG,  # Change from INFO to DEBUG
    format="%(asctime)s - %(levelname)s - %(message)s"
)
```

This will show:
- All received messages
- Parsing details
- Message routing decisions
- Component ID filtering

### Add Debug Prints

Add temporary debug statements:

```python
# In mavlink_comm.py
logging.debug(f"Received text: {text}")
logging.debug(f"Message parts: {parts}")
logging.debug(f"Parsed result: {result}")

# In main.py
logging.debug(f"Message type: {msg_type}")
logging.debug(f"Message data: {data}")
logging.debug(f"Statistics: {extinguish_stats}")
```

### Test Message Parsing Independently

Create a test script:

```python
from groundside.mavlink_comm import MavlinkReceiver

receiver = MavlinkReceiver.__new__(MavlinkReceiver)
test_msg = "e_0_43.123_-80.456_10.5_RED_SUCCESS"
result = receiver._parse_extinguish_status(test_msg)
print(f"Result: {result}")
```

### Verify Message Flow

Add logging at each step:

1. **Message reception:** Check `process_messages()` receives messages
2. **Component filtering:** Verify component ID check
3. **Prefix routing:** Check message routing to correct parser
4. **Parsing:** Verify parsing succeeds
5. **Handler:** Check handler receives parsed message
6. **Statistics:** Verify statistics update

## Getting Help

### Information to Provide

When reporting issues, include:

1. **Error messages:**
   - Full error traceback
   - Log output
   - Warning messages

2. **System information:**
   - Python version: `python --version`
   - Operating system
   - pymavlink version: `pip show pymavlink`

3. **Configuration:**
   - MAVLink settings from `util.py`
   - Any custom modifications

4. **Message examples:**
   - Sample messages that fail
   - Expected vs actual behavior

5. **Steps to reproduce:**
   - Exact commands run
   - Sequence of events
   - When issue occurs

### Log Files

Check log output for:
- Connection errors
- Parsing warnings/errors
- Message format issues
- Component ID mismatches

### Common Error Messages Reference

| Error Message | Meaning | Solution |
|--------------|---------|----------|
| "Failed to connect" | Cannot establish MAVLink connection | Check network, verify airside running |
| "Invalid extinguish status format" | Message format incorrect | Verify message format matches spec |
| "Failed to parse extinguish status" | Parsing error (ValueError/IndexError) | Check numeric values, field count |
| "Ignoring message from component X" | Wrong component ID | Verify component ID is 191 |
| "Unknown message format" | Message doesn't match any prefix | Check message starts with `e_` |

## Prevention

### Best Practices

1. **Validate messages before sending:**
   - Check format before transmission
   - Verify all fields present
   - Test with sample messages

2. **Monitor logs regularly:**
   - Check for warnings
   - Review error patterns
   - Track message rates

3. **Test thoroughly:**
   - Test with various message formats
   - Test edge cases
   - Test error conditions

4. **Keep configuration consistent:**
   - Document any changes
   - Version control config
   - Test after changes

## Still Having Issues?

1. Review [GETTING_STARTED.md](GETTING_STARTED.md) for setup steps
2. Check [IMPLEMENTATION_GUIDE.md](IMPLEMENTATION_GUIDE.md) for implementation details
3. Consult [EXTINGUISH_SYSTEM_DOCUMENTATION.md](EXTINGUISH_SYSTEM_DOCUMENTATION.md) for technical reference
4. Verify message format matches specification exactly
5. Check that all dependencies are installed correctly
