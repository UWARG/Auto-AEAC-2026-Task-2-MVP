# import subprocess
# import os
# from pymavlink import mavutil

# # --- CONFIGURATION ---
# # For Raspberry Pi GPIO UART, use /dev/serial0
# # Ensure Pixhawk SERIALx_BAUD matches this (57600 is default for Telem ports)
# MAV_PORT = '/dev/serial0' 
# BAUD_RATE = 57600

# # Store active processes in a dictionary for easy management
# processes = {
#     "Auto": [],
#     "SLAM": [],
#     "OSS": []
# }

# def stop_all_scripts():
#     """Kills all currently running subprocesses across all modes."""
#     for mode in processes:
#         for p in processes[mode]:
#             if p.poll() is None:  # Check if process is still running
#                 print(f"Terminating {mode} process (PID: {p.pid})...")
#                 p.terminate()
#                 try:
#                     p.wait(timeout=2) # Give it a moment to close cleanly
#                 except subprocess.TimeoutExpired:
#                     p.kill() # Force kill if it doesn't close
#         processes[mode] = []

# def start_script(mode):
#     stop_all_scripts()
    
#     home = os.path.expanduser("~")
#     print(f"Starting {mode} mode...")
    
#     try:
#         if mode == "Auto":
#             processes["Auto"].append(subprocess.Popen(["python3", "auto.py"]))
            
#         elif mode == "SLAM":
#             processes["SLAM"].append(subprocess.Popen(["python3", "slam.py"]))
            
#         elif mode == "OSS":
#             # Paths based on your previous structure
#             oss_base = f"{home}/home/oakd" # Double check if /home/oakd/home/oakd is intended
            
#             processes["OSS"].append(subprocess.Popen(["./feature_tracker"], 
#                                     cwd=f"{oss_base}/oak_d_vins_cpp/"))
            
#             processes["OSS"].append(subprocess.Popen(["./vins_fusion", "oak_d.yaml"], 
#                                     cwd=f"{oss_base}/VINS-Fusion/vins_estimator"))
            
#             processes["OSS"].append(subprocess.Popen(["./mavlink_udp"], 
#                                     cwd=f"{oss_base}/mavlink-udp-proxy/"))
#     except Exception as e:
#         print(f"Error starting {mode}: {e}")

# # --- MAIN EXECUTION ---

# print(f"Connecting to Pixhawk via UART ({MAV_PORT})...")
# try:
#     # autoreconnect=True helps if the physical wire is loose
#     master = mavutil.mavlink_connection(MAV_PORT, baud=BAUD_RATE, autoreconnect=True)
#     master.wait_heartbeat()
#     print("Connected! Heartbeat received.")
# except Exception as e:
#     print(f"Failed to connect: {e}")
#     exit(1)

# current_mode = None

# try:
#     while True:
#         # Wait for RC_CHANNELS message
#         msg = master.recv_match(type='RC_CHANNELS', blocking=True, timeout=1.0)
        
#         if not msg:
#             continue

#         # Using Channel 11 for mode switching
#         pwm = msg.chan11_raw
        
#         if pwm <= 1300:
#             new_mode = "Auto"
#         elif pwm <= 1800:
#             new_mode = "SLAM"
#         else:
#             new_mode = "OSS"

#         if current_mode != new_mode:
#             print(f"\n--- Mode Change Detected: {new_mode} (PWM: {pwm}) ---")
#             current_mode = new_mode
#             start_script(new_mode)

# except KeyboardInterrupt:
#     print("\nUser interrupted. Shutting down...")
# finally:
#     stop_all_scripts()
#     print("Cleanup complete.")

import serial
import time

# Try the primary hardware port for Pi 5 / Pi 4
PORT = '/dev/ttyAMA0' 
BAUD = 57600

try:
    # Initialize the port
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"--- Starting Hardware Loopback on {PORT} ---")
    
    # Clear any old data sitting in the buffer
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    
    # Send a test string
    test_string = b"HELLO_PI\n"
    print(f"Sending: {test_string.decode().strip()}")
    ser.write(test_string)
    
    # Give the hardware a tiny moment to process
    time.sleep(0.1)
    
    # Read the data back
    incoming_data = ser.readline()
    
    if incoming_data == test_string:
        print(">>> SUCCESS: The Pi sent and received the data perfectly!")
        print("Your Raspberry Pi hardware and software settings are CORRECT.")
    elif len(incoming_data) > 0:
        print(f">>> PARTIAL SUCCESS: Received data, but it was corrupted: {incoming_data}")
    else:
        print(">>> FAILED: Sent data but received NOTHING back.")
        print("Check: Is the jumper wire securely touching Pins 8 and 10?")

    ser.close()

except Exception as e:
    print(f"ERROR: Could not open {PORT}. {e}")