from pymavlink import mavutil
import subprocess
import os

# Store active processes in a dictionary for easy management
processes = {
    "Auto": [],
    "SLAM": [],
    "OSS": []
}

def stop_all_scripts():
    """Kills all currently running subprocesses across all modes."""
    for mode in processes:
        for p in processes[mode]:
            if p.poll() is None: # Check if process is still running
                print(f"Terminating {mode} process...")
                p.terminate()
        processes[mode] = []

def start_script(mode):
    stop_all_scripts() # Ensure a clean slate before starting new mode
    
    home = os.path.expanduser("~") # Safely handle the tilde (~)
    
    if mode == "Auto":
        processes["Auto"].append(subprocess.Popen(["python3", "auto.py"]))
        
    elif mode == "SLAM":
        processes["SLAM"].append(subprocess.Popen(["python3", "slam.py"]))
        
    elif mode == "OSS":
        # Note: Arguments must be separate items in the list
        processes["OSS"].append(subprocess.Popen(["./feature_tracker"], cwd=f"{home}/home/oakd/oak_d_vins_cpp/"))
        processes["OSS"].append(subprocess.Popen(["./vins_fusion", "oak_d.yaml"], cwd=f"{home}/home/oakd/VINS-Fusion/vins_estimator"))
        processes["OSS"].append(subprocess.Popen(["./mavlink_udp"], cwd=f"{home}/home/oakd/mavlink-udp-proxy/"))

print("Connecting to Pixhawk...")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master.wait_heartbeat()
print("Connected!")

current_mode = None

try:
    while True:
        msg = master.recv_match(type='RC_CHANNELS', blocking=True)
        if not msg:
            continue

        pwm = msg.chan11_raw
        
        # Determine mode based on PWM
        if pwm <= 1300:
            new_mode = "Auto"
        elif pwm <= 1800:
            new_mode = "SLAM"
        else:
            new_mode = "OSS"

        # Only trigger change if the mode actually shifts
        if current_mode != new_mode:
            print(f"--- Mode Change Detected: {new_mode} ---")
            current_mode = new_mode
            start_script(new_mode)

except KeyboardInterrupt:
    print("Shutting down...")
    stop_all_scripts()