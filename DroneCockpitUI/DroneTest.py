import sys
import os
import time

# --- 1. PORTABLE PATH RESOLUTION ---
# Get the absolute path of the directory where THIS script is saved
script_dir = os.path.dirname(os.path.abspath(__file__))

# Move up one level from 'DroneCockpitUI' to 'Project RAD' and into 'x64/Debug'
# This works regardless of where the project is moved on your computer.
backend_path = os.path.abspath(os.path.join(script_dir, '..', 'x64', 'Debug'))

print("--- Project R.A.D: System Check ---")
print(f"Targeting Backend: {backend_path}")

# --- 2. PHYSICAL FILE VERIFICATION ---
if not os.path.exists(backend_path):
    print(f"❌ Error: Folder '{backend_path}' does not exist.")
    print("Action: Rebuild your C++ project in Visual Studio.")
    sys.exit()

# Check if the .pyd file is actually inside the folder
files_in_dir = os.listdir(backend_path)
pyd_found = any(f.startswith("DroneBackend") and f.endswith(".pyd") for f in files_in_dir)

if pyd_found:
    print("✅ System: DroneBackend.pyd located.")
    sys.path.append(backend_path)
    # Required for Python 3.8+ on Windows to load the binary link
    if hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(backend_path)
else:
    print("❌ Error: 'DroneBackend.pyd' not found in the Debug folder.")
    print(f"Files found in folder: {files_in_dir}")
    sys.exit()

# --- 3. HARDWARE LINK & TELEMETRY ---
try:
    import DroneBackend
    print("🚀 SUCCESS: C++ Bridge Active.")
    
    # Create the instance
    drone = DroneBackend.DroneLink()
    
    # Set your COM port (Check Device Manager)
    port = "COM3" 
    
    if drone.connect(port):
        print(f"✅ Connected to F405 on {port}")
        print("Telemetry active. Press Ctrl+C to exit.\n")
        
        while True:
            # Fetch data from the C++ Backend
            volts = drone.getBatteryVoltage()
            angles = drone.getAttitude() # Returns [Roll, Pitch, Yaw]
            
            # Print update in-place (no scrolling)
            # Roll and Pitch are usually indices 0 and 1
            output = f"Battery: {volts:.2f}V | Roll: {angles[0]:>6.1f}° | Pitch: {angles[1]:>6.1f}°"
            print(output, end='\r')
            
            time.sleep(0.05) # 20Hz refresh rate
    else:
        print(f"❌ Connection Failed: Could not open {port}.")
        print("Ensure Betaflight is closed and the drone is plugged in.")

except ImportError as e:
    print(f"❌ Logic Error: Python found the file but failed to load it.")
    print(f"Detail: {e}")
    print("\nTIP: If you built in Debug mode, ensure you have the Debug Python headers,")
    print("or try switching your C++ project to 'Release' mode and rebuild.")

except KeyboardInterrupt:
    print("\n\nExiting System... Fly safe.")