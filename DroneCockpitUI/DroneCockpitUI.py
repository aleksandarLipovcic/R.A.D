import sys
import os
import time

# This looks for the C++ 'DroneBackend' in your Debug folder
# Adjust the path if your build output is in a different location
sys.path.append(os.path.join(os.getcwd(), '..', 'x64', 'Debug'))

try:
    import DroneBackend
    print("--- Project R.A.D: Backend Test ---")
    
    # Create the instance
    drone = DroneBackend.DroneLink()
    
    # Replace 'COM3' with the port you see in Device Manager
    port = "COM3" 
    
    if drone.connect(port):
        print(f"✅ Connected to F405 on {port}")
        print("Moving the drone should change the values below.")
        print("Press Ctrl+C to stop.\n")
        
        while True:
            # Call the C++ functions we defined
            volts = drone.getBatteryVoltage()
            angles = drone.getAttitude() # [Roll, Pitch, Yaw]
            
            # Print in one line (updates in place)
            output = f"Battery: {volts:.2f}V | Roll: {angles[0]:>6.1f} | Pitch: {angles[1]:>6.1f}"
            print(output, end='\r')
            
            time.sleep(0.05) 
    else:
        print(f"❌ Could not open {port}. Check USB or Betaflight status.")

except ImportError:
    print("❌ Error: Could not find 'DroneBackend.pyd'.")
    print("Check that you built the DroneBackend project in x64 Debug mode.")
except KeyboardInterrupt:
    print("\n\nExiting... Fly safe.")