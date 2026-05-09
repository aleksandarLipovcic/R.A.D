import os
import subprocess
import sys
import platform

def install_and_import(package, import_name=None):
    if import_name is None:
        import_name = package
    try:
        __import__(import_name)
        return True
    except ImportError:
        print(f"📦 Installing missing dependency: {package}...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", package])
            return True
        except Exception as e:
            print(f"❌ Failed to install {package}: {e}")
            return False

def check_dependencies():
    print("--- 1. Checking Python Dependencies ---")
    # pybind11 is needed for the C++ interface
    install_and_import("pybind11")
    # pyserial is used for hardware detection logic
    install_and_import("pyserial", "serial")
    print("✅ All Python dependencies are ready.")

def check_cpp_backend():
    print("\n--- 2. Checking C++ Threaded Backend ---")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    root_dir = os.path.dirname(base_dir)
    
    # Updated to look for 'DroneLink' instead of 'DroneBackend'
    # as we renamed the module to reflect the new threading logic
    search_paths = [
        os.path.join(root_dir, "x64", "Release", "DroneLink.pyd"),
        os.path.join(root_dir, "x64", "Debug", "DroneLink.pyd")
    ]
    
    found = False
    for p in search_paths:
        if os.path.exists(p):
            print(f"✅ Found compiled backend: {os.path.basename(p)}")
            found = True
            break
            
    if not found:
        print("⚠️  Warning: DroneLink.pyd not found.")
        print("   Action: Rebuild the 'DroneLink' project in Visual Studio (x64 Release).")

def check_hardware():
    print("\n--- 3. Checking Flight Controller (F405) Connection ---")
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    
    drone_found = False
    for port in ports:
        # SpeedyBee F405 usually identifies as STM32 Virtual Com Port
        if "STM" in port.description or "USB Serial" in port.description:
            print(f"✅ Drone detected on {port.device} ({port.description})")
            drone_found = True
            break
            
    if not drone_found:
        print("❌ Flight Controller not detected.")
        print("   Note: Ensure the drone is powered (Battery or USB) and drivers are installed.")

if __name__ == "__main__":
    print("========================================")
    print("   Project R.A.D - System Readiness    ")
    print("========================================\n")
    
    check_dependencies()
    check_cpp_backend()
    check_hardware()
    
    print("\nSetup check complete. You are ready to run DroneCockpitApp.py.")