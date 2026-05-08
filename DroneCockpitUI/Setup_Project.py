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
        subprocess.check_call([sys.executable, "-m", "pip", "install", package])
        return True

def check_dependencies():
    print("--- 1. Checking Python Dependencies ---")
    install_and_import("pybind11")
    install_and_import("pyserial", "serial")
    print("✅ All Python dependencies are ready.")

def check_cpp_backend():
    print("\n--- 2. Checking C++ Backend Build ---")
    base_dir = os.path.dirname(os.path.abspath(__file__))
    # Moves up to 'Project RAD' level to find the x64 folder
    root_dir = os.path.dirname(base_dir)
    
    search_paths = [
        os.path.join(root_dir, "x64", "Release", "DroneBackend.pyd"),
        os.path.join(root_dir, "x64", "Debug", "DroneBackend.pyd")
    ]
    
    found = False
    for p in search_paths:
        if os.path.exists(p):
            print(f"✅ Found compiled backend at: {p}")
            found = True
            break
            
    if not found:
        print("⚠️  Warning: DroneBackend.pyd not found.")
        print("   Action: Open Visual Studio and 'Rebuild' the DroneBackend project.")

def check_hardware():
    print("\n--- 3. Checking Drone Connectivity ---")
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    
    for port in ports:
        if "STM" in port.description or "USB Serial" in port.description:
            print(f"✅ Drone detected on {port.device} ({port.description})")
            return
            
    print("❌ Drone not detected. Check USB connection and drivers.")

if __name__ == "__main__":
    print("========================================")
    print("   Project R.A.D - Portable Setup      ")
    print("========================================\n")
    
    check_dependencies()
    check_cpp_backend()
    check_hardware()
    
    print("\nInitialization complete. Run DroneTest.py to start.")