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
    # Install pybind11 and pyserial if they are missing
    install_and_import("pybind11")
    install_and_import("pyserial", "serial")
    print("✅ All Python dependencies are ready.")

def check_cpp_backend():
    print("\n--- 2. Checking C++ Backend Build ---")
    # Get the directory where this setup script lives
    base_dir = os.path.dirname(os.path.abspath(__file__))
    # Look one level up in the 'x64' folder
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
        print(f"   Looked in: {root_dir}\\x64\\...")
        print("   Action: Open Visual Studio and 'Rebuild' the DroneBackend project.")

def check_hardware():
    print("\n--- 3. Checking Drone Connectivity ---")
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    
    if not ports:
        print("❌ No COM ports found. Is the drone plugged in?")
        return

    for port in ports:
        # F405 usually identifies as STM32 or USB Serial
        if "STM" in port.description or "USB Serial" in port.description or "COM3" in port.device:
            print(f"✅ Drone detected on {port.device} ({port.description})")
            return
            
    print("❓ Devices found, but none look like an F405 Flight Controller.")
    for p in ports:
        print(f"   - Found: {p.device} ({p.description})")

if __name__ == "__main__":
    print("========================================")
    print("   Project R.A.D - Portable Setup      ")
    print("========================================\n")
    
    check_dependencies()
    check_cpp_backend()
    check_hardware()
    
    print("\n" + "="*40)
    print("Initialization complete.")