import sys
import os
import tkinter as tk
from IMUWidget import IMUWidget

# Ensure we can find the C++ Backend
script_dir = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.abspath(os.path.join(script_dir, '..', 'x64', 'Debug'))
sys.path.append(backend_path)
if hasattr(os, 'add_dll_directory'):
    os.add_dll_directory(backend_path)

import DroneBackend

class DroneCockpitApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Project R.A.D - Drone Cockpit")
        self.root.geometry("400x500")

        # Initialize Backend Hub
        self.hub = DroneBackend.DroneLink()
        self.imu_sensor = None
        
        # Setup UI Components
        self.setup_ui()
        
        # Start Connection Scan
        self.auto_connect()

    def setup_ui(self):
        # Connection Status
        self.status_label = tk.Label(self.root, text="Status: Disconnected", fg="red")
        self.status_label.pack(pady=10)

        # Integrate our Modular IMU Widget
        self.imu_view = IMUWidget(self.root)
        self.imu_view.pack(padx=20, pady=10, fill="x")

    def auto_connect(self):
        port = DroneBackend.AutoDetectF405()
        if port != "NOT_FOUND":
            if self.hub.connect(port):
                self.status_label.config(text=f"Status: Connected on {port}", fg="green")
                self.imu_sensor = DroneBackend.IMUSensor(self.hub)
                self.update_loop()
        else:
            self.status_label.config(text="Status: Searching for Drone...", fg="orange")
            self.root.after(2000, self.auto_connect)

    def update_loop(self):
        if self.imu_sensor:
            data = self.imu_sensor.getRawData()
            self.imu_view.update_data(data)
            
        # Run at ~20Hz (every 50ms)
        self.root.after(50, self.update_loop)

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneCockpitApp(root)
    root.mainloop()