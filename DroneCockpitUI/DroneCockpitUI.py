import sys
import os
import tkinter as tk
from IMUWidget import IMUWidget

# --- Backend Path Configuration ---
# This ensures Python finds the .pyd file in your Visual Studio output folder
script_dir = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.abspath(os.path.join(script_dir, '..', 'x64', 'Debug'))
sys.path.append(backend_path)

if hasattr(os, 'add_dll_directory'):
    try:
        os.add_dll_directory(backend_path)
    except Exception as e:
        print(f"Note: DLL directory already added or inaccessible: {e}")

import DroneBackend

class DroneCockpitApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Project R.A.D - Master's Research Cockpit")
        self.root.geometry("450x550")
        self.root.configure(bg="#f0f0f0")

        # Initialize Backend Hub
        self.hub = DroneBackend.DroneLink()
        self.imu_sensor = None
        
        # Setup UI Components
        self.setup_ui()
        
        # Start Connection Scan
        self.auto_connect()

    def setup_ui(self):
        # Header Label
        header = tk.Label(self.root, text="DRONE TELEMETRY SYSTEM", 
                         font=('Arial', 12, 'bold'), bg="#f0f0f0")
        header.pack(pady=(10, 0))

        # Connection Status
        self.status_label = tk.Label(self.root, text="Status: Searching...", 
                                    fg="orange", bg="#f0f0f0", font=('Arial', 10))
        self.status_label.pack(pady=5)

        # Integrate our Modular IMU Widget (Now with Thesis Latency fields)
        self.imu_view = IMUWidget(self.root)
        self.imu_view.pack(padx=20, pady=10, fill="both", expand=True)

        # Control Frame (Optional: for manual disconnect/reconnect)
        self.btn_frame = tk.Frame(self.root, bg="#f0f0f0")
        self.btn_frame.pack(pady=10)
        
        self.reconnect_btn = tk.Button(self.btn_frame, text="Force Reconnect", 
                                      command=self.auto_connect)
        self.reconnect_btn.pack()

    def auto_connect(self):
        # Attempt to find the F405 automatically
        port = DroneBackend.AutoDetectF405()
        
        if port != "NOT_FOUND":
            if self.hub.connect(port):
                self.status_label.config(text=f"Status: Connected on {port}", fg="green")
                self.imu_sensor = DroneBackend.IMUSensor(self.hub)
                self.update_loop()
            else:
                self.status_label.config(text="Status: Connection Failed", fg="red")
        else:
            self.status_label.config(text="Status: Searching for Drone...", fg="orange")
            # Poll every 2 seconds if not found
            self.root.after(2000, self.auto_connect)

    def update_loop(self):
        """
        Main telemetry loop. Fetches 6-axis data + RTT + FC Cycle time
        in a single call for precision research.
        """
        if self.imu_sensor:
            try:
                # Call the new thesis-specific C++ method
                # This returns a dict: {'data': {...}, 'rtt_ms': float, 'fc_cycle_ms': float}
                result = self.imu_sensor.getThesisData(self.hub)
                
                # Pass the complex result to the Widget for processing
                self.imu_view.update_ui(result)
                
                # Run at ~20Hz (50ms) to keep UI smooth without flooding the serial bus
                self.root.after(50, self.update_loop)
            except Exception as e:
                print(f"Telemetry Lost: {e}")
                self.status_label.config(text="Status: Data Stream Interrupted", fg="red")
                self.auto_connect()

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneCockpitApp(root)
    
    # Clean exit logic
    def on_closing():
        if app.hub:
            app.hub.disconnect()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()