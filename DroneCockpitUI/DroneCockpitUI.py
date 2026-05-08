import sys
import os
import tkinter as tk
from tkinter import ttk
from IMUWidget import IMUWidget
from Drone3DView import Drone3DView

# --- Backend Path Configuration ---
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
        self.root.configure(bg="#f0f0f0")

        self.hub = DroneBackend.DroneLink()
        self.imu_sensor = None
        
        self.setup_ui()
        
        # --- FIX: Dynamic Geometry to prevent window clipping ---
        self.root.update_idletasks()
        # Calculate required width/height and add a small buffer for padding
        req_width = self.root.winfo_reqwidth() + 20
        req_height = self.root.winfo_reqheight() + 20
        
        # Set geometry and center the window on screen
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        x = (screen_width // 2) - (req_width // 2)
        y = (screen_height // 2) - (req_height // 2)
        
        self.root.geometry(f"{req_width}x{req_height}+{x}+{y}")
        self.root.minsize(req_width, req_height)

        self.auto_connect()

    def setup_ui(self):
        # --- Top Navigation/Status Bar ---
        self.top_frame = tk.Frame(self.root, bg="#f0f0f0")
        self.top_frame.pack(side="top", fill="x", pady=10)

        header = tk.Label(self.top_frame, text="DRONE TELEMETRY SYSTEM", 
                         font=('Arial', 14, 'bold'), bg="#f0f0f0")
        header.pack()

        self.status_label = tk.Label(self.top_frame, text="Status: Searching...", 
                                    fg="orange", bg="#f0f0f0", font=('Arial', 10))
        self.status_label.pack()

        # --- Main Workspace (Grid Layout) ---
        self.main_container = tk.Frame(self.root, bg="#f0f0f0")
        self.main_container.pack(fill="both", expand=True, padx=20) # Increased padding

        # Left Column: Telemetry Table
        self.imu_view = IMUWidget(self.main_container)
        self.imu_view.grid(row=0, column=0, sticky="nsew", padx=10)

        # Right Column: 3D Visualization
        self.visual_frame = ttk.LabelFrame(self.main_container, text="Spatial Orientation")
        self.visual_frame.grid(row=0, column=1, sticky="nsew", padx=10)
        
        # Using the improved Drone3DView with isometric projection
        self.drone_3d = Drone3DView(self.visual_frame, width=400, height=350)
        self.drone_3d.pack(padx=10, pady=10, expand=True, fill="both")

        # --- Bottom Control Bar ---
        self.btn_frame = tk.Frame(self.root, bg="#f0f0f0")
        self.btn_frame.pack(side="bottom", fill="x", pady=20)
        
        self.reconnect_btn = ttk.Button(self.btn_frame, text="Force Reconnect", 
                                      command=self.auto_connect)
        self.reconnect_btn.pack()

    def auto_connect(self):
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
            self.root.after(2000, self.auto_connect)

    def update_loop(self):
        if self.imu_sensor:
            try:
                result = self.imu_sensor.getThesisData(self.hub)
                
                # 1. Update the Telemetry Numbers (This handles its own calibration/filtering)
                self.imu_view.update_ui(result)
                
                # 2. Update the 3D Model using the IMU's filtered angles
                # We pull the processed pitch/roll directly from the widget for consistency
                self.drone_3d.update_orientation(
                    roll=self.imu_view.roll_angle,
                    pitch=self.imu_view.pitch_angle,
                    yaw=0 # Magnetometer logic will go here next
                )
                
                self.root.after(30, self.update_loop) # Faster 30ms refresh for smoother 3D
            except Exception as e:
                print(f"Telemetry Lost: {e}")
                self.status_label.config(text="Status: Data Stream Interrupted", fg="red")
                self.auto_connect()

if __name__ == "__main__":
    root = tk.Tk()
    app = DroneCockpitApp(root)
    
    def on_closing():
        if app.hub:
            app.hub.disconnect()
        root.destroy()
        
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()
