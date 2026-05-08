import tkinter as tk
from tkinter import ttk

class IMUWidget(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="MPU-6500 Long-Range Flight Hub", padding=10)
        
        # --- 1. Pilot-Grade UI Colors ---
        self.CLR_SAFE = "#99FF99"      # Green: Stable cruising
        self.CLR_WARN = "#FFFF99"      # Yellow: Unexpected turbulence/movement
        self.CLR_CRITICAL = "#FF4444"  # Red: Potential loss of control
        self.CLR_TEXT = "#000000"

        # --- 2. Research Data Toggle ---
        self.show_diag = tk.BooleanVar(value=False)
        self.toggle_btn = ttk.Checkbutton(
            self, text="Show Thesis Latency Data", 
            variable=self.show_diag, 
            command=self.refresh_layout
        )
        self.toggle_btn.grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 10))

        self.create_pfd_grid()

        # --- 3. Diagnostics Frame (Latency Tracking) ---
        self.diag_frame = ttk.LabelFrame(self, text="Link Performance Breakdown (ms)")
        self.rtt_val = ttk.Label(self.diag_frame, text="Total RTT: 0.00ms")
        self.fc_val = ttk.Label(self.diag_frame, text="FC Internal: 0.00ms")
        self.link_val = ttk.Label(self.diag_frame, text="One-Way Link: 0.00ms")
        self.rtt_val.pack(anchor="w")
        self.fc_val.pack(anchor="w")
        self.link_val.pack(anchor="w")

    def create_pfd_grid(self):
        """Creates the grid where data backgrounds change color."""
        self.grid_container = tk.Frame(self, bd=1, relief="solid", padx=5, pady=5)
        self.grid_container.grid(row=1, column=0, columnspan=3, sticky="nsew")

        headers = ["FLIGHT AXIS", "ROTATION (deg/s)", "G-FORCE (g)"]
        for i, h in enumerate(headers):
            tk.Label(self.grid_container, text=h, font=('Arial', 10, 'bold')).grid(row=0, column=i, padx=15, pady=5)

        self.axes = {}
        axis_names = [("roll", "ROLL (X)"), ("pitch", "PITCH (Y)"), ("yaw", "YAW (Z)")]

        for i, (key, label) in enumerate(axis_names, 1):
            tk.Label(self.grid_container, text=f"{label}:", font=('Arial', 10)).grid(row=i, column=0, sticky="e")
            
            # Peak-Hold Rotation Cells
            rot_lbl = tk.Label(self.grid_container, text="0.00", font=('Consolas', 12, 'bold'), width=10, bg=self.CLR_SAFE)
            rot_lbl.grid(row=i, column=1, padx=2, pady=2)
            
            # Static Angle/G-Force Cells
            acc_lbl = tk.Label(self.grid_container, text="0.00", font=('Consolas', 12, 'bold'), width=10, bg=self.CLR_SAFE)
            acc_lbl.grid(row=i, column=2, padx=2, pady=2)
            
            self.axes[key] = {'rot_lbl': rot_lbl, 'acc_lbl': acc_lbl, 'reset_id': None}

    def reset_rotation_color(self, axis_key):
        """Returns the specific rotation cell back to green."""
        if axis_key in self.axes:
            self.axes[axis_key]['rot_lbl'].config(bg=self.CLR_SAFE)
            self.axes[axis_key]['reset_id'] = None

    def get_accel_color(self, val, is_z=False):
        """Tuned for 10-inch cruiser stability limits."""
        abs_v = abs(val)
        if is_z:
            # Z-axis: Safety is about vertical lift
            if val < 0.50: return self.CLR_CRITICAL # Critical lift loss
            if val < 0.85: return self.CLR_WARN     # Cruiser warning: excessive tilt
            return self.CLR_SAFE
        else:
            # X/Y: Safety is about staying level for efficient long-range cruise
            if abs_v > 0.70: return self.CLR_CRITICAL
            if abs_v > 0.25: return self.CLR_WARN
            return self.CLR_SAFE

    def update_ui(self, result):
        """Updates UI with Peak-Hold logic for 10-inch frame dynamics."""
        data = result['data']
        GYRO_SCALE = 16.4   #
        ACCEL_SCALE = 2048.0 #

        axis_mapping = {
            'roll':  (data['gx']/GYRO_SCALE, data['ax']/ACCEL_SCALE, False),
            'pitch': (data['gy']/GYRO_SCALE, data['ay']/ACCEL_SCALE, False),
            'yaw':   (data['gz']/GYRO_SCALE, data['az']/ACCEL_SCALE, True)
        }

        for key, (rot, acc, is_z) in axis_mapping.items():
            # 1. Update G-Force (Current Position)
            self.axes[key]['acc_lbl'].config(text=f"{acc:>7.2f}", bg=self.get_accel_color(acc, is_z))

            # 2. Update Rotation (Long-Range Sensitive Peak-Hold)
            self.axes[key]['rot_lbl'].config(text=f"{rot:>7.2f}")
            
            abs_rot = abs(rot)
            spike_color = None
            hold_time = 0

            # On a 10-inch cruiser, 80+ deg/s is a massive, sharp disturbance.
            if abs_rot > 80:
                spike_color = self.CLR_CRITICAL
                hold_time = 2500 # Keep alert for 2.5s (Long distance needs more notice)
            elif abs_rot > 30:
                spike_color = self.CLR_WARN
                hold_time = 1500 # Keep alert for 1.5s

            if spike_color:
                if self.axes[key]['reset_id']:
                    self.after_cancel(self.axes[key]['reset_id'])
                self.axes[key]['rot_lbl'].config(bg=spike_color)
                self.axes[key]['reset_id'] = self.after(hold_time, lambda k=key: self.reset_rotation_color(k))

        # 3. Update Thesis Diagnostics
        if self.show_diag.get():
            rtt, fc = result['rtt_ms'], result['fc_cycle_ms']
            self.rtt_val.config(text=f"Total RTT:     {rtt:>6.2f} ms") #
            self.fc_val.config(text=f"FC Internal:   {fc:>6.2f} ms") #
            self.link_val.config(text=f"One-Way Link:  {max(0, (rtt-fc)/2):>6.2f} ms") #

    def refresh_layout(self):
        if self.show_diag.get():
            self.diag_frame.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        else:
            self.diag_frame.grid_forget()