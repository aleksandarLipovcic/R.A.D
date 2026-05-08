import tkinter as tk
from tkinter import ttk

class IMUWidget(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="MPU-6500 Long-Range Flight Hub", padding=10)
        
        # --- 1. Pilot-Grade UI Colors ---
        self.CLR_SAFE = "#99FF99"      # Green
        self.CLR_WARN = "#FFFF99"      # Yellow
        self.CLR_CRITICAL = "#FF4444"  # Red
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

        # --- 3. Diagnostics Frame ---
        self.diag_frame = ttk.LabelFrame(self, text="Link Performance Breakdown (ms)")
        self.rtt_val = ttk.Label(self.diag_frame, text="Total RTT: 0.00ms")
        self.fc_val = ttk.Label(self.diag_frame, text="FC Internal: 0.00ms")
        self.link_val = ttk.Label(self.diag_frame, text="One-Way Link: 0.00ms")
        self.rtt_val.pack(anchor="w")
        self.fc_val.pack(anchor="w")
        self.link_val.pack(anchor="w")

    def create_pfd_grid(self):
        self.grid_container = tk.Frame(self, bd=1, relief="solid", padx=5, pady=5)
        self.grid_container.grid(row=1, column=0, columnspan=3, sticky="nsew")

        headers = ["FLIGHT AXIS", "ROTATION (deg/s)", "G-FORCE (g)"]
        for i, h in enumerate(headers):
            tk.Label(self.grid_container, text=h, font=('Arial', 10, 'bold')).grid(row=0, column=i, padx=15, pady=5)

        self.axes = {}
        axis_names = [("roll", "ROLL (X)"), ("pitch", "PITCH (Y)"), ("yaw", "YAW (Z)")]

        for i, (key, label) in enumerate(axis_names, 1):
            tk.Label(self.grid_container, text=f"{label}:", font=('Arial', 10)).grid(row=i, column=0, sticky="e")
            
            rot_lbl = tk.Label(self.grid_container, text="0.00", font=('Consolas', 12, 'bold'), width=10, bg=self.CLR_SAFE)
            rot_lbl.grid(row=i, column=1, padx=2, pady=2)
            
            acc_lbl = tk.Label(self.grid_container, text="0.00", font=('Consolas', 12, 'bold'), width=10, bg=self.CLR_SAFE)
            acc_lbl.grid(row=i, column=2, padx=2, pady=2)
            
            # alert_level: 0=Green, 1=Yellow, 2=Red
            self.axes[key] = {
                'rot_lbl': rot_lbl, 
                'acc_lbl': acc_lbl, 
                'reset_id': None, 
                'alert_level': 0 
            }

    def reset_rotation_color(self, axis_key):
        """Returns the specific rotation cell back to green and clears priority."""
        if axis_key in self.axes:
            self.axes[axis_key]['rot_lbl'].config(bg=self.CLR_SAFE)
            self.axes[axis_key]['reset_id'] = None
            self.axes[axis_key]['alert_level'] = 0

    def get_accel_color(self, val, is_z=False):
        """Tuned for 10-inch cruiser stability limits."""
        abs_v = abs(val)
        if is_z:
            if val < 0.50: return self.CLR_CRITICAL
            if val < 0.85: return self.CLR_WARN
            return self.CLR_SAFE
        else:
            if abs_v > 0.70: return self.CLR_CRITICAL
            if abs_v > 0.25: return self.CLR_WARN
            return self.CLR_SAFE

    def update_ui(self, result):
        """Updates UI with Priority-Locked Peak-Hold for rotation."""
        data = result['data']
        GYRO_SCALE = 16.4   #
        ACCEL_SCALE = 2048.0 #

        axis_mapping = {
            'roll':  (data['gx']/GYRO_SCALE, data['ax']/ACCEL_SCALE, False),
            'pitch': (data['gy']/GYRO_SCALE, data['ay']/ACCEL_SCALE, False),
            'yaw':   (data['gz']/GYRO_SCALE, data['az']/ACCEL_SCALE, True)
        }

        for key, (rot, acc, is_z) in axis_mapping.items():
            # 1. Update G-Force (Instant/Standard behavior)
            self.axes[key]['acc_lbl'].config(text=f"{acc:>7.2f}", bg=self.get_accel_color(acc, is_z))

            # 2. Update Rotation (Priority Peak-Hold)
            self.axes[key]['rot_lbl'].config(text=f"{rot:>7.2f}")
            
            abs_rot = abs(rot)
            current_level = self.axes[key]['alert_level']
            
            # Determine potential new alert level
            new_level = 0
            spike_color = None
            hold_time = 0

            if abs_rot > 80:
                new_level = 2 # Red Priority
                spike_color = self.CLR_CRITICAL
                hold_time = 2500
            elif abs_rot > 30:
                new_level = 1 # Yellow Priority
                spike_color = self.CLR_WARN
                hold_time = 1500

            # PRIORITY RULE: Only update if the new spike is higher or equal to current color
            if new_level >= current_level and new_level > 0:
                # Cancel existing reset timer
                if self.axes[key]['reset_id']:
                    self.after_cancel(self.axes[key]['reset_id'])
                
                # Apply high-priority color and level
                self.axes[key]['rot_lbl'].config(bg=spike_color)
                self.axes[key]['alert_level'] = new_level
                
                # Schedule reset back to green (Level 0)
                self.axes[key]['reset_id'] = self.after(hold_time, lambda k=key: self.reset_rotation_color(k))

        # 3. Update Thesis Diagnostics
        if self.show_diag.get():
            rtt, fc = result['rtt_ms'], result['fc_cycle_ms']
            self.rtt_val.config(text=f"Total RTT:     {rtt:>6.2f} ms")
            self.fc_val.config(text=f"FC Internal:   {fc:>6.2f} ms")
            self.link_val.config(text=f"One-Way Link:  {max(0, (rtt-fc)/2):>6.2f} ms")

    def refresh_layout(self):
        if self.show_diag.get():
            self.diag_frame.grid(row=5, column=0, columnspan=3, sticky="ew", pady=(10, 0))
        else:
            self.diag_frame.grid_forget()