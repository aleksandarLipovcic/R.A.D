import tkinter as tk
from tkinter import ttk
import math
import time

class IMUWidget(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="MPU-6500 Long-Range Flight Hub", padding=10)
        
        # --- 1. DEFINE COLORS FIRST ---
        # Ensures UI elements can find these variables during creation
        self.CLR_SAFE = "#99FF99"      # Green
        self.CLR_WARN = "#FFFF99"      # Yellow
        self.CLR_CRITICAL = "#FF4444"  # Red
        self.CLR_OFF = "#E0E0E0"       # Grey (for Yaw N/A)

        # --- 2. INITIALIZE STATE & CALIBRATION ---
        self.last_time = time.time()
        self.pitch_angle = 0.0
        self.roll_angle = 0.0
        
        # Captured during 'Calibrate Sensors' routine
        self.gyro_offsets = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.accel_offsets = {'pitch': 0.0, 'roll': 0.0}
        
        self.is_calibrating = False
        self.calib_samples = []

        # --- 3. UI LAYOUT ---
        self.setup_header_controls()
        self.create_pfd_grid()

        # Diagnostics Frame (Thesis Latency Data)
        self.diag_frame = ttk.LabelFrame(self, text="Link Performance Breakdown (ms)")
        self.rtt_val = ttk.Label(self.diag_frame, text="Total RTT: 0.00ms")
        self.fc_val = ttk.Label(self.diag_frame, text="FC Internal: 0.00ms")
        self.link_val = ttk.Label(self.diag_frame, text="One-Way Link: 0.00ms")
        self.rtt_val.pack(anchor="w")
        self.fc_val.pack(anchor="w")
        self.link_val.pack(anchor="w")

    def setup_header_controls(self):
        ctrl_frame = tk.Frame(self)
        ctrl_frame.grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 10))
        
        self.show_diag = tk.BooleanVar(value=False)
        ttk.Checkbutton(ctrl_frame, text="Show Thesis Latency Data", 
                        variable=self.show_diag, command=self.refresh_layout).pack(side="left")
        
        # Calibration button that zeroes rotation and angles
        self.calib_btn = ttk.Button(ctrl_frame, text="Calibrate Sensors", command=self.start_calibration)
        self.calib_btn.pack(side="left", padx=10)

    def start_calibration(self):
        """Initializes sampling to zero-out sensor bias"""
        self.is_calibrating = True
        self.calib_samples = []
        self.calib_btn.config(text="CALIBRATING...", state="disabled")

    def create_pfd_grid(self):
        self.grid_container = tk.Frame(self, bd=1, relief="solid", padx=5, pady=5)
        self.grid_container.grid(row=1, column=0, columnspan=4, sticky="nsew")

        headers = ["FLIGHT AXIS", "ROTATION (deg/s)", "G-FORCE (g)", "ANGLE (deg)"]
        for i, h in enumerate(headers):
            tk.Label(self.grid_container, text=h, font=('Arial', 10, 'bold')).grid(row=0, column=i, padx=12, pady=5)

        self.axes = {}
        axis_names = [("roll", "ROLL (X)"), ("pitch", "PITCH (Y)"), ("yaw", "YAW (Z)")]

        for i, (key, label) in enumerate(axis_names, 1):
            tk.Label(self.grid_container, text=f"{label}:", font=('Arial', 10)).grid(row=i, column=0, sticky="e")
            
            rot_lbl = tk.Label(self.grid_container, text="0.00", font=('Consolas', 12, 'bold'), width=10, bg=self.CLR_SAFE)
            acc_lbl = tk.Label(self.grid_container, text="0.00", font=('Consolas', 12, 'bold'), width=10, bg=self.CLR_SAFE)
            ang_lbl = tk.Label(self.grid_container, text="0.0", font=('Consolas', 12, 'bold'), width=10, bg=self.CLR_SAFE)
            
            rot_lbl.grid(row=i, column=1, padx=2, pady=2)
            acc_lbl.grid(row=i, column=2, padx=2, pady=2)
            ang_lbl.grid(row=i, column=3, padx=2, pady=2)
            
            self.axes[key] = {
                'rot_lbl': rot_lbl, 'acc_lbl': acc_lbl, 'ang_lbl': ang_lbl,
                'rot_reset_id': None, 'ang_reset_id': None,
                'rot_level': 0, 'ang_level': 0
            }

    def update_ui(self, result):
        data = result['data']
        GYRO_SCALE, ACCEL_SCALE = 16.4, 2048.0 # From MPU-6500 Datasheet
        
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        gx_raw, gy_raw, gz_raw = data['gx']/GYRO_SCALE, data['gy']/GYRO_SCALE, data['gz']/GYRO_SCALE
        ax, ay, az = data['ax']/ACCEL_SCALE, data['ay']/ACCEL_SCALE, data['az']/ACCEL_SCALE

        # 1. CALIBRATION LOGIC (Average 50 samples)
        if self.is_calibrating:
            cur_p = math.atan2(ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)
            cur_r = math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180 / math.pi)
            self.calib_samples.append((gx_raw, gy_raw, gz_raw, cur_p, cur_r))
            
            if len(self.calib_samples) >= 50:
                avgs = [sum(col)/len(col) for col in zip(*self.calib_samples)]
                self.gyro_offsets = {'x': avgs[0], 'y': avgs[1], 'z': avgs[2]}
                self.accel_offsets = {'pitch': avgs[3], 'roll': avgs[4]}
                self.is_calibrating = False
                self.calib_btn.config(text="Calibrate Sensors", state="normal")
            return

        # 2. APPLY CALIBRATION OFFSETS
        gx, gy, gz = gx_raw - self.gyro_offsets['x'], gy_raw - self.gyro_offsets['y'], gz_raw - self.gyro_offsets['z']
        acc_p_final = (math.atan2(ax, math.sqrt(ay**2 + az**2)) * (180 / math.pi)) - self.accel_offsets['pitch']
        acc_r_final = (math.atan2(ay, math.sqrt(ax**2 + az**2)) * (180 / math.pi)) - self.accel_offsets['roll']

        # 3. COMPLEMENTARY FILTER (Betaflight Sync)
        self.pitch_angle = 0.98 * (self.pitch_angle + gy * dt) + 0.02 * acc_p_final
        self.roll_angle = 0.98 * (self.roll_angle + gx * dt) + 0.02 * acc_r_final

        axis_map = {
            'roll':  (gx, ax, self.roll_angle, False),
            'pitch': (gy, ay, self.pitch_angle, False),
            'yaw':   (gz, az, 0.0, True) 
        }

        for key, (rot, acc, ang, is_z) in axis_map.items():
            self.axes[key]['rot_lbl'].config(text=f"{rot:>7.2f}")
            self.axes[key]['acc_lbl'].config(text=f"{acc:>7.2f}", bg=self.get_accel_color(acc, is_z))
            
            if key == 'yaw':
                self.axes[key]['ang_lbl'].config(text="N/A", bg=self.CLR_OFF) # Heading needs Magnetometer
            else:
                self.axes[key]['ang_lbl'].config(text=f"{ang:>7.1f}")

            self.apply_priority_alerts(key, rot, ang)

        # 4. THESIS DIAGNOSTICS
        if self.show_diag.get():
            rtt, fc = result['rtt_ms'], result['fc_cycle_ms']
            self.rtt_val.config(text=f"Total RTT:     {rtt:>6.2f} ms")
            self.fc_val.config(text=f"FC Internal:   {fc:>6.2f} ms")
            self.link_val.config(text=f"One-Way Link:  {max(0, (rtt-fc)/2):>6.2f} ms")

    def get_accel_color(self, val, is_z=False):
        abs_v = abs(val)
        if is_z:
            if val < 0.50: return self.CLR_CRITICAL
            if val < 0.85: return self.CLR_WARN
            return self.CLR_SAFE
        else:
            if abs_v > 0.70: return self.CLR_CRITICAL
            if abs_v > 0.25: return self.CLR_WARN
            return self.CLR_SAFE

    def apply_priority_alerts(self, key, rot, ang):
        # Rotation Alerts
        abs_rot = abs(rot)
        r_lvl, r_clr, r_h = (2, self.CLR_CRITICAL, 2500) if abs_rot > 80 else (1, self.CLR_WARN, 1500) if abs_rot > 30 else (0, None, 0)
        if r_lvl >= self.axes[key]['rot_level'] and r_lvl > 0:
            if self.axes[key]['rot_reset_id']: self.after_cancel(self.axes[key]['rot_reset_id'])
            self.axes[key]['rot_lbl'].config(bg=r_clr)
            self.axes[key]['rot_level'] = r_lvl
            self.axes[key]['rot_reset_id'] = self.after(r_h, lambda k=key: self.reset_rot_alert(k))

        # Angle Alerts
        abs_ang = abs(ang)
        a_lvl, a_clr, a_h = (2, self.CLR_CRITICAL, 3000) if abs_ang > 30 else (1, self.CLR_WARN, 2000) if abs_ang > 15 else (0, None, 0)
        if key != 'yaw' and a_lvl >= self.axes[key]['ang_level'] and a_lvl > 0:
            if self.axes[key]['ang_reset_id']: self.after_cancel(self.axes[key]['ang_reset_id'])
            self.axes[key]['ang_lbl'].config(bg=a_clr)
            self.axes[key]['ang_level'] = a_lvl
            self.axes[key]['ang_reset_id'] = self.after(a_h, lambda k=key: self.reset_ang_alert(k))

    def reset_rot_alert(self, key):
        self.axes[key]['rot_lbl'].config(bg=self.CLR_SAFE)
        self.axes[key]['rot_level'] = 0

    def reset_ang_alert(self, key):
        self.axes[key]['ang_lbl'].config(bg=self.CLR_SAFE)
        self.axes[key]['ang_level'] = 0

    def refresh_layout(self):
        if self.show_diag.get():
            self.diag_frame.grid(row=5, column=0, columnspan=4, sticky="ew", pady=(10, 0))
        else:
            self.diag_frame.grid_forget()