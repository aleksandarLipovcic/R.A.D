import tkinter as tk
from tkinter import ttk

class IMUWidget(ttk.LabelFrame):
    def __init__(self, parent):
        super().__init__(parent, text="MPU-6500 Telemetry", padding=10)
        
        # Gyro Labels
        self.gx_label = ttk.Label(self, text="Gyro X: 0.0")
        self.gy_label = ttk.Label(self, text="Gyro Y: 0.0")
        self.gz_label = ttk.Label(self, text="Gyro Z: 0.0")
        
        self.gx_label.grid(row=0, column=0, sticky="w")
        self.gy_label.grid(row=1, column=0, sticky="w")
        self.gz_label.grid(row=2, column=0, sticky="w")
        
        # Add a small separator
        ttk.Separator(self, orient='horizontal').grid(row=3, column=0, sticky="ew", pady=5)
        
        # Accel Labels
        self.ax_label = ttk.Label(self, text="Accel X: 0.0")
        self.ay_label = ttk.Label(self, text="Accel Y: 0.0")
        self.az_label = ttk.Label(self, text="Accel Z: 0.0")
        
        self.ax_label.grid(row=4, column=0, sticky="w")
        self.ay_label.grid(row=5, column=0, sticky="w")
        self.az_label.grid(row=6, column=0, sticky="w")

    def update_data(self, data):
        """Expects a dictionary from our C++ backend"""
        self.gx_label.config(text=f"Gyro X: {data['gx']:>6}")
        self.gy_label.config(text=f"Gyro Y: {data['gy']:>6}")
        self.gz_label.config(text=f"Gyro Z: {data['gz']:>6}")
        
        self.ax_label.config(text=f"Accel X: {data['ax']:>6}")
        self.ay_label.config(text=f"Accel Y: {data['ay']:>6}")
        self.az_label.config(text=f"Accel Z: {data['az']:>6}")