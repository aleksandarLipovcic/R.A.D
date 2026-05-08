import tkinter as tk
import math

class Drone3DView(tk.Canvas):
    def __init__(self, parent, width=400, height=350):
        super().__init__(parent, width=width, height=height, bg="#1a1a1a", highlightthickness=0)
        self.center = (width // 2, height // 2)
        self.scale = 70
        
        # Static Viewpoint (Isometric) - This prevents the "feedback loss"
        # We tilt the camera slightly so all axes are always visible
        self.view_pitch = math.radians(25) 
        self.view_yaw = math.radians(-35)

        # 3D points for an "X" frame + Central Body
        self.points = {
            'FL': [1, 0, 1],   'FR': [1, 0, -1],
            'RR': [-1, 0, -1], 'RL': [-1, 0, 1],
            'NOSE': [1.8, 0, 0], 'TAIL': [-0.5, 0, 0]
        }

    def project_point(self, x, y, z, roll, pitch, yaw):
        # 1. Pilot Orientation (The Drone's actual movement)
        r, p, y_rad = map(math.radians, [roll, pitch, yaw])

        # Pitch (X)
        ty = y * math.cos(p) - z * math.sin(p); tz = y * math.sin(p) + z * math.cos(p)
        y, z = ty, tz
        # Roll (Z)
        tx = x * math.cos(r) - y * math.sin(r); ty = x * math.sin(r) + y * math.cos(r)
        x, y = tx, ty
        # Yaw (Y)
        tx = x * math.cos(y_rad) + z * math.sin(y_rad); tz = -x * math.sin(y_rad) + z * math.cos(y_rad)
        x, z = tx, tz

        # 2. Isometric Camera Projection (The "Chase Cam" effect)
        # Apply static offsets so the drone is viewed from an angle
        cy = y * math.cos(self.view_pitch) - z * math.sin(self.view_pitch)
        cz = y * math.sin(self.view_pitch) + z * math.cos(self.view_pitch)
        y, z = cy, cz
        
        cx = x * math.cos(self.view_yaw) + z * math.sin(self.view_yaw)
        x = cx

        # Simple 2D Screen Mapping
        px = x * self.scale + self.center[0]
        py = -y * self.scale + self.center[1]
        return px, py

    def update_orientation(self, roll, pitch, yaw):
        self.delete("all")
        
        # Artificial Horizon Reference
        self.create_line(0, self.center[1]+40, 400, self.center[1]+40, fill="#333333")

        # Project vertices
        pts = {k: self.project_point(*v, roll, pitch, yaw) for k, v in self.points.items()}

        # Draw Frame Arms (Color-coded for front/back)
        # Front (Green/Blue)
        self.create_line(self.center[0], self.center[1], pts['FL'], fill="#00FF00", width=4)
        self.create_line(self.center[0], self.center[1], pts['FR'], fill="#00FF00", width=4)
        # Rear (Red)
        self.create_line(self.center[0], self.center[1], pts['RL'], fill="#FF4444", width=4)
        self.create_line(self.center[0], self.center[1], pts['RR'], fill="#FF4444", width=4)

        # Draw Nose Arrow (Directional Feedback)
        self.create_line(pts['TAIL'], pts['NOSE'], fill="#FFFFFF", arrow=tk.LAST, width=2)

        # Propeller Disks (Provides immediate visual scale for tilt)
        for motor in ['FL', 'FR', 'RL', 'RR']:
            p = pts[motor]
            color = "#00FF00" if "F" in motor else "#FF4444"
            self.create_oval(p[0]-15, p[1]-5, p[0]+15, p[1]+5, outline=color, width=1)