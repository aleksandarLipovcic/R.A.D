import tkinter as tk
import math


class Drone3DView(tk.Canvas):
    """
    Lightweight software 3D renderer for a 10-inch X-frame drone.

    Coordinate system (right-hand, matches Betaflight / MSP convention):
        X = Forward
        Y = Up
        Z = Right

    Rotation order: Yaw → Pitch → Roll  (intrinsic, body-frame)
    """

    # Critical angle thresholds — canvas tint changes colour above these
    WARN_ANGLE     = 35.0   # degrees
    CRITICAL_ANGLE = 60.0   # degrees

    def __init__(self, parent, width=500, height=450):
        super().__init__(
            parent,
            width=width, height=height,
            bg="#1a1a1a", highlightthickness=0
        )
        self.cx     = width  // 2
        self.cy     = height // 2
        self.scale  = 80

        # Fixed isometric camera angles (not driven by telemetry)
        self._cam_pitch = math.radians(20)
        self._cam_yaw   = math.radians(-30)

        # 10-inch X-frame geometry  [X_fwd, Y_up, Z_right]
        # Motor positions at ±1.2 units, body markers slightly inboard
        self._pts = {
            "FL":     [ 1.2,  0.0, -1.2],
            "FR":     [ 1.2,  0.0,  1.2],
            "RL":     [-1.2,  0.0, -1.2],
            "RR":     [-1.2,  0.0,  1.2],
            "BODY_F": [ 0.8,  0.0,  0.0],
            "BODY_B": [-0.8,  0.0,  0.0],
            "NOSE":   [ 1.8,  0.0,  0.0],
        }

    # ── Projection ────────────────────────────────────────────────────────────

    def _rotate(self, x, y, z, roll_r, pitch_r, yaw_r):
        """
        Apply body-frame rotation: Yaw first, then Pitch, then Roll.
        All inputs in radians.
        Returns (x, y, z) in world frame.
        """
        # --- Yaw (rotation around Y axis) ---
        x1 =  x * math.cos(yaw_r) + z * math.sin(yaw_r)
        z1 = -x * math.sin(yaw_r) + z * math.cos(yaw_r)
        x, z = x1, z1

        # --- Pitch (rotation around Z axis) ---
        x2 = x * math.cos(pitch_r) - y * math.sin(pitch_r)
        y2 = x * math.sin(pitch_r) + y * math.cos(pitch_r)
        x, y = x2, y2

        # --- Roll (rotation around X axis) ---
        y3 = y * math.cos(roll_r) - z * math.sin(roll_r)
        z3 = y * math.sin(roll_r) + z * math.cos(roll_r)
        y, z = y3, z3

        return x, y, z

    def _camera(self, x, y, z):
        """Apply fixed isometric camera transform and perspective projection."""
        # Camera pitch (tilt down)
        y1 = y * math.cos(self._cam_pitch) - z * math.sin(self._cam_pitch)
        z1 = y * math.sin(self._cam_pitch) + z * math.cos(self._cam_pitch)
        y, z = y1, z1

        # Camera yaw (orbit around drone)
        x2 =  x * math.cos(self._cam_yaw) + z * math.sin(self._cam_yaw)
        z2 = -x * math.sin(self._cam_yaw) + z * math.cos(self._cam_yaw)
        x, z = x2, z2

        # Perspective divide
        fov    = 300
        factor = fov / (fov + z)
        px = x * self.scale * factor + self.cx
        py = -y * self.scale * factor + self.cy
        return px, py, z   # z returned for depth sorting / prop sizing

    def _project(self, x, y, z, roll_r, pitch_r, yaw_r):
        x, y, z = self._rotate(x, y, z, roll_r, pitch_r, yaw_r)
        return self._camera(x, y, z)

    # ── Rendering ─────────────────────────────────────────────────────────────

    def update_orientation(self, roll: float, pitch: float, yaw: float = 0.0):
        """
        Redraw the drone model for the given orientation.
        roll, pitch, yaw are in degrees (already ÷10 from main.py).
        """
        self.delete("all")

        roll_r  = math.radians(roll)
        pitch_r = math.radians(pitch)
        yaw_r   = math.radians(yaw)

        # ── Background tint for critical attitude warning ──────────────────
        max_tilt = max(abs(roll), abs(pitch))
        if max_tilt >= self.CRITICAL_ANGLE:
            self.config(bg="#3a0000")
        elif max_tilt >= self.WARN_ANGLE:
            self.config(bg="#2a2000")
        else:
            self.config(bg="#1a1a1a")

        # ── Static ground reference grid (never rotates) ──────────────────
        for i in range(-5, 6):
            p1 = self._camera(*self._rotate(float(i), -1.0, -5.0, 0, 0, 0))
            p2 = self._camera(*self._rotate(float(i), -1.0,  5.0, 0, 0, 0))
            self.create_line(p1[0], p1[1], p2[0], p2[1], fill="#2a2a2a", width=1)

        # ── Project all vertices ──────────────────────────────────────────
        proj = {
            k: self._project(*v, roll_r, pitch_r, yaw_r)
            for k, v in self._pts.items()
        }
        center = self._project(0, 0, 0, roll_r, pitch_r, yaw_r)

        # ── Arms + propeller ovals ────────────────────────────────────────
        # FPV colour convention: green = front, red = rear
        motors = [
            ("FL", "#00FF00"),
            ("FR", "#00FF00"),
            ("RL", "#FF4444"),
            ("RR", "#FF4444"),
        ]
        for key, color in motors:
            p = proj[key]
            # Arm
            self.create_line(
                center[0], center[1], p[0], p[1],
                fill=color, width=5
            )
            # Propeller oval — perspective-aware radius
            r  = 20 * (300 / (300 + max(p[2], -290)))  # clamp z so factor stays positive
            self.create_oval(
                p[0] - r,    p[1] - r / 3,
                p[0] + r,    p[1] + r / 3,
                outline=color, width=1
            )

        # ── Fuselage / nose arrow ────────────────────────────────────────
        self.create_line(
            proj["BODY_B"][0], proj["BODY_B"][1],
            proj["NOSE"][0],   proj["NOSE"][1],
            fill="#FFFFFF", width=3, arrow=tk.LAST
        )

        # ── HUD text overlay ─────────────────────────────────────────────
        hud_color = "#FF4444" if max_tilt >= self.CRITICAL_ANGLE else "#00FF00"
        self.create_text(10, 10, anchor="nw",
                         text=f"R: {roll:>6.1f}°",  fill=hud_color, font=("Consolas", 10))
        self.create_text(10, 25, anchor="nw",
                         text=f"P: {pitch:>6.1f}°", fill=hud_color, font=("Consolas", 10))
        self.create_text(10, 40, anchor="nw",
                         text=f"Y: {yaw:>6.1f}°",   fill=hud_color, font=("Consolas", 10))

        # Critical angle banner
        if max_tilt >= self.CRITICAL_ANGLE:
            self.create_text(
                self.cx, self.cy - 60,
                text="⚠ CRITICAL ATTITUDE",
                fill="#FF4444", font=("Consolas", 12, "bold")
            )