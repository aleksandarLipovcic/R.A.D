import tkinter as tk
import math


class Drone3DView(tk.Canvas):
    """
    Software 3D renderer for a 10-inch X-frame long-range quadcopter.

    ═══════════════════════════════════════════════════════════════════════════
    COORDINATE SYSTEM  (right-hand, body frame)
    ═══════════════════════════════════════════════════════════════════════════
        +X  =  forward  (nose direction)
        +Y  =  right    (starboard)
        +Z  =  up       (away from ground when level)

    ═══════════════════════════════════════════════════════════════════════════
    BETAFLIGHT MSP_ATTITUDE SIGN CONVENTION
    ═══════════════════════════════════════════════════════════════════════════
        roll  > 0  →  right side drops  (right-wing-down)
        pitch > 0  →  nose drops        (nose-down)
        yaw   > 0  →  nose turns right  (CW from above)

    ═══════════════════════════════════════════════════════════════════════════
    WARNING THRESHOLDS  — must match IMUWidget exactly
    ═══════════════════════════════════════════════════════════════════════════
    10-inch long-range quad flight envelope:
        WARN_ANGLE     = 20°   gentle bank, start alerting
        CRITICAL_ANGLE = 40°   aggressive for LR, definitely alert

    ═══════════════════════════════════════════════════════════════════════════
    PERSPECTIVE
    ═══════════════════════════════════════════════════════════════════════════
    FOV = 1000 → near-orthographic.  Near/far size ratio ≈ 1.35× instead of
    2.5× at FOV=350.  The model no longer balloons toward the camera on pitch.
    """

    # ── Warning thresholds — MUST match IMUWidget.ANGLE_WARN_DEG / ANGLE_CRIT_DEG
    WARN_ANGLE     = 20.0
    CRITICAL_ANGLE = 40.0

    # ── Colours shared with IMUWidget for visual consistency ──────────────────
    CLR_SAFE     = "#99FF99"
    CLR_WARN     = "#FFFF99"
    CLR_CRITICAL = "#FF4444"

    def __init__(self, parent, width=500, height=450):
        super().__init__(
            parent,
            width=width, height=height,
            bg="#1a1a1a", highlightthickness=0
        )
        self.width  = width
        self.height = height
        self.cx     = width  // 2
        self.cy     = height // 2
        self.scale  = 75            # world-unit → pixels

        # ── Camera ────────────────────────────────────────────────────────────
        # Fixed isometric-style camera sitting above-right-behind the drone.
        # Drone rotates; camera never moves.
        self._cam_azimuth   = math.radians(35)   # orbit left/right
        self._cam_elevation = math.radians(28)   # tilt down onto the drone

        # ── Perspective FOV ───────────────────────────────────────────────────
        # High value = near-orthographic = stable size during pitch/roll.
        # 1000 gives near/far ratio ≈ 1.35× (was 2.5× at 350).
        self._fov = 1000.0

        # ── 10-inch X-frame geometry (body frame) ─────────────────────────────
        # +X forward, +Y right, +Z up
        d = 1.15   # arm length from centre to motor
        self._motors = {
            "FL": [ d, -d, 0.0],   # Front-Left  — green
            "FR": [ d,  d, 0.0],   # Front-Right — green
            "RL": [-d, -d, 0.0],   # Rear-Left   — red
            "RR": [-d,  d, 0.0],   # Rear-Right  — red
        }
        self._body = {
            "CTR":  [ 0.0,  0.0,  0.0],
            "NOSE": [ 1.6,  0.0,  0.0],
            "TAIL": [-0.9,  0.0,  0.0],
            "BF_L": [ 0.4, -0.25, 0.05],
            "BF_R": [ 0.4,  0.25, 0.05],
            "BB_L": [-0.4, -0.25, 0.05],
            "BB_R": [-0.4,  0.25, 0.05],
        }

    # ══════════════════════════════════════════════════════════════════════════
    # MATH CORE
    # ══════════════════════════════════════════════════════════════════════════

    @staticmethod
    def _rot_roll(x, y, z, phi):
        """Around X (forward)."""
        cp, sp = math.cos(phi), math.sin(phi)
        return x, y * cp - z * sp, y * sp + z * cp

    @staticmethod
    def _rot_pitch(x, y, z, theta):
        """Around Y (right)."""
        ct, st = math.cos(theta), math.sin(theta)
        return x * ct + z * st, y, -x * st + z * ct

    @staticmethod
    def _rot_yaw(x, y, z, psi):
        """Around Z (up)."""
        cp, sp = math.cos(psi), math.sin(psi)
        return x * cp - y * sp, x * sp + y * cp, z

    def _body_to_world(self, x, y, z, roll_r, pitch_r, yaw_r):
        """Roll → Pitch → Yaw intrinsic body-frame rotations."""
        x, y, z = self._rot_roll(x, y, z, roll_r)
        x, y, z = self._rot_pitch(x, y, z, pitch_r)
        x, y, z = self._rot_yaw(x, y, z, yaw_r)
        return x, y, z

    def _world_to_screen(self, x, y, z):
        """
        Fixed camera transform then perspective projection.

        After both camera rotations, X is depth (into screen), Y is
        camera-right, Z is camera-up.  Canvas Y is inverted so we negate Z.

        Key: depth = FOV + x*scale.  High FOV keeps the factor near 1.0
        even when x swings ±1.5, preventing the balloon effect on pitch.
        """
        # Camera elevation — tilt so we see the top of the drone
        el  = self._cam_elevation
        x1  =  x * math.cos(el) + z * math.sin(el)
        z1  = -x * math.sin(el) + z * math.cos(el)
        x, z = x1, z1

        # Camera azimuth — orbit around the drone
        az  = self._cam_azimuth
        x2  =  x * math.cos(az) + y * math.sin(az)
        y2  = -x * math.sin(az) + y * math.cos(az)
        x, y = x2, y2

        # Perspective divide — FOV=1000 makes this near-orthographic
        depth  = self._fov + x * self.scale
        factor = self._fov / max(depth, 1.0)

        sx =  y * self.scale * factor + self.cx
        sy = -z * self.scale * factor + self.cy

        return sx, sy, x   # x = raw depth for sorting / prop sizing

    def _project_pt(self, pt, roll_r, pitch_r, yaw_r):
        x, y, z = self._body_to_world(*pt, roll_r, pitch_r, yaw_r)
        return self._world_to_screen(x, y, z)

    # ══════════════════════════════════════════════════════════════════════════
    # RENDERING
    # ══════════════════════════════════════════════════════════════════════════

    def update_orientation(self, roll: float, pitch: float, yaw: float = 0.0):
        """
        Redraw the drone model.
        roll, pitch, yaw in degrees — already ÷10 from MSP in main.py.
        """
        self.delete("all")

        roll_r  =  math.radians(roll)
        pitch_r =  math.radians(pitch)   # BF pitch+ = nose-down = correct as-is
        yaw_r   =  math.radians(yaw)     # BF yaw+ = CW from above.
                                          # _rot_yaw: nose(1,0,0) at +90 -> (0,1,0) = RIGHT ✓

        max_tilt = max(abs(roll), abs(pitch))

        # ── Background tint — matches IMUWidget colour thresholds ─────────────
        if max_tilt >= self.CRITICAL_ANGLE:
            self.config(bg="#3a0000")
        elif max_tilt >= self.WARN_ANGLE:
            self.config(bg="#1e1e00")
        else:
            self.config(bg="#1a1a1a")

        # ── Static ground grid (world space — never rotates with drone) ───────
        self._draw_ground_grid()

        # ── Project all motor positions ───────────────────────────────────────
        proj_motors = {
            k: self._project_pt(v, roll_r, pitch_r, yaw_r)
            for k, v in self._motors.items()
        }
        ctr = self._project_pt(self._body["CTR"], roll_r, pitch_r, yaw_r)

        # Draw far-to-near so near propellers render on top
        motor_order = sorted(
            proj_motors.items(), key=lambda kv: kv[1][2], reverse=True
        )

        # ── Arms + propeller discs ────────────────────────────────────────────
        for key, (sx, sy, depth) in motor_order:
            is_front   = key.startswith("F")
            arm_color  = "#00DD00" if is_front else "#DD2222"
            prop_color = "#00FF44" if is_front else "#FF4444"

            self.create_line(ctr[0], ctr[1], sx, sy,
                             fill=arm_color, width=4, capstyle=tk.ROUND)
            self.create_oval(sx - 5, sy - 5, sx + 5, sy + 5,
                             fill=arm_color, outline="")

            # Prop disc — near-orthographic means depth_factor stays ~1.0
            # so props stay consistent size regardless of attitude
            depth_factor = max(0.5, min(1.0, 1.0 / (1.0 + depth * 0.15)))
            pr  = 26 * depth_factor
            pry = pr * 0.28

            self.create_oval(sx - pr,  sy - pry,
                             sx + pr,  sy + pry,
                             outline=prop_color, width=2)
            self.create_line(sx - pr, sy, sx + pr, sy,
                             fill=prop_color, width=1)
            self.create_line(sx, sy - pry, sx, sy + pry,
                             fill=prop_color, width=1)

        # ── Fuselage plate ────────────────────────────────────────────────────
        body_pts   = ["BF_L", "BF_R", "BB_R", "BB_L"]
        proj_body  = [self._project_pt(self._body[k], roll_r, pitch_r, yaw_r)
                      for k in body_pts]
        flat_body  = [coord for sx, sy, _ in proj_body for coord in (sx, sy)]
        if len(flat_body) >= 6:
            self.create_polygon(flat_body, fill="#444444",
                                outline="#888888", width=1)

        # ── Nose arrow ────────────────────────────────────────────────────────
        nose = self._project_pt(self._body["NOSE"], roll_r, pitch_r, yaw_r)
        tail = self._project_pt(self._body["TAIL"], roll_r, pitch_r, yaw_r)
        self.create_line(tail[0], tail[1], nose[0], nose[1],
                         fill="#FFFFFF", width=2,
                         arrow=tk.LAST, arrowshape=(10, 12, 4))

        # ── Attitude indicator bars ───────────────────────────────────────────
        self._draw_attitude_bars(roll, pitch)

        # ── HUD text overlay ──────────────────────────────────────────────────
        hud = "#FF4444" if max_tilt >= self.CRITICAL_ANGLE else \
              "#FFFF44" if max_tilt >= self.WARN_ANGLE      else "#00FF00"

        for i, txt in enumerate([
            f"R: {roll:>+7.1f}°",
            f"P: {pitch:>+7.1f}°",
            f"Y: {yaw:>+7.1f}°",
        ]):
            self.create_text(10, 10 + i * 16, anchor="nw",
                             text=txt, fill=hud, font=("Consolas", 10, "bold"))

        # ── Warning banner ────────────────────────────────────────────────────
        if max_tilt >= self.CRITICAL_ANGLE:
            self.create_text(self.cx, 22, text="⚠  CRITICAL ATTITUDE  ⚠",
                             fill="#FF4444", font=("Consolas", 11, "bold"))
        elif max_tilt >= self.WARN_ANGLE:
            self.create_text(self.cx, 22, text="ATTITUDE CAUTION",
                             fill="#FFFF44", font=("Consolas", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # HELPERS
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_ground_grid(self):
        """Flat reference grid on the world Z=-1.8 plane — never rotates."""
        gz   = -1.8
        ext  = 5.0
        step = 1.0
        v = -ext
        while v <= ext + 0.001:
            x1, y1, _ = self._world_to_screen( ext, v, gz)
            x2, y2, _ = self._world_to_screen(-ext, v, gz)
            self.create_line(x1, y1, x2, y2, fill="#303030", width=1)
            x3, y3, _ = self._world_to_screen(v,  ext, gz)
            x4, y4, _ = self._world_to_screen(v, -ext, gz)
            self.create_line(x3, y3, x4, y4, fill="#303030", width=1)
            v += step
        xh1, yh1, _ = self._world_to_screen( ext, 0, gz)
        xh2, yh2, _ = self._world_to_screen(-ext, 0, gz)
        self.create_line(xh1, yh1, xh2, yh2, fill="#505050", width=2)

    def _draw_attitude_bars(self, roll: float, pitch: float):
        """
        2D overlay attitude bars — same colour thresholds as the 3D model
        and the IMUWidget table, so all three displays agree.
        """
        def bar_color(deg):
            a = abs(deg)
            if   a >= self.CRITICAL_ANGLE: return self.CLR_CRITICAL
            elif a >= self.WARN_ANGLE:     return self.CLR_WARN
            else:                          return self.CLR_SAFE

        # ── Roll bar (bottom-centre, horizontal) ──────────────────────────────
        bar_w  = 80
        bar_cx = self.cx
        bar_y  = self.height - 20
        pct_r  = max(-1.0, min(1.0, roll / 45.0))   # ±45° = full scale
        clr_r  = bar_color(roll)

        self.create_line(bar_cx - bar_w, bar_y, bar_cx + bar_w, bar_y,
                         fill="#444444", width=4)
        self.create_line(bar_cx, bar_y - 6, bar_cx, bar_y + 6,
                         fill="#888888", width=2)
        bx = bar_cx + int(pct_r * bar_w)
        self.create_oval(bx - 6, bar_y - 6, bx + 6, bar_y + 6,
                         fill=clr_r, outline="")
        self.create_text(bar_cx, bar_y + 14,
                         text="ROLL", fill="#666666", font=("Consolas", 8))

        # ── Pitch bar (right side, vertical) ─────────────────────────────────
        bar_h  = 80
        bar_x  = self.width - 18
        bar_cy = self.cy
        pct_p  = max(-1.0, min(1.0, pitch / 45.0))   # positive = nose down = bar moves down
        clr_p  = bar_color(pitch)

        self.create_line(bar_x, bar_cy - bar_h, bar_x, bar_cy + bar_h,
                         fill="#444444", width=4)
        self.create_line(bar_x - 6, bar_cy, bar_x + 6, bar_cy,
                         fill="#888888", width=2)
        by = bar_cy + int(pct_p * bar_h)
        self.create_oval(bar_x - 6, by - 6, bar_x + 6, by + 6,
                         fill=clr_p, outline="")
        self.create_text(bar_x, bar_cy - bar_h - 12,
                         text="PTCH", fill="#666666", font=("Consolas", 8))