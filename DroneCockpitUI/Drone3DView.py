import tkinter as tk
import math


class Drone3DView(tk.Canvas):
    """
    Attitude Indicator (ADI / artificial horizon) display for a 10-inch
    long-range quadcopter.

    ═══════════════════════════════════════════════════════════════════════════
    LAYOUT
    ═══════════════════════════════════════════════════════════════════════════

        ┌──────────────────────────────────────────────────────────────┐
        │          ╔═ bank arc + graduation marks ═╗                  │
        │          ║  ▼ roll pointer moves here   ║                  │
        │   ───────╫──────────────────────────────╫───────           │
        │           ║    SKY  (dark blue)          ║                  │
        │   +10°  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─                   │
        │   ════════════════ horizon ═══════════════  ← tilts/shifts  │
        │            ──[drone symbol, fixed]──                        │
        │   -10°  ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─                   │
        │           GROUND (dark amber)                               │
        │   ──────────────────────────────────────────────            │
        │   [    yaw heading compass strip    Y: +53°    ]           │
        └──────────────────────────────────────────────────────────────┘

    ═══════════════════════════════════════════════════════════════════════════
    WHY ADI INSTEAD OF 3D MODEL
    ═══════════════════════════════════════════════════════════════════════════
    A fixed isometric 3D camera creates ambiguity: at large angles the
    perspective changes so much that the pilot loses situational awareness.
    ADI removes that ambiguity:
      • Roll  — horizon tilt, bank arc pointer.  Immediately obvious.
      • Pitch — horizon shift up/down.  Immediately obvious.
      • Yaw   — compass strip.  Heading readout.
      • Warnings — background colour + arm colours on drone symbol.

    ═══════════════════════════════════════════════════════════════════════════
    WARNING THRESHOLDS  (must match IMUWidget)
    ═══════════════════════════════════════════════════════════════════════════
    WARN_ANGLE     = 20°
    CRITICAL_ANGLE = 40°
    """

    # ── Warning thresholds — must match IMUWidget.ANGLE_WARN_DEG / ANGLE_CRIT_DEG
    WARN_ANGLE     = 20.0
    CRITICAL_ANGLE = 40.0

    # ── Pitch scale: pixels per degree of pitch ───────────────────────────────
    # 3.5 px/° → ±30° covers ±105 px, keeps the ladder readable.
    PITCH_PX_PER_DEG = 3.5

    # ── Colours ───────────────────────────────────────────────────────────────
    CLR_SAFE     = "#00FF44"
    CLR_WARN     = "#FFFF44"
    CLR_CRITICAL = "#FF4444"

    def __init__(self, parent, width=500, height=450):
        super().__init__(
            parent,
            width=width, height=height,
            bg="#0d1b2a", highlightthickness=0
        )
        self.width   = width
        self.height  = height
        self.cx      = width  // 2

        # ADI area takes the top 78% of the canvas; compass strip gets 22%.
        self.adi_h   = int(height * 0.78)
        self.comp_y0 = self.adi_h          # top of compass strip
        self.adi_cy  = int(self.adi_h * 0.50)   # vertical centre of ADI area

    # ══════════════════════════════════════════════════════════════════════════
    # PUBLIC API
    # ══════════════════════════════════════════════════════════════════════════

    def update_orientation(self, roll: float, pitch: float, yaw: float = 0.0):
        """
        Redraw the full display.
        roll, pitch, yaw in degrees (already ÷10 from MSP in main.py).
        Betaflight sign convention:
            roll  > 0  →  right side drops
            pitch > 0  →  nose drops
            yaw   > 0  →  nose turns right (CW from above)
        """
        self.delete("all")

        max_tilt = max(abs(roll), abs(pitch))

        if   max_tilt >= self.CRITICAL_ANGLE: level = "critical"
        elif max_tilt >= self.WARN_ANGLE:     level = "warn"
        else:                                 level = "safe"

        self._draw_adi_background(roll, pitch, level)
        self._draw_pitch_ladder(roll, pitch)
        self._draw_bank_arc(roll, level)
        self._draw_drone_symbol(roll, level)
        self._draw_centre_marker()
        self._draw_adi_border()
        self._draw_compass(yaw)
        self._draw_hud(roll, pitch, yaw, level)

        if level != "safe":
            self._draw_warning_banner(level, max_tilt)

    # ══════════════════════════════════════════════════════════════════════════
    # ADI BACKGROUND  (sky / ground / horizon line)
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_adi_background(self, roll: float, pitch: float, level: str):
        """
        Fill sky (above horizon) and ground (below) using a rotated/shifted
        horizon line.  The horizon:
          • tilts  by roll  (right side drops for positive roll)
          • shifts by pitch (nose-down → horizon shifts UP, showing more sky)
        """
        roll_r = math.radians(roll)

        # Pitch shifts the horizon DOWN when pitch > 0 (nose drops → horizon goes up
        # from pilot's view, i.e. we shift the horizon DOWN in screen coords).
        pitch_offset = pitch * self.PITCH_PX_PER_DEG    # positive = horizon shifts down

        hcx = self.cx
        hcy = self.adi_cy + pitch_offset

        # Unit vectors along and perpendicular to the tilted horizon
        hdx = math.cos(roll_r)      # along horizon
        hdy = math.sin(roll_r)
        sdx =  math.sin(roll_r)     # toward sky  (perpendicular, upward in tilted frame)
        sdy = -math.cos(roll_r)

        L = self.width + self.height   # large enough to cover canvas

        # Sky polygon
        sky = [
            hcx - L * hdx + L * sdx,  hcy - L * hdy + L * sdy,
            hcx + L * hdx + L * sdx,  hcy + L * hdy + L * sdy,
            hcx + L * hdx,            hcy + L * hdy,
            hcx - L * hdx,            hcy - L * hdy,
        ]
        # Ground polygon
        gnd = [
            hcx - L * hdx - L * sdx,  hcy - L * hdy - L * sdy,
            hcx + L * hdx - L * sdx,  hcy + L * hdy - L * sdy,
            hcx + L * hdx,            hcy + L * hdy,
            hcx - L * hdx,            hcy - L * hdy,
        ]

        sky_clr = {
            "safe":     "#0d1b2a",
            "warn":     "#1a1a04",
            "critical": "#280404",
        }[level]
        gnd_clr = {
            "safe":     "#1f1006",
            "warn":     "#1f1404",
            "critical": "#240404",
        }[level]

        # Clip ADI to its area (draw sky/ground clipped vertically)
        self.create_rectangle(0, 0, self.width, self.adi_h,
                              fill=sky_clr, outline="")   # base sky fill
        self.create_polygon(gnd, fill=gnd_clr, outline="")
        self.create_polygon(sky, fill=sky_clr, outline="")

        # Horizon line
        self.create_line(
            hcx - L * hdx, hcy - L * hdy,
            hcx + L * hdx, hcy + L * hdy,
            fill="#AAAAAA", width=2
        )

    # ══════════════════════════════════════════════════════════════════════════
    # PITCH LADDER
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_pitch_ladder(self, roll: float, pitch: float):
        """
        Horizontal tick marks at ±5°, ±10°, ±20°, ±30° from the horizon.
        They move with the horizon (roll + pitch) so the pilot can read
        pitch angle directly.
        """
        roll_r = math.radians(roll)
        pitch_offset = pitch * self.PITCH_PX_PER_DEG

        # Horizon centre
        hcx = self.cx
        hcy = self.adi_cy + pitch_offset

        # Along-horizon unit vector (for drawing horizontal ticks)
        hdx = math.cos(roll_r)
        hdy = math.sin(roll_r)
        # Sky direction (perpendicular, toward sky)
        sdx =  math.sin(roll_r)
        sdy = -math.cos(roll_r)

        for deg in (-30, -20, -10, -5, 5, 10, 20, 30):
            half_w = 55 if abs(deg) % 10 == 0 else 28
            offset_px = -deg * self.PITCH_PX_PER_DEG    # neg: + pitch → line above

            lx = hcx + offset_px * sdx
            ly = hcy + offset_px * sdy

            # Clamp to ADI area
            if not (0 < ly < self.adi_h):
                continue

            self.create_line(
                lx - half_w * hdx, ly - half_w * hdy,
                lx + half_w * hdx, ly + half_w * hdy,
                fill="#556677", width=1
            )
            # Label on the right side for multiples of 10
            if abs(deg) % 10 == 0:
                lbl_x = lx + (half_w + 8) * hdx
                lbl_y = ly + (half_w + 8) * hdy
                if 0 < lbl_y < self.adi_h:
                    self.create_text(
                        lbl_x, lbl_y,
                        text=f"{abs(deg)}",
                        fill="#556677", font=("Consolas", 7)
                    )

    # ══════════════════════════════════════════════════════════════════════════
    # BANK ARC  (roll angle arc at top of ADI)
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_bank_arc(self, roll: float, level: str):
        """
        Semicircular arc with graduation marks at ±10°, ±20°, ±30°, ±45°, ±60°.
        A pointer triangle moves along the arc to show current bank angle.
        The arc lives at the top of the ADI area.
        """
        arc_cx = self.cx
        arc_cy = int(self.adi_h * 0.92)   # near top, radius points upward
        arc_r  = int(self.adi_h * 0.88)

        arc_color = {
            "safe": "#446688", "warn": "#888822", "critical": "#882222"
        }[level]

        # Draw partial arc (bottom semicircle: from 180° to 360° in standard coords,
        # but in Tk the arc goes from startangle counterclockwise.
        # We want an arc that opens UPWARD (like a ∩ shape) centred at arc_cy.
        self.create_arc(
            arc_cx - arc_r, arc_cy - arc_r,
            arc_cx + arc_r, arc_cy + arc_r,
            start=0, extent=180,
            style=tk.ARC, outline=arc_color, width=2
        )

        # Graduation marks
        for deg in (-60, -45, -30, -20, -10, 10, 20, 30, 45, 60):
            # On the top semicircle: deg=0 → top (270° in Tk = -90° math)
            # Betaflight convention: positive roll = right side down
            # Bank arc: pointer at 0° is top; positive roll pointer moves right
            angle_rad = math.radians(-deg - 90)   # screen angle for this bank mark
            tick_outer_x = arc_cx + arc_r * math.cos(angle_rad)
            tick_outer_y = arc_cy + arc_r * math.sin(angle_rad)
            tick_len = 10 if abs(deg) % 30 == 0 else 6
            tick_inner_x = arc_cx + (arc_r - tick_len) * math.cos(angle_rad)
            tick_inner_y = arc_cy + (arc_r - tick_len) * math.sin(angle_rad)
            self.create_line(tick_inner_x, tick_inner_y,
                             tick_outer_x, tick_outer_y,
                             fill=arc_color, width=1)

        # Roll pointer triangle — tracks current bank angle
        ptr_angle_rad = math.radians(-roll - 90)
        ptr_tip_x = arc_cx + (arc_r - 2) * math.cos(ptr_angle_rad)
        ptr_tip_y = arc_cy + (arc_r - 2) * math.sin(ptr_angle_rad)
        # Pointer base (slightly inside the arc)
        base_r = arc_r - 14
        perp   = ptr_angle_rad + math.pi / 2
        base_x1 = arc_cx + base_r * math.cos(ptr_angle_rad) + 6 * math.cos(perp)
        base_y1 = arc_cy + base_r * math.sin(ptr_angle_rad) + 6 * math.sin(perp)
        base_x2 = arc_cx + base_r * math.cos(ptr_angle_rad) - 6 * math.cos(perp)
        base_y2 = arc_cy + base_r * math.sin(ptr_angle_rad) - 6 * math.sin(perp)

        ptr_color = {
            "safe": "#FFFFFF", "warn": "#FFFF44", "critical": "#FF4444"
        }[level]
        self.create_polygon(
            ptr_tip_x, ptr_tip_y,
            base_x1, base_y1,
            base_x2, base_y2,
            fill=ptr_color, outline=""
        )

    # ══════════════════════════════════════════════════════════════════════════
    # DRONE SYMBOL  (fixed position, centred in ADI — arms change colour)
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_drone_symbol(self, roll: float, level: str):
        """
        The drone symbol stays fixed at the ADI centre.  In a true ADI the
        aircraft symbol never moves — the horizon moves around it, and the
        pilot reads bank from the horizon tilt and the bank pointer.

        Arms are coloured by warning level so the pilot has an immediate
        at-a-glance status even without reading the table.

        Front arms (top two) are always green-tinted at SAFE so the pilot
        can identify drone orientation; both pairs go WARN/CRITICAL together.
        """
        cx  = self.cx
        cy  = self.adi_cy
        arm = 55    # pixel length from centre to motor

        # Warning colours override the normal green/red arm split
        if level == "critical":
            front_clr = rear_clr = "#FF4444"
            body_clr  = "#441111"
        elif level == "warn":
            front_clr = rear_clr = "#FFFF44"
            body_clr  = "#333310"
        else:
            front_clr = "#00DD44"   # green — front
            rear_clr  = "#DD2222"   # red   — rear
            body_clr  = "#444444"

        # Arm direction: X-frame, 45° arms
        # Viewed from behind: front-right goes upper-right, front-left upper-left
        angles = {
            "FR": math.radians(-45),    # upper-right
            "FL": math.radians(-135),   # upper-left
            "RR": math.radians( 45),    # lower-right
            "RL": math.radians( 135),   # lower-left
        }
        colors = {
            "FR": front_clr, "FL": front_clr,
            "RR": rear_clr,  "RL": rear_clr,
        }

        prop_r = 18

        for key, ang in angles.items():
            ex = cx + arm * math.cos(ang)
            ey = cy + arm * math.sin(ang)
            clr = colors[key]

            # Arm
            self.create_line(cx, cy, ex, ey, fill=clr, width=4, capstyle=tk.ROUND)
            # Motor hub
            self.create_oval(ex - 5, ey - 5, ex + 5, ey + 5,
                             fill=clr, outline="")
            # Propeller disc
            self.create_oval(ex - prop_r, ey - prop_r * 0.35,
                             ex + prop_r, ey + prop_r * 0.35,
                             outline=clr, width=2)
            # Prop cross-hair
            self.create_line(ex - prop_r, ey, ex + prop_r, ey,
                             fill=clr, width=1)
            self.create_line(ex, ey - prop_r * 0.35,
                             ex, ey + prop_r * 0.35,
                             fill=clr, width=1)

        # Central body
        self.create_oval(cx - 12, cy - 12, cx + 12, cy + 12,
                         fill=body_clr, outline="#888888", width=2)

        # Forward direction indicator: small triangle pointing upward
        # (toward "front" in the behind-view projection)
        tri = [cx, cy - 22, cx - 7, cy - 12, cx + 7, cy - 12]
        self.create_polygon(tri, fill=front_clr, outline="")

    # ══════════════════════════════════════════════════════════════════════════
    # CENTRE REFERENCE MARKER
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_centre_marker(self):
        """Fixed horizon reference lines at the centre — like real ADI."""
        cx, cy = self.cx, self.adi_cy
        clr = "#DDDDDD"
        # Left wing bar
        self.create_line(cx - 80, cy, cx - 30, cy, fill=clr, width=3)
        self.create_line(cx - 30, cy, cx - 30, cy + 12, fill=clr, width=3)
        # Right wing bar
        self.create_line(cx + 30, cy, cx + 80, cy, fill=clr, width=3)
        self.create_line(cx + 30, cy, cx + 30, cy + 12, fill=clr, width=3)
        # Centre dot
        self.create_oval(cx - 4, cy - 4, cx + 4, cy + 4,
                         fill="#FFFFFF", outline="")

    # ══════════════════════════════════════════════════════════════════════════
    # ADI BORDER
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_adi_border(self):
        self.create_rectangle(0, 0, self.width - 1, self.adi_h - 1,
                              outline="#333333", width=1)

    # ══════════════════════════════════════════════════════════════════════════
    # COMPASS STRIP  (yaw / heading)
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_compass(self, yaw: float):
        """
        Scrolling compass tape at the bottom.  The tape moves so the current
        heading is always centred.  Marks every 10°, labels every 30°.
        Cardinal directions labelled N/E/S/W.
        """
        y0   = self.comp_y0
        y1   = self.height
        cy_c = (y0 + y1) // 2
        cx   = self.cx

        self.create_rectangle(0, y0, self.width, y1,
                              fill="#111111", outline="#333333")

        px_per_deg = 3.5    # pixels per degree of heading

        # Draw ticks centred on current yaw
        for offset_deg in range(-90, 91, 10):
            hdg = (yaw + offset_deg) % 360
            x   = cx + offset_deg * px_per_deg

            tick_h = 10 if hdg % 30 == 0 else 5
            self.create_line(x, y0 + 2, x, y0 + 2 + tick_h,
                             fill="#666666", width=1)

            if hdg % 30 == 0:
                label = {0: "N", 90: "E", 180: "S", 270: "W"}.get(int(hdg),
                         f"{int(hdg)}")
                self.create_text(x, y0 + 18,
                                 text=label, fill="#888888",
                                 font=("Consolas", 8))

        # Centre pointer
        self.create_line(cx, y0, cx, y0 + 6, fill="#FFFFFF", width=2)

        # Heading readout
        self.create_text(cx, cy_c + 6,
                         text=f"HDG  {yaw % 360:>5.1f}°",
                         fill="#00FF88", font=("Consolas", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # HUD TEXT
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_hud(self, roll: float, pitch: float, yaw: float, level: str):
        hud = {
            "safe":     "#00FF00",
            "warn":     "#FFFF44",
            "critical": "#FF4444",
        }[level]

        for i, txt in enumerate([
            f"R: {roll:>+7.1f}°",
            f"P: {pitch:>+7.1f}°",
        ]):
            self.create_text(10, 10 + i * 16, anchor="nw",
                             text=txt, fill=hud, font=("Consolas", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # WARNING BANNER
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_warning_banner(self, level: str, max_tilt: float):
        if level == "critical":
            self.create_text(
                self.cx, 22,
                text=f"⚠  CRITICAL  {max_tilt:.0f}°  ⚠",
                fill="#FF4444", font=("Consolas", 11, "bold")
            )
        else:
            self.create_text(
                self.cx, 22,
                text=f"CAUTION  {max_tilt:.0f}°",
                fill="#FFFF44", font=("Consolas", 10, "bold")
            )