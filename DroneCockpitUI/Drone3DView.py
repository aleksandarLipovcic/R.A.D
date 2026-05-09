import tkinter as tk
import math


class Drone3DView(tk.Canvas):
    """
    Professional Attitude Direction Indicator (ADI / Artificial Horizon)
    for a 10-inch long-range quadcopter — aviation instrument standard.

    FIXES vs previous version
    ─────────────────────────
    1. Sky/ground direction vectors corrected.
       Previous: sdx = -sin(r), sdy = +cos(r)  → "sky" pointed DOWNWARD at roll=0
       Fixed:    sdx = +sin(r), sdy = -cos(r)  → "sky" points screen-UP at roll=0
       Result:   level flight now shows sky-blue top half, brown bottom half.

    2. Compass tape always visible — complete rewrite of tick loop.
       Previous: step=5°, condition round(hdg)%30==0 → missed labels whenever
                 yaw+offset doesn't land exactly on a multiple of 30.
       Fixed:    pre-compute the nearest lower 10° snap of yaw, then iterate
                 over multiples of 10 across the visible range.  Every label
                 position is exact — no floating-point miss.

    BETAFLIGHT SIGN CONVENTION  (MSP_ATTITUDE, verified)
    ──────────────────────────────────────────────────────
        roll  > 0  →  right side drops
        pitch > 0  →  nose drops
        yaw   > 0  →  nose turns right (CW from above)

    ADI behaviour:
        roll  > 0  →  horizon tilts:  right end goes DOWN
        pitch > 0  →  horizon shifts UP on screen (nose drops → more sky)
        yaw        →  compass tape scrolls right
    """

    # ── Warning thresholds — must match IMUWidget ─────────────────────────────
    WARN_ANGLE     = 15.0
    CRITICAL_ANGLE = 30.0

    # ── Pitch ladder scale ────────────────────────────────────────────────────
    PITCH_PX_PER_DEG = 4.0

    # ── Aviation colour palette ───────────────────────────────────────────────
    C_SKY_SAFE  = "#1565C0"    # deep aviation blue
    C_SKY_WARN  = "#0a2a0a"    # dark olive-green
    C_SKY_CRIT  = "#3a0808"    # dark red

    C_GND_SAFE  = "#7B3F00"    # warm brown
    C_GND_WARN  = "#4a3000"    # darker amber
    C_GND_CRIT  = "#4a0808"    # reddish brown

    C_HORIZON      = "#FFFFFF"
    C_AIRCRAFT     = "#FFD700"   # aviation gold
    C_PITCH_LADDER = "#FFFFFF"
    C_BANK_ARC     = "#FFFFFF"
    C_BANK_PTR     = "#FFD700"
    C_WARN         = "#FFD700"
    C_CRITICAL     = "#FF3333"
    C_SAFE_HUD     = "#00FF44"

    def __init__(self, parent, width=500, height=450):
        super().__init__(
            parent,
            width=width, height=height,
            bg="#000000", highlightthickness=0
        )
        self.W  = width
        self.H  = height
        self.cx = width // 2

        # ADI = top 78 %, compass strip = bottom 22 %
        self.adi_h   = int(height * 0.78)
        self.comp_y0 = self.adi_h
        self.adi_cy  = int(self.adi_h * 0.52)   # ADI vertical centre

        # Bank arc geometry
        self.arc_cy = int(self.adi_h * 0.91)
        self.arc_r  = int(self.adi_h * 0.86)

    # ══════════════════════════════════════════════════════════════════════════
    # PUBLIC API
    # ══════════════════════════════════════════════════════════════════════════

    def update_orientation(self, roll: float, pitch: float, yaw: float = 0.0):
        """
        Redraw complete ADI.
        roll, pitch, yaw in degrees (already ÷10 from MSP in main.py).
        """
        self.delete("all")

        max_tilt = max(abs(roll), abs(pitch))
        if   max_tilt >= self.CRITICAL_ANGLE: level = "critical"
        elif max_tilt >= self.WARN_ANGLE:     level = "warn"
        else:                                 level = "safe"

        self._draw_adi_ball(roll, pitch, level)
        self._draw_pitch_ladder(roll, pitch)
        self._draw_bank_arc(roll, level)
        self._draw_aircraft_symbol(level)
        self._draw_horizon_ref_bars()
        self._draw_adi_bezel()
        self._draw_hud_numerics(roll, pitch, level)
        self._draw_compass_strip(yaw)
        if level != "safe":
            self._draw_warning_banner(level, max_tilt)

    # ══════════════════════════════════════════════════════════════════════════
    # 1. ADI BALL — sky / ground fill with tilting horizon
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_adi_ball(self, roll: float, pitch: float, level: str):
        """
        Sky is ABOVE the horizon, ground BELOW.

        Coordinate convention
        ─────────────────────
        Canvas X grows rightward, canvas Y grows DOWNward.

        Along-horizon unit vector at bank angle r:
            hdx = cos(r),  hdy = sin(r)

        Perpendicular pointing toward SKY (screen-up direction rotated by r):
            At r=0: sky is straight up → screen-dy = -1
            sdx =  sin(r),  sdy = -cos(r)      ← CORRECTED  (was -sin, +cos)

        Ground polygon = the half-plane below the horizon
            = horizon centre  -  large*sky_direction
        """
        roll_r = math.radians(roll)

        # Horizon centre: pitch+ = nose-down → horizon moves UP on screen (−y)
        hcx = self.cx
        hcy = self.adi_cy - pitch * self.PITCH_PX_PER_DEG

        # Along-horizon unit vector
        hdx =  math.cos(roll_r)
        hdy =  math.sin(roll_r)

        # Sky direction (perpendicular, screen-upward when level)  ← KEY FIX
        sdx =  math.sin(roll_r)
        sdy = -math.cos(roll_r)

        L = self.W + self.H + 100   # large enough to overpaint the whole canvas

        sky_clr = {"safe": self.C_SKY_SAFE,
                   "warn": self.C_SKY_WARN,
                   "critical": self.C_SKY_CRIT}[level]
        gnd_clr = {"safe": self.C_GND_SAFE,
                   "warn": self.C_GND_WARN,
                   "critical": self.C_GND_CRIT}[level]

        # Base fill — everything starts as sky (handles extreme-pitch case)
        self.create_rectangle(0, 0, self.W, self.adi_h, fill=sky_clr, outline="")

        # Ground polygon — the half-plane on the opposite side of sky
        # Four corners: go far along horizon in both directions, then far in
        # the ground direction (-sky).
        gnd = [
            hcx - L * hdx - L * sdx,  hcy - L * hdy - L * sdy,
            hcx + L * hdx - L * sdx,  hcy + L * hdy - L * sdy,
            hcx + L * hdx + L * sdx,  hcy + L * hdy + L * sdy,
            hcx - L * hdx + L * sdx,  hcy - L * hdy + L * sdy,
        ]
        # Only the bottom two corners (the "+sdx/sdy" ones) are on the sky
        # side of the horizon — we want the ground polygon to cover the region
        # below. Using: far-left-ground, far-right-ground, far-right-sky (=horizon),
        # far-left-sky (=horizon) gives a half-plane:
        gnd_poly = [
            hcx - L * hdx - L * sdx,  hcy - L * hdy - L * sdy,   # far-left  ground
            hcx + L * hdx - L * sdx,  hcy + L * hdy - L * sdy,   # far-right ground
            hcx + L * hdx,             hcy + L * hdy,              # far-right horizon
            hcx - L * hdx,             hcy - L * hdy,              # far-left  horizon
        ]
        self.create_polygon(gnd_poly, fill=gnd_clr, outline="")

        # Horizon line
        self.create_line(
            hcx - L * hdx, hcy - L * hdy,
            hcx + L * hdx, hcy + L * hdy,
            fill=self.C_HORIZON, width=3
        )

    # ══════════════════════════════════════════════════════════════════════════
    # 2. PITCH LADDER
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_pitch_ladder(self, roll: float, pitch: float):
        """
        Horizontal tick marks at ±5°, ±10°, ±15°, ±20°, ±25°, ±30°.
        Labels on both sides at multiples of 10°.
        Lines scroll with the horizon (pitch) and tilt with roll.
        """
        roll_r = math.radians(roll)
        hcx = self.cx
        hcy = self.adi_cy - pitch * self.PITCH_PX_PER_DEG

        hdx =  math.cos(roll_r);  hdy =  math.sin(roll_r)
        sdx =  math.sin(roll_r);  sdy = -math.cos(roll_r)   # sky direction

        for deg in (-30, -25, -20, -15, -10, -5, 5, 10, 15, 20, 25, 30):
            is_major = (abs(deg) % 10 == 0)
            half_w   = 50 if is_major else 28
            lw       = 2  if is_major else 1

            # Offset in sky direction: positive deg = above horizon
            offset_px = deg * self.PITCH_PX_PER_DEG
            lx = hcx + offset_px * sdx
            ly = hcy + offset_px * sdy

            if ly < -10 or ly > self.adi_h + 10:
                continue

            x1 = lx - half_w * hdx;  y1 = ly - half_w * hdy
            x2 = lx + half_w * hdx;  y2 = ly + half_w * hdy
            self.create_line(x1, y1, x2, y2, fill=self.C_PITCH_LADDER, width=lw)

            # End-cap ticks on major lines pointing toward horizon
            if is_major:
                cap = 8
                for ex, ey in [(x1, y1), (x2, y2)]:
                    # Tick points toward horizon (opposite sky direction)
                    tx = ex - cap * sdx
                    ty = ey - cap * sdy
                    self.create_line(ex, ey, tx, ty,
                                     fill=self.C_PITCH_LADDER, width=lw)

            # Labels on both sides (major lines only)
            if is_major:
                gap = half_w + 14
                for sign in (-1, 1):
                    lbl_x = lx + sign * gap * hdx
                    lbl_y = ly + sign * gap * hdy
                    if 0 < lbl_y < self.adi_h:
                        self.create_text(lbl_x, lbl_y,
                                         text=f"{abs(deg)}",
                                         fill=self.C_PITCH_LADDER,
                                         font=("Consolas", 9, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # 3. BANK ARC
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_bank_arc(self, roll: float, level: str):
        """
        Upper semicircle with graduation ticks at ±10, ±20, ±30, ±45, ±60°.
        Gold triangle pointer moves along the arc to show current bank.
        Convention: 0° bank = pointer at 12-o'clock (top).
                    positive roll (right-down) = pointer moves right.
        """
        acx = self.cx
        acy = self.arc_cy
        ar  = self.arc_r

        arc_clr = {"safe":     self.C_BANK_ARC,
                   "warn":     self.C_WARN,
                   "critical": self.C_CRITICAL}[level]

        # Upper semicircle (Tk arc: start=0 is 3-o'clock, extent=180 → upper half)
        self.create_arc(
            acx - ar, acy - ar, acx + ar, acy + ar,
            start=0, extent=180,
            style=tk.ARC, outline=arc_clr, width=2
        )

        # Fixed graduation marks
        for bank_deg in (-60, -45, -30, -20, -10, 10, 20, 30, 45, 60):
            math_angle = math.radians(90 - bank_deg)   # 90° = top of circle
            ox = acx + ar * math.cos(math_angle)
            oy = acy - ar * math.sin(math_angle)       # canvas Y inverted
            if oy > acy + 5:
                continue
            tick_len = 14 if abs(bank_deg) % 30 == 0 else 8
            ix = acx + (ar - tick_len) * math.cos(math_angle)
            iy = acy - (ar - tick_len) * math.sin(math_angle)
            self.create_line(ix, iy, ox, oy, fill=arc_clr, width=2)
            if abs(bank_deg) in (30, 60):
                lx = acx + (ar + 12) * math.cos(math_angle)
                ly = acy - (ar + 12) * math.sin(math_angle)
                if ly < acy:
                    self.create_text(lx, ly, text=str(abs(bank_deg)),
                                     fill=arc_clr, font=("Consolas", 8))

        # Roll pointer triangle — tip inward, base on arc
        ptr_angle = math.radians(90 - roll)
        tip_x = acx + (ar - 18) * math.cos(ptr_angle)
        tip_y = acy - (ar - 18) * math.sin(ptr_angle)
        bas_x = acx + (ar - 4)  * math.cos(ptr_angle)
        bas_y = acy - (ar - 4)  * math.sin(ptr_angle)
        perp  = ptr_angle + math.pi / 2
        b1x = bas_x + 7 * math.cos(perp);  b1y = bas_y - 7 * math.sin(perp)
        b2x = bas_x - 7 * math.cos(perp);  b2y = bas_y + 7 * math.sin(perp)
        ptr_clr = {"safe": self.C_BANK_PTR,
                   "warn": self.C_WARN,
                   "critical": self.C_CRITICAL}[level]
        self.create_polygon(tip_x, tip_y, b1x, b1y, b2x, b2y,
                            fill=ptr_clr, outline=self.C_BANK_ARC, width=1)

        # Fixed 12-o'clock reference tick
        self.create_line(acx, acy - ar + 2, acx, acy - ar + 18,
                         fill=self.C_BANK_ARC, width=3)

    # ══════════════════════════════════════════════════════════════════════════
    # 4. AIRCRAFT SYMBOL — fixed yellow T-bar
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_aircraft_symbol(self, level: str):
        """
        Classic fixed ADI aircraft symbol: two horizontal wing bars + centre dot.
        Colour tracks warning level so pilot gets immediate status at a glance.
        """
        cx  = self.cx
        cy  = self.adi_cy
        clr = {"safe":     self.C_AIRCRAFT,
               "warn":     self.C_WARN,
               "critical": self.C_CRITICAL}[level]

        bar_len = 55
        bar_w   = 5
        drop    = 12
        gap     = 12

        # Left wing ─┐
        self.create_line(cx - gap, cy, cx - gap - bar_len, cy,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
        self.create_line(cx - gap - bar_len, cy,
                         cx - gap - bar_len, cy + drop,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
        # Right wing └─
        self.create_line(cx + gap, cy, cx + gap + bar_len, cy,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
        self.create_line(cx + gap + bar_len, cy,
                         cx + gap + bar_len, cy + drop,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
        # Centre dot
        r = 6
        self.create_oval(cx - r, cy - r, cx + r, cy + r,
                         fill=clr, outline="#333333", width=1)

    # ══════════════════════════════════════════════════════════════════════════
    # 5. HORIZON REFERENCE BARS
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_horizon_ref_bars(self):
        cx = self.cx;  cy = self.adi_cy
        self.create_line(cx - 11, cy, cx - 4,  cy, fill="#CCCCCC", width=2)
        self.create_line(cx + 4,  cy, cx + 11, cy, fill="#CCCCCC", width=2)

    # ══════════════════════════════════════════════════════════════════════════
    # 6. ADI BEZEL
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_adi_bezel(self):
        self.create_rectangle(0, 0, self.W - 1, self.adi_h - 1,
                              outline="#555555", width=2)
        self.create_rectangle(2, 2, self.W - 3, self.adi_h - 3,
                              outline="#222222", width=1)

    # ══════════════════════════════════════════════════════════════════════════
    # 7. HUD NUMERICS
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_hud_numerics(self, roll: float, pitch: float, level: str):
        hud = {"safe": self.C_SAFE_HUD,
               "warn": self.C_WARN,
               "critical": self.C_CRITICAL}[level]
        self.create_text(8,  8, anchor="nw",
                         text=f"R  {roll:>+6.1f}°",
                         fill=hud, font=("Consolas", 10, "bold"))
        self.create_text(8, 26, anchor="nw",
                         text=f"P  {pitch:>+6.1f}°",
                         fill=hud, font=("Consolas", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # 8. COMPASS STRIP — always-visible heading tape
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_compass_strip(self, yaw: float):
        """
        Scrolling heading tape that is ALWAYS fully populated with ticks and
        labels regardless of the current yaw value.

        Root-cause fix
        ──────────────
        Previous approach: iterate offset_deg in steps of 5, compute
        hdg = (yaw + offset_deg) % 360, check round(hdg) % 30 == 0.
        Problem: when yaw = 79°, offset = -19 → hdg = 60 → should label,
        but offset step=5 never lands on -19.  Labels only appeared when
        the drone happened to be pointing near a multiple of 30°.

        Correct approach
        ────────────────
        1. Find the first label heading to the LEFT of the visible strip:
               left_hdg = floor((yaw - strip_half_deg) / 10) * 10
        2. Iterate label headings at every 10° from left_hdg rightward.
        3. Compute the screen X for each label heading directly from its
           offset from the current yaw.  No floating-point rounding needed.
        4. Draw small intermediate ticks at every 5° separately.

        This guarantees every 10° label and every cardinal appear exactly
        once, positioned correctly, no matter what yaw is.
        """
        y0  = self.comp_y0
        y1  = self.H
        ccy = (y0 + y1) // 2
        cx  = self.cx
        ppd = 3.2          # pixels per degree

        # Background
        self.create_rectangle(0, y0, self.W, y1, fill="#111111", outline="")
        self.create_line(0, y0, self.W, y0, fill="#444444", width=1)

        strip_half_deg = self.W / ppd / 2 + 15   # degrees visible left or right

        # ── Small intermediate ticks at every 5° ──────────────────────────────
        offset = -strip_half_deg
        while offset <= strip_half_deg:
            sx = cx + offset * ppd
            if 0 <= sx <= self.W:
                self.create_line(sx, y0 + 3, sx, y0 + 8,
                                 fill="#666666", width=1)
            offset += 5.0

        # ── Major ticks + labels at every 10° ────────────────────────────────
        # Start from the nearest 10° boundary left of the visible strip
        left_hdg_raw = yaw - strip_half_deg
        start_hdg    = math.floor(left_hdg_raw / 10.0) * 10.0   # exact multiple of 10

        hdg = start_hdg
        while True:
            # Offset of this heading from current yaw in degrees
            offset_deg = hdg - yaw
            # Wrap: keep offset in [-180, +180) so the tape scrolls correctly
            while offset_deg >  180: offset_deg -= 360
            while offset_deg < -180: offset_deg += 360

            sx = cx + offset_deg * ppd
            if sx > self.W + 20:
                break

            if 0 <= sx <= self.W:
                hdg_norm = int(round(hdg)) % 360
                if hdg_norm < 0:
                    hdg_norm += 360

                is_30 = (hdg_norm % 30 == 0)
                tick_h  = 14 if is_30 else 9
                tick_clr = "#DDDDDD" if is_30 else "#AAAAAA"

                self.create_line(sx, y0 + 3, sx, y0 + 3 + tick_h,
                                 fill=tick_clr, width=2 if is_30 else 1)

                # Label at every 30°
                if is_30:
                    label = {0: "N", 90: "E", 180: "S", 270: "W"}.get(
                        hdg_norm, f"{hdg_norm:03d}"
                    )
                    is_cardinal = hdg_norm in (0, 90, 180, 270)
                    lbl_clr = "#FFFFFF" if is_cardinal else "#BBBBBB"
                    lbl_fnt = ("Consolas", 9, "bold") if is_cardinal \
                              else ("Consolas", 8)
                    self.create_text(sx, y0 + 24,
                                     text=label,
                                     fill=lbl_clr, font=lbl_fnt)

            hdg += 10.0

        # ── Centre reference triangle ─────────────────────────────────────────
        self.create_polygon(cx - 6, y0 + 1,
                            cx + 6, y0 + 1,
                            cx,     y0 + 10,
                            fill="#FFFFFF", outline="")

        # ── Heading readout box ───────────────────────────────────────────────
        hdg_str  = f"HDG  {yaw % 360:05.1f}\u00b0"
        box_w, box_h = 116, 22
        bx = cx - box_w // 2
        by = ccy + 2
        self.create_rectangle(bx, by, bx + box_w, by + box_h,
                              fill="#0a1a0a", outline="#00AA44", width=1)
        self.create_text(cx, by + box_h // 2,
                         text=hdg_str,
                         fill="#00FF88",
                         font=("Consolas", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # 9. WARNING BANNER
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_warning_banner(self, level: str, max_tilt: float):
        if level == "critical":
            txt = f"\u26a0  CRITICAL  {max_tilt:.0f}\u00b0  \u26a0"
            clr = self.C_CRITICAL
            fnt = ("Consolas", 11, "bold")
        else:
            txt = f"CAUTION  {max_tilt:.0f}\u00b0"
            clr = self.C_WARN
            fnt = ("Consolas", 10, "bold")

        self.create_rectangle(self.cx - 112, 36,
                              self.cx + 112, 56,
                              fill="#000000", outline=clr, width=1)
        self.create_text(self.cx, 46, text=txt, fill=clr, font=fnt)