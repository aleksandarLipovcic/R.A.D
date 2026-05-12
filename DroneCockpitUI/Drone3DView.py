import tkinter as tk
import math


class Drone3DView(tk.Canvas):
    """
    Professional Attitude Direction Indicator (ADI / Artificial Horizon)
    for a 10-inch long-range quadcopter — aviation instrument standard.

    BETAFLIGHT SIGN CONVENTION  (MSP_ATTITUDE, verified)
        roll  > 0  →  right side drops
        pitch > 0  →  nose drops
        yaw   > 0  →  nose turns right (CW from above)

    ADI behaviour:
        roll  > 0  →  horizon tilts: right end goes DOWN
        pitch > 0  →  horizon shifts UP on screen (nose drops → more sky)

    COMPASS STRIP — DUAL SOURCE (aviation cross-check standard)
    ────────────────────────────────────────────────────────────
    The compass tape scrolls on the MAGNETOMETER heading (mag_heading_deg).
    This is the primary, drift-free reference — equivalent to the magnetic
    compass on a real aircraft.

    A secondary "ghost" marker (cyan diamond) shows where the FC gyro-
    integrated yaw sits on the same tape.  When both overlap the aircraft
    is tracking true; when they diverge the pilot sees it immediately as
    the gap widens.

    A small divergence badge (Δ value) is printed inside the HDG box when
    |mag − gyro| > 5° so the crew always has a numeric cross-check.

    Colour coding of the HDG box outline:
        green   |Δ| <  15°   headings agree
        yellow  |Δ| <  30°   monitor
        red     |Δ| ≥  30°   heading adjustment required
    """

    MARGIN = 2

    WARN_ANGLE     = 15.0
    CRITICAL_ANGLE = 30.0

    PITCH_PX_PER_DEG = 4.5

    # ── Heading drift thresholds (shared with IMUWidget) ─────────────────────
    DRIFT_WARN_DEG = 15.0
    DRIFT_CRIT_DEG = 30.0

    # ── Aviation colour palette ───────────────────────────────────────────────
    C_SKY_SAFE  = "#1565C0"
    C_SKY_WARN  = "#0a2a0a"
    C_SKY_CRIT  = "#3a0808"
    C_GND_SAFE  = "#7B3F00"
    C_GND_WARN  = "#4a3000"
    C_GND_CRIT  = "#4a0808"
    C_HORIZON      = "#FFFFFF"
    C_AIRCRAFT     = "#FFD700"
    C_PITCH_LADDER = "#FFFFFF"
    C_BANK_ARC     = "#FFFFFF"
    C_BANK_PTR     = "#FFD700"
    C_WARN         = "#FFD700"
    C_CRITICAL     = "#FF3333"
    C_SAFE_HUD     = "#00FF44"
    C_BLACK        = "#000000"
    C_COMP_BG      = "#0a0a0a"
    C_COMP_SEP     = "#444444"

    # ── Dual-source compass colours ───────────────────────────────────────────
    C_MAG_MARKER   = "#FFFFFF"   # primary mag heading  — white centre triangle
    C_GYRO_MARKER  = "#00CCFF"   # secondary gyro yaw   — cyan ghost diamond
    C_HDG_AGREE    = "#00AA44"   # HDG box border when headings agree
    C_HDG_WARN     = "#CCAA00"   # HDG box border when warn
    C_HDG_CRIT     = "#FF2222"   # HDG box border when critical

    def __init__(self, parent, width=500, height=450):
        super().__init__(
            parent,
            width=width, height=height,
            bg=self.C_BLACK,
            highlightthickness=0
        )
        self._nominal_w = width
        self._nominal_h = height
        self._recompute_geometry(width, height)

    def _recompute_geometry(self, W: int, H: int):
        M = self.MARGIN
        self.W  = W
        self.H  = H
        self.cx = W // 2

        self.adi_x0 = M
        self.adi_x1 = W - M
        self.adi_y0 = M
        self.adi_h  = int(H * 0.76)
        self.adi_y1 = self.adi_h
        self.adi_cy = M + int((self.adi_h - M) * 0.54)

        self.comp_y0 = self.adi_y1
        self.comp_y1 = H - M
        self.comp_cx = self.cx

        self.arc_cx = self.cx
        self.arc_cy = self.adi_cy
        self.arc_r  = min(self.adi_cy - M - 10,
                          self.cx     - M - 20)

    # ══════════════════════════════════════════════════════════════════════════
    # PUBLIC API
    # ══════════════════════════════════════════════════════════════════════════

    def update_orientation(self,
                           roll:        float,
                           pitch:       float,
                           yaw:         float = 0.0,
                           mag_heading: float = None,
                           mag_valid:   bool  = False):
        """
        roll, pitch  — FC attitude in degrees
        yaw          — FC gyro-integrated yaw (degrees)
        mag_heading  — magnetometer heading 0–360° (or None / invalid)
        mag_valid    — True when the mag has a good lock
        """
        W = self.winfo_width()
        H = self.winfo_height()
        if W < 10 or H < 10:
            W = self._nominal_w
            H = self._nominal_h
        self._recompute_geometry(W, H)

        self.delete("all")
        self.create_rectangle(0, 0, W, H, fill=self.C_BLACK, outline="")

        max_tilt = max(abs(roll), abs(pitch))
        if   max_tilt >= self.CRITICAL_ANGLE: level = "critical"
        elif max_tilt >= self.WARN_ANGLE:     level = "warn"
        else:                                 level = "safe"

        # ── Heading sources ───────────────────────────────────────────────────
        # The compass tape ALWAYS scrolls on the FC gyro-integrated yaw.
        # The magnetometer is shown as a ghost marker for cross-check only.
        # Gyro drives the HSI; mag is the independent reference — never the
        # other way around.
        tape_heading = yaw % 360.0          # primary: gyro yaw, always

        if mag_valid and mag_heading is not None:
            mag_marker = mag_heading % 360.0
            drift = (tape_heading - mag_marker + 540) % 360 - 180
        else:
            mag_marker = None
            drift      = 0.0

        self._draw_adi_ball(roll, pitch, level)
        self._draw_pitch_ladder(roll, pitch)
        self._draw_bank_arc(roll, level)
        self._draw_aircraft_symbol(level)
        self._draw_horizon_ref_bars()

        M   = self.MARGIN
        clr = self.C_BLACK
        self.create_rectangle(0,     0,       M,    H,    fill=clr, outline="")
        self.create_rectangle(W - M, 0,       W,    H,    fill=clr, outline="")
        self.create_rectangle(0,     0,       W,    M,    fill=clr, outline="")
        self.create_rectangle(0,     self.adi_y1, W, H,   fill=clr, outline="")

        self._draw_adi_bezel()
        self._draw_hud_numerics(roll, pitch, level)
        self._draw_compass_strip(tape_heading, mag_marker, drift, mag_valid)

        if level != "safe":
            self._draw_warning_banner(level, max_tilt)

    # ══════════════════════════════════════════════════════════════════════════
    # 1. ADI BALL
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_adi_ball(self, roll: float, pitch: float, level: str):
        roll_r = math.radians(roll)
        hcx = self.cx
        hcy = self.adi_cy - pitch * self.PITCH_PX_PER_DEG

        hdx =  math.cos(roll_r);  hdy =  math.sin(roll_r)
        sdx =  math.sin(roll_r);  sdy = -math.cos(roll_r)

        L = self.W + self.H + 200

        sky_clr = {"safe": self.C_SKY_SAFE,
                   "warn": self.C_SKY_WARN,
                   "critical": self.C_SKY_CRIT}[level]
        gnd_clr = {"safe": self.C_GND_SAFE,
                   "warn": self.C_GND_WARN,
                   "critical": self.C_GND_CRIT}[level]

        self.create_rectangle(self.adi_x0, self.adi_y0,
                              self.adi_x1, self.adi_y1,
                              fill=sky_clr, outline="")

        gnd_poly = [
            hcx - L * hdx - L * sdx,  hcy - L * hdy - L * sdy,
            hcx + L * hdx - L * sdx,  hcy + L * hdy - L * sdy,
            hcx + L * hdx,             hcy + L * hdy,
            hcx - L * hdx,             hcy - L * hdy,
        ]
        self.create_polygon(gnd_poly, fill=gnd_clr, outline="")

        hw = 2.0
        pts = [
            hcx - L * hdx + hw * sdx,  hcy - L * hdy + hw * sdy,
            hcx + L * hdx + hw * sdx,  hcy + L * hdy + hw * sdy,
            hcx + L * hdx - hw * sdx,  hcy + L * hdy - hw * sdy,
            hcx - L * hdx - hw * sdx,  hcy - L * hdy - hw * sdy,
        ]
        self.create_polygon(pts, fill=self.C_HORIZON, outline="")

    # ══════════════════════════════════════════════════════════════════════════
    # 2. PITCH LADDER
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_pitch_ladder(self, roll: float, pitch: float):
        roll_r = math.radians(roll)
        hcx = self.cx
        hcy = self.adi_cy - pitch * self.PITCH_PX_PER_DEG

        hdx =  math.cos(roll_r);  hdy =  math.sin(roll_r)
        sdx =  math.sin(roll_r);  sdy = -math.cos(roll_r)

        for deg in (-30, -25, -20, -15, -10, -5, 5, 10, 15, 20, 25, 30):
            is_major = (abs(deg) % 10 == 0)
            half_w   = 52 if is_major else 30
            hw       = 1.5 if is_major else 0.8

            offset_px = deg * self.PITCH_PX_PER_DEG
            lx = hcx + offset_px * sdx
            ly = hcy + offset_px * sdy

            if ly < self.adi_y0 - 5 or ly > self.adi_y1 + 5:
                continue

            x1 = lx - half_w * hdx;  y1 = ly - half_w * hdy
            x2 = lx + half_w * hdx;  y2 = ly + half_w * hdy

            pts = [
                x1 + hw * sdx, y1 + hw * sdy,
                x2 + hw * sdx, y2 + hw * sdy,
                x2 - hw * sdx, y2 - hw * sdy,
                x1 - hw * sdx, y1 - hw * sdy,
            ]
            self.create_polygon(pts, fill=self.C_PITCH_LADDER, outline="")

            if is_major:
                cap = 9;  cap_hw = 0.8
                for ex, ey in [(x1, y1), (x2, y2)]:
                    tx = ex - cap * sdx;  ty = ey - cap * sdy
                    cpts = [
                        ex + cap_hw * hdx, ey + cap_hw * hdy,
                        tx + cap_hw * hdx, ty + cap_hw * hdy,
                        tx - cap_hw * hdx, ty - cap_hw * hdy,
                        ex - cap_hw * hdx, ey - cap_hw * hdy,
                    ]
                    self.create_polygon(cpts, fill=self.C_PITCH_LADDER, outline="")

                gap = half_w + 16
                for sign in (-1, 1):
                    lbl_x = lx + sign * gap * hdx
                    lbl_y = ly + sign * gap * hdy
                    if self.adi_y0 < lbl_y < self.adi_y1:
                        self.create_text(lbl_x, lbl_y,
                                         text=f"{abs(deg)}",
                                         fill=self.C_PITCH_LADDER,
                                         font=("Consolas", 9, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # 3. BANK ARC
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_bank_arc(self, roll: float, level: str):
        acx = self.arc_cx
        acy = self.arc_cy
        ar  = self.arc_r

        arc_clr = {"safe":     self.C_BANK_ARC,
                   "warn":     self.C_WARN,
                   "critical": self.C_CRITICAL}[level]

        self.create_arc(
            acx - ar, acy - ar, acx + ar, acy + ar,
            start=0, extent=180,
            style=tk.ARC, outline=arc_clr, width=2
        )

        for bank_deg, tick_len, label_str in [
            (-90, 16, "90"), (-60, 16, "60"), (-45, 12, None),
            (-30, 16, "30"), (-20,  8, None), (-10,  8, None),
            ( 10,  8, None), ( 20,  8, None), ( 30, 16, "30"),
            ( 45, 12, None), ( 60, 16, "60"), ( 90, 16, "90"),
        ]:
            ma = math.radians(90 - bank_deg)
            ox = acx + ar * math.cos(ma)
            oy = acy - ar * math.sin(ma)
            if oy > acy + 4:
                continue
            ix = acx + (ar - tick_len) * math.cos(ma)
            iy = acy - (ar - tick_len) * math.sin(ma)
            self.create_line(ix, iy, ox, oy, fill=arc_clr, width=2)

            if label_str:
                lx = acx + (ar + 14) * math.cos(ma)
                ly = acy - (ar + 14) * math.sin(ma)
                if self.adi_x0 < lx < self.adi_x1 and self.adi_y0 < ly < self.adi_y1:
                    self.create_text(lx, ly, text=label_str,
                                     fill=arc_clr, font=("Consolas", 8))

        pa = math.radians(90 - roll)
        tip_x = acx + (ar - 20) * math.cos(pa)
        tip_y = acy - (ar - 20) * math.sin(pa)
        bas_x = acx + ar * math.cos(pa)
        bas_y = acy - ar * math.sin(pa)
        perp  = pa + math.pi / 2
        b1x = bas_x + 8 * math.cos(perp);  b1y = bas_y - 8 * math.sin(perp)
        b2x = bas_x - 8 * math.cos(perp);  b2y = bas_y + 8 * math.sin(perp)
        ptr_clr = {"safe":     self.C_BANK_PTR,
                   "warn":     self.C_WARN,
                   "critical": self.C_CRITICAL}[level]
        self.create_polygon(tip_x, tip_y, b1x, b1y, b2x, b2y,
                            fill=ptr_clr, outline=arc_clr, width=1)

        self.create_line(acx, acy - ar, acx, acy - ar + 20,
                         fill=self.C_BANK_ARC, width=3)

    # ══════════════════════════════════════════════════════════════════════════
    # 4. AIRCRAFT SYMBOL
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_aircraft_symbol(self, level: str):
        cx  = self.cx
        cy  = self.adi_cy
        clr = {"safe":     self.C_AIRCRAFT,
               "warn":     self.C_WARN,
               "critical": self.C_CRITICAL}[level]

        bar_len = 55;  bar_w = 5;  drop = 12;  gap = 12

        self.create_line(cx - gap, cy, cx - gap - bar_len, cy,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
        self.create_line(cx - gap - bar_len, cy,
                         cx - gap - bar_len, cy + drop,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
        self.create_line(cx + gap, cy, cx + gap + bar_len, cy,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
        self.create_line(cx + gap + bar_len, cy,
                         cx + gap + bar_len, cy + drop,
                         fill=clr, width=bar_w, capstyle=tk.ROUND)
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
        M = self.MARGIN
        self.create_rectangle(M, M, self.W - M - 1, self.adi_y1 - 1,
                              outline="#666666", width=1, fill="")
        self.create_line(M, self.adi_y1, self.W - M, self.adi_y1,
                         fill=self.C_COMP_SEP, width=2)

    # ══════════════════════════════════════════════════════════════════════════
    # 7. HUD NUMERICS
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_hud_numerics(self, roll: float, pitch: float, level: str):
        hud = {"safe":     self.C_SAFE_HUD,
               "warn":     self.C_WARN,
               "critical": self.C_CRITICAL}[level]
        M = self.MARGIN + 4
        self.create_text(M, M + 4,  anchor="nw",
                         text=f"R  {roll:>+6.1f}\u00b0",
                         fill=hud, font=("Consolas", 10, "bold"))
        self.create_text(M, M + 22, anchor="nw",
                         text=f"P  {pitch:>+6.1f}\u00b0",
                         fill=hud, font=("Consolas", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # 8. COMPASS STRIP — dual-source heading display
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_compass_strip(self,
                            tape_heading: float,
                            mag_marker:   float | None,
                            drift:        float,
                            mag_valid:    bool):
        """
        tape_heading  — drives the scrolling tape: always the FC gyro yaw.
        mag_marker    — magnetometer heading plotted as a cyan ghost diamond
                        on the tape for cross-check. None when mag invalid.
        drift         — signed difference (gyro − mag), degrees.
        mag_valid     — controls source badge and whether ghost marker shows.
        """
        x0  = self.adi_x0
        x1  = self.adi_x1
        y0  = self.comp_y0
        y1  = self.comp_y1
        cx  = self.comp_cx
        ppd = 3.0

        # ── Background ────────────────────────────────────────────────────────
        self.create_rectangle(x0, y0, x1, y1,
                              fill=self.C_COMP_BG, outline="")
        self.create_line(x0, y1 - 1, x1, y1 - 1, fill="#444444", width=1)

        strip_half_deg = (x1 - x0) / ppd / 2.0 + 15.0

        # ── Small ticks every 5° ─────────────────────────────────────────────
        off = -strip_half_deg
        while off <= strip_half_deg:
            sx = cx + off * ppd
            if x0 <= sx <= x1:
                self.create_line(sx, y0 + 3, sx, y0 + 8,
                                 fill="#555555", width=1)
            off += 5.0

        # ── Major ticks + labels every 10° ───────────────────────────────────
        start_hdg = math.floor((tape_heading - strip_half_deg) / 10.0) * 10.0
        hdg = start_hdg
        while True:
            offset_deg = hdg - tape_heading
            while offset_deg >  180.0: offset_deg -= 360.0
            while offset_deg < -180.0: offset_deg += 360.0

            sx = cx + offset_deg * ppd
            if sx > x1 + 20:
                break

            if x0 <= sx <= x1:
                hdg_norm = int(round(hdg)) % 360
                if hdg_norm < 0:
                    hdg_norm += 360

                is_30    = (hdg_norm % 30 == 0)
                tick_h   = 14 if is_30 else 9
                tick_w   =  2 if is_30 else 1
                tick_clr = "#EEEEEE" if is_30 else "#888888"

                self.create_line(sx, y0 + 3, sx, y0 + 3 + tick_h,
                                 fill=tick_clr, width=tick_w)

                if is_30:
                    label = {0: "N", 90: "E", 180: "S", 270: "W"}.get(
                        hdg_norm, f"{hdg_norm:03d}"
                    )
                    is_card = hdg_norm in (0, 90, 180, 270)
                    self.create_text(
                        sx, y0 + 26,
                        text=label,
                        fill="#FFFFFF" if is_card else "#BBBBBB",
                        font=("Consolas", 9, "bold") if is_card else ("Consolas", 8)
                    )

            hdg += 10.0

        # ── Primary centre reference triangle (mag / tape) ───────────────────
        self.create_polygon(cx - 6, y0,
                            cx + 6, y0,
                            cx,     y0 + 9,
                            fill=self.C_MAG_MARKER, outline="")

        # ── Mag ghost marker (cyan diamond) ─────────────────────────────────
        # Shown when mag is valid so the pilot can cross-check gyro vs mag.
        # The diamond sits at the mag heading position on the gyro-driven tape.
        if mag_valid and mag_marker is not None:
            mag_offset = (mag_marker - tape_heading + 540) % 360 - 180
            mx_px = cx + mag_offset * ppd

            if x0 + 4 <= mx_px <= x1 - 4:
                d = 6   # diamond half-size in px
                self.create_polygon(
                    mx_px,     y0 + 2,
                    mx_px + d, y0 + 2 + d,
                    mx_px,     y0 + 2 + d * 2,
                    mx_px - d, y0 + 2 + d,
                    fill="",
                    outline=self.C_GYRO_MARKER,
                    width=2
                )
                # Vertical stem down from diamond
                self.create_line(mx_px, y0 + 2 + d * 2,
                                 mx_px, y0 + 14,
                                 fill=self.C_GYRO_MARKER, width=1,
                                 dash=(3, 2))

        # ── Source badge (top-left of strip) ─────────────────────────────────
        # Tape is always GYRO.  Show MAG LOCK when mag provides a ghost marker.
        badge_txt = "GYRO"
        badge_clr = "#FFAA00"
        self.create_text(x0 + 4, y0 + 4, anchor="nw",
                         text=badge_txt,
                         fill=badge_clr,
                         font=("Consolas", 7, "bold"))
        if mag_valid:
            self.create_text(x0 + 4, y0 + 14, anchor="nw",
                             text="MAG◆",
                             fill=self.C_GYRO_MARKER,
                             font=("Consolas", 7, "bold"))
        # ── HDG readout box — colour reflects divergence ───────────────────────
        abs_drift = abs(drift)
        if   abs_drift >= self.DRIFT_CRIT_DEG: box_outline = self.C_HDG_CRIT
        elif abs_drift >= self.DRIFT_WARN_DEG: box_outline = self.C_HDG_WARN
        else:                                  box_outline = self.C_HDG_AGREE

        box_w, box_h = 130, 22
        ccy = (y0 + y1) // 2
        bx  = cx - box_w // 2
        by  = ccy + 1
        self.create_rectangle(bx, by, bx + box_w, by + box_h,
                              fill="#0a1a0a", outline=box_outline, width=2)
        self.create_text(cx, by + box_h // 2,
                         text=f"HDG  {tape_heading % 360:05.1f}\u00b0",
                         fill="#00FF88",
                         font=("Consolas", 10, "bold"))

        # ── Drift badge inside the HDG box (only when meaningful) ────────────
        if mag_valid and mag_marker is not None and abs_drift >= 5.0:
            drift_txt = f"\u0394{drift:>+5.1f}\u00b0"
            drift_clr = self.C_HDG_CRIT if abs_drift >= self.DRIFT_CRIT_DEG \
                        else self.C_HDG_WARN
            # Draw to the right of the main HDG text
            self.create_text(bx + box_w - 3, by + box_h // 2,
                             anchor="e",
                             text=drift_txt,
                             fill=drift_clr,
                             font=("Consolas", 7, "bold"))

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

        M  = self.MARGIN
        bw = 120
        self.create_rectangle(self.cx - bw, M + 36,
                              self.cx + bw, M + 57,
                              fill=self.C_BLACK, outline=clr, width=1)
        self.create_text(self.cx, M + 46,
                         text=txt, fill=clr, font=fnt)