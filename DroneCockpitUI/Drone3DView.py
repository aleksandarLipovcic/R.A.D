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
        yaw        →  compass tape scrolls right

    FIXES IN THIS VERSION
    ─────────────────────
    1.  BORDER / BLEED FIX
        All drawing is confined to the inset region [M, M, W-M, H-M] where
        M = MARGIN = 2 px. Two opaque "curtain" rectangles are drawn last,
        one over the full left/right edges and one over top/bottom, covering
        any polygon overshoot from the sky/ground fill (which uses L >> W to
        guarantee full coverage). The curtains use the same black as the
        canvas background so the instrument appears perfectly bounded.

    2.  ADI / COMPASS BOUNDARY
        The ADI occupies rows [M .. adi_h-1].
        The compass strip occupies rows [adi_h .. H-M-1].
        A solid separator line is drawn at exactly y=adi_h.
        Both regions start at x=M and end at x=W-M so left/right edges align.

    3.  STAIRCASE / ALIASING ON TILTED LINES
        The horizon line is drawn as a filled polygon (a thick stripe) instead
        of a single Tkinter line. Tkinter rounds line endpoints to integer
        pixels which causes a staircase on near-horizontal lines. Drawing a
        thin polygon whose top and bottom edges are the two y-values of a
        thick line forces Tk to scanline-fill between them, producing a smooth
        appearance without the staircase.
        Pitch-ladder lines use the same polygon technique.

    4.  COMPASS ALWAYS VISIBLE AND ALIGNED
        The compass background exactly covers [M, adi_h, W-M, H-M].
        Tick/label positions are computed by snapping to exact 10° multiples
        so they never miss due to floating-point rounding.
    """

    # ── Margin — all drawing inset from canvas edge by this many pixels ───────
    MARGIN = 2

    # ── Warning thresholds — must match IMUWidget ─────────────────────────────
    WARN_ANGLE     = 15.0
    CRITICAL_ANGLE = 30.0

    # ── Pitch ladder scale ────────────────────────────────────────────────────
    PITCH_PX_PER_DEG = 4.5   # slightly increased for better readability

    # ── Aviation colour palette ───────────────────────────────────────────────
    C_SKY_SAFE  = "#1565C0"    # deep aviation blue
    C_SKY_WARN  = "#0a2a0a"
    C_SKY_CRIT  = "#3a0808"
    C_GND_SAFE  = "#7B3F00"    # warm brown
    C_GND_WARN  = "#4a3000"
    C_GND_CRIT  = "#4a0808"
    C_HORIZON      = "#FFFFFF"
    C_AIRCRAFT     = "#FFD700"   # aviation gold
    C_PITCH_LADDER = "#FFFFFF"
    C_BANK_ARC     = "#FFFFFF"
    C_BANK_PTR     = "#FFD700"
    C_WARN         = "#FFD700"
    C_CRITICAL     = "#FF3333"
    C_SAFE_HUD     = "#00FF44"
    C_BLACK        = "#000000"
    C_COMP_BG      = "#0a0a0a"   # near-black for compass strip
    C_COMP_SEP     = "#444444"   # separator line colour

    def __init__(self, parent, width=500, height=450):
        super().__init__(
            parent,
            width=width, height=height,
            bg=self.C_BLACK,
            highlightthickness=0
        )
        # Nominal size used only before the widget is mapped.
        # _recompute_geometry() refreshes these from winfo_width/height each frame.
        self._nominal_w = width
        self._nominal_h = height
        self._recompute_geometry(width, height)

    def _recompute_geometry(self, W: int, H: int):
        """
        Compute all layout variables from the actual rendered canvas size.
        Called at the start of every update_orientation() so the instrument
        fills whatever pixel area the canvas actually occupies, regardless of
        how the widget was packed/placed by the parent.
        """
        M = self.MARGIN
        self.W  = W
        self.H  = H
        self.cx = W // 2

        # ADI drawing region
        self.adi_x0 = M
        self.adi_x1 = W - M
        self.adi_y0 = M
        # ADI takes 76 % of height; compass strip gets the remaining 24 %
        self.adi_h  = int(H * 0.76)
        self.adi_y1 = self.adi_h
        self.adi_cy = M + int((self.adi_h - M) * 0.54)   # aircraft-symbol row

        # Compass strip
        self.comp_y0 = self.adi_y1
        self.comp_y1 = H - M
        self.comp_cx = self.cx

        # Bank arc — centred at aircraft symbol
        self.arc_cx = self.cx
        self.arc_cy = self.adi_cy
        self.arc_r  = min(self.adi_cy - M - 10,
                          self.cx     - M - 20)

    # ══════════════════════════════════════════════════════════════════════════
    # PUBLIC API
    # ══════════════════════════════════════════════════════════════════════════

    def update_orientation(self, roll: float, pitch: float, yaw: float = 0.0):
        # ── Read actual rendered canvas size every frame ───────────────────────
        # winfo_width/height return 1 before the widget is mapped; fall back to
        # the nominal size passed to __init__ in that case.
        W = self.winfo_width()
        H = self.winfo_height()
        if W < 10 or H < 10:
            W = self._nominal_w
            H = self._nominal_h
        self._recompute_geometry(W, H)

        self.delete("all")

        # Full-canvas black base — covers every pixel at the true rendered size
        self.create_rectangle(0, 0, W, H, fill=self.C_BLACK, outline="")

        max_tilt = max(abs(roll), abs(pitch))
        if   max_tilt >= self.CRITICAL_ANGLE: level = "critical"
        elif max_tilt >= self.WARN_ANGLE:     level = "warn"
        else:                                 level = "safe"

        # ── ADI layers (back → front) ─────────────────────────────────────────
        self._draw_adi_ball(roll, pitch, level)
        self._draw_pitch_ladder(roll, pitch)
        self._draw_bank_arc(roll, level)
        self._draw_aircraft_symbol(level)
        self._draw_horizon_ref_bars()

        # ── Edge curtains — hide polygon overshoot at every canvas edge ──────
        M   = self.MARGIN
        clr = self.C_BLACK
        self.create_rectangle(0,     0,       M,    H,    fill=clr, outline="")  # left
        self.create_rectangle(W - M, 0,       W,    H,    fill=clr, outline="")  # right
        self.create_rectangle(0,     0,       W,    M,    fill=clr, outline="")  # top
        self.create_rectangle(0,     self.adi_y1, W, H,   fill=clr, outline="")  # below ADI

        # ── ADI bezel (drawn over curtains so border is sharp) ────────────────
        self._draw_adi_bezel()

        # ── HUD text (drawn after bezel so text is on top) ────────────────────
        self._draw_hud_numerics(roll, pitch, level)

        # ── Compass strip (fills exactly from adi_y1 to comp_y1) ─────────────
        self._draw_compass_strip(yaw)

        # ── Warning banner (topmost layer) ────────────────────────────────────
        if level != "safe":
            self._draw_warning_banner(level, max_tilt)

    # ══════════════════════════════════════════════════════════════════════════
    # 1. ADI BALL — sky / ground fill
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_adi_ball(self, roll: float, pitch: float, level: str):
        """
        Sky above horizon (blue), ground below (brown).

        Vector maths
        ────────────
        Along-horizon unit vector at bank angle r (r > 0 → right side drops):
            hdx = cos(r),   hdy = sin(r)

        Sky-direction perpendicular (points screen-UP at r=0):
            sdx = sin(r),   sdy = -cos(r)

        Betaflight pitch+ = nose-down → horizon moves UP on screen:
            hcy = adi_cy - pitch * PITCH_PX_PER_DEG
        """
        roll_r = math.radians(roll)
        hcx = self.cx
        hcy = self.adi_cy - pitch * self.PITCH_PX_PER_DEG

        hdx =  math.cos(roll_r);  hdy =  math.sin(roll_r)
        sdx =  math.sin(roll_r);  sdy = -math.cos(roll_r)

        # L large enough to guarantee full coverage; curtains hide the excess
        L = self.W + self.H + 200

        sky_clr = {"safe": self.C_SKY_SAFE,
                   "warn": self.C_SKY_WARN,
                   "critical": self.C_SKY_CRIT}[level]
        gnd_clr = {"safe": self.C_GND_SAFE,
                   "warn": self.C_GND_WARN,
                   "critical": self.C_GND_CRIT}[level]

        # Sky base fill (entire ADI area; ground polygon overwrites lower half)
        self.create_rectangle(self.adi_x0, self.adi_y0,
                              self.adi_x1, self.adi_y1,
                              fill=sky_clr, outline="")

        # Ground half-plane polygon
        gnd_poly = [
            hcx - L * hdx - L * sdx,  hcy - L * hdy - L * sdy,
            hcx + L * hdx - L * sdx,  hcy + L * hdy - L * sdy,
            hcx + L * hdx,             hcy + L * hdy,
            hcx - L * hdx,             hcy - L * hdy,
        ]
        self.create_polygon(gnd_poly, fill=gnd_clr, outline="")

        # ── Horizon rendered as a filled polygon stripe (fixes staircase) ─────
        # A thick horizon "band" is a parallelogram of height `hw` pixels,
        # centred on the horizon line. Scanline-filling this polygon is smooth
        # whereas a Tkinter line would round each endpoint separately.
        hw = 2.0   # half-width of horizon stripe in pixels
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
        """
        Pitch graduation lines drawn as thin filled polygons (not lines) to
        avoid the staircase effect on tilted segments.
        Labels on both sides of major (10°-multiple) lines.
        """
        roll_r = math.radians(roll)
        hcx = self.cx
        hcy = self.adi_cy - pitch * self.PITCH_PX_PER_DEG

        hdx =  math.cos(roll_r);  hdy =  math.sin(roll_r)
        sdx =  math.sin(roll_r);  sdy = -math.cos(roll_r)

        for deg in (-30, -25, -20, -15, -10, -5, 5, 10, 15, 20, 25, 30):
            is_major = (abs(deg) % 10 == 0)
            half_w   = 52 if is_major else 30
            hw       = 1.5 if is_major else 0.8   # polygon half-height

            # Centre of this tick line in screen coords
            offset_px = deg * self.PITCH_PX_PER_DEG
            lx = hcx + offset_px * sdx
            ly = hcy + offset_px * sdy

            if ly < self.adi_y0 - 5 or ly > self.adi_y1 + 5:
                continue

            # Four corners of the line-as-polygon
            x1 = lx - half_w * hdx;  y1 = ly - half_w * hdy
            x2 = lx + half_w * hdx;  y2 = ly + half_w * hdy

            pts = [
                x1 + hw * sdx, y1 + hw * sdy,
                x2 + hw * sdx, y2 + hw * sdy,
                x2 - hw * sdx, y2 - hw * sdy,
                x1 - hw * sdx, y1 - hw * sdy,
            ]
            self.create_polygon(pts, fill=self.C_PITCH_LADDER, outline="")

            # End-cap ticks on major lines (also as polygons)
            if is_major:
                cap = 9
                cap_hw = 0.8
                for ex, ey in [(x1, y1), (x2, y2)]:
                    tx = ex - cap * sdx;  ty = ey - cap * sdy
                    # Cap as a thin polygon along the sky direction
                    cpts = [
                        ex + cap_hw * hdx, ey + cap_hw * hdy,
                        tx + cap_hw * hdx, ty + cap_hw * hdy,
                        tx - cap_hw * hdx, ty - cap_hw * hdy,
                        ex - cap_hw * hdx, ey - cap_hw * hdy,
                    ]
                    self.create_polygon(cpts, fill=self.C_PITCH_LADDER,
                                        outline="")

                # Labels on both sides
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
        """
        Upper semicircle bank scale.
        Centre = (arc_cx, arc_cy) = (cx, adi_cy) = aircraft symbol centre.
        0° mark at top (12-o'clock), ±90° at aircraft-symbol height.
        Tick marks at ±10°, ±20°, ±30°, ±45°, ±60°, ±90°.
        Moving gold pointer triangle: tip inward, base on arc.
        Fixed white reference tick at 0°.
        """
        acx = self.arc_cx
        acy = self.arc_cy
        ar  = self.arc_r

        arc_clr = {"safe":     self.C_BANK_ARC,
                   "warn":     self.C_WARN,
                   "critical": self.C_CRITICAL}[level]

        # Semicircular arc — Tk arc: start=0 is 3-o'clock, extent=180 → top half
        self.create_arc(
            acx - ar, acy - ar, acx + ar, acy + ar,
            start=0, extent=180,
            style=tk.ARC, outline=arc_clr, width=2
        )

        # Graduation marks
        for bank_deg, tick_len, label_str in [
            (-90, 16, "90"), (-60, 16, "60"), (-45, 12, None),
            (-30, 16, "30"), (-20,  8, None), (-10,  8, None),
            ( 10,  8, None), ( 20,  8, None), ( 30, 16, "30"),
            ( 45, 12, None), ( 60, 16, "60"), ( 90, 16, "90"),
        ]:
            # math_angle: bank=0 → 90° (top); positive bank → pointer moves right
            ma = math.radians(90 - bank_deg)
            ox = acx + ar * math.cos(ma)
            oy = acy - ar * math.sin(ma)
            if oy > acy + 4:
                continue   # skip bottom-half ticks
            ix = acx + (ar - tick_len) * math.cos(ma)
            iy = acy - (ar - tick_len) * math.sin(ma)
            self.create_line(ix, iy, ox, oy, fill=arc_clr, width=2)

            if label_str:
                lx = acx + (ar + 14) * math.cos(ma)
                ly = acy - (ar + 14) * math.sin(ma)
                if self.adi_x0 < lx < self.adi_x1 and self.adi_y0 < ly < self.adi_y1:
                    self.create_text(lx, ly, text=label_str,
                                     fill=arc_clr, font=("Consolas", 8))

        # Moving roll-pointer triangle (tip inward, base on arc)
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

        # Fixed 0° reference tick (pointing inward from top)
        self.create_line(acx, acy - ar,
                         acx, acy - ar + 20,
                         fill=self.C_BANK_ARC, width=3)

    # ══════════════════════════════════════════════════════════════════════════
    # 4. AIRCRAFT SYMBOL — fixed yellow T-bar
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
    # 5. HORIZON REFERENCE BARS (fixed, white)
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_horizon_ref_bars(self):
        cx = self.cx;  cy = self.adi_cy
        self.create_line(cx - 11, cy, cx - 4,  cy, fill="#CCCCCC", width=2)
        self.create_line(cx + 4,  cy, cx + 11, cy, fill="#CCCCCC", width=2)

    # ══════════════════════════════════════════════════════════════════════════
    # 6. ADI BEZEL
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_adi_bezel(self):
        """
        Two-pixel border drawn exactly at the inset boundary.
        This sits on top of the edge curtains, so the instrument edge is crisp.
        """
        M = self.MARGIN
        # Outer border
        self.create_rectangle(M, M, self.W - M - 1, self.adi_y1 - 1,
                              outline="#666666", width=1, fill="")
        # Separator between ADI and compass
        self.create_line(M, self.adi_y1,
                         self.W - M, self.adi_y1,
                         fill=self.C_COMP_SEP, width=2)

    # ══════════════════════════════════════════════════════════════════════════
    # 7. HUD NUMERICS
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_hud_numerics(self, roll: float, pitch: float, level: str):
        hud = {"safe":     self.C_SAFE_HUD,
               "warn":     self.C_WARN,
               "critical": self.C_CRITICAL}[level]
        M = self.MARGIN + 4
        self.create_text(M,      M + 4,  anchor="nw",
                         text=f"R  {roll:>+6.1f}\u00b0",
                         fill=hud, font=("Consolas", 10, "bold"))
        self.create_text(M,      M + 22, anchor="nw",
                         text=f"P  {pitch:>+6.1f}\u00b0",
                         fill=hud, font=("Consolas", 10, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # 8. COMPASS STRIP — always visible, aligned with ADI edges
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_compass_strip(self, yaw: float):
        """
        Scrolling heading tape that fills exactly [adi_x0, comp_y0, adi_x1, comp_y1].
        Left and right edges align perfectly with the ADI bezel.

        Tick algorithm
        ──────────────
        Snap the leftmost heading to the nearest lower multiple of 10°, then
        step 10° at a time computing the exact screen X for each.  No
        floating-point rounding miss possible — each label position is exact.

        Small ticks (5° marks) are added in a separate pass.
        """
        x0  = self.adi_x0        # left  edge — same as ADI
        x1  = self.adi_x1        # right edge — same as ADI
        y0  = self.comp_y0       # top of compass
        y1  = self.comp_y1       # bottom of compass
        cx  = self.comp_cx
        ppd = 3.0                # pixels per degree — tuned to strip width

        # ── Background — exactly fills the compass region ─────────────────────
        self.create_rectangle(x0, y0, x1, y1,
                              fill=self.C_COMP_BG, outline="")

        # ── Top separator already drawn by _draw_adi_bezel; draw bottom border
        self.create_line(x0, y1 - 1, x1, y1 - 1,
                         fill="#444444", width=1)

        strip_half_deg = (x1 - x0) / ppd / 2.0 + 15.0

        # Small ticks every 5°
        off = -strip_half_deg
        while off <= strip_half_deg:
            sx = cx + off * ppd
            if x0 <= sx <= x1:
                self.create_line(sx, y0 + 3, sx, y0 + 8,
                                 fill="#555555", width=1)
            off += 5.0

        # Major ticks + labels every 10°
        start_hdg = math.floor((yaw - strip_half_deg) / 10.0) * 10.0

        hdg = start_hdg
        while True:
            offset_deg = hdg - yaw
            # Wrap offset into [-180, +180)
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
                        font=("Consolas", 9, "bold") if is_card
                             else ("Consolas", 8)
                    )

            hdg += 10.0

        # ── Centre reference triangle ─────────────────────────────────────────
        self.create_polygon(cx - 6, y0,
                            cx + 6, y0,
                            cx,     y0 + 9,
                            fill="#FFFFFF", outline="")

        # ── Heading readout box — centred in strip ────────────────────────────
        box_w, box_h = 120, 22
        ccy = (y0 + y1) // 2
        bx  = cx - box_w // 2
        by  = ccy + 1
        self.create_rectangle(bx, by, bx + box_w, by + box_h,
                              fill="#0a1a0a", outline="#00AA44", width=1)
        self.create_text(cx, by + box_h // 2,
                         text=f"HDG  {yaw % 360:05.1f}\u00b0",
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
            
        M = self.MARGIN
        bw = 120
        self.create_rectangle(self.cx - bw, M + 36,
                              self.cx + bw, M + 57,
                              fill=self.C_BLACK, outline=clr, width=1)
        self.create_text(self.cx, M + 46,
                         text=txt, fill=clr, font=fnt)