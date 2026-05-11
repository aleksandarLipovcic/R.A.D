import tkinter as tk


class BaroWidget(tk.Frame):
    """
    Aviation PFD-style Altitude Tape + Vertical Speed Indicator (VSI).

    TAPE LAYOUT — box LEFT, tip points RIGHT ► into spine on RIGHT
    ─────────────────────────────────────────────────────────────────────────
      x=0  BOX_L  SHOULDER  TIP_X  MIN_L  MAJ_L  SPINE_X   x=W
        │    │       │        ►      │      │        │
        │    ├───────┘        │      │      ├────────┤  +390
        │    │ +161.0  tip►───┤      ├──────┤        │  (spine)
        │    ├───────┐        │      │      ├────────┤  +380
        │    │       │        │      │      │        │

    Pentagon (tip ► points RIGHT toward spine):
      A = (TIP_X,    cy)       rightmost tip — sits at spine edge
      B = (SHOULDER, cy - bh)  upper-right shoulder of box
      C = (BOX_L,    cy - bh)  upper-left corner
      D = (BOX_L,    cy + bh)  lower-left corner
      E = (SHOULDER, cy + bh)  lower-right shoulder of box

    Scale numbers are centred inside the box body (BOX_L … SHOULDER).
    Ticks extend LEFT from SPINE_X.
    Spine is the rightmost vertical line.

    ALTITUDE THRESHOLDS (drone operational envelope):
      Green   0 – 800 m
      Amber 800 – 1000 m
      Red   > 1000 m
    """

    # ── Canvas geometry ───────────────────────────────────────────────────────
    TAPE_W   = 160
    VSI_W    =  60
    TAPE_H   = 340

    PX_PER_M   = 6.0
    LABEL_STEP = 10
    TICK_STEP  =  5

    # ── Tape x-coordinate layout (all measured from left = 0) ────────────────
    #
    #   BOX_L   SHOULDER   TIP_X   MIN_L   MAJ_L   SPINE_X
    #     5       100       110     114     120      150
    #
    #   Box body  : BOX_L … SHOULDER  (95 px wide — fits ±9999.9)
    #   Angled tip: SHOULDER … TIP_X  (10 px ramp)
    #   Gap       : TIP_X … MIN_L     (4 px clear of ticks)
    #   Minor tick: MIN_L … SPINE_X   (36 px)   — actually just 7 px:
    #               ticks hang LEFT from SPINE_X
    #
    SPINE_X  = 150   # rightmost spine line
    MAJ_L    = 138   # left end of major tick  (12 px wide tick)
    MIN_L    = 143   # left end of minor tick  ( 7 px wide tick)
    TIP_X    = 130   # rightmost tip of pentagon (≈ MAJ_L - 8, clears ticks)
    SHOULDER = 120   # angled corner x  (TIP_X - 10)
    BOX_L    =   5   # left wall of box

    # ── Warning thresholds ────────────────────────────────────────────────────
    WARN_ALT_M  =  800.0
    CRIT_ALT_M  = 1000.0
    WARN_VARIO  =   3.0
    CRIT_VARIO  =   8.0
    VSI_MAX_MPS =  10.0

    # ── Anti-flicker gates ────────────────────────────────────────────────────
    ALT_THR   = 0.05
    VARIO_THR = 0.03

    # ── Cockpit palette ───────────────────────────────────────────────────────
    C_BG      = "#0d0d1a"
    C_FRAME   = "#111120"
    C_SPINE   = "#6868a8"
    C_LABEL   = "#e8e8ff"
    C_MIN     = "#484870"
    C_BOX_BG  = "#000010"
    C_SAFE    = "#00e07a"
    C_WARN    = "#ffd700"
    C_CRIT    = "#ff3333"
    C_DIM     = "#9090b8"
    C_SEP     = "#2a2a44"

    CM_TO_M = 0.01
    M_TO_FT = 3.28084

    # ─────────────────────────────────────────────────────────────────────────

    def __init__(self, parent):
        super().__init__(parent, bg=self.C_FRAME)
        self._alt_m     = 0.0
        self._vario_mps = 0.0
        self._valid     = False
        self._qnh_ref_m = 0.0

        self._last_alt   = None
        self._last_vario = None
        self._last_valid = None

        self._build_ui()

    # ── Construction ──────────────────────────────────────────────────────────

    def _build_ui(self):
        hdr = tk.Frame(self, bg=self.C_FRAME)
        hdr.pack(side="top", fill="x", padx=4, pady=(5, 2))
        tk.Label(
            hdr, text="ALTITUDE / VSI",
            bg=self.C_FRAME, fg=self.C_DIM,
            font=("Consolas", 9, "bold")
        ).pack(side="left")

        cv_row = tk.Frame(self, bg=self.C_FRAME)
        cv_row.pack(side="top", padx=4)

        self._tape_cv = tk.Canvas(
            cv_row,
            width=self.TAPE_W, height=self.TAPE_H,
            bg=self.C_BG,
            highlightthickness=1,
            highlightbackground=self.C_SPINE
        )
        self._tape_cv.pack(side="left")

        self._vsi_cv = tk.Canvas(
            cv_row,
            width=self.VSI_W, height=self.TAPE_H,
            bg=self.C_BG,
            highlightthickness=1,
            highlightbackground=self.C_SPINE
        )
        self._vsi_cv.pack(side="left", padx=(3, 0))

        imp_row = tk.Frame(self, bg=self.C_FRAME)
        imp_row.pack(side="top", fill="x", padx=6, pady=(6, 0))

        self._ft_lbl = tk.Label(
            imp_row, text="--- ft",
            bg=self.C_FRAME, fg=self.C_DIM,
            font=("Consolas", 11, "bold"), anchor="w"
        )
        self._ft_lbl.pack(side="left")

        self._agl_lbl = tk.Label(
            imp_row, text="AGL: ---",
            bg=self.C_FRAME, fg=self.C_DIM,
            font=("Consolas", 9), anchor="e"
        )
        self._agl_lbl.pack(side="right")

        tk.Frame(self, bg=self.C_SEP, height=1).pack(
            side="top", fill="x", padx=4, pady=(5, 3)
        )

        qnh_row = tk.Frame(self, bg=self.C_FRAME)
        qnh_row.pack(side="top", fill="x", padx=6, pady=(0, 6))

        self._qnh_lbl = tk.Label(
            qnh_row, text="QNH ref: not set",
            bg=self.C_FRAME, fg=self.C_DIM,
            font=("Consolas", 9), anchor="w"
        )
        self._qnh_lbl.pack(side="left", fill="x", expand=True)

        self._qnh_btn = tk.Button(
            qnh_row, text="SET QNH",
            command=self._set_qnh,
            bg="#1e1e38", fg="#c0c0e0",
            activebackground="#2e2e58", activeforeground="#ffffff",
            font=("Consolas", 10, "bold"),
            relief="raised", bd=2, padx=14, pady=6, cursor="hand2"
        )
        self._qnh_btn.pack(side="right")

        self._draw_tape()
        self._draw_vsi()

    # ── QNH ──────────────────────────────────────────────────────────────────

    def _set_qnh(self):
        self._qnh_ref_m = self._alt_m
        self._last_alt  = None
        self._refresh_labels()
        self._draw_tape()

    def _refresh_labels(self):
        agl = self._alt_m - self._qnh_ref_m
        clr = self._alt_color(self._alt_m)
        self._ft_lbl.config(
            text=f"{self._alt_m * self.M_TO_FT:>+8.0f} ft", fg=clr
        )
        self._agl_lbl.config(text=f"AGL {agl:+.1f} m", fg=clr)
        self._qnh_lbl.config(text=f"QNH ref: {self._qnh_ref_m:+.1f} m")

    # ── Altitude tape ─────────────────────────────────────────────────────────

    def _draw_tape(self):
        cv = self._tape_cv
        cv.delete("all")
        W, H = self.TAPE_W, self.TAPE_H
        cy   = H // 2

        if not self._valid:
            cv.create_text(
                W // 2, H // 2, text="NO\nSIG",
                fill=self.C_CRIT, font=("Consolas", 12, "bold"),
                justify="center"
            )
            return

        alt  = self._alt_m
        ppm  = self.PX_PER_M
        half = H / (2.0 * ppm)

        raw_lo = int(alt - half) - self.LABEL_STEP
        lo = raw_lo - (raw_lo % self.TICK_STEP)
        hi = int(alt + half) + self.LABEL_STEP

        # ── 1. Spine — right side ──────────────────────────────────────────────
        cv.create_line(
            self.SPINE_X, 0, self.SPINE_X, H,
            fill=self.C_SPINE, width=2
        )

        # ── 2. Ticks + scale numbers ───────────────────────────────────────────
        bh       = 14        # pentagon half-height
        label_cx = (self.BOX_L + self.SHOULDER) // 2   # centre of box body

        for a in range(lo, hi + 1, self.TICK_STEP):
            y = cy - (a - alt) * ppm
            if y < 2 or y > H - 2:
                continue

            behind_box = abs(y - cy) < bh + 2

            if a % self.LABEL_STEP == 0:
                # major tick — always drawn
                cv.create_line(
                    self.MAJ_L, y, self.SPINE_X, y,
                    fill=self.C_SPINE, width=2
                )
                # label — only hidden when behind the box
                if not behind_box:
                    cv.create_text(
                        label_cx, y,
                        text=f"{a:+d}",
                        fill=self.C_LABEL,
                        font=("Consolas", 10, "bold"),
                        anchor="center"
                    )
            else:
                # minor tick — always drawn
                cv.create_line(
                    self.MIN_L, y, self.SPINE_X, y,
                    fill=self.C_MIN, width=1
                )

        # ── 3. Pentagon altitude box  ◄ tip points RIGHT ► into spine ──────────
        #
        #   C(BOX_L, cy-bh) ──── B(SHOULDER, cy-bh)
        #   |                                        \
        #   |   readout text                 tip► A(TIP_X, cy)
        #   |                                        /
        #   D(BOX_L, cy+bh) ──── E(SHOULDER, cy+bh)
        #
        box_col = self._alt_color(alt)

        A = (self.TIP_X,    cy)
        B = (self.SHOULDER, cy - bh)
        C = (self.BOX_L,    cy - bh)
        D = (self.BOX_L,    cy + bh)
        E = (self.SHOULDER, cy + bh)

        pts = [A[0],A[1], B[0],B[1], C[0],C[1], D[0],D[1], E[0],E[1]]
        cv.create_polygon(pts, fill=self.C_BOX_BG, outline=box_col, width=2)

        # Readout centred in the rectangular body (BOX_L … SHOULDER)
        text_cx = (self.BOX_L + self.SHOULDER) // 2
        cv.create_text(
            text_cx, cy,
            text=f"{alt:+.1f}",
            fill=box_col,
            font=("Consolas", 11, "bold"),
            anchor="center"
        )

    # ── VSI ───────────────────────────────────────────────────────────────────

    def _draw_vsi(self):
        cv = self._vsi_cv
        cv.delete("all")
        W, H  = self.VSI_W, self.TAPE_H
        cy    = H // 2
        MAX   = self.VSI_MAX_MPS
        ax    = W // 2

        scale_h = (H - 30) // 2

        def v2y(v: float) -> int:
            return cy - int((v / MAX) * scale_h)

        cv.create_line(ax, 12, ax, H - 18, fill=self.C_SPINE, width=1)

        for v in range(2, int(MAX) + 1, 2):
            for sign in (1, -1):
                y = v2y(sign * v)
                cv.create_line(ax - 8, y, ax + 8, y, fill=self.C_LABEL, width=1)
                cv.create_text(
                    ax + 12, y, text=str(v),
                    fill=self.C_LABEL, font=("Consolas", 9, "bold"), anchor="w"
                )

        cv.create_line(ax - 10, cy, ax + 10, cy, fill=self.C_LABEL, width=2)
        cv.create_text(
            ax + 12, cy, text="0",
            fill=self.C_LABEL, font=("Consolas", 9, "bold"), anchor="w"
        )

        cv.create_text(ax, 6,     text="▲", fill=self.C_DIM, font=("Consolas", 8))
        cv.create_text(ax, H - 6, text="▼", fill=self.C_DIM, font=("Consolas", 8))
        cv.create_text(ax, 4,     text="V/S", fill=self.C_DIM,
                       font=("Consolas", 7, "bold"), anchor="n")

        if not self._valid:
            cv.create_text(
                ax, H // 2, text="NO\nSIG",
                fill=self.C_CRIT, font=("Consolas", 8, "bold"), justify="center"
            )
            return

        vario    = max(-MAX, min(MAX, self._vario_mps))
        needle_y = v2y(vario)
        color    = (self.C_CRIT if abs(vario) >= self.CRIT_VARIO else
                    self.C_WARN if abs(vario) >= self.WARN_VARIO else
                    self.C_SAFE)

        cv.create_line(ax, cy, ax, needle_y,
                       fill=color, width=5, capstyle=tk.ROUND)
        cv.create_oval(
            ax - 5, needle_y - 5, ax + 5, needle_y + 5,
            fill=color, outline=""
        )
        cv.create_text(
            ax, H - 8, text=f"{vario:+.1f}",
            fill=color, font=("Consolas", 8, "bold"), anchor="s"
        )

    # ── Public update (called from main.py at 50 Hz) ──────────────────────────

    def update_baro(self, data: dict):
        """
        Keys expected in ui_data:
            baro_altitude_cm       int   FC-fused altitude above home, cm
            baro_vario_cm_per_sec  int   vertical speed, cm/s
            baro_valid             bool  True when MSP_ALTITUDE decoded OK
        """
        valid    = data.get("baro_valid", False)
        alt_cm   = int(data.get("baro_altitude_cm",      0)) if valid else 0
        vario_cm = int(data.get("baro_vario_cm_per_sec", 0)) if valid else 0

        new_alt   = alt_cm   * self.CM_TO_M
        new_vario = vario_cm * self.CM_TO_M

        alt_dirty   = (self._last_alt   is None or
                       abs(new_alt   - self._last_alt)   > self.ALT_THR)
        vario_dirty = (self._last_vario is None or
                       abs(new_vario - self._last_vario) > self.VARIO_THR)
        valid_dirty = (self._last_valid != valid)

        self._alt_m     = new_alt
        self._vario_mps = new_vario
        self._valid     = valid

        if valid:
            self._refresh_labels()
        else:
            self._ft_lbl.config( text="--- ft",   fg=self.C_DIM)
            self._agl_lbl.config(text="AGL: ---", fg=self.C_DIM)

        if alt_dirty or valid_dirty:
            self._draw_tape()
            self._last_alt   = new_alt
            self._last_valid = valid

        if vario_dirty or valid_dirty:
            self._draw_vsi()
            self._last_vario = new_vario

    # ── Colour helper ─────────────────────────────────────────────────────────

    def _alt_color(self, alt_m: float) -> str:
        if alt_m >= self.CRIT_ALT_M: return self.C_CRIT
        if alt_m >= self.WARN_ALT_M: return self.C_WARN
        return self.C_SAFE