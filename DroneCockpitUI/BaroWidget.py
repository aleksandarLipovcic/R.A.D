import tkinter as tk
import time
import collections


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

    VARIO NOTE
    ──────────
    Betaflight's MSP_ALTITUDE varioCmPerSec field is often zero or near-zero
    because the FC's internal vario estimator is tuned for stabilisation, not
    for a readable climb-rate display.  We therefore compute our own vario by
    differentiating altitude over a short rolling window (VARIO_WINDOW_S) and
    applying a simple exponential smoothing filter (VARIO_ALPHA).

    The FC vario is accepted only as a fallback when we have fewer than
    MIN_SAMPLES_FOR_DERIVED samples in the window.

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

    # ── Tape x-coordinate layout ──────────────────────────────────────────────
    SPINE_X  = 150
    MAJ_L    = 138
    MIN_L    = 143
    TIP_X    = 130
    SHOULDER = 120
    BOX_L    =   5

    # ── Vario derivation parameters ───────────────────────────────────────────
    # Rolling window of (timestamp, altitude_m) samples used to compute dh/dt.
    # A 2-second window smooths noise without lagging too much.
    VARIO_WINDOW_S       = 2.0    # seconds of history to keep
    VARIO_ALPHA          = 0.25   # EMA smoothing  (lower = smoother, more lag)
    MIN_SAMPLES_FOR_DERIVED = 3   # need at least this many samples before
                                  # we trust the derived value over FC vario

    # ── Warning thresholds ────────────────────────────────────────────────────
    WARN_ALT_M  =  800.0
    CRIT_ALT_M  = 1000.0
    WARN_VARIO  =   3.0
    CRIT_VARIO  =   8.0
    VSI_MAX_MPS =  10.0

    # ── Anti-flicker gates ────────────────────────────────────────────────────
    ALT_THR   = 0.05
    VARIO_THR = 0.02   # tighter than before since derived vario is smoother

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
        self._vario_mps = 0.0   # smoothed derived vario
        self._valid     = False
        self._qnh_ref_m = 0.0

        self._last_alt   = None
        self._last_vario = None
        self._last_valid = None

        # ── Vario derivation state ─────────────────────────────────────────────
        # deque of (time_s, alt_m) — automatically discards old samples
        self._alt_history: collections.deque = collections.deque()
        self._vario_ema   = 0.0   # exponential moving average of raw dh/dt

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

    # ── Vario derivation ──────────────────────────────────────────────────────

    def _update_vario(self, alt_m: float, fc_vario_mps: float) -> float:
        """
        Compute a smoothed vertical speed from the altitude history.

        Algorithm:
          1. Push (now, alt_m) onto a rolling deque.
          2. Evict samples older than VARIO_WINDOW_S.
          3. If we have enough samples, estimate dh/dt by linear regression
             over the window (least-squares slope = Σ(t·h) / Σ(t²) after
             mean-centring).  This is more robust than a simple endpoint diff.
          4. Apply exponential smoothing to suppress noise.
          5. Fall back to the FC-supplied vario until the window fills up.

        Returns the smoothed vario in m/s.
        """
        now = time.monotonic()
        self._alt_history.append((now, alt_m))

        # Evict old samples
        cutoff = now - self.VARIO_WINDOW_S
        while self._alt_history and self._alt_history[0][0] < cutoff:
            self._alt_history.popleft()

        n = len(self._alt_history)

        if n < self.MIN_SAMPLES_FOR_DERIVED:
            # Not enough data yet — trust the FC value and seed the EMA
            self._vario_ema = fc_vario_mps
            return fc_vario_mps

        # ── Linear regression slope (dh/dt) over the window ──────────────────
        times = [s[0] for s in self._alt_history]
        alts  = [s[1] for s in self._alt_history]

        t_mean = sum(times) / n
        h_mean = sum(alts)  / n

        num = sum((t - t_mean) * (h - h_mean) for t, h in zip(times, alts))
        den = sum((t - t_mean) ** 2            for t      in times)

        raw_vario = (num / den) if den > 1e-9 else 0.0

        # ── Exponential moving average ────────────────────────────────────────
        self._vario_ema = (self.VARIO_ALPHA * raw_vario
                           + (1.0 - self.VARIO_ALPHA) * self._vario_ema)

        return self._vario_ema

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
        bh       = 14
        label_cx = (self.BOX_L + self.SHOULDER) // 2

        for a in range(lo, hi + 1, self.TICK_STEP):
            y = cy - (a - alt) * ppm
            if y < 2 or y > H - 2:
                continue

            behind_box = abs(y - cy) < bh + 2

            if a % self.LABEL_STEP == 0:
                cv.create_line(
                    self.MAJ_L, y, self.SPINE_X, y,
                    fill=self.C_SPINE, width=2
                )
                if not behind_box:
                    cv.create_text(
                        label_cx, y,
                        text=f"{a:+d}",
                        fill=self.C_LABEL,
                        font=("Consolas", 10, "bold"),
                        anchor="center"
                    )
            else:
                cv.create_line(
                    self.MIN_L, y, self.SPINE_X, y,
                    fill=self.C_MIN, width=1
                )

        # ── 3. Pentagon altitude box ───────────────────────────────────────────
        box_col = self._alt_color(alt)

        A = (self.TIP_X,    cy)
        B = (self.SHOULDER, cy - bh)
        C = (self.BOX_L,    cy - bh)
        D = (self.BOX_L,    cy + bh)
        E = (self.SHOULDER, cy + bh)

        pts = [A[0],A[1], B[0],B[1], C[0],C[1], D[0],D[1], E[0],E[1]]
        cv.create_polygon(pts, fill=self.C_BOX_BG, outline=box_col, width=2)

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
        Keys expected in data:
            baro_altitude_cm       int   FC-fused altitude above home, cm
            baro_vario_cm_per_sec  int   vertical speed from FC, cm/s
                                         (used only as fallback seed)
            baro_valid             bool  True when MSP_ALTITUDE decoded OK

        Vario is derived internally from the altitude history rather than
        relying on the FC value, which Betaflight often reports as zero.
        """
        valid    = data.get("baro_valid", False)
        alt_cm   = int(data.get("baro_altitude_cm",      0)) if valid else 0
        vario_cm = int(data.get("baro_vario_cm_per_sec", 0)) if valid else 0

        new_alt      = alt_cm   * self.CM_TO_M
        fc_vario_mps = vario_cm * self.CM_TO_M

        # Derive vario from altitude history (ignores FC value unless warming up)
        if valid:
            new_vario = self._update_vario(new_alt, fc_vario_mps)
        else:
            # Invalid signal — reset history so we start fresh on reconnect
            self._alt_history.clear()
            self._vario_ema = 0.0
            new_vario = 0.0

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