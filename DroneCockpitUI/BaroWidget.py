"""
BaroWidget.py  — Aviation PFD-style Altitude Tape + VSI
========================================================
FULLY RESPONSIVE with correct aviation proportions:

  ┌─────────────────────────┬──────┐
  │                         │ V/S  │
  │   ALTITUDE TAPE  (~75%) │ (25%)│
  │                         │      │
  └─────────────────────────┴──────┘

The VSI has a FIXED minimum width and a MAXIMUM cap so it never
dominates.  The altitude tape takes everything that's left.
Both canvases bind <Configure> and redraw from actual pixel sizes —
nothing is hardcoded.

Key proportions (tape canvas):
  SPINE  at 92% of tape width   (vertical scale line, right side)
  MAJ_L  at 84%                 (major tick start)
  MIN_L  at 89%                 (minor tick start)
  TIP_X  at 76%                 (pentagon pointer tip)
  SHLDR  at 66%                 (pentagon shoulder)
  BOX_L  at  3%                 (left wall of readout box)

VSI canvas:
  Fixed width = clamp(W_total * 0.22, 44, 80) px
  Axis at 40% of VSI width; labels to the right of axis
"""

import tkinter as tk
import time
import collections


class BaroWidget(tk.Frame):

    # ── Scale / units ─────────────────────────────────────────────────────────
    PX_PER_M_BASE = 6.0
    REF_H         = 340.0
    LABEL_STEP    = 10
    TICK_STEP     =  5

    # ── Tape proportions (fraction of TAPE canvas width) ──────────────────────
    _P_SPINE   = 0.92
    _P_MAJ_L   = 0.84
    _P_MIN_L   = 0.89
    _P_TIP_X   = 0.76
    _P_SHLDR   = 0.66
    _P_BOX_L   = 0.03

    # ── VSI sizing ────────────────────────────────────────────────────────────
    VSI_MIN_W  = 44    # pixels minimum
    VSI_MAX_W  = 80    # pixels maximum — prevents the giant-dot problem
    VSI_FRAC   = 0.22  # preferred fraction of total widget width

    # ── Vario derivation ──────────────────────────────────────────────────────
    VARIO_WINDOW_S          = 2.0
    VARIO_ALPHA             = 0.25
    MIN_SAMPLES_FOR_DERIVED = 3

    # ── Warning thresholds ────────────────────────────────────────────────────
    WARN_ALT_M  =  800.0
    CRIT_ALT_M  = 1000.0
    WARN_VARIO  =   3.0
    CRIT_VARIO  =   8.0
    VSI_MAX_MPS =  10.0

    # ── Anti-flicker gates ────────────────────────────────────────────────────
    ALT_THR   = 0.05
    VARIO_THR = 0.02

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

        self._alt_history: collections.deque = collections.deque()
        self._vario_ema   = 0.0

        self._tape_redraw_id = None
        self._vsi_redraw_id  = None

        self._build_ui()

    # ── Construction ──────────────────────────────────────────────────────────

    def _build_ui(self):
        self.grid_columnconfigure(0, weight=1)
        self.grid_rowconfigure(1, weight=1)

        # Row 0 — header
        hdr = tk.Frame(self, bg=self.C_FRAME)
        hdr.grid(row=0, column=0, sticky="ew", padx=4, pady=(5, 2))
        tk.Label(
            hdr, text="ALTITUDE / VSI",
            bg=self.C_FRAME, fg=self.C_DIM,
            font=("Consolas", 9, "bold")
        ).pack(side="left")

        # Row 1 — canvas area managed by place() inside _cv_outer
        self._cv_outer = tk.Frame(self, bg=self.C_FRAME)
        self._cv_outer.grid(row=1, column=0, sticky="nsew", padx=4)

        self._tape_cv = tk.Canvas(
            self._cv_outer, bg=self.C_BG,
            highlightthickness=1, highlightbackground=self.C_SPINE,
        )
        self._vsi_cv = tk.Canvas(
            self._cv_outer, bg=self.C_BG,
            highlightthickness=1, highlightbackground=self.C_SPINE,
        )

        self._tape_cv.place(x=0, y=0, relheight=1.0)
        self._vsi_cv.place(relheight=1.0)

        self._cv_outer.bind("<Configure>", self._on_outer_configure)
        self._tape_cv.bind("<Configure>",  lambda e: self._schedule_redraw("tape"))
        self._vsi_cv.bind("<Configure>",   lambda e: self._schedule_redraw("vsi"))

        # Row 2 — ft / AGL labels
        imp_row = tk.Frame(self, bg=self.C_FRAME)
        imp_row.grid(row=2, column=0, sticky="ew", padx=6, pady=(6, 0))

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

        # Row 3 — separator
        tk.Frame(self, bg=self.C_SEP, height=1).grid(
            row=3, column=0, sticky="ew", padx=4, pady=(5, 3)
        )

        # Row 4 — QNH
        qnh_row = tk.Frame(self, bg=self.C_FRAME)
        qnh_row.grid(row=4, column=0, sticky="ew", padx=6, pady=(0, 6))

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

    # ── Canvas layout — enforces VSI width cap ────────────────────────────────

    def _on_outer_configure(self, event):
        total_w = event.width
        gap     = 3

        vsi_w = int(total_w * self.VSI_FRAC)
        vsi_w = max(self.VSI_MIN_W, min(self.VSI_MAX_W, vsi_w))

        tape_w = max(60, total_w - vsi_w - gap)

        self._tape_cv.place(x=0,            y=0, width=tape_w, relheight=1.0)
        self._vsi_cv.place( x=tape_w + gap, y=0, width=vsi_w,  relheight=1.0)

    # ── Debounced redraws ─────────────────────────────────────────────────────

    def _schedule_redraw(self, which: str):
        if which == "tape":
            if self._tape_redraw_id:
                self._tape_cv.after_cancel(self._tape_redraw_id)
            self._tape_redraw_id = self._tape_cv.after(12, self._draw_tape)
        else:
            if self._vsi_redraw_id:
                self._vsi_cv.after_cancel(self._vsi_redraw_id)
            self._vsi_redraw_id = self._vsi_cv.after(12, self._draw_vsi)

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
        now = time.monotonic()
        self._alt_history.append((now, alt_m))

        cutoff = now - self.VARIO_WINDOW_S
        while self._alt_history and self._alt_history[0][0] < cutoff:
            self._alt_history.popleft()

        n = len(self._alt_history)
        if n < self.MIN_SAMPLES_FOR_DERIVED:
            self._vario_ema = fc_vario_mps
            return fc_vario_mps

        times  = [s[0] for s in self._alt_history]
        alts   = [s[1] for s in self._alt_history]
        t_mean = sum(times) / n
        h_mean = sum(alts)  / n
        num    = sum((t - t_mean) * (h - h_mean) for t, h in zip(times, alts))
        den    = sum((t - t_mean) ** 2            for t in times)
        raw    = (num / den) if den > 1e-9 else 0.0

        self._vario_ema = (self.VARIO_ALPHA * raw
                           + (1.0 - self.VARIO_ALPHA) * self._vario_ema)
        return self._vario_ema

    # ── Altitude tape ─────────────────────────────────────────────────────────

    def _draw_tape(self):
        cv = self._tape_cv
        cv.delete("all")

        W = cv.winfo_width()
        H = cv.winfo_height()
        if W < 20 or H < 20:
            return

        cy = H // 2

        spine_x = int(W * self._P_SPINE)
        maj_l   = int(W * self._P_MAJ_L)
        min_l   = int(W * self._P_MIN_L)
        tip_x   = int(W * self._P_TIP_X)
        shldr   = int(W * self._P_SHLDR)
        box_l   = max(2, int(W * self._P_BOX_L))
        lbl_cx  = (box_l + shldr) // 2

        ppm     = self.PX_PER_M_BASE * (H / self.REF_H)
        ppm     = max(2.0, min(18.0, ppm))
        font_sz = max(7, min(11, int(W * 0.072)))

        if not self._valid:
            cv.create_text(W // 2, H // 2, text="NO\nSIG",
                           fill=self.C_CRIT,
                           font=("Consolas", font_sz + 1, "bold"),
                           justify="center")
            return

        alt  = self._alt_m
        half = H / (2.0 * ppm)
        raw_lo = int(alt - half) - self.LABEL_STEP
        lo = raw_lo - (raw_lo % self.TICK_STEP)
        hi = int(alt + half) + self.LABEL_STEP

        # Spine
        cv.create_line(spine_x, 0, spine_x, H, fill=self.C_SPINE, width=2)

        # Ticks + labels
        bh = max(10, int(H * 0.040))

        for a in range(lo, hi + 1, self.TICK_STEP):
            y = cy - (a - alt) * ppm
            if y < 2 or y > H - 2:
                continue
            behind_box = abs(y - cy) < bh + 2
            if a % self.LABEL_STEP == 0:
                cv.create_line(maj_l, y, spine_x, y,
                               fill=self.C_SPINE, width=2)
                if not behind_box:
                    cv.create_text(lbl_cx, y, text=f"{a:+d}",
                                   fill=self.C_LABEL,
                                   font=("Consolas", font_sz, "bold"),
                                   anchor="center")
            else:
                cv.create_line(min_l, y, spine_x, y,
                               fill=self.C_MIN, width=1)

        # Pentagon readout box
        box_col = self._alt_color(alt)
        A = (tip_x, cy);       B = (shldr, cy - bh)
        C = (box_l, cy - bh);  D = (box_l, cy + bh);  E = (shldr, cy + bh)
        cv.create_polygon([A[0],A[1], B[0],B[1], C[0],C[1], D[0],D[1], E[0],E[1]],
                          fill=self.C_BOX_BG, outline=box_col, width=2)
        cv.create_text((box_l + shldr) // 2, cy,
                       text=f"{alt:+.1f}",
                       fill=box_col,
                       font=("Consolas", font_sz, "bold"),
                       anchor="center")

    # ── VSI ───────────────────────────────────────────────────────────────────

    def _draw_vsi(self):
        cv = self._vsi_cv
        cv.delete("all")

        W = cv.winfo_width()
        H = cv.winfo_height()
        if W < 10 or H < 10:
            return

        cy    = H // 2
        MAX   = self.VSI_MAX_MPS

        # Because VSI_MAX_W caps the canvas at 80 px, everything here
        # is sized to look right at 44–80 px wide.
        ax      = max(6, int(W * 0.40))
        pad_t   = max(8,  int(H * 0.04))
        pad_b   = max(14, int(H * 0.06))
        scale_h = max(10, (H - pad_t - pad_b) // 2)

        font_sz   = max(6, min(8, int(W * 0.14)))
        tick_half = max(3, int(W * 0.12))

        def v2y(v: float) -> int:
            return cy - int((v / MAX) * scale_h)

        # Centre axis line
        cv.create_line(ax, pad_t, ax, H - pad_b,
                       fill=self.C_SPINE, width=1)

        # Tick marks + labels every 2 m/s
        for v in range(2, int(MAX) + 1, 2):
            for sign in (1, -1):
                y = v2y(sign * v)
                cv.create_line(ax - tick_half, y, ax + tick_half, y,
                               fill=self.C_LABEL, width=1)
                lbl_x = ax + tick_half + 2
                if lbl_x + font_sz * 2 <= W - 1:
                    cv.create_text(lbl_x, y, text=str(v),
                                   fill=self.C_LABEL,
                                   font=("Consolas", font_sz, "bold"),
                                   anchor="w")

        # Zero mark
        cv.create_line(ax - tick_half - 2, cy, ax + tick_half + 2, cy,
                       fill=self.C_LABEL, width=2)
        lbl_x = ax + tick_half + 2
        if lbl_x + font_sz * 2 <= W - 1:
            cv.create_text(lbl_x, cy, text="0",
                           fill=self.C_LABEL,
                           font=("Consolas", font_sz, "bold"),
                           anchor="w")

        # "V/S" label + arrows
        cv.create_text(ax, 2, text="V/S",
                       fill=self.C_DIM,
                       font=("Consolas", max(5, font_sz - 1), "bold"),
                       anchor="n")
        cv.create_text(ax, pad_t - 1, text="▲",
                       fill=self.C_DIM, font=("Consolas", font_sz), anchor="s")
        cv.create_text(ax, H - pad_b + 1, text="▼",
                       fill=self.C_DIM, font=("Consolas", font_sz), anchor="n")

        if not self._valid:
            cv.create_text(ax, cy, text="--",
                           fill=self.C_CRIT,
                           font=("Consolas", font_sz, "bold"),
                           anchor="center")
            return

        vario    = max(-MAX, min(MAX, self._vario_mps))
        needle_y = v2y(vario)
        color    = (self.C_CRIT if abs(vario) >= self.CRIT_VARIO else
                    self.C_WARN if abs(vario) >= self.WARN_VARIO else
                    self.C_SAFE)

        # Needle width and dot radius: hard-capped small values
        needle_w = max(2, min(4,  int(W * 0.07)))
        dot_r    = max(3, min(6,  int(W * 0.10)))

        cv.create_line(ax, cy, ax, needle_y,
                       fill=color, width=needle_w, capstyle=tk.ROUND)
        cv.create_oval(ax - dot_r, needle_y - dot_r,
                       ax + dot_r, needle_y + dot_r,
                       fill=color, outline="")

        # Numeric readout at bottom
        cv.create_text(ax, H - 2, text=f"{vario:+.1f}",
                       fill=color,
                       font=("Consolas", font_sz, "bold"),
                       anchor="s")

    # ── Public update ─────────────────────────────────────────────────────────

    def update_baro(self, data: dict):
        valid    = data.get("baro_valid", False)
        alt_cm   = int(data.get("baro_altitude_cm",      0)) if valid else 0
        vario_cm = int(data.get("baro_vario_cm_per_sec", 0)) if valid else 0

        new_alt      = alt_cm   * self.CM_TO_M
        fc_vario_mps = vario_cm * self.CM_TO_M

        if valid:
            new_vario = self._update_vario(new_alt, fc_vario_mps)
        else:
            self._alt_history.clear()
            self._vario_ema = 0.0
            new_vario = 0.0

        alt_dirty   = self._last_alt   is None or abs(new_alt   - self._last_alt)   > self.ALT_THR
        vario_dirty = self._last_vario is None or abs(new_vario - self._last_vario) > self.VARIO_THR
        valid_dirty = self._last_valid != valid

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