import tkinter as tk
import time


class IMUWidget(tk.Frame):
    """
    MPU-6500 IMU / Attitude display — fully adaptive cockpit instrument.

    Layout philosophy:
      Primary data (rotation, g-force, angle) is NEVER hidden as long as the
      panel has enough room to draw even one row.  Secondary chrome (column
      header labels, checkbox) shrinks or drops first.  The diag panel hides
      only when the widget is genuinely too small to fit it after the grid.

    Tier definitions drive font size and chrome visibility only:
      FULL    (h>=210, w>=400): full column headers + checkbox + diag panel
      MEDIUM  (h>=150, w>=290): short column headers + adjust btn + diag panel
      COMPACT (h>=90,  w>=190): no column headers + axis rows + diag if it fits
      TINY    (h< 90  or w<190): single summary line

    The diag panel uses grid() / grid_forget() dynamically after every resize
    based on actual remaining pixel space — not on tier alone.
    """

    # ── Scale factors ─────────────────────────────────────────────────────────
    GYRO_SCALE  = 16.4
    ACCEL_SCALE = 2048.0

    # ── Thresholds ────────────────────────────────────────────────────────────
    ANGLE_WARN_DEG = 15.0
    ANGLE_CRIT_DEG = 30.0
    GYRO_WARN_DPS  = 30.0
    GYRO_CRIT_DPS  = 100.0
    HOLD_WARN_SEC  = 2.0
    HOLD_CRIT_SEC  = 4.0
    ACC_LAT_WARN   = 0.26
    ACC_LAT_CRIT   = 0.50
    ACC_VERT_LO_OK = 0.87
    ACC_VERT_HI_OK = 1.13
    ACC_VERT_LO_CR = 0.64
    ACC_VERT_HI_CR = 1.36
    DRIFT_WARN_DEG = 15.0
    DRIFT_CRIT_DEG = 30.0

    # Approximate pixel heights for layout decisions
    _HDR_H    = 28    # header bar
    _ROW_H    = 26    # one axis data row
    _COLHDR_H = 22    # column header row
    _DIAG_H   = 54    # diag panel (3 lines)
    _PAD      = 10    # total vertical padding budget

    # ── Cockpit colour palette ────────────────────────────────────────────────
    C_BG          = "#0a0a10"
    C_HEADER_BG   = "#0d1520"
    C_GRID_BG     = "#0b0b15"
    C_CELL_BG     = "#0d1520"
    C_CELL_BORDER = "#1e2a3a"
    C_COL_HDR_BG  = "#111828"
    C_COL_HDR_FG  = "#4a7a9a"
    C_AXIS_FG     = "#6a8a9a"
    C_TEXT        = "#c0d0e0"
    C_LABEL       = "#3a5a6a"
    C_SEP         = "#1e2a3a"
    C_SAFE        = "#00ff88"
    C_WARN        = "#FFD700"
    C_CRIT        = "#FF3333"
    C_NEUTRAL     = "#00d4ff"
    C_BTN_NORMAL  = "#0a1520"
    C_BTN_FG      = "#00d4ff"
    C_BTN_FLASH_A = "#FF2222"
    C_BTN_FLASH_B = "#880000"

    def __init__(self, parent, on_adjust_heading=None):
        super().__init__(parent, bg=self.C_BG)
        self._on_adjust_heading = on_adjust_heading

        self.gyro_offsets   = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.is_calibrating = False
        self.calib_samples  = []

        self._gyro_state = {
            axis: {"state": "safe", "hold_until": 0.0}
            for axis in ("roll", "pitch", "yaw")
        }

        self._flash_job = None
        self._flash_on  = False
        self._flashing  = False

        self._last_mag_heading = 0.0
        self._mag_valid        = False
        self._last_data        = {}

        self._current_tier  = None
        self._tier_widgets  = {}
        self._diag_visible  = False   # tracks actual diag panel state

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self._container = tk.Frame(self, bg=self.C_BG)
        self._container.grid(row=0, column=0, sticky="nsew")

        self.bind("<Configure>", self._on_resize)

    # ══════════════════════════════════════════════════════════════════════════
    # Resize handling
    # ══════════════════════════════════════════════════════════════════════════

    def _on_resize(self, event=None):
        w = self.winfo_width()
        h = self.winfo_height()
        tier = self._classify(w, h)

        if tier != self._current_tier:
            self._build_tier(tier)
            self._current_tier = tier
            if self._last_data:
                self._render(self._last_data)

        # After (re)build, always re-evaluate diag visibility based on real space
        self.after_idle(lambda: self._refit_diag(h))

    def _classify(self, w, h):
        if h < 90 or w < 190:
            return "tiny"
        if h < 150 or w < 290:
            return "compact"
        if h < 210 or w < 400:
            return "medium"
        return "full"

    def _refit_diag(self, total_h=None):
        """
        Show or hide the diag panel depending on whether it actually fits
        below the grid in the current pixel height.  Called after every resize
        and after every tier rebuild.
        """
        diag = self._tier_widgets.get("diag")
        if diag is None:
            return  # tier doesn't have a diag panel (compact/tiny)

        if total_h is None:
            total_h = self.winfo_height()

        tier = self._current_tier
        # Estimate pixels consumed by fixed chrome above the diag panel
        if tier == "full":
            chrome = self._HDR_H + self._COLHDR_H + 3 * self._ROW_H + self._PAD
        elif tier == "medium":
            chrome = self._HDR_H + self._COLHDR_H + 3 * self._ROW_H + self._PAD
        else:
            chrome = 3 * self._ROW_H + self._PAD

        fits = (total_h - chrome) >= self._DIAG_H

        if fits and not self._diag_visible:
            diag.grid(row=3, column=0, sticky="ew", padx=4, pady=(2, 4))
            self._diag_visible = True
        elif not fits and self._diag_visible:
            diag.grid_forget()
            self._diag_visible = False

    # ══════════════════════════════════════════════════════════════════════════
    # Tier builders
    # ══════════════════════════════════════════════════════════════════════════

    def _clear_container(self):
        for w in self._container.winfo_children():
            w.destroy()
        self._tier_widgets  = {}
        self._diag_visible  = False
        for i in range(8):
            self._container.columnconfigure(i, weight=0, minsize=0)
            self._container.rowconfigure(i,    weight=0, minsize=0)

    def _build_tier(self, tier):
        self._clear_container()
        {
            "full":    self._build_full,
            "medium":  self._build_medium,
            "compact": self._build_compact,
            "tiny":    self._build_tiny,
        }[tier]()

    # ── FULL ──────────────────────────────────────────────────────────────────
    def _build_full(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.rowconfigure(0, weight=0)   # header
        c.rowconfigure(1, weight=0)   # col headers
        c.rowconfigure(2, weight=1)   # grid rows
        c.rowconfigure(3, weight=0)   # diag (shown/hidden by _refit_diag)

        self._build_header(c, row=0, full=True)
        self._build_col_headers(c, row=1, font_size=8, labels=[
            "FLIGHT AXIS", "ROTATION  °/s", "G-FORCE  g", "ANGLE / HDG  °"
        ])
        self._build_axis_grid(c, row=2, font_size=12)
        self._build_diag(c)

    # ── MEDIUM ────────────────────────────────────────────────────────────────
    def _build_medium(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.rowconfigure(0, weight=0)
        c.rowconfigure(1, weight=0)
        c.rowconfigure(2, weight=1)
        c.rowconfigure(3, weight=0)

        self._build_header(c, row=0, full=False)
        self._build_col_headers(c, row=1, font_size=7, labels=[
            "AXIS", "ROT °/s", "G  g", "ANGLE °"
        ])
        self._build_axis_grid(c, row=2, font_size=10)
        self._build_diag(c)

    # ── COMPACT ───────────────────────────────────────────────────────────────
    def _build_compact(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.rowconfigure(0, weight=0)   # slim header (adjust btn + drift only)
        c.rowconfigure(1, weight=1)   # grid rows
        # No diag — will be added by _refit_diag if space allows
        # (compact is h>=90, so there's rarely room, but we still check)

        self._build_header(c, row=0, full=False)
        self._build_axis_grid(c, row=1, font_size=9)
        self._build_diag(c)   # created but initially hidden; _refit_diag decides

    # ── TINY ──────────────────────────────────────────────────────────────────
    def _build_tiny(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.rowconfigure(0, weight=1)

        summary = tk.Label(
            c, text="R: ---  P: ---  Y: ---",
            font=("Consolas", 9, "bold"),
            fg=self.C_SAFE, bg=self.C_BG,
        )
        summary.grid(row=0, column=0)
        self._tier_widgets["summary"] = summary

    # ── Sub-builders ──────────────────────────────────────────────────────────

    def _build_header(self, parent, row, full: bool):
        hdr = tk.Frame(parent, bg=self.C_HEADER_BG,
                       highlightbackground=self.C_SEP, highlightthickness=1)
        hdr.grid(row=row, column=0, sticky="ew", padx=4, pady=(4, 1))
        hdr.columnconfigure(10, weight=1)

        col = 0

        if full:
            show_diag = tk.BooleanVar(value=True)
            self._tier_widgets["show_diag"] = show_diag
            chk = tk.Checkbutton(
                hdr, text="Link latency",
                variable=show_diag,
                command=self._on_latency_toggle,
                font=("Consolas", 8),
                fg=self.C_LABEL, bg=self.C_HEADER_BG,
                selectcolor=self.C_CELL_BG,
                activebackground=self.C_HEADER_BG,
                activeforeground=self.C_NEUTRAL,
                bd=0, highlightthickness=0,
            )
            chk.grid(row=0, column=col, padx=(8, 4), pady=4)
            col += 1

        adj_btn = tk.Button(
            hdr,
            text="⊕  ADJUST HEADING" if full else "⊕  ADJUST HDG",
            font=("Consolas", 8, "bold"),
            fg=self.C_BTN_FG, bg=self.C_BTN_NORMAL,
            activeforeground=self.C_BG, activebackground=self.C_NEUTRAL,
            relief="flat", bd=0, padx=8, pady=3,
            cursor="hand2",
            command=self._on_adjust_pressed,
        )
        adj_btn.grid(row=0, column=col, padx=4, pady=4)
        self._tier_widgets["adj_btn"] = adj_btn
        col += 1

        tk.Label(hdr, text="Δ HDG",
                 font=("Consolas", 7), fg=self.C_LABEL,
                 bg=self.C_HEADER_BG,
                 ).grid(row=0, column=col, padx=(4, 1), pady=4)
        col += 1

        drift_lbl = tk.Label(hdr, text="---",
                             font=("Consolas", 8, "bold"),
                             fg=self.C_LABEL, bg=self.C_HEADER_BG)
        drift_lbl.grid(row=0, column=col, padx=(0, 8), pady=4)
        self._tier_widgets["drift_lbl"] = drift_lbl

    def _build_col_headers(self, parent, row, font_size, labels):
        frm = tk.Frame(parent, bg=self.C_COL_HDR_BG)
        frm.grid(row=row, column=0, sticky="ew", padx=4, pady=0)
        for i, w in enumerate([2, 1, 1, 1]):
            frm.columnconfigure(i, weight=w)
        for ci, txt in enumerate(labels):
            tk.Label(
                frm, text=txt,
                font=("Consolas", font_size, "bold"),
                fg=self.C_COL_HDR_FG, bg=self.C_COL_HDR_BG,
                padx=4, pady=3, anchor="center",
            ).grid(row=0, column=ci, sticky="ew", padx=1)

    def _build_axis_grid(self, parent, row, font_size):
        frm = tk.Frame(parent, bg=self.C_GRID_BG,
                       highlightbackground=self.C_SEP, highlightthickness=1)
        frm.grid(row=row, column=0, sticky="nsew", padx=4, pady=1)
        for i, w in enumerate([2, 1, 1, 1]):
            frm.columnconfigure(i, weight=w)
        frm.rowconfigure(0, weight=1)
        frm.rowconfigure(1, weight=1)
        frm.rowconfigure(2, weight=1)

        axes = {}
        for i, (key, label) in enumerate([
            ("roll",  "ROLL"),
            ("pitch", "PITCH"),
            ("yaw",   "YAW"),
        ]):
            if i > 0:
                tk.Frame(frm, bg=self.C_SEP, height=1).grid(
                    row=i * 2 - 1, column=0, columnspan=4, sticky="ew")

            actual_row = i * 2  # interleave separators

            tk.Label(
                frm, text=label,
                font=("Consolas", font_size - 1, "bold"),
                fg=self.C_AXIS_FG, bg=self.C_GRID_BG,
                padx=6, anchor="w",
            ).grid(row=actual_row, column=0, sticky="ew", padx=1, pady=2)

            rot = self._make_cell(frm, font_size)
            acc = self._make_cell(frm, font_size)
            ang = self._make_cell(frm, font_size)
            rot.grid(row=actual_row, column=1, sticky="ew", padx=1, pady=2)
            acc.grid(row=actual_row, column=2, sticky="ew", padx=1, pady=2)
            ang.grid(row=actual_row, column=3, sticky="ew", padx=1, pady=2)
            axes[key] = {"rot": rot, "acc": acc, "ang": ang}

        self._tier_widgets["axes"] = axes

    def _build_diag(self, parent):
        """Build diag panel but do NOT show it — _refit_diag decides."""
        diag = tk.Frame(parent, bg=self.C_HEADER_BG,
                        highlightbackground=self.C_SEP, highlightthickness=1)
        # NOT grid()'d here — _refit_diag calls grid() / grid_forget()
        self._tier_widgets["diag"] = diag

        for key, label in [
            ("rtt_lbl",    "TOTAL RTT"),
            ("oneway_lbl", "EST ONE-WAY"),
            ("cycle_lbl",  "FC CYCLE"),
        ]:
            row_f = tk.Frame(diag, bg=self.C_HEADER_BG)
            row_f.pack(fill="x", padx=8, pady=1)
            tk.Label(row_f, text=f"{label}:",
                     font=("Consolas", 8),
                     fg=self.C_LABEL, bg=self.C_HEADER_BG,
                     width=13, anchor="w",
                     ).pack(side="left")
            val = tk.Label(row_f, text="— ms",
                           font=("Consolas", 8, "bold"),
                           fg=self.C_NEUTRAL, bg=self.C_HEADER_BG)
            val.pack(side="left")
            self._tier_widgets[key] = val

    def _make_cell(self, parent, font_size):
        return tk.Label(
            parent,
            text="  ---",
            font=("Consolas", font_size, "bold"),
            fg=self.C_SAFE,
            bg=self.C_CELL_BG,
            highlightbackground=self.C_CELL_BORDER,
            highlightthickness=1,
            padx=4, pady=3,
            anchor="e",
        )

    # ══════════════════════════════════════════════════════════════════════════
    # Latency checkbox
    # ══════════════════════════════════════════════════════════════════════════

    def _on_latency_toggle(self):
        """User toggled the 'Link latency' checkbox — honour it immediately."""
        show_var = self._tier_widgets.get("show_diag")
        if show_var is None:
            return
        diag = self._tier_widgets.get("diag")
        if diag is None:
            return
        if show_var.get():
            # re-run refit so it re-checks available space
            self._refit_diag()
        else:
            diag.grid_forget()
            self._diag_visible = False

    # ══════════════════════════════════════════════════════════════════════════
    # Heading adjust + flash
    # ══════════════════════════════════════════════════════════════════════════

    def _on_adjust_pressed(self):
        if not self._mag_valid:
            return
        if self._on_adjust_heading:
            self._on_adjust_heading()

    def _start_flash(self):
        if self._flashing:
            return
        self._flashing = True
        self._flash_on = True
        self._do_flash()

    def _stop_flash(self):
        if self._flash_job is not None:
            try:
                self.after_cancel(self._flash_job)
            except Exception:
                pass
            self._flash_job = None
        self._flashing = False
        self._flash_on = False
        btn = self._tier_widgets.get("adj_btn")
        if btn:
            full = self._current_tier == "full"
            btn.config(
                bg=self.C_BTN_NORMAL, fg=self.C_BTN_FG,
                relief="flat",
                text="⊕  ADJUST HEADING" if full else "⊕  ADJUST HDG",
            )

    def _do_flash(self):
        if not self._flashing:
            return
        self._flash_on = not self._flash_on
        btn = self._tier_widgets.get("adj_btn")
        if btn:
            if self._flash_on:
                btn.config(bg=self.C_BTN_FLASH_A, fg="#ffffff",
                           text="⚠  ADJUST HEADING")
            else:
                btn.config(bg=self.C_BTN_FLASH_B, fg="#FF9999",
                           text="⚠  ADJUST HEADING")
        self._flash_job = self.after(500, self._do_flash)

    # ══════════════════════════════════════════════════════════════════════════
    # Public update
    # ══════════════════════════════════════════════════════════════════════════

    def update_ui(self, data: dict):
        self._last_data = data
        w = self.winfo_width()
        h = self.winfo_height()
        tier = self._classify(w, h)
        if tier != self._current_tier:
            self._build_tier(tier)
            self._current_tier = tier
            self.after_idle(lambda: self._refit_diag(h))
        self._render(data)

    def _render(self, data: dict):
        gx = data.get("gx", 0) / self.GYRO_SCALE
        gy = data.get("gy", 0) / self.GYRO_SCALE
        gz = data.get("gz", 0) / self.GYRO_SCALE
        ax = data.get("ax", 0) / self.ACCEL_SCALE
        ay = data.get("ay", 0) / self.ACCEL_SCALE
        az = data.get("az", 0) / self.ACCEL_SCALE

        if self.is_calibrating:
            self.calib_samples.append((gx, gy, gz))
            if len(self.calib_samples) >= 50:
                self._finish_calibration(self.calib_samples)
            return

        gx -= self.gyro_offsets["x"]
        gy -= self.gyro_offsets["y"]
        gz -= self.gyro_offsets["z"]

        roll           = data.get("roll",  0.0)
        pitch          = data.get("pitch", 0.0)
        now            = time.monotonic()
        mag_heading    = data.get("mag_heading_deg", 0.0)
        mag_valid      = data.get("mag_valid", False)
        fc_yaw_trimmed = data.get("yaw", 0.0) % 360

        self._last_mag_heading = mag_heading
        self._mag_valid        = mag_valid

        drift     = (fc_yaw_trimmed - mag_heading + 540) % 360 - 180 if mag_valid else 0.0
        abs_drift = abs(drift)

        # ── Drift label ───────────────────────────────────────────────────────
        drift_lbl = self._tier_widgets.get("drift_lbl")
        if drift_lbl:
            if mag_valid:
                clr = (self.C_CRIT if abs_drift >= self.DRIFT_CRIT_DEG else
                       self.C_WARN if abs_drift >= self.DRIFT_WARN_DEG else
                       self.C_SAFE)
                drift_lbl.config(text=f"{drift:>+6.1f}°", fg=clr)
            else:
                drift_lbl.config(text="NO MAG", fg=self.C_LABEL)

        # ── Flash control ─────────────────────────────────────────────────────
        if mag_valid and abs_drift >= self.DRIFT_CRIT_DEG:
            self._start_flash()
        else:
            self._stop_flash()

        # ── Tiny: single summary line ─────────────────────────────────────────
        summary = self._tier_widgets.get("summary")
        if summary:
            rc = self._angle_color(roll)
            pc = self._angle_color(pitch)
            yc = self._drift_color(abs_drift) if mag_valid else self.C_SAFE
            summary.config(
                text=f"R:{roll:>5.1f}°  P:{pitch:>5.1f}°  Y:{fc_yaw_trimmed:>5.1f}°",
                fg=self._worst_color(rc, pc, yc),
            )
            return

        # ── Axis cells ────────────────────────────────────────────────────────
        axes = self._tier_widgets.get("axes")
        if not axes:
            return

        for key, rot_val, acc_val, ang_val, is_yaw in [
            ("roll",  gx, ax, roll,           False),
            ("pitch", gy, ay, pitch,          False),
            ("yaw",   gz, az, fc_yaw_trimmed, True),
        ]:
            cells = axes.get(key)
            if not cells:
                continue

            rot_clr = self._gyro_color(key, abs(rot_val), now)
            acc_clr = self._accel_color(acc_val, is_yaw)
            ang_clr = (self._drift_color(abs_drift) if (is_yaw and mag_valid)
                       else self._angle_color(ang_val))

            def border(clr):
                return ("#3a0000" if clr == self.C_CRIT else
                        "#3a3000" if clr == self.C_WARN else
                        self.C_CELL_BORDER)

            cells["rot"].config(text=f"{rot_val:>7.2f}", fg=rot_clr,
                                highlightbackground=border(rot_clr))
            cells["acc"].config(text=f"{acc_val:>7.3f}", fg=acc_clr,
                                highlightbackground=border(acc_clr))
            cells["ang"].config(text=f"{ang_val:>7.1f}", fg=ang_clr,
                                highlightbackground=border(ang_clr))

        # ── Diag panel values ─────────────────────────────────────────────────
        if self._diag_visible:
            # Respect user checkbox if present
            show_var = self._tier_widgets.get("show_diag")
            if show_var is None or show_var.get():
                rtt      = data.get("rtt_ms",      0.0)
                fc_cycle = data.get("fc_cycle_ms", 0.0)
                rl = self._tier_widgets.get("rtt_lbl")
                ol = self._tier_widgets.get("oneway_lbl")
                cl = self._tier_widgets.get("cycle_lbl")
                if rl: rl.config(text=f"{rtt:>6.2f} ms")
                if ol: ol.config(text=f"{rtt/2:>6.2f} ms")
                if cl: cl.config(text=f"{fc_cycle:>6.2f} ms")

    # ══════════════════════════════════════════════════════════════════════════
    # Calibration
    # ══════════════════════════════════════════════════════════════════════════

    def _finish_calibration(self, samples):
        n = len(samples)
        self.gyro_offsets = {
            "x": sum(s[0] for s in samples) / n,
            "y": sum(s[1] for s in samples) / n,
            "z": sum(s[2] for s in samples) / n,
        }
        self.is_calibrating = False

    # ══════════════════════════════════════════════════════════════════════════
    # Colour helpers
    # ══════════════════════════════════════════════════════════════════════════

    def _gyro_color(self, axis: str, abs_dps: float, now: float) -> str:
        s = self._gyro_state[axis]
        if abs_dps >= self.GYRO_CRIT_DPS:
            s["state"]      = "critical"
            s["hold_until"] = now + self.HOLD_CRIT_SEC
        elif abs_dps >= self.GYRO_WARN_DPS:
            if s["state"] != "critical":
                s["state"]      = "warn"
                s["hold_until"] = now + self.HOLD_WARN_SEC
        else:
            if now >= s["hold_until"]:
                s["state"]      = "safe"
                s["hold_until"] = 0.0
        return (self.C_CRIT if s["state"] == "critical" else
                self.C_WARN if s["state"] == "warn"     else
                self.C_SAFE)

    def _accel_color(self, val: float, is_vertical: bool) -> str:
        if is_vertical:
            if   self.ACC_VERT_LO_OK < val < self.ACC_VERT_HI_OK: return self.C_SAFE
            elif self.ACC_VERT_LO_CR < val < self.ACC_VERT_HI_CR: return self.C_WARN
            else:                                                  return self.C_CRIT
        else:
            a = abs(val)
            if   a < self.ACC_LAT_WARN: return self.C_SAFE
            elif a < self.ACC_LAT_CRIT: return self.C_WARN
            else:                       return self.C_CRIT

    def _angle_color(self, deg: float) -> str:
        a = abs(deg)
        if   a < self.ANGLE_WARN_DEG: return self.C_SAFE
        elif a < self.ANGLE_CRIT_DEG: return self.C_WARN
        else:                         return self.C_CRIT

    def _drift_color(self, abs_drift: float) -> str:
        if   abs_drift >= self.DRIFT_CRIT_DEG: return self.C_CRIT
        elif abs_drift >= self.DRIFT_WARN_DEG: return self.C_WARN
        else:                                  return self.C_SAFE

    def _worst_color(self, *colors: str) -> str:
        priority = {self.C_CRIT: 2, self.C_WARN: 1, self.C_SAFE: 0, self.C_NEUTRAL: 0}
        return max(colors, key=lambda c: priority.get(c, 0))