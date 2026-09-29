import tkinter as tk
import time

import link_mode


class IMUWidget(tk.Frame):
    """
    MPU-6500 IMU / Attitude display — fully adaptive cockpit instrument.

    Aviation alerting standard (EASA CS-25 / FAA AC 25.1322):
      SAFE   — dark cell background (#0d1520),  green  text  (#00ff88)
      WARN   — solid amber fill   (#C87000),    black  text  (#000000)
               Amber = CAUTION level.  Black-on-amber passes WCAG AA (4.6:1).
      CRIT   — flashing red fill alternating between #CC0000 and #500000,
               white text (#FFFFFF) always — white-on-#CC0000 = 5.1:1 (WCAG AA).
               Flash cadence 500 ms ON / 500 ms OFF per DO-160 / EASA conventions.

    All three states keep the numeric value fully legible at all times.

    Link-aware (see link_mode.py):
      USB   — every cell live, as above.
      RADIO — the ELRS link carries attitude only. Rotation and G-force
              cells show a grey "USB" placeholder (never a fake 0 that would
              flash the vertical-G cell red), the heading-drift readout and
              ADJUST HEADING are disabled (the radio heading IS the FC yaw,
              so there is nothing to cross-check), and the latency footer
              shows attitude age / rate / link quality instead of MSP RTT.
              Attitude older than link_mode.STALE_ATTITUDE_MS is drawn in the
              muted "stale" colour so a frozen horizon is never read as live.
    Flashing is managed by a single shared _tick() loop that walks a registry
    of currently-flashing cells.  Adding / removing cells is O(1).

    Layout tiers (unchanged from original):
      FULL    (h>=210, w>=400)
      MEDIUM  (h>=150, w>=290)
      COMPACT (h>=90,  w>=190)
      TINY    (h< 90  or w<190)
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

    # ── Layout pixel budgets ──────────────────────────────────────────────────
    _HDR_H    = 28
    _ROW_H    = 26
    _COLHDR_H = 22
    _DIAG_H   = 54
    _PAD      = 10

    # ── Base cockpit palette ──────────────────────────────────────────────────
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
    C_NEUTRAL     = "#00d4ff"
    C_BTN_NORMAL  = "#0a1520"
    C_BTN_FG      = "#00d4ff"
    C_BTN_FLASH_A = "#FF2222"
    C_BTN_FLASH_B = "#880000"

    # ── Alert state colours (aviation standard) ───────────────────────────────
    # SAFE
    C_SAFE_BG   = "#0d1520"   # dark cockpit cell
    C_SAFE_FG   = "#00ff88"   # green value text
    C_SAFE_BDR  = "#1e2a3a"   # subtle border

    # WARN  — amber caution (EASA CS-25 / FAA AC 25.1322 amber = caution)
    C_WARN_BG   = "#C87000"   # solid amber fill
    C_WARN_FG   = "#000000"   # black text — contrast 4.6:1 on amber (WCAG AA)
    C_WARN_BDR  = "#FF9900"   # bright amber border for extra pop

    # CRIT  — red warning, flashing
    C_CRIT_BG_A = "#CC0000"   # flash phase A — bright red   (white-on-red 5.1:1)
    C_CRIT_BG_B = "#500000"   # flash phase B — dark red     (white-on-dark 8.0:1)
    C_CRIT_FG   = "#FFFFFF"   # white text — readable on both phases
    C_CRIT_BDR  = "#FF4444"   # red border

    # Flash cadence: 500 ms per half-period (1 Hz total)
    _FLASH_MS = 500

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

        # Registry of cells currently in CRIT state: label widget → True
        self._crit_cells: dict[tk.Label, bool] = {}
        self._cell_flash_phase = True   # True = phase-A (bright)
        self._cell_flash_job   = None

        self._last_mag_heading = 0.0
        self._mag_valid        = False
        self._last_data        = {}

        self._radio         = False   # True while the ELRS link feeds us
        # Latency / link footer (RTT, one-way, FC cycle — or attitude age /
        # rate / LQ on radio). Off by default: constantly changing numbers
        # distract the pilot. One setting shared by every layout tier.
        self._show_latency  = tk.BooleanVar(self, value=False)

        self._current_tier  = None
        self._tier_widgets  = {}
        self._diag_visible  = False

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self._container = tk.Frame(self, bg=self.C_BG)
        self._container.grid(row=0, column=0, sticky="nsew")

        self.bind("<Configure>", self._on_resize)

    # ══════════════════════════════════════════════════════════════════════════
    # Cell flash engine — single shared ticker
    # ══════════════════════════════════════════════════════════════════════════

    def _start_cell_flash(self):
        if self._cell_flash_job is not None:
            return
        try:
            self._cell_flash_ticker()
        except tk.TclError:
            self._cell_flash_job = None

    def _cell_flash_ticker(self):
        """Toggle all CRIT cells between phase-A and phase-B every _FLASH_MS."""
        self._cell_flash_phase = not self._cell_flash_phase
        bg = self.C_CRIT_BG_A if self._cell_flash_phase else self.C_CRIT_BG_B

        dead = []
        for lbl in list(self._crit_cells):
            try:
                lbl.config(bg=bg)
            except tk.TclError:
                dead.append(lbl)

        for lbl in dead:
            self._crit_cells.pop(lbl, None)

        if self._crit_cells:
            try:
                self._cell_flash_job = self.after(self._FLASH_MS,
                                                  self._cell_flash_ticker)
            except tk.TclError:
                self._cell_flash_job = None
        else:
            self._cell_flash_job = None

    def _register_crit(self, lbl: tk.Label):
        if lbl not in self._crit_cells:
            self._crit_cells[lbl] = True
            self._start_cell_flash()

    def _unregister_crit(self, lbl: tk.Label):
        self._crit_cells.pop(lbl, None)
        # Ticker stops itself automatically when dict empties

    # ══════════════════════════════════════════════════════════════════════════
    # Cell state application
    # ══════════════════════════════════════════════════════════════════════════

    def _apply_cell_state(self, lbl: tk.Label, value_text: str, state: str):
        """
        Apply SAFE / WARN / CRIT visual state to a data cell.

        SAFE  — dark bg, green text, subtle border
        WARN  — solid amber bg, black text, amber border
        CRIT  — enter flash registry (flashing red), white text, red border
        """
        if state == "na":        # not carried by the active link
            self._unregister_crit(lbl)
            lbl.config(
                text=value_text,
                bg=link_mode.C_NA_BG,
                fg=link_mode.C_NA_FG,
                highlightbackground=self.C_SAFE_BDR,
            )
        elif state == "stale":   # real value, but too old to trust
            self._unregister_crit(lbl)
            lbl.config(
                text=value_text,
                bg=self.C_SAFE_BG,
                fg=link_mode.C_STALE_FG,
                highlightbackground=self.C_SAFE_BDR,
            )
        elif state == "warn":
            self._unregister_crit(lbl)
            lbl.config(
                text=value_text,
                bg=self.C_WARN_BG,
                fg=self.C_WARN_FG,
                highlightbackground=self.C_WARN_BDR,
            )
        elif state == "crit":
            lbl.config(
                text=value_text,
                bg=self.C_CRIT_BG_A,   # ticker will alternate from here
                fg=self.C_CRIT_FG,
                highlightbackground=self.C_CRIT_BDR,
            )
            self._register_crit(lbl)
        else:  # safe
            self._unregister_crit(lbl)
            lbl.config(
                text=value_text,
                bg=self.C_SAFE_BG,
                fg=self.C_SAFE_FG,
                highlightbackground=self.C_SAFE_BDR,
            )

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
        diag = self._tier_widgets.get("diag")
        if diag is None:
            return
        if not self._show_latency.get():
            if self._diag_visible:
                diag.grid_forget()
                self._diag_visible = False
            return

        if total_h is None:
            total_h = self.winfo_height()

        tier = self._current_tier
        if tier in ("full", "medium"):
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
        # Remove all cells from the flash registry before destroying them
        self._crit_cells.clear()
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
        self._apply_link_mode()   # new widgets start with USB labels

    def _build_full(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.rowconfigure(0, weight=0)
        c.rowconfigure(1, weight=0)
        c.rowconfigure(2, weight=1)
        c.rowconfigure(3, weight=0)

        self._build_header(c, row=0, full=True)
        self._build_col_headers(c, row=1, font_size=8, labels=[
            "FLIGHT AXIS", "ROTATION  °/s", "G-FORCE  g", "ANGLE / HDG  °"
        ])
        self._build_axis_grid(c, row=2, font_size=12)
        self._build_diag(c)

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

    def _build_compact(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.rowconfigure(0, weight=0)
        c.rowconfigure(1, weight=1)

        self._build_header(c, row=0, full=False)
        self._build_axis_grid(c, row=1, font_size=9)
        self._build_diag(c)

    def _build_tiny(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.rowconfigure(0, weight=1)

        summary = tk.Label(
            c, text="R: ---  P: ---  Y: ---",
            font=("Consolas", 9, "bold"),
            fg=self.C_SAFE_FG, bg=self.C_BG,
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
            chk = tk.Checkbutton(
                hdr, text="Link latency",
                variable=self._show_latency,
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
        else:
            # Narrower tiers: a compact toggle for the same setting
            lat_btn = tk.Checkbutton(
                hdr, text="⏱", indicatoron=False,
                variable=self._show_latency,
                command=self._on_latency_toggle,
                font=("Consolas", 8), width=2,
                fg=self.C_LABEL, bg=self.C_HEADER_BG,
                selectcolor="#1e3a50",
                activebackground=self.C_HEADER_BG,
                activeforeground=self.C_NEUTRAL,
                bd=0, highlightthickness=0,
            )
            lat_btn.grid(row=0, column=col, padx=(6, 2), pady=4)
            self._tier_widgets["lat_btn"] = lat_btn
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

        # Source tag, right-aligned (column 10 is the stretch column)
        src_lbl = tk.Label(hdr, text="", font=("Consolas", 7, "bold"),
                           fg=link_mode.C_USB, bg=self.C_HEADER_BG)
        src_lbl.grid(row=0, column=11, padx=(4, 8), pady=4)
        self._tier_widgets["src_lbl"] = src_lbl

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

        frm.rowconfigure(0, weight=1, minsize=0)
        frm.rowconfigure(1, weight=0, minsize=1)
        frm.rowconfigure(2, weight=1, minsize=0)
        frm.rowconfigure(3, weight=0, minsize=1)
        frm.rowconfigure(4, weight=1, minsize=0)

        axes = {}
        for i, (key, label) in enumerate([
            ("roll",  "ROLL"),
            ("pitch", "PITCH"),
            ("yaw",   "YAW"),
        ]):
            actual_row = i * 2

            if i > 0:
                tk.Frame(frm, bg=self.C_SEP, height=1).grid(
                    row=actual_row - 1, column=0, columnspan=4, sticky="ew")

            tk.Label(
                frm, text=label,
                font=("Consolas", font_size - 1, "bold"),
                fg=self.C_AXIS_FG, bg=self.C_GRID_BG,
                padx=6, anchor="w",
            ).grid(row=actual_row, column=0, sticky="nsew", padx=1, pady=2)

            rot = self._make_cell(frm, font_size)
            acc = self._make_cell(frm, font_size)
            ang = self._make_cell(frm, font_size)
            rot.grid(row=actual_row, column=1, sticky="nsew", padx=1, pady=2)
            acc.grid(row=actual_row, column=2, sticky="nsew", padx=1, pady=2)
            ang.grid(row=actual_row, column=3, sticky="nsew", padx=1, pady=2)
            axes[key] = {"rot": rot, "acc": acc, "ang": ang}

        self._tier_widgets["axes"] = axes

    def _build_diag(self, parent):
        diag = tk.Frame(parent, bg=self.C_HEADER_BG,
                        highlightbackground=self.C_SEP, highlightthickness=1)
        self._tier_widgets["diag"] = diag

        for key, label in [
            ("rtt_lbl",    "TOTAL RTT"),
            ("oneway_lbl", "EST ONE-WAY"),
            ("cycle_lbl",  "FC CYCLE"),
        ]:
            row_f = tk.Frame(diag, bg=self.C_HEADER_BG)
            row_f.pack(fill="x", padx=8, pady=1)
            name = tk.Label(row_f, text=f"{label}:",
                            font=("Consolas", 8),
                            fg=self.C_LABEL, bg=self.C_HEADER_BG,
                            width=13, anchor="w")
            name.pack(side="left")
            self._tier_widgets[key + "_name"] = name
            val = tk.Label(row_f, text="— ms",
                           font=("Consolas", 8, "bold"),
                           fg=self.C_NEUTRAL, bg=self.C_HEADER_BG)
            val.pack(side="left")
            self._tier_widgets[key] = val

    def _make_cell(self, parent, font_size):
        """Create a data cell in its initial SAFE state."""
        return tk.Label(
            parent,
            text="  ---",
            font=("Consolas", font_size, "bold"),
            fg=self.C_SAFE_FG,
            bg=self.C_SAFE_BG,
            highlightbackground=self.C_SAFE_BDR,
            highlightthickness=1,
            padx=4, pady=3,
            anchor="e",
        )

    # ══════════════════════════════════════════════════════════════════════════
    # Latency checkbox
    # ══════════════════════════════════════════════════════════════════════════

    def _on_latency_toggle(self):
        # _refit_diag() shows the footer if the setting is on and it fits,
        # and hides it otherwise.
        self._refit_diag()

    # ══════════════════════════════════════════════════════════════════════════
    # Heading adjust + heading-button flash
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
            # FIX: wrap btn.config() in try/except tk.TclError.
            # Without this, if a flash job fires on a widget that has been
            # partially torn down (e.g. during test teardown), the TclError
            # propagates through Tkinter's report_callback_exception() handler
            # instead of the normal Python exception path, bypassing any
            # surrounding try/except and failing whichever pytest test is
            # currently cleaning up.
            try:
                if self._flash_on:
                    btn.config(bg=self.C_BTN_FLASH_A, fg="#ffffff",
                               text="⚠  ADJUST HEADING")
                else:
                    btn.config(bg=self.C_BTN_FLASH_B, fg="#FF9999",
                               text="⚠  ADJUST HEADING")
            except tk.TclError:
                # Widget is gone — stop flashing silently.
                self._flashing   = False
                self._flash_job  = None
                return

        # FIX: same guard on the reschedule call.
        try:
            self._flash_job = self.after(500, self._do_flash)
        except tk.TclError:
            self._flash_job = None
            self._flashing  = False

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
        radio   = link_mode.is_radio(data)
        raw_imu = link_mode.available(data, "raw_imu")
        att_old = link_mode.stale_ms(data, "attitude")
        if radio != self._radio:
            self._radio = radio
            self._apply_link_mode()

        gx = data.get("gx", 0) / self.GYRO_SCALE
        gy = data.get("gy", 0) / self.GYRO_SCALE
        gz = data.get("gz", 0) / self.GYRO_SCALE
        ax = data.get("ax", 0) / self.ACCEL_SCALE
        ay = data.get("ay", 0) / self.ACCEL_SCALE
        az = data.get("az", 0) / self.ACCEL_SCALE

        if self.is_calibrating and raw_imu:
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
        # On the radio link "mag heading" is the FC's own fused yaw — there
        # is no independent magnetometer to cross-check against.
        mag_valid      = data.get("mag_valid", False) and link_mode.available(data, "raw_mag")
        fc_yaw_trimmed = data.get("yaw", 0.0) % 360

        self._last_mag_heading = mag_heading
        self._mag_valid        = mag_valid

        drift     = (fc_yaw_trimmed - mag_heading + 540) % 360 - 180 if mag_valid else 0.0
        abs_drift = abs(drift)

        # ── Drift label ───────────────────────────────────────────────────────
        drift_lbl = self._tier_widgets.get("drift_lbl")
        if drift_lbl:
            if mag_valid:
                clr = (self.C_CRIT_BG_A if abs_drift >= self.DRIFT_CRIT_DEG else
                       self.C_WARN_BG    if abs_drift >= self.DRIFT_WARN_DEG else
                       self.C_SAFE_FG)
                drift_lbl.config(text=f"{drift:>+6.1f}°", fg=clr)
            elif not link_mode.available(data, "raw_mag"):
                drift_lbl.config(text=link_mode.NA_LONG, fg=link_mode.C_NA_FG)
            else:
                drift_lbl.config(text="NO MAG", fg=self.C_LABEL)

        src_lbl = self._tier_widgets.get("src_lbl")
        if src_lbl:
            txt, clr = link_mode.source_tag(data)
            if att_old:
                txt, clr = link_mode.stale_text(att_old), link_mode.C_STALE_FG
            src_lbl.config(text=txt, fg=clr)

        # ── Heading-button flash ──────────────────────────────────────────────
        if mag_valid and abs_drift >= self.DRIFT_CRIT_DEG:
            self._start_flash()
        else:
            self._stop_flash()

        # ── TINY: single summary line ─────────────────────────────────────────
        summary = self._tier_widgets.get("summary")
        if summary:
            rc = self._angle_state(roll)
            pc = self._angle_state(pitch)
            yc = self._drift_state(abs_drift) if mag_valid else "safe"
            worst = self._worst_state(rc, pc, yc)
            fg = {
                "crit": self.C_CRIT_FG,
                "warn": self.C_WARN_FG,
                "safe": self.C_SAFE_FG,
            }[worst]
            bg = {
                "crit": self.C_CRIT_BG_A,
                "warn": self.C_WARN_BG,
                "safe": self.C_BG,
            }[worst]
            if att_old:
                fg, bg = link_mode.C_STALE_FG, self.C_BG
            summary.config(
                text=f"R:{roll:>5.1f}°  P:{pitch:>5.1f}°  Y:{fc_yaw_trimmed:>5.1f}°",
                fg=fg, bg=bg,
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

            if is_yaw:
                # A heading is not a tilt: without a mag reference there is
                # nothing to alarm on (the old code ran it through the
                # roll/pitch limits, so any heading past 30° flashed red).
                ang_state = self._drift_state(abs_drift) if mag_valid else "safe"
            else:
                ang_state = self._angle_state(ang_val)
            if att_old:
                ang_state = "stale"
            self._apply_cell_state(cells["ang"], f"{ang_val:>7.1f}", ang_state)

            if not raw_imu:
                self._apply_cell_state(cells["rot"], link_mode.NA_SHORT, "na")
                self._apply_cell_state(cells["acc"], link_mode.NA_SHORT, "na")
                continue

            rot_state = self._gyro_state_label(key, abs(rot_val), now)
            acc_state = self._accel_state(acc_val, is_yaw)
            self._apply_cell_state(cells["rot"], f"{rot_val:>7.2f}", rot_state)
            self._apply_cell_state(cells["acc"], f"{acc_val:>7.3f}", acc_state)

        # ── Diag panel ────────────────────────────────────────────────────────
        if self._diag_visible:
            if radio:
                att_age = link_mode.age_ms(data, "attitude")
                rate    = float(data.get("rate_attitude_hz", 0.0))
                lq      = int(data.get("rc_link_quality", -1))
                rl = self._tier_widgets.get("rtt_lbl")
                ol = self._tier_widgets.get("oneway_lbl")
                cl = self._tier_widgets.get("cycle_lbl")
                if rl: rl.config(text=f"{att_age:>6d} ms" if att_age >= 0 else "  never",
                                 fg=link_mode.C_STALE_FG if att_old else self.C_NEUTRAL)
                if ol: ol.config(text=f"{rate:>6.1f} Hz", fg=self.C_NEUTRAL)
                if cl: cl.config(text=f"{lq:>6d} %" if lq >= 0 else "  --- %",
                                 fg=self.C_NEUTRAL)
            else:
                rtt      = data.get("rtt_ms",      0.0)
                fc_cycle = data.get("fc_cycle_ms", 0.0)
                rl = self._tier_widgets.get("rtt_lbl")
                ol = self._tier_widgets.get("oneway_lbl")
                cl = self._tier_widgets.get("cycle_lbl")
                if rl: rl.config(text=f"{rtt:>6.2f} ms")
                if ol: ol.config(text=f"{rtt/2:>6.2f} ms")
                if cl: cl.config(text=f"{fc_cycle:>6.2f} ms")

    # ══════════════════════════════════════════════════════════════════════════
    # Link mode (USB ↔ radio)
    # ══════════════════════════════════════════════════════════════════════════

    _DIAG_NAMES = {
        False: {"rtt_lbl": "TOTAL RTT", "oneway_lbl": "EST ONE-WAY",
                "cycle_lbl": "FC CYCLE"},
        True:  {"rtt_lbl": "ATT AGE", "oneway_lbl": "ATT RATE",
                "cycle_lbl": "RADIO LQ"},
    }

    def _apply_link_mode(self):
        """Relabel/enable the parts that differ between USB and radio."""
        names = self._DIAG_NAMES[self._radio]
        for key, text in names.items():
            lbl = self._tier_widgets.get(key + "_name")
            if lbl:
                lbl.config(text=f"{text}:")
        btn = self._tier_widgets.get("adj_btn")
        if btn:
            if self._radio:
                self._stop_flash()
                btn.config(state="disabled",
                           disabledforeground=link_mode.C_NA_FG)
            else:
                btn.config(state="normal")
        if self._radio and self.is_calibrating:
            # No raw gyro on the radio link — abandon a running calibration.
            self.is_calibrating = False
            self.calib_samples = []

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
    # State helpers  (return "safe" | "warn" | "crit")
    # ══════════════════════════════════════════════════════════════════════════

    def _gyro_state_label(self, axis: str, abs_dps: float, now: float) -> str:
        s = self._gyro_state[axis]
        if abs_dps >= self.GYRO_CRIT_DPS:
            s["state"]      = "crit"
            s["hold_until"] = now + self.HOLD_CRIT_SEC
        elif abs_dps >= self.GYRO_WARN_DPS:
            if s["state"] == "crit" and now < s["hold_until"]:
                pass  # CRIT hold active — suppress downgrade to WARN
            else:
                s["state"]      = "warn"
                s["hold_until"] = now + self.HOLD_WARN_SEC
        else:
            if now >= s["hold_until"]:
                s["state"]      = "safe"
                s["hold_until"] = 0.0
        return s["state"]

    def _accel_state(self, val: float, is_vertical: bool) -> str:
        if is_vertical:
            if   self.ACC_VERT_LO_OK < val < self.ACC_VERT_HI_OK: return "safe"
            elif self.ACC_VERT_LO_CR < val < self.ACC_VERT_HI_CR: return "warn"
            else:                                                  return "crit"
        else:
            a = abs(val)
            if   a < self.ACC_LAT_WARN: return "safe"
            elif a < self.ACC_LAT_CRIT: return "warn"
            else:                       return "crit"

    def _angle_state(self, deg: float) -> str:
        a = abs(deg)
        if   a < self.ANGLE_WARN_DEG: return "safe"
        elif a < self.ANGLE_CRIT_DEG: return "warn"
        else:                         return "crit"

    def _drift_state(self, abs_drift: float) -> str:
        if   abs_drift >= self.DRIFT_CRIT_DEG: return "crit"
        elif abs_drift >= self.DRIFT_WARN_DEG: return "warn"
        else:                                  return "safe"

    def _worst_state(self, *states: str) -> str:
        priority = {"crit": 2, "warn": 1, "safe": 0}
        return max(states, key=lambda s: priority.get(s, 0))

    # Legacy colour helpers retained for any external callers
    def _gyro_color(self, axis, abs_dps, now):
        s = self._gyro_state_label(axis, abs_dps, now)
        return self.C_CRIT_BG_A if s == "crit" else self.C_WARN_BG if s == "warn" else self.C_SAFE_FG

    def _accel_color(self, val, is_vertical):
        s = self._accel_state(val, is_vertical)
        return self.C_CRIT_BG_A if s == "crit" else self.C_WARN_BG if s == "warn" else self.C_SAFE_FG

    def _angle_color(self, deg):
        s = self._angle_state(deg)
        return self.C_CRIT_BG_A if s == "crit" else self.C_WARN_BG if s == "warn" else self.C_SAFE_FG

    def _drift_color(self, abs_drift):
        s = self._drift_state(abs_drift)
        return self.C_CRIT_BG_A if s == "crit" else self.C_WARN_BG if s == "warn" else self.C_SAFE_FG

    def _worst_color(self, *colors):
        priority = {self.C_CRIT_BG_A: 2, self.C_WARN_BG: 1,
                    self.C_SAFE_FG: 0, self.C_NEUTRAL: 0}
        return max(colors, key=lambda c: priority.get(c, 0))

    def destroy(self):
        # Cancel all pending after() jobs before Tkinter destroys the widget
        for job in [self._cell_flash_job, self._flash_job]:
            if job is not None:
                try:
                    self.after_cancel(job)
                except Exception:
                    pass
        self._cell_flash_job = None
        self._flash_job      = None
        self._crit_cells.clear()
        super().destroy()