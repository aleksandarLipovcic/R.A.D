"""
ArmingWidget.py  —  Arming Diagnostics Panel
=============================================
Aviation pre-flight checklist style.  Fully responsive — columns reflow
when panel width changes.

Behaviour:
  All flags clear  →  large green  "READY TO ARM"
  Any flag set     →  red "BLOCKED" header + count + coloured checklist rows

Each row:
  ● REASON NAME    ← red filled dot  = blocking
  ○ REASON NAME    ← dim  empty dot  = clear / OK

Feed via  update_arming(data_dict)  every UI tick.
"""

import tkinter as tk

# ── Palette ───────────────────────────────────────────────────────────────────
_BG       = "#0f0f1a"
_BG2      = "#12121f"
_BORDER   = "#1e2a3a"
_LABEL_FG = "#4a6080"
_VALUE_FG = "#c8dff0"
_GREEN    = "#00ff88"
_ORANGE   = "#ffaa00"
_RED      = "#ff3355"
_DIM      = "#1a2a3a"
_DIM_FG   = "#263a4a"

_F_SMALL  = ("Consolas",  8, "bold")
_F_READY  = ("Consolas", 14, "bold")
_F_REASON = ("Consolas",  9, "bold")
_F_COUNT  = ("Consolas", 10, "bold")
_F_BODY   = ("Consolas",  8)

# Full ordered list — matches decodeArmingDisable() in DroneLink.cpp
_BITS = [
    (1 << 0,  "NO GYRO"),
    (1 << 1,  "FAILSAFE"),
    (1 << 2,  "RX FAILSAFE"),
    (1 << 3,  "BAD RX"),
    (1 << 4,  "BOX FAILSAFE"),
    (1 << 5,  "RUNAWAY TKOFF"),
    (1 << 6,  "CRASH DETECT"),
    (1 << 7,  "THROTTLE HIGH"),
    (1 << 8,  "NOT LEVEL"),
    (1 << 9,  "BOOT GRACE"),
    (1 << 10, "NO PREARM"),
    (1 << 11, "CPU OVERLOAD"),
    (1 << 12, "CALIBRATING"),
    (1 << 13, "CLI ACTIVE"),
    (1 << 14, "OSD MENU"),
    (1 << 15, "BST"),
    (1 << 16, "MSP OVERRIDE"),
    (1 << 17, "PARALYZE"),
    (1 << 18, "GPS NOT READY"),
    (1 << 21, "RESCUE SW"),
    (1 << 23, "DSHOT BITBANG"),
    (1 << 24, "ACC CAL NEEDED"),
    (1 << 25, "MOTOR PROTOCOL"),
    (1 << 26, "ARM SW OFF"),
]


class ArmingWidget(tk.Frame):
    """Aviation-style arming diagnostics checklist."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=_BG, **kwargs)
        self._last_flags: int = -1   # force first draw
        self._build_ui()
        self.bind("<Configure>", self._on_resize)

    # =========================================================================
    # Build
    # =========================================================================

    def _build_ui(self):

        # ── Header ────────────────────────────────────────────────────────────
        hdr = tk.Frame(self, bg=_BG2,
                       highlightthickness=1, highlightbackground=_BORDER)
        hdr.pack(fill="x", padx=6, pady=(6, 4))

        tk.Label(hdr, text="PRE-FLIGHT ARMING CHECK",
                 bg=_BG2, fg=_LABEL_FG, font=_F_SMALL).pack(
            side="left", padx=10, pady=5)

        self._count_lbl = tk.Label(hdr, text="",
                                    bg=_BG2, fg=_ORANGE, font=_F_COUNT)
        self._count_lbl.pack(side="right", padx=8, pady=5)

        self._status_lbl = tk.Label(hdr, text="WAITING…",
                                     bg=_BG2, fg=_ORANGE, font=_F_READY)
        self._status_lbl.pack(side="right", padx=(4, 0), pady=5)

        # ── Checklist container (holds col frames, rebuilt on resize) ─────────
        self._list_frame = tk.Frame(self, bg=_BG)
        self._list_frame.pack(fill="both", expand=True, padx=6, pady=(0, 6))

        # Build row widgets once — move them between columns on resize
        self._rows: dict[int, tuple] = {}   # bit → (row_frame, canvas, dot, lbl)
        for bit, name in _BITS:
            rf = tk.Frame(self._list_frame, bg=_BG)
            cv = tk.Canvas(rf, width=10, height=10,
                           bg=_BG, highlightthickness=0)
            cv.pack(side="left", padx=(0, 4))
            dot = cv.create_oval(1, 1, 9, 9, fill=_DIM_FG, outline="")
            lbl = tk.Label(rf, text=name, bg=_BG, fg=_DIM_FG,
                           font=_F_REASON)
            lbl.pack(side="left")
            self._rows[bit] = (rf, cv, dot, lbl)

        # Initial column layout
        self._n_cols  = 0
        self._col_frames: list[tk.Frame] = []
        self._layout_columns(2)

    # =========================================================================
    # Responsive column layout
    # =========================================================================

    def _on_resize(self, event):
        w = event.width
        # Choose column count based on available width
        if w >= 560:
            n = 3
        elif w >= 340:
            n = 2
        else:
            n = 1
        if n != self._n_cols:
            self._layout_columns(n)

    def _layout_columns(self, n: int):
        self._n_cols = n

        # Destroy old column frames (they hold grid references, not widgets)
        for cf in self._col_frames:
            cf.destroy()
        self._col_frames = []

        # Create fresh column frames
        for i in range(n):
            cf = tk.Frame(self._list_frame, bg=_BG)
            cf.grid(row=0, column=i, sticky="nw", padx=(0, 10))
            self._list_frame.columnconfigure(i, weight=1)
            self._col_frames.append(cf)

        # Distribute rows round-robin across columns
        total = len(_BITS)
        rows_per_col = (total + n - 1) // n   # ceiling division

        for idx, (bit, _) in enumerate(_BITS):
            rf, cv, dot, lbl = self._rows[bit]
            col_idx = idx // rows_per_col
            col_idx = min(col_idx, n - 1)     # clamp for last short column
            row_idx = idx % rows_per_col

            rf.grid(in_=self._col_frames[col_idx],
                    row=row_idx, column=0,
                    sticky="w", pady=1)

    # =========================================================================
    # Public update
    # =========================================================================

    def update_arming(self, data: dict):
        flags = int(data.get("arming_disable_flags", 0))
        if flags == self._last_flags:
            return   # nothing changed — skip redraw
        self._last_flags = flags

        # Header
        active = bin(flags).count("1")
        if flags == 0:
            self._status_lbl.config(text="READY TO ARM", fg=_GREEN)
            self._count_lbl.config(text="")
        else:
            self._status_lbl.config(text="BLOCKED", fg=_RED)
            n_word = f"{active} reason{'s' if active != 1 else ''}"
            self._count_lbl.config(text=n_word, fg=_ORANGE)

        # Row colours
        for bit, _ in _BITS:
            rf, cv, dot, lbl = self._rows[bit]
            active_bit = bool(flags & bit)
            if active_bit:
                cv.itemconfig(dot, fill=_RED)
                lbl.config(fg=_RED)
                cv.config(bg=_BG)
                rf.config(bg=_BG)
            else:
                cv.itemconfig(dot, fill=_DIM_FG)
                lbl.config(fg=_DIM_FG)
                cv.config(bg=_BG)
                rf.config(bg=_BG)