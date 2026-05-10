import tkinter as tk
from tkinter import ttk
import time


class BaroWidget(ttk.LabelFrame):
    """
    BMP280 Barometer display widget — aviation altimeter style.

    ┌─────────────────────────────────────────────────────────────────┐
    │  BMP280 Barometer                             [Set QNH]        │
    ├──────────────────────┬──────────────────────────────────────────┤
    │  ALT (m)             │  ALT (ft)                               │
    │   ±XXXX.X m          │   ±XXXXX ft                             │
    ├──────────────────────┼──────────────────────────────────────────┤
    │  VARIO (m/s)         │  VARIO (ft/min)                         │
    │   ±XX.XX m/s         │   ±XXXX ft/min                          │
    ├──────────────────────┴──────────────────────────────────────────┤
    │  QNH ref: ±XXXX.X m   Home alt: ±XXXX.X m                     │
    └─────────────────────────────────────────────────────────────────┘

    DESIGN
    ──────
    Altitude is displayed in both metric and imperial (aviation standard).
    QNH zeroing: pressing "Set QNH" records the current altitude as the
    reference point so the display shows height above that point — useful
    for zeroing on the launch pad.

    Colour coding (matches IMUWidget conventions):
        green   — normal / safe
        yellow  — caution  (altitude > WARN_ALT_M or |vario| > WARN_VARIO)
        red     — critical (altitude > CRIT_ALT_M or |vario| > CRIT_VARIO)

    THRESHOLDS  (sensible for a 10-inch LR outdoor quad)
        Altitude warn:     50 m AGL
        Altitude critical: 120 m AGL  (legal limit in many countries)
        Vario warn:        3 m/s
        Vario critical:    8 m/s
    """

    # ── Scale factors (match BaroBMP280.h constants) ──────────────────────────
    CM_TO_M   = 0.01
    CM_TO_FT  = 0.0328084
    MPS_TO_FPM = 196.85   # 1 m/s = 196.85 ft/min

    # ── Warning thresholds ────────────────────────────────────────────────────
    WARN_ALT_M   =  50.0   # metres AGL
    CRIT_ALT_M   = 120.0   # metres AGL
    WARN_VARIO   =   3.0   # m/s
    CRIT_VARIO   =   8.0   # m/s

    # ── Colours (match IMUWidget) ─────────────────────────────────────────────
    CLR_SAFE     = "#99FF99"
    CLR_WARN     = "#FFFF99"
    CLR_CRITICAL = "#FF4444"
    CLR_OFF      = "#E0E0E0"   # used when baro_valid = False
    CLR_BG       = "#f0f0f0"

    def __init__(self, parent):
        super().__init__(parent, text="BMP280 Barometer", padding=10)

        # QNH offset — raw altitude (cm) recorded when pilot presses Set QNH
        self._qnh_ref_cm: int = 0

        # Last known raw altitude cm (for QNH zeroing)
        self._last_alt_cm: int = 0

        self._build_ui()

    # ── UI construction ───────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Header row: "Set QNH" button ────────────────────────────────────
        hdr = tk.Frame(self)
        hdr.grid(row=0, column=0, columnspan=4, sticky="ew", pady=(0, 8))

        tk.Label(hdr, text="Altitude above QNH reference",
                 font=("Arial", 9, "italic")).pack(side="left")

        self._qnh_btn = tk.Button(
            hdr, text="Set QNH (zero)",
            command=self._set_qnh,
            relief="raised", padx=6, pady=2
        )
        self._qnh_btn.pack(side="right")

        # ── Main data grid ──────────────────────────────────────────────────
        grid = tk.Frame(self, bd=1, relief="solid", padx=4, pady=4)
        grid.grid(row=1, column=0, columnspan=4, sticky="nsew")

        # Column headers
        for col, txt in enumerate(["Parameter", "Value (metric)",
                                   "Value (imperial)", "Status"]):
            tk.Label(grid, text=txt,
                     font=("Arial", 9, "bold")).grid(
                row=0, column=col, padx=10, pady=4, sticky="w")

        # ── Row helper ────────────────────────────────────────────────────
        self._cells = {}

        rows_def = [
            ("alt_m",    "Altitude",      "m",      "ft"),
            ("alt_qnh",  "Alt (QNH)",     "m",      "ft"),
            ("vario",    "Vert. speed",   "m/s",    "ft/min"),
        ]

        for r, (key, label, unit_m, unit_imp) in enumerate(rows_def, start=1):
            tk.Label(grid, text=f"{label}:",
                     font=("Arial", 10)).grid(
                row=r, column=0, sticky="e", padx=6)

            val_m = tk.Label(grid, text=f"--- {unit_m}",
                             font=("Consolas", 12, "bold"),
                             width=14, bg=self.CLR_OFF, anchor="e")
            val_i = tk.Label(grid, text=f"--- {unit_imp}",
                             font=("Consolas", 12, "bold"),
                             width=14, bg=self.CLR_OFF, anchor="e")
            status = tk.Label(grid, text="---",
                              font=("Arial", 9), width=10)

            val_m.grid(  row=r, column=1, padx=4, pady=3)
            val_i.grid(  row=r, column=2, padx=4, pady=3)
            status.grid( row=r, column=3, padx=4, pady=3, sticky="w")

            self._cells[key] = {
                "val_m":  val_m,
                "val_i":  val_i,
                "status": status,
            }

        # ── QNH reference readout ────────────────────────────────────────
        ref_row = tk.Frame(self)
        ref_row.grid(row=2, column=0, columnspan=4, sticky="ew", pady=(8, 0))

        self._qnh_ref_lbl = tk.Label(
            ref_row,
            text="QNH ref: not set  |  Raw alt: ---",
            font=("Consolas", 9),
            fg="#555555"
        )
        self._qnh_ref_lbl.pack(side="left")

        # ── No-signal indicator ──────────────────────────────────────────
        self._nosig_lbl = tk.Label(
            self,
            text="BARO: NO SIGNAL",
            font=("Consolas", 11, "bold"),
            fg="#FF4444", bg=self.CLR_BG
        )
        # Hidden by default; shown when baro_valid = False
        self._nosig_lbl.grid(row=3, column=0, columnspan=4, pady=(6, 0))
        self._nosig_lbl.grid_remove()

    # ── QNH zeroing ───────────────────────────────────────────────────────────

    def _set_qnh(self):
        """Record current raw altitude as the QNH reference (pilot zeroing)."""
        self._qnh_ref_cm = self._last_alt_cm
        self._update_qnh_label()

    def _update_qnh_label(self):
        ref_m = self._qnh_ref_cm * self.CM_TO_M
        raw_m = self._last_alt_cm * self.CM_TO_M
        self._qnh_ref_lbl.config(
            text=f"QNH ref: {ref_m:+.1f} m  |  Raw alt (AGL): {raw_m:+.1f} m"
        )

    # ── Main update ───────────────────────────────────────────────────────────

    def update_baro(self, data: dict):
        """
        Called from main.py update loop with the ui_data dict.

        Expected keys:
            baro_altitude_cm      int    FC-fused altitude, cm above home
            baro_vario_cm_per_sec int    Vertical speed, cm/s
            baro_valid            bool   True if MSP_ALTITUDE frame was valid

        All unit conversion is done here so the C++ side stays clean.
        """
        valid = data.get("baro_valid", False)

        if not valid:
            self._show_no_signal()
            return

        self._nosig_lbl.grid_remove()

        alt_cm   = int(data.get("baro_altitude_cm",      0))
        vario_cm = int(data.get("baro_vario_cm_per_sec", 0))

        self._last_alt_cm = alt_cm
        self._update_qnh_label()

        # Unit conversions
        alt_m   = alt_cm   * self.CM_TO_M
        alt_ft  = alt_cm   * self.CM_TO_FT
        vario_m = vario_cm * self.CM_TO_M
        vario_f = vario_m  * self.MPS_TO_FPM

        qnh_cm  = alt_cm - self._qnh_ref_cm
        qnh_m   = qnh_cm * self.CM_TO_M
        qnh_ft  = qnh_cm * self.CM_TO_FT

        # ── Colour logic ──────────────────────────────────────────────────────
        alt_clr   = self._alt_color(abs(alt_m))
        qnh_clr   = self._alt_color(abs(qnh_m))
        vario_clr = self._vario_color(abs(vario_m))

        # ── Update cells ──────────────────────────────────────────────────────
        self._update_row("alt_m",
                         f"{alt_m:>+8.1f} m", alt_clr,
                         f"{alt_ft:>+9.0f} ft", alt_clr,
                         self._alt_status(abs(alt_m)))

        self._update_row("alt_qnh",
                         f"{qnh_m:>+8.1f} m", qnh_clr,
                         f"{qnh_ft:>+9.0f} ft", qnh_clr,
                         self._alt_status(abs(qnh_m)))

        self._update_row("vario",
                         f"{vario_m:>+7.2f} m/s", vario_clr,
                         f"{vario_f:>+8.0f} fpm", vario_clr,
                         self._vario_status(vario_m))

    # ── Colour and status helpers ─────────────────────────────────────────────

    def _alt_color(self, abs_m: float) -> str:
        if   abs_m >= self.CRIT_ALT_M: return self.CLR_CRITICAL
        elif abs_m >= self.WARN_ALT_M: return self.CLR_WARN
        else:                          return self.CLR_SAFE

    def _vario_color(self, abs_mps: float) -> str:
        if   abs_mps >= self.CRIT_VARIO: return self.CLR_CRITICAL
        elif abs_mps >= self.WARN_VARIO:  return self.CLR_WARN
        else:                             return self.CLR_SAFE

    def _alt_status(self, abs_m: float) -> str:
        if   abs_m >= self.CRIT_ALT_M: return "CRITICAL"
        elif abs_m >= self.WARN_ALT_M: return "CAUTION"
        else:                          return "NORMAL"

    def _vario_status(self, mps: float) -> str:
        abs_mps = abs(mps)
        if   abs_mps >= self.CRIT_VARIO: return "CRITICAL"
        elif abs_mps >= self.WARN_VARIO:  return "CAUTION"
        else:
            if   mps >  0.1: return "CLIMBING"
            elif mps < -0.1: return "DESCENDING"
            else:            return "LEVEL"

    def _update_row(self, key: str,
                    txt_m: str, clr_m: str,
                    txt_i: str, clr_i: str,
                    status: str):
        c = self._cells[key]
        c["val_m"].config( text=txt_m, bg=clr_m)
        c["val_i"].config( text=txt_i, bg=clr_i)
        c["status"].config(text=status,
                           fg="#CC0000" if "CRITICAL" in status else
                              "#888800" if "CAUTION"  in status else
                              "#005500")

    def _show_no_signal(self):
        for key in self._cells:
            self._update_row(key,
                             "--- m",   self.CLR_OFF,
                             "--- ft",  self.CLR_OFF,
                             "NO SIG")
        self._nosig_lbl.grid()