"""
theme.py -- one visual theme for the whole cockpit (pilot + copilot windows)

Every color literal in the UI goes through C("#rrggbb"). With the
"classic" theme C() returns the color unchanged (the original look); with
the "military" theme (default) it maps the color onto a military glass-
cockpit palette by its ROLE, so all widgets change together and keep
their meaning:

  role (aviation convention)        military palette
  --------------------------------  -----------------------------------
  WARNING  - act now                 red      (red hues)
  CAUTION  - be aware                amber    (orange / yellow hues)
  NORMAL   - ok / engaged / safe     green    (green hues)
  ADVISORY - selected / info / cyan  cyan     (cyan / bright blue hues)
  TARGET   - target / nav cue        magenta  (purple / pink hues)
  DATA     - measured values         near-white (light neutrals)
  LABEL    - legends, units, scales  green-grey (mid neutrals, grey-blue)
  BEZEL    - backgrounds, frames     black-olive scale (dark neutrals and
                                     the old dark-navy chrome)

Brightness is kept: a dimmed green stays a dimmed green, a background
stays darker than the frame around it. Red/amber/green are reserved for
state (AC 25-11B / MIL-STD-1472 color coding); decoration never uses
them.

Selection: config/ui_theme.json {"theme": "military" | "classic"} or the
environment variable RAD_UI_THEME. Read once at import: widget modules
build their colors at import time, so changing the theme needs a restart.

Fonts: F(family) maps the old UI fonts. Military: labels and titles in
"Bahnschrift" (DIN-style, ships with Windows 10/11; Tk falls back to the
default font where it is missing), numbers stay in "Consolas" (fixed
width, digits do not jump while they change).
"""
from __future__ import annotations

import colorsys
import json
import os
from functools import lru_cache

_APP_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONFIG_PATH = os.path.join(_APP_DIR, "config", "ui_theme.json")
THEMES = ("military", "classic")


def _load_name() -> str:
    name = os.environ.get("RAD_UI_THEME", "").strip().lower()
    if not name:
        try:
            with open(CONFIG_PATH, encoding="utf-8") as f:
                name = str(json.load(f).get("theme", "")).strip().lower()
        except (OSError, ValueError, AttributeError):
            name = ""
    return name if name in THEMES else "military"


NAME = _load_name()
MILITARY = NAME == "military"


def save_choice(name: str) -> None:
    """Persist the theme for the next start (used by the toolbar menu)."""
    if name not in THEMES:
        return
    try:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump({"theme": name}, f)
    except OSError:
        pass


# ── Semantic palette (use these in new code) ────────────────────────────────
class P:
    if MILITARY:
        BG = "#080b08"            # workspace
        PANEL = "#050705"         # instrument face
        BEZEL = "#151b15"         # panel frame / title bar
        BEZEL_HI = "#2c372c"      # frame line
        BEZEL_LO = "#0d110d"
        GRID = "#0f150f"
        DATA = "#e6eee0"
        LABEL = "#8a9c84"
        DIM = "#4a5848"
        NORMAL = "#3ce870"
        ADVISORY = "#28d8f0"
        CAUTION = "#ffb000"
        WARNING = "#ff3030"
        TARGET = "#ff4ef0"
        WARNING_BG = "#5a0a0a"
        CAUTION_BG = "#4a3000"
        BUTTON = "#1a221a"
        BUTTON_HI = "#253025"
        SELECT = "#1d3a2a"
    else:
        BG = "#1a1a2e"
        PANEL = "#0f0f1a"
        BEZEL = "#16213e"
        BEZEL_HI = "#2a3a5a"
        BEZEL_LO = "#0d0d1a"
        GRID = "#1e1e34"
        DATA = "#e6e6e6"
        LABEL = "#7a8599"
        DIM = "#445566"
        NORMAL = "#00ff88"
        ADVISORY = "#00d4ff"
        CAUTION = "#ffaa00"
        WARNING = "#ff3355"
        TARGET = "#cc88ff"
        WARNING_BG = "#b00020"
        CAUTION_BG = "#c86400"
        BUTTON = "#162040"
        BUTTON_HI = "#1c2c54"
        SELECT = "#1e3060"


# Colors that must never change (transparency keys, chroma keys).
_KEEP = {"#010203"}

# Explicit mappings where the role mapper would lose the meaning:
# attitude-indicator sky / ground stay blue / brown (every pilot reads the
# horizon by them), only toned down for a dark cockpit.
_OVERRIDE = {
    "#1565c0": "#22548a",     # ADI sky
    "#7b3f00": "#6b4220",     # ADI ground
    "#0a2a0a": "#0f2a12",     # ADI sky, warning tint
    "#4a3000": "#4a3008",     # ADI ground, warning tint
}

# ── Role colors for the mapper (HSV of the military palette) ───────────────
def _hsv(hex_: str):
    r, g, b = (int(hex_[i:i + 2], 16) / 255.0 for i in (1, 3, 5))
    return colorsys.rgb_to_hsv(r, g, b)


def _hex(h: float, s: float, v: float) -> str:
    r, g, b = colorsys.hsv_to_rgb(h % 1.0, max(0.0, min(1.0, s)), max(0.0, min(1.0, v)))
    return "#%02x%02x%02x" % (round(r * 255), round(g * 255), round(b * 255))


_ROLE = {
    "warning": _hsv("#ff3030"),
    "caution": _hsv("#ffb000"),
    "normal": _hsv("#3ce870"),
    "advisory": _hsv("#28d8f0"),
    "target": _hsv("#ff4ef0"),
}
_NEUTRAL_HUE = 100.0 / 360.0          # olive


def _neutral(v: float, s_in: float) -> str:
    """Dark-to-light neutral on the olive axis. Very light colors become the
    DATA near-white; mid tones the LABEL green-grey; darks the bezel scale."""
    if v >= 0.86:
        return _hex(_NEUTRAL_HUE, 0.05, 0.93)
    if v <= 0.45:
        # backgrounds: compress toward black, keep their order
        vv = 0.02 + v * 0.62
        return _hex(_NEUTRAL_HUE, 0.28, vv)
    return _hex(_NEUTRAL_HUE, 0.12 + 0.06 * min(1.0, s_in * 2), 0.30 + (v - 0.45) * 0.95)


def _role_of(h: float, s: float, v: float):
    deg = h * 360.0
    if s < 0.20 or v < 0.20:
        return None
    # dark / greyish blue chrome (old navy panels, blue-grey labels) = neutral
    if 185 <= deg <= 265 and (v < 0.50 or s < 0.45):
        return None
    if deg < 18 or deg >= 330:
        return "warning"
    if deg < 66:
        return "caution"
    if deg < 165:
        return "normal"
    if deg < 255:
        return "advisory"
    return "target"


@lru_cache(maxsize=None)
def _military(hex_: str) -> str:
    h, s, v = _hsv(hex_)
    role = _role_of(h, s, v)
    if role is None:
        return _neutral(v, s)
    rh, rs, rv = _ROLE[role]
    # keep the source's brightness relative to full, and how saturated it was
    out_v = rv * min(1.0, v / 0.9)
    out_s = rs * min(1.0, 0.35 + s * 0.75)
    if v < 0.45:                        # dark tinted backgrounds (e.g. dark red alarm bg)
        out_v = v * 0.9
    return _hex(rh, out_s, out_v)


def C(color: str) -> str:
    """Theme a color literal ("#rrggbb"). Anything else is returned as is."""
    if not MILITARY or not isinstance(color, str) or len(color) != 7 or color[0] != "#":
        return color
    c = color.lower()
    if c in _KEEP:
        return color
    if c in _OVERRIDE:
        return _OVERRIDE[c]
    try:
        return _military(c)
    except ValueError:
        return color


_FONT_MAP = {"Segoe UI": "Bahnschrift", "Segoe UI Semibold": "Bahnschrift SemiBold",
             "Arial": "Bahnschrift", "Helvetica": "Bahnschrift", "Verdana": "Bahnschrift"}


def F(family: str) -> str:
    return _FONT_MAP.get(family, family) if MILITARY else family


# Fonts for new code (title bars, annunciators, toolbar)
FONT_TITLE = (F("Segoe UI"), 9, "bold")
FONT_UI = (F("Segoe UI"), 10)
FONT_UI_B = (F("Segoe UI"), 10, "bold")
FONT_DATA = ("Consolas", 10, "bold")
FONT_ANN = (F("Segoe UI"), 10, "bold")


def title(text: str) -> str:
    """Panel/legend text: military look uses upper case."""
    return text.upper() if MILITARY else text


def apply_ttk(root) -> None:
    """Style ttk widgets (Treeview, Combobox, Spinbox, PanedWindow,
    Scrollbar, Notebook, Checkbutton, Radiobutton) for the theme."""
    try:
        from tkinter import ttk
        st = ttk.Style(root)
        if "clam" in st.theme_names():
            st.theme_use("clam")
    except Exception:
        return
    if not MILITARY:
        return
    p = P
    base = dict(background=p.PANEL, foreground=p.DATA, fieldbackground=p.PANEL,
                bordercolor=p.BEZEL_HI, lightcolor=p.BEZEL, darkcolor=p.BEZEL_LO,
                troughcolor=p.BEZEL_LO, arrowcolor=p.LABEL, insertcolor=p.DATA,
                selectbackground=p.SELECT, selectforeground=p.DATA)
    try:
        st.configure(".", font=FONT_UI, **base)
        st.configure("Treeview", rowheight=22, **base)
        st.configure("Treeview.Heading", background=p.BEZEL, foreground=p.LABEL,
                     font=FONT_TITLE, relief="flat", bordercolor=p.BEZEL_HI)
        st.map("Treeview", background=[("selected", p.SELECT)],
               foreground=[("selected", p.NORMAL)])
        st.map("Treeview.Heading", background=[("active", p.BUTTON_HI)])
        st.configure("TPanedwindow", background=p.BEZEL)
        st.configure("Sash", sashthickness=6, gripcount=0, background=p.BEZEL_HI)
        for w in ("TCombobox", "TSpinbox", "TEntry"):
            st.configure(w, **base)
            st.map(w, fieldbackground=[("readonly", p.PANEL)],
                   foreground=[("readonly", p.DATA)],
                   selectbackground=[("readonly", p.PANEL)],
                   selectforeground=[("readonly", p.DATA)])
        for sb in ("Vertical.TScrollbar", "Horizontal.TScrollbar"):
            st.configure(sb, background=p.BUTTON_HI, troughcolor=p.BEZEL_LO,
                         bordercolor=p.BEZEL_LO, lightcolor=p.BUTTON_HI,
                         darkcolor=p.BUTTON_HI, arrowcolor=p.LABEL, gripcount=0)
            st.map(sb, background=[("active", p.BEZEL_HI)])
        st.configure("TNotebook", background=p.BG, bordercolor=p.BEZEL_HI)
        st.configure("TNotebook.Tab", background=p.BEZEL, foreground=p.LABEL,
                     padding=(10, 3), font=FONT_TITLE)
        st.map("TNotebook.Tab", background=[("selected", p.BUTTON_HI)],
               foreground=[("selected", p.NORMAL)])
        for w in ("TCheckbutton", "TRadiobutton", "TLabel", "TFrame", "TLabelframe",
                  "TLabelframe.Label", "TButton"):
            st.configure(w, background=p.PANEL, foreground=p.DATA)
        st.map("TCheckbutton", indicatorcolor=[("selected", p.NORMAL), ("!selected", p.BEZEL_LO)])
        st.map("TRadiobutton", indicatorcolor=[("selected", p.NORMAL), ("!selected", p.BEZEL_LO)])
        st.configure("TButton", background=p.BUTTON, bordercolor=p.BEZEL_HI)
        st.map("TButton", background=[("active", p.BUTTON_HI)])
        # dropdown list of comboboxes is a plain Tk listbox
        root.option_add("*TCombobox*Listbox.background", p.PANEL)
        root.option_add("*TCombobox*Listbox.foreground", p.DATA)
        root.option_add("*TCombobox*Listbox.selectBackground", p.SELECT)
        root.option_add("*TCombobox*Listbox.selectForeground", p.NORMAL)
    except Exception:
        pass


def apply_tk_defaults(root) -> None:
    """Defaults for plain Tk widgets created without explicit colors
    (dialogs, menus, message boxes on Linux)."""
    if not MILITARY:
        return
    p = P
    opts = [
        ("*Menu.background", p.BEZEL), ("*Menu.foreground", p.DATA),
        ("*Menu.activeBackground", p.SELECT), ("*Menu.activeForeground", p.NORMAL),
        ("*Menu.relief", "flat"), ("*Menu.font", FONT_UI),
        # widgets created without explicit colors (dialogs, small buttons)
        ("*Button.background", p.BUTTON), ("*Button.foreground", p.DATA),
        ("*Button.activeBackground", p.BUTTON_HI), ("*Button.activeForeground", p.NORMAL),
        ("*Button.highlightBackground", p.BEZEL_HI), ("*Button.relief", "flat"),
        ("*Button.borderWidth", 1),
        ("*Label.background", p.PANEL), ("*Label.foreground", p.DATA),
        ("*Frame.background", p.PANEL), ("*Toplevel.background", p.BG),
        ("*Checkbutton.background", p.PANEL), ("*Checkbutton.foreground", p.DATA),
        ("*Checkbutton.selectColor", p.BEZEL_LO), ("*Checkbutton.activeBackground", p.BUTTON_HI),
        ("*Radiobutton.background", p.PANEL), ("*Radiobutton.foreground", p.DATA),
        ("*Radiobutton.selectColor", p.BEZEL_LO),
        ("*Entry.background", p.BEZEL_LO), ("*Entry.foreground", p.DATA),
        ("*Entry.insertBackground", p.DATA),
        ("*Spinbox.background", p.BEZEL_LO), ("*Spinbox.foreground", p.DATA),
        ("*Spinbox.buttonBackground", p.BUTTON),
        ("*Listbox.background", p.BEZEL_LO), ("*Listbox.foreground", p.DATA),
        ("*Listbox.selectBackground", p.SELECT), ("*Listbox.selectForeground", p.NORMAL),
        ("*Text.background", p.BEZEL_LO), ("*Text.foreground", p.DATA),
        ("*Scale.background", p.PANEL), ("*Scale.troughColor", p.BEZEL_LO),
    ]
    for k, v in opts:
        root.option_add(k, v)
