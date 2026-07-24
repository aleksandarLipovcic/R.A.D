import tkinter as tk
import tkinter.font as tkfont
import math


class MagWidget(tk.Frame):
    """
    QMC5883L Magnetometer display widget — aviation-standard, fully responsive.

    AVIATION SAFETY INVARIANTS (never broken regardless of window size):
      ① Heading readout (degrees) is always fully visible, never clipped.
      ② Lock/signal status indicator is always visible.
      ③ Both calibration buttons are always fully accessible/clickable.
      The widget enforces a hard minimum size so all three are always present
      and none of them ever overlaps or crops another.

    Layout tiers (progressive enhancement, chosen from available w/h):
      FULL     Rose + heading/status/buttons column + raw XYZ bars.
               (plenty of both width and height)
      COMPACT  Rose + heading/status/buttons column, no bars.
               (decent width and height, not enough room for bars)
      SIDE     No rose. Heading box on the LEFT, buttons stacked
               vertically on the RIGHT. Used when the panel is wide but
               short -- putting the heading box beside the buttons
               (instead of on top of them) means neither has to fight
               the other for vertical space.
      MINI     No rose. Heading box on top, both buttons in a row below.
               The fallback for small panels -- this is the tier the hard
               minimum size (_MIN_W/_MIN_H) is sized to guarantee fits.

    Heading box sizing (this is the fix for the box overflowing/cropping):
      In every tier, the heading box's row/column is given weight=1 while
      the status/hint/button rows are given weight=0 with an explicit
      minsize. That means the heading box only ever gets whatever space is
      LEFT OVER after the status text and both buttons have already
      claimed their guaranteed minimum space -- so a bigger heading font
      can never push the buttons out of view or off screen.
      grid_propagate(False) is set on the heading box itself so it never
      grows past the size its row/column assigns it, no matter how large
      a font is requested for it -- previously the box sized itself to
      fit its own label, so a large font could make it demand more room
      than the panel actually had, and the excess simply got clipped by
      whichever ancestor's size actually was fixed (this is what caused
      the cropped "188.3° S" readout).

    Adaptive heading font:
      Binary-searches for the largest font size (Consolas Bold) where the
      worst-case string "270.0° NW" fits inside the heading box's ACTUAL
      assigned width *and* height (both are measured now -- previously
      only width was checked, which is what let the text overflow
      vertically even when it technically fit horizontally).
    """

    # ── Cockpit colour palette ───────────────────────────────────────────────
    C_BG         = "#0a0a10"
    C_ROSE_BG    = "#0d1117"
    C_ROSE_RING  = "#1e3040"
    C_LUBBER     = "#FFD700"
    C_CARDINAL   = "#c8d8e8"
    C_CARDINAL_N = "#FF4444"
    C_TICK_MINOR = "#1e2e3e"
    C_SAFE       = "#00d4ff"
    C_GREEN      = "#00ff88"
    C_WARN       = "#FFD700"
    C_CRIT       = "#FF4444"
    C_OFF        = "#444455"
    C_BAR_BG     = "#111118"
    C_TEXT       = "#a0b8c8"
    C_LABEL      = "#445566"
    C_NOSIG_FG   = "#FF4444"
    C_HINT       = "#664400"
    C_SEP        = "#1e2a3a"
    C_HDG_BG     = "#080f18"
    C_HDG_BORDER = "#1e4a6a"
    C_BTN_MAG_BG = "#082210"
    C_BTN_MAG_FG = "#00ff88"
    C_BTN_ACC_BG = "#080a20"
    C_BTN_ACC_FG = "#44aaff"

    MAG_SCALE_MIN = 800.0
    _mag_scale    = 800.0

    PAD = 6   # uniform outer padding (px)

    # ── Heading font scaling ─────────────────────────────────────────────────
    # Worst-case string: "270.0° NW" — font is measured directly via
    # font.measure()/font.metrics() against the box's ACTUAL assigned size,
    # so these are just the search bounds, not the fit itself.
    _HDG_MIN_PT = 9
    _HDG_MAX_PT = 48
    _HDG_PAD_X  = 22   # total horizontal padding+border inside the heading box (px)
    _HDG_PAD_Y  = 18   # total vertical padding+border inside the heading box (px)

    # ── Rose side-gutter heading (the fix for wasted space beside the compass) ──
    # The rose is circular and height-bound, so a wide-but-not-tall window
    # leaves blank canvas to either side of it that was previously unused.
    # When that gutter is wide enough to be worth using, we draw a big
    # heading readout directly in it, right next to the compass, and hide
    # the smaller duplicate box in the side panel so there's one prominent
    # number, not two. Below this threshold everything reverts exactly to
    # the original layout (small box in the side panel, nothing beside the
    # rose) -- so narrower/shorter windows are completely unaffected.
    _GUTTER_MIN_PX   = 130
    _SIDE_HDG_MAX_PT = 64
    # Fixed visual gap between the rose's circle and the side heading box,
    # and between the compass and its "combined group" and the canvas edges.
    _SIDE_HDG_GAP_PX = 28

    # ── Aviation safety: guaranteed minimum space for the non-negotiable
    #    elements. The heading box only ever gets what's left over after
    #    these have taken their share, so it can never crowd them out.
    _STATUS_MINSIZE  = 16
    _HINT_MINSIZE    = 20
    _LABEL_MINSIZE   = 14
    _HDG_ROW_MINSIZE = 30
    _BTN_ROW_MINSIZE = 34   # buttons side-by-side (mini)
    _BTN_COL_MINSIZE = 34   # each button when stacked (side tier)

    # ── Aviation safety: hard minimum widget dimensions ──────────────────────
    # Sized so the MINI tier -- the most cramped layout -- always has room
    # for the heading box, the status indicator, and both buttons without
    # any of them clipping.
    _MIN_W = 285
    _MIN_H = 145

    # ── Tier breakpoints ──────────────────────────────────────────────────────
    # COMPACT's floor is set to the same size that used to be MINI's floor
    # in the old two-tier version, so the rose keeps showing in every case
    # it used to -- SIDE only takes over for windows shorter than that,
    # which previously had no rose either (they were forced into MINI).
    # SIDE is a strict improvement over the old behaviour, never a regression.
    _FULL_W, _FULL_H       = 455, 225
    _COMPACT_W, _COMPACT_H = 285, 145
    _SIDE_MIN_W            = 340   # SIDE only kicks in when short on height
                                    # but there's enough width to go sideways

    # All 8 cardinal & intercardinal points
    CARDINALS = {
        0:   "N",  45: "NE",  90:  "E",  135: "SE",
        180: "S", 225: "SW", 270:  "W",  315: "NW",
    }

    def __init__(self, parent, on_mag_calibrate=None, on_acc_calibrate=None):
        super().__init__(parent, bg=self.C_BG)
        self._last_heading  = 0.0
        self._valid         = False
        self._on_mag_cal    = on_mag_calibrate
        self._on_acc_cal    = on_acc_calibrate
        self._current_tier  = None
        self._tier_widgets  = {}
        self._hdg_font_size = 0    # loop guard
        self._side_heading_active = False   # is the rose currently showing
                                             # its own big side heading?

        # NOTE: previously this was `self.minsize = (self._MIN_W, self._MIN_H)`,
        # which does nothing — tk.Frame has no .minsize attribute/method (that
        # belongs to Tk/Toplevel), so it silently failed to protect anything.
        # The actual fix: tell Tk this frame's own natural (requested) size is
        # _MIN_W x _MIN_H, and freeze it there with grid_propagate(False) so
        # nothing internal can shrink that request. A widget's requested size
        # is what a parent grid/pack layout treats as its floor — so whatever
        # container this is placed in (sticky="nsew", weight>0) can still grow
        # it larger, but can never give it less than this floor.
        self.configure(width=self._MIN_W, height=self._MIN_H)
        self.grid_propagate(False)

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

        self._container = tk.Frame(self, bg=self.C_BG)
        self._container.grid(row=0, column=0, sticky="nsew",
                             padx=self.PAD, pady=self.PAD)

        self.bind("<Configure>", self._on_resize)

    # ══════════════════════════════════════════════════════════════════════════
    # Resize / tier management
    # ══════════════════════════════════════════════════════════════════════════

    def _on_resize(self, event=None):
        w    = self.winfo_width()
        h    = self.winfo_height()
        tier = self._classify(w, h)
        if tier != self._current_tier:
            self._build_tier(tier)
            self._current_tier = tier
        self._redraw_dynamic()

    def _classify(self, w, h):
        if w >= self._FULL_W and h >= self._FULL_H:
            return "full"
        if w >= self._COMPACT_W and h >= self._COMPACT_H:
            return "compact"
        if w >= self._SIDE_MIN_W and h < self._COMPACT_H:
            return "side"
        return "mini"

    # ══════════════════════════════════════════════════════════════════════════
    # Tier builders
    # ══════════════════════════════════════════════════════════════════════════

    def _clear_container(self):
        for widget in self._container.winfo_children():
            widget.destroy()
        self._tier_widgets  = {}
        self._hdg_font_size = 0
        self._side_heading_active = False
        for i in range(10):
            self._container.columnconfigure(i, weight=0, minsize=0)
            self._container.rowconfigure(i,    weight=0, minsize=0)

    def _build_tier(self, tier):
        self._clear_container()
        {"full": self._build_full,
         "compact": self._build_compact,
         "side": self._build_side,
         "mini": self._build_mini}[tier]()

    # ── shared: a self-contained heading box that never grows past the
    #    cell its parent's layout assigns it (see class docstring) ───────────
    def _make_hdg_box(self, parent, font_size):
        box = tk.Frame(parent, bg=self.C_HDG_BG,
                       highlightbackground=self.C_HDG_BORDER,
                       highlightthickness=2)
        box.grid_propagate(False)
        box.columnconfigure(0, weight=1)
        box.rowconfigure(0, weight=1)

        val = tk.Label(box, text="---.- ---",
                       font=("Consolas", font_size, "bold"),
                       fg=self.C_CRIT, bg=self.C_HDG_BG, anchor="center")
        val.grid(row=0, column=0, sticky="nsew", padx=8, pady=6)

        box.bind("<Configure>",
                lambda e: self._resize_hdg_font(e.width, e.height))

        self._tier_widgets["hdg_val"] = val
        self._tier_widgets["hdg_box"] = box
        return box

    # ── FULL ─────────────────────────────────────────────────────────────────
    def _build_full(self):
        c = self._container
        c.columnconfigure(0, weight=2)
        c.columnconfigure(1, weight=2, minsize=230)
        c.rowconfigure(0, weight=1)
        c.rowconfigure(1, weight=0, minsize=54)

        rose_cv = tk.Canvas(c, bg=self.C_BG, highlightthickness=0)
        rose_cv.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=(0, 4))
        rose_cv.bind("<Configure>", lambda e: self._draw_rose())
        self._tier_widgets["rose"] = rose_cv

        right = tk.Frame(c, bg=self.C_BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=(0, 4))
        self._build_right_panel(right, font_card=10, font_status=9,
                                font_btn=9, tier="full")

        bars_frame = tk.Frame(c, bg=self.C_BG)
        bars_frame.grid(row=1, column=0, columnspan=2,
                        sticky="ew", pady=(4, 0))
        bars_frame.columnconfigure(1, weight=1)
        tk.Label(bars_frame, text="RAW FIELD COMPONENTS",
                 font=("Consolas", 7, "bold"),
                 fg=self.C_LABEL, bg=self.C_BG,
                 ).grid(row=0, column=0, columnspan=3, sticky="w", pady=(0, 2))
        self._build_bars_into(bars_frame, start_row=1, bar_height=13)

    # ── COMPACT ───────────────────────────────────────────────────────────────
    def _build_compact(self):
        c = self._container
        c.columnconfigure(0, weight=3)
        c.columnconfigure(1, weight=2, minsize=155)
        c.rowconfigure(0, weight=1)

        rose_cv = tk.Canvas(c, bg=self.C_BG, highlightthickness=0)
        rose_cv.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        rose_cv.bind("<Configure>", lambda e: self._draw_rose())
        self._tier_widgets["rose"] = rose_cv

        right = tk.Frame(c, bg=self.C_BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        self._build_right_panel(right, font_card=9, font_status=8,
                                font_btn=8, tier="compact")

    # ── SIDE (wide but short: heading box beside the buttons) ─────────────────
    def _build_side(self):
        """
        Used when there's enough width to put the heading box and the
        buttons next to each other, but not enough height to comfortably
        stack the heading on top of them (see FULL/COMPACT). Putting them
        side by side means the heading box can use the full available
        height for its font without pushing the buttons off screen, and
        the buttons get a guaranteed minimum height regardless of how
        short the panel gets (down to the hard minimum).
        """
        c = self._container
        c.columnconfigure(0, weight=3, minsize=140)
        c.columnconfigure(1, weight=2, minsize=110)
        c.rowconfigure(0, weight=1)

        left = tk.Frame(c, bg=self.C_BG)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 4))
        left.columnconfigure(0, weight=1)
        left.rowconfigure(0, weight=1, minsize=self._HDG_ROW_MINSIZE)  # heading — flexible
        left.rowconfigure(1, weight=0, minsize=self._STATUS_MINSIZE)

        hdg_box = self._make_hdg_box(left, font_size=16)
        hdg_box.grid(row=0, column=0, sticky="nsew")

        status = tk.Label(left, text="⬤  NO SIGNAL",
                          font=("Consolas", 8, "bold"),
                          fg=self.C_NOSIG_FG, bg=self.C_BG, anchor="center")
        status.grid(row=1, column=0, pady=(2, 0))
        self._tier_widgets["status"] = status

        right = tk.Frame(c, bg=self.C_BG)
        right.grid(row=0, column=1, sticky="nsew", padx=(4, 0))
        right.columnconfigure(0, weight=1)
        right.rowconfigure(0, weight=1, minsize=self._BTN_COL_MINSIZE)
        right.rowconfigure(1, weight=1, minsize=self._BTN_COL_MINSIZE)

        mag_btn = tk.Button(
            right, text="⊕ CAL MAG",
            font=("Consolas", 9, "bold"),
            fg=self.C_BTN_MAG_FG, bg=self.C_BTN_MAG_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_MAG_FG,
            relief="flat", bd=0, cursor="hand2",
            command=self._on_mag_cal_pressed,
            state="normal" if self._on_mag_cal else "disabled")
        mag_btn.grid(row=0, column=0, sticky="nsew", pady=(0, 2))
        self._tier_widgets["mag_btn"] = mag_btn

        acc_btn = tk.Button(
            right, text="⊕ CAL GYRO",
            font=("Consolas", 9, "bold"),
            fg=self.C_BTN_ACC_FG, bg=self.C_BTN_ACC_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_ACC_FG,
            relief="flat", bd=0, cursor="hand2",
            command=self._on_acc_cal_pressed,
            state="normal" if self._on_acc_cal else "disabled")
        acc_btn.grid(row=1, column=0, sticky="nsew", pady=(2, 0))
        self._tier_widgets["acc_btn"] = acc_btn

    # ── MINI ──────────────────────────────────────────────────────────────────
    def _build_mini(self):
        c = self._container
        c.columnconfigure(0, weight=1)
        c.columnconfigure(1, weight=1)
        c.rowconfigure(0, weight=1, minsize=self._HDG_ROW_MINSIZE)   # heading — flexible
        c.rowconfigure(1, weight=0, minsize=self._STATUS_MINSIZE)
        c.rowconfigure(2, weight=0, minsize=self._BTN_ROW_MINSIZE)

        hdg_box = self._make_hdg_box(c, font_size=16)
        hdg_box.grid(row=0, column=0, columnspan=2, sticky="nsew", pady=(0, 3))

        status = tk.Label(c, text="⬤  NO SIGNAL",
                          font=("Consolas", 8, "bold"),
                          fg=self.C_NOSIG_FG, bg=self.C_BG, anchor="center")
        status.grid(row=1, column=0, columnspan=2, pady=(0, 3))
        self._tier_widgets["status"] = status

        mag_btn = tk.Button(
            c, text="⊕ CAL MAG",
            font=("Consolas", 9, "bold"),
            fg=self.C_BTN_MAG_FG, bg=self.C_BTN_MAG_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_MAG_FG,
            relief="flat", bd=0, padx=4, cursor="hand2",
            command=self._on_mag_cal_pressed,
            state="normal" if self._on_mag_cal else "disabled")
        mag_btn.grid(row=2, column=0, sticky="nsew", padx=(0, 2))
        self._tier_widgets["mag_btn"] = mag_btn

        acc_btn = tk.Button(
            c, text="⊕ CAL GYRO",
            font=("Consolas", 9, "bold"),
            fg=self.C_BTN_ACC_FG, bg=self.C_BTN_ACC_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_ACC_FG,
            relief="flat", bd=0, padx=4, cursor="hand2",
            command=self._on_acc_cal_pressed,
            state="normal" if self._on_acc_cal else "disabled")
        acc_btn.grid(row=2, column=1, sticky="nsew", padx=(2, 0))
        self._tier_widgets["acc_btn"] = acc_btn

    # ══════════════════════════════════════════════════════════════════════════
    # Shared right panel (FULL / COMPACT)
    # ══════════════════════════════════════════════════════════════════════════

    def _build_right_panel(self, parent, font_card, font_status, font_btn, tier):
        parent.columnconfigure(0, weight=1)
        # Both the heading area and the button area are weighted now (not
        # weight=0/content-driven), so when the panel is given more room
        # than its minimum, that extra room is actually used — growing the
        # heading font and enlarging the buttons into bigger, more prominent
        # tap targets — instead of sitting at minimum size with the leftover
        # space going unused (which is what made things look small/cramped
        # even in a large window).
        parent.rowconfigure(0, weight=3)   # top (label/heading/status/hint) — flexible
        parent.rowconfigure(1, weight=0)   # separator — fixed, thin
        parent.rowconfigure(2, weight=2)   # buttons — flexible, grows too

        top = tk.Frame(parent, bg=self.C_BG)
        top.grid(row=0, column=0, sticky="nsew")
        top.columnconfigure(0, weight=1)

        trow = 0
        if tier == "full":
            label_row = trow
            top.rowconfigure(label_row, weight=0, minsize=self._LABEL_MINSIZE)
            hdg_label = tk.Label(top, text="MAGNETIC HEADING",
                     font=("Consolas", 7, "bold"),
                     fg=self.C_LABEL, bg=self.C_BG,
                     )
            hdg_label.grid(row=label_row, column=0, pady=(2, 2))
            self._tier_widgets["hdg_label"]     = hdg_label
            self._tier_widgets["hdg_label_row"] = label_row
            trow += 1

        # The heading row is the ONLY flexible row in `top` -- it gets
        # whatever's left after the label/status/hint rows (all fixed,
        # weight=0) have taken their guaranteed minimum. That leftover
        # amount is what _resize_hdg_font() fits the font to, so growing
        # the font can never encroach on the other rows.
        hdg_row = trow
        top.rowconfigure(hdg_row, weight=1, minsize=self._HDG_ROW_MINSIZE)
        hdg_box = self._make_hdg_box(top, font_size=20)
        hdg_box.grid(row=hdg_row, column=0, sticky="nsew")
        # Keep refs so the rose can hide this box (and reclaim its row) when
        # there's enough gutter space beside the compass to show a bigger,
        # more prominent heading readout right next to it instead -- see
        # _apply_side_heading_state().
        self._tier_widgets["hdg_panel"]     = top
        self._tier_widgets["hdg_row"]       = hdg_row
        trow += 1

        # Re-measure whenever the box itself resizes AND whenever the
        # surrounding panel resizes (covers layout passes that change the
        # box's assigned size without the box itself firing Configure).
        parent.bind("<Configure>",
                    lambda e: self._trigger_hdg_resize(), add="+")

        status_row = trow
        top.rowconfigure(status_row, weight=0, minsize=self._STATUS_MINSIZE)
        status = tk.Label(top, text="⬤  NO SIGNAL",
                          font=("Consolas", font_status, "bold"),
                          fg=self.C_NOSIG_FG, bg=self.C_BG, anchor="center")
        status.grid(row=status_row, column=0, pady=(5, 2))
        self._tier_widgets["status"] = status
        trow += 1

        if tier == "full":
            hint_row = trow
            top.rowconfigure(hint_row, weight=0, minsize=self._HINT_MINSIZE)
            hint = tk.Label(top,
                            text="Set debug_mode = MAG_CALIB\nin Betaflight CLI",
                            font=("Consolas", 7),
                            fg=self.C_HINT, bg=self.C_BG, justify="center")
            hint.grid(row=hint_row, column=0, pady=(0, 2))
            self._tier_widgets["hint"] = hint

        tk.Frame(parent, bg=self.C_SEP, height=1).grid(
            row=1, column=0, sticky="ew", padx=2, pady=3)

        btn = tk.Frame(parent, bg=self.C_BG)
        btn.grid(row=2, column=0, sticky="nsew")
        parent.rowconfigure(2, weight=2, minsize=self._BTN_ROW_MINSIZE * 2)
        btn.columnconfigure(0, weight=1)

        brow = 0

        label_mag = "⊕  CALIBRATE MAG" if tier == "full" else "⊕  CAL MAG"
        mag_btn = tk.Button(
            btn, text=label_mag,
            font=("Consolas", font_btn, "bold"),
            fg=self.C_BTN_MAG_FG, bg=self.C_BTN_MAG_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_MAG_FG,
            relief="flat", bd=0, padx=8, pady=9,
            cursor="hand2",
            command=self._on_mag_cal_pressed,
            state="normal" if self._on_mag_cal else "disabled")
        btn.rowconfigure(brow, weight=1, minsize=self._BTN_ROW_MINSIZE)
        mag_btn.grid(row=brow, column=0, sticky="nsew", pady=(0, 2))
        self._tier_widgets["mag_btn"] = mag_btn
        brow += 1

        if tier == "full":
            mag_lbl = tk.Label(btn, text="", font=("Consolas", 7),
                               fg=self.C_WARN, bg=self.C_BG, justify="center")
            btn.rowconfigure(brow, weight=0)
            mag_lbl.grid(row=brow, column=0)
            self._tier_widgets["mag_lbl"] = mag_lbl
            brow += 1

        label_acc = "⊕  CALIBRATE GYRO/ACC" if tier == "full" else "⊕  CAL GYRO"
        acc_btn = tk.Button(
            btn, text=label_acc,
            font=("Consolas", font_btn, "bold"),
            fg=self.C_BTN_ACC_FG, bg=self.C_BTN_ACC_BG,
            activeforeground=self.C_BG, activebackground=self.C_BTN_ACC_FG,
            relief="flat", bd=0, padx=8, pady=10,
            cursor="hand2",
            command=self._on_acc_cal_pressed,
            state="normal" if self._on_acc_cal else "disabled")
        btn.rowconfigure(brow, weight=1, minsize=self._BTN_ROW_MINSIZE)
        acc_btn.grid(row=brow, column=0, sticky="nsew", pady=(0, 2))
        self._tier_widgets["acc_btn"] = acc_btn
        brow += 1

        if tier == "full":
            acc_lbl = tk.Label(btn, text="", font=("Consolas", 7),
                               fg=self.C_BTN_ACC_FG, bg=self.C_BG,
                               justify="center")
            btn.rowconfigure(brow, weight=0)
            acc_lbl.grid(row=brow, column=0)
            self._tier_widgets["acc_lbl"] = acc_lbl

    # ══════════════════════════════════════════════════════════════════════════
    # Adaptive heading font  ← FIXED (width AND height, bounded box)
    # ══════════════════════════════════════════════════════════════════════════

    def _trigger_hdg_resize(self):
        """Called when the surrounding panel resizes — re-measure using the
        heading box's own current assigned size."""
        hdg_box = self._tier_widgets.get("hdg_box")
        if hdg_box:
            w = hdg_box.winfo_width()
            h = hdg_box.winfo_height()
            if w > 1 and h > 1:
                self._resize_hdg_font(w, h)

    def _resize_hdg_font(self, box_width: int, box_height: int):
        """
        Binary-search for the largest font size (Consolas Bold) where the
        WORST-CASE heading string '270.0° NW' fits inside the heading box's
        *actual assigned* width AND height, minus padding. Because the box
        has grid_propagate(False) and only ever receives whatever space its
        row/column allocates it (see _make_hdg_box and the weight layout in
        each tier builder), that assigned size can never itself grow to
        chase the font -- so a fit found here is guaranteed to render
        without clipping, in either dimension.
        """
        hdg_val = self._tier_widgets.get("hdg_val")
        if not hdg_val:
            return

        available_w = box_width  - self._HDG_PAD_X
        available_h = box_height - self._HDG_PAD_Y
        if available_w < 20 or available_h < 10:
            return

        # Worst-case text: 3-digit degrees + two-char cardinal e.g. "270.0° NW"
        test_text = "270.0° NW"

        # Binary search between min and max for the largest fitting size,
        # checking BOTH dimensions -- previously only width was checked,
        # which let a font be chosen that fit horizontally but was taller
        # than the box, so it overflowed vertically and got clipped.
        lo, hi = self._HDG_MIN_PT, self._HDG_MAX_PT
        best   = lo

        while lo <= hi:
            mid  = (lo + hi) // 2
            font = tkfont.Font(family="Consolas", size=mid, weight="bold")
            text_w = font.measure(test_text)
            text_h = font.metrics("linespace")
            if text_w <= available_w and text_h <= available_h:
                best = mid
                lo   = mid + 1
            else:
                hi   = mid - 1

        if best == self._hdg_font_size:
            return
        self._hdg_font_size = best
        hdg_val.config(font=("Consolas", best, "bold"))

    # ── Bar builder helper ─────────────────────────────────────────────────
    def _build_bars_into(self, parent, start_row=0, bar_height=13):
        bars = {}
        for i, (axis, color) in enumerate(
                [("X", "#4488FF"), ("Y", "#FF8844"), ("Z", "#44FF88")]):
            r = start_row + i

            tk.Label(parent, text=f" {axis} ",
                     font=("Consolas", 9, "bold"),
                     fg=color, bg=self.C_BG, width=3,
                     ).grid(row=r, column=0, sticky="w")

            bar_cv = tk.Canvas(parent, height=bar_height,
                               bg=self.C_BG, highlightthickness=0)
            bar_cv.grid(row=r, column=1, sticky="ew", padx=(4, 8))

            val_lbl = tk.Label(parent, text="    0",
                               font=("Consolas", 9),
                               fg=self.C_TEXT, bg=self.C_BG,
                               width=6, anchor="e")
            val_lbl.grid(row=r, column=2, sticky="e", padx=(0, 4))

            bars[axis] = {"canvas": bar_cv, "label": val_lbl, "color": color}

        self._tier_widgets["bars"] = bars

    # ══════════════════════════════════════════════════════════════════════════
    # Dynamic redraw dispatcher
    # ══════════════════════════════════════════════════════════════════════════

    def _redraw_dynamic(self):
        self._draw_rose()
        self._update_readout_widgets(self._last_heading, self._valid)
        self._trigger_hdg_resize()

    # ══════════════════════════════════════════════════════════════════════════
    # Compass Rose
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_rose(self):
        cv = self._tier_widgets.get("rose")
        if cv is None:
            return

        cv.delete("all")
        W = cv.winfo_width()
        H = cv.winfo_height()
        if W < 20 or H < 20:
            return

        heading = self._last_heading
        valid   = self._valid
        cy = H // 2

        r_from_size   = min(W, H) // 2 - 5
        r_from_bottom = int((H / 2 - 5) / 0.92)
        r = max(15, min(r_from_size, r_from_bottom))

        f_cardinal      = max(8,  int(r * 0.17))
        f_intercardinal = max(6,  int(r * 0.12))
        f_numeral       = max(6,  int(r * 0.12))
        f_overlay       = max(8,  int(r * 0.15))
        f_nosig         = max(9,  int(r * 0.17))

        if not valid:
            cx = W // 2
            self._apply_side_heading_state(False)
            cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                           fill=self.C_ROSE_BG, outline=self.C_ROSE_RING, width=2)
            r2 = int(r * 0.88)
            cv.create_oval(cx - r2, cy - r2, cx + r2, cy + r2,
                           fill="", outline="#152535", width=1)
            cv.create_text(cx, cy - int(r * 0.12), text="NO SIG",
                           fill=self.C_NOSIG_FG,
                           font=("Consolas", f_nosig, "bold"))
            if r > 40:
                cv.create_text(cx, cy + int(r * 0.22), text="debug_mode?",
                               fill=self.C_HINT,
                               font=("Consolas", max(6, f_nosig - 3)))
            return

        # ── Decide, BEFORE drawing anything, whether the side-gutter
        # heading readout will be shown, and if so how wide it needs to
        # be -- so the compass and the heading box can be centered as a
        # single group (with a fixed gap between them) rather than the
        # compass sitting dead-center and the box being tacked onto
        # whatever space happened to be left on one side.
        gutter_estimate = int(W / 2 - r) - 14
        side_active = gutter_estimate >= self._GUTTER_MIN_PX

        box_w = box_h = best_font = 0
        if side_active:
            best_font, box_w, box_h = self._compute_side_heading_layout(
                gutter_estimate, r, H)
            if box_w <= 0:
                side_active = False

        if side_active:
            group_w = 2 * r + self._SIDE_HDG_GAP_PX + box_w
            cx = int((W - group_w) / 2) + r
        else:
            cx = W // 2

        self._apply_side_heading_state(side_active)

        cv.create_oval(cx - r, cy - r, cx + r, cy + r,
                       fill=self.C_ROSE_BG, outline=self.C_ROSE_RING, width=2)
        r2 = int(r * 0.88)
        cv.create_oval(cx - r2, cy - r2, cx + r2, cy + r2,
                       fill="", outline="#152535", width=1)

        tick_step = 5 if r > 70 else (10 if r > 40 else 30)
        for deg in range(0, 360, tick_step):
            is_major = (deg % 90 == 0)
            is_semi  = (deg % 45 == 0)
            is_10    = (deg % 10 == 0)

            if is_major:
                tick_len = max(5, int(r * 0.18))
                tick_w, tick_clr = 2, self.C_CARDINAL
            elif is_semi:
                tick_len = max(4, int(r * 0.13))
                tick_w, tick_clr = 1, "#3a6a8a"
            elif is_10:
                tick_len = max(3, int(r * 0.09))
                tick_w, tick_clr = 1, "#254555"
            else:
                tick_len = max(1, int(r * 0.05))
                tick_w, tick_clr = 1, self.C_TICK_MINOR

            rad = math.radians((deg - heading) - 90)
            ox  = cx + r              * math.cos(rad)
            oy  = cy + r              * math.sin(rad)
            ix  = cx + (r - tick_len) * math.cos(rad)
            iy  = cy + (r - tick_len) * math.sin(rad)
            cv.create_line(ix, iy, ox, oy, fill=tick_clr, width=tick_w)

        if r > 55:
            num_r = int(r * 0.70)
            for deg in range(30, 360, 30):
                if deg % 45 == 0:
                    continue
                rad = math.radians((deg - heading) - 90)
                lx  = cx + num_r * math.cos(rad)
                ly  = cy + num_r * math.sin(rad)
                cv.create_text(lx, ly, text=str(deg // 10),
                               fill="#2a5a7a",
                               font=("Consolas", f_numeral))

        label_r = int(r * 0.72)
        for hdg_fixed, letter in [(0, "N"), (90, "E"), (180, "S"), (270, "W")]:
            rad   = math.radians((hdg_fixed - heading) - 90)
            lx    = cx + label_r * math.cos(rad)
            ly    = cy + label_r * math.sin(rad)
            color = self.C_CARDINAL_N if letter == "N" else self.C_CARDINAL
            cv.create_text(lx, ly, text=letter, fill=color,
                           font=("Consolas", f_cardinal, "bold"))

        if r > 50:
            inter_r = int(r * 0.68)
            for hdg_fixed, label in [(45, "NE"), (135, "SE"),
                                     (225, "SW"), (315, "NW")]:
                rad = math.radians((hdg_fixed - heading) - 90)
                lx  = cx + inter_r * math.cos(rad)
                ly  = cy + inter_r * math.sin(rad)
                cv.create_text(lx, ly, text=label, fill="#3a7a9a",
                               font=("Consolas", f_intercardinal))

        tip_y  = cy - r + 2
        base_y = cy - r + max(7, int(r * 0.20))
        half_w = max(3, int(r * 0.07))
        cv.create_polygon(
            cx - half_w, base_y,
            cx + half_w, base_y,
            cx,          tip_y,
            fill=self.C_LUBBER, outline="#b09000", width=1)

        ptr_r = int(r * 0.55)
        arrow = (max(4, int(r * 0.10)),
                 max(5, int(r * 0.12)),
                 max(2, int(r * 0.04)))
        cv.create_line(cx, cy, cx, cy - ptr_r,
                       fill=self.C_LUBBER, width=2,
                       arrow=tk.LAST, arrowshape=arrow)

        dot = max(2, int(r * 0.05))
        cv.create_oval(cx - dot, cy - dot, cx + dot, cy + dot,
                       fill=self.C_LUBBER, outline=self.C_ROSE_BG, width=1)

        if side_active:
            x_c = cx + r + self._SIDE_HDG_GAP_PX + box_w / 2
            self._draw_side_heading_box(cv, x_c, cy, box_w, box_h,
                                        best_font, heading, valid=True)
        else:
            ov_y  = cy + int(r * 0.76)
            ov_hw = int(r * 0.46)
            ov_hh = max(9, int(r * 0.16))
            cv.create_rectangle(cx - ov_hw, ov_y - ov_hh,
                                 cx + ov_hw, ov_y + ov_hh,
                                 fill=self.C_HDG_BG,
                                 outline=self.C_HDG_BORDER, width=1)
            cv.create_text(cx, ov_y,
                           text=f"{heading % 360:05.1f}°",
                           fill=self.C_SAFE,
                           font=("Consolas", f_overlay, "bold"))

    # ══════════════════════════════════════════════════════════════════════════
    # Rose side-gutter heading — uses the blank space beside a height-bound
    # compass rose to show ONE big, prominent heading readout right next to
    # it, instead of a small duplicate tucked in the side panel.
    #
    # Sizing and drawing are split into two steps: _compute_side_heading_layout()
    # figures out how big the box/font need to be *before* the rose itself is
    # drawn, so the caller can center the compass+box pair as a single group
    # (with a fixed gap between them) rather than drawing the compass
    # dead-center first and bolting the box onto whatever space is left on
    # one side.
    # ══════════════════════════════════════════════════════════════════════════

    def _compute_side_heading_layout(self, gutter_px, r, H):
        """Binary-search the largest font (bounded by the worst-case string
        '270.0° NW') that fits in the estimated gutter, and return
        (font_size, box_w, box_h). Returns (0, 0, 0) if there isn't enough
        room to bother."""
        avail_w = gutter_px - 6
        avail_h = min(H - 16, max(40, int(r * 1.6)))
        if avail_w < 30 or avail_h < 20:
            return 0, 0, 0

        test_text = "270.0° NW"
        lo, hi = self._HDG_MIN_PT, self._SIDE_HDG_MAX_PT
        best = lo
        while lo <= hi:
            mid  = (lo + hi) // 2
            font = tkfont.Font(family="Consolas", size=mid, weight="bold")
            if font.measure(test_text) <= avail_w and \
               font.metrics("linespace") <= avail_h:
                best = mid
                lo = mid + 1
            else:
                hi = mid - 1

        box_w = avail_w + 16
        box_h = min(avail_h + 16, H - 6)
        return best, box_w, box_h

    def _draw_side_heading_box(self, cv, x_c, y_c, box_w, box_h, font_size,
                               heading, valid):
        """Draw the large digital heading readout box, pre-sized by
        _compute_side_heading_layout(), centered at (x_c, y_c)."""
        cv.create_rectangle(x_c - box_w / 2, y_c - box_h / 2,
                             x_c + box_w / 2, y_c + box_h / 2,
                             fill=self.C_HDG_BG,
                             outline=self.C_HDG_BORDER, width=2)

        if valid:
            hdg_norm = heading % 360.0
            cardinal = self._cardinal_for(hdg_norm)
            text = f"{hdg_norm:05.1f}° {cardinal}"
            color = self.C_SAFE
        else:
            text  = "---.- ---"
            color = self.C_CRIT

        cv.create_text(x_c, y_c, text=text, fill=color,
                       font=("Consolas", font_size, "bold"))

    def _apply_side_heading_state(self, active: bool):
        """Show/hide the side panel's own (smaller) heading box + label in
        favor of the rose's big side readout, and reclaim its row so status
        and the calibration buttons get to use the freed vertical space too.
        No-op in tiers that don't have a side panel (mini/side have no rose,
        so this is simply never triggered for them)."""
        if active == self._side_heading_active:
            return
        self._side_heading_active = active

        panel   = self._tier_widgets.get("hdg_panel")
        hdg_box = self._tier_widgets.get("hdg_box")
        hdg_row = self._tier_widgets.get("hdg_row")
        if panel is None or hdg_box is None or hdg_row is None:
            return

        label     = self._tier_widgets.get("hdg_label")
        label_row = self._tier_widgets.get("hdg_label_row")

        if active:
            hdg_box.grid_remove()
            panel.rowconfigure(hdg_row, weight=0, minsize=0)
            if label is not None:
                label.grid_remove()
                panel.rowconfigure(label_row, weight=0, minsize=0)
        else:
            hdg_box.grid(row=hdg_row, column=0, sticky="nsew")
            panel.rowconfigure(hdg_row, weight=1, minsize=self._HDG_ROW_MINSIZE)
            if label is not None:
                label.grid(row=label_row, column=0, pady=(2, 2))
                panel.rowconfigure(label_row, weight=0, minsize=self._LABEL_MINSIZE)

    # ══════════════════════════════════════════════════════════════════════════
    # Readout widget update
    # ══════════════════════════════════════════════════════════════════════════

    def _cardinal_for(self, hdg_norm: float) -> str:
        nearest = min(self.CARDINALS.keys(),
                      key=lambda k: abs((k - hdg_norm + 180) % 360 - 180))
        return self.CARDINALS[nearest]

    def _update_readout_widgets(self, heading: float, valid: bool):
        hdg_val = self._tier_widgets.get("hdg_val")
        status  = self._tier_widgets.get("status")
        hint    = self._tier_widgets.get("hint")

        if not valid:
            if hdg_val:
                hdg_val.config(text="---.- ---", fg=self.C_CRIT)
            if status:
                status.config(text="⬤  NO SIGNAL", fg=self.C_NOSIG_FG)
            if hint:
                hint.config(fg=self.C_HINT)
            return

        if hint:
            hint.config(fg=self.C_BG)

        hdg_norm = heading % 360.0
        cardinal = self._cardinal_for(hdg_norm)
        hdg_text = f"{hdg_norm:05.1f}° {cardinal}"

        if hdg_val:
            hdg_val.config(text=hdg_text, fg=self.C_SAFE)
            # Re-measure font fit whenever text content changes (cardinal
            # length changes between single-char "N" and two-char "NW" etc.)
            hdg_box = self._tier_widgets.get("hdg_box")
            if hdg_box:
                w = hdg_box.winfo_width()
                h = hdg_box.winfo_height()
                if w > 1 and h > 1:
                    # Force re-check even if size didn't change — text did
                    self._hdg_font_size = 0
                    self._resize_hdg_font(w, h)

        if status:
            status.config(text="⬤  LOCK", fg=self.C_GREEN)

    # ══════════════════════════════════════════════════════════════════════════
    # Raw field bars
    # ══════════════════════════════════════════════════════════════════════════

    def _draw_bars(self, mx: int, my: int, mz: int):
        bars = self._tier_widgets.get("bars")
        if not bars:
            return

        peak = max(abs(mx), abs(my), abs(mz), 1)
        self._mag_scale = max(self.MAG_SCALE_MIN, peak * 1.25)

        for axis, raw in [("X", mx), ("Y", my), ("Z", mz)]:
            if axis not in bars:
                continue
            entry = bars[axis]
            cv    = entry["canvas"]
            color = entry["color"]
            cv.delete("all")

            W   = cv.winfo_width()  or 120
            H   = cv.winfo_height() or 13
            mid = W // 2

            cv.create_rectangle(1, 1, W - 1, H - 1,
                                 fill=self.C_BAR_BG, outline="#1a2535")
            cv.create_line(mid, 1, mid, H - 1, fill="#1e2e40", width=1)

            norm   = max(-1.0, min(1.0, raw / self._mag_scale))
            bar_px = int(abs(norm) * (mid - 3))

            if norm >= 0:
                cv.create_rectangle(mid,          2, mid + bar_px, H - 2,
                                    fill=color, outline="")
            else:
                cv.create_rectangle(mid - bar_px, 2, mid,          H - 2,
                                    fill=color, outline="")

            entry["label"].config(text=f"{raw:>5d}")

    # ══════════════════════════════════════════════════════════════════════════
    # Calibration button handlers
    # ══════════════════════════════════════════════════════════════════════════

    def _on_mag_cal_pressed(self):
        if self._on_mag_cal:
            self._on_mag_cal()
        btn = self._tier_widgets.get("mag_btn")
        if btn:
            btn.config(state="disabled", text="⊕  CALIBRATING MAG…")

    def _on_acc_cal_pressed(self):
        if self._on_acc_cal:
            self._on_acc_cal()
        btn = self._tier_widgets.get("acc_btn")
        if btn:
            btn.config(state="disabled", text="⊕  CALIBRATING…")

    # ══════════════════════════════════════════════════════════════════════════
    # Public update — called every telemetry tick
    # ══════════════════════════════════════════════════════════════════════════

    def update_mag(self, data: dict):
        valid       = data.get("mag_valid",                  False)
        heading     = data.get("mag_heading_deg",            0.0)
        mx          = data.get("mag_x",                      0)
        my          = data.get("mag_y",                      0)
        mz          = data.get("mag_z",                      0)
        mag_cal_act = data.get("mag_cal_active",             False)
        mag_cal_rem = data.get("mag_cal_seconds_remaining",  0)
        acc_cal_act = data.get("acc_cal_active",             False)
        acc_cal_rem = data.get("acc_cal_seconds_remaining",  0)

        self._valid        = valid
        self._last_heading = heading

        w    = self.winfo_width()
        h    = self.winfo_height()
        tier = self._classify(w, h)
        if tier != self._current_tier:
            self._build_tier(tier)
            self._current_tier = tier

        self._draw_rose()
        self._draw_bars(mx, my, mz)
        self._update_readout_widgets(heading, valid)
        self._update_mag_cal_ui(mag_cal_act, mag_cal_rem)
        self._update_acc_cal_ui(acc_cal_act, acc_cal_rem)

    # ══════════════════════════════════════════════════════════════════════════
    # Calibration UI state
    # ══════════════════════════════════════════════════════════════════════════

    def _update_mag_cal_ui(self, cal_active: bool, seconds_remaining: int):
        mag_btn = self._tier_widgets.get("mag_btn")
        mag_lbl = self._tier_widgets.get("mag_lbl")
        if cal_active:
            if mag_btn:
                mag_btn.config(state="disabled", text="⊕  CALIBRATING MAG…")
            if mag_lbl:
                mag_lbl.config(
                    text=f"Rotate on all axes — {seconds_remaining:2d}s",
                    fg=self.C_WARN)
        else:
            if mag_btn:
                mag_btn.config(
                    state="normal" if self._on_mag_cal else "disabled",
                    text="⊕  CALIBRATE MAG" if self._current_tier == "full"
                    else "⊕  CAL MAG")
            if mag_lbl:
                mag_lbl.config(text="")

    def _update_acc_cal_ui(self, cal_active: bool, seconds_remaining: int):
        acc_btn = self._tier_widgets.get("acc_btn")
        acc_lbl = self._tier_widgets.get("acc_lbl")
        if cal_active:
            if acc_btn:
                acc_btn.config(state="disabled", text="⊕  CALIBRATING…")
            if acc_lbl:
                acc_lbl.config(
                    text=f"Keep level & still — {seconds_remaining:2d}s",
                    fg=self.C_BTN_ACC_FG)
        else:
            if acc_btn:
                acc_btn.config(
                    state="normal" if self._on_acc_cal else "disabled",
                    text="⊕  CALIBRATE GYRO/ACC" if self._current_tier == "full"
                    else "⊕  CAL GYRO")
            if acc_lbl:
                acc_lbl.config(text="")


# ══════════════════════════════════════════════════════════════════════════════
# Quick demo / test harness
# ══════════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    root.title("MagWidget — resize test")
    root.geometry("720x320")
    root.configure(bg="#0a0a10")
    root.minsize(285, 145)

    widget = MagWidget(
        root,
        on_mag_calibrate=lambda: print("MAG CAL triggered"),
        on_acc_calibrate=lambda: print("ACC CAL triggered"),
    )
    widget.pack(fill="both", expand=True)

    # Cycle through all 8 headings to stress-test label sizing
    headings = [0, 27.5, 45, 90, 135, 139.1, 180, 225, 270, 315, 359.9]
    idx = [0]

    def next_heading():
        h = headings[idx[0] % len(headings)]
        idx[0] += 1
        widget.update_mag({
            "mag_valid":       True,
            "mag_heading_deg": h,
            "mag_x":           int(500 * math.cos(math.radians(h))),
            "mag_y":           int(500 * math.sin(math.radians(h))),
            "mag_z":           -650,
        })
        root.after(1200, next_heading)

    root.after(300, next_heading)
    root.mainloop()