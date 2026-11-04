"""
command_panel.py  --  "Laptop Control" window of the cockpit
============================================================
Top-level Tk window driving UplinkController. Opened from the toolbar
(🕹 Laptop Control). Tk thread only; reads controller.get_status() at 10 Hz.

Keys (this window must have keyboard focus -- see the "Keyboard" line):
    W / S      forward / back            A / D   left / right
    Q / E      yaw left / right          Space   stop all movement
    R / F      altitude target +0.5 / -0.5 m (while holding)
    H          altitude hold on / off
Movement is press-and-hold: releasing the key returns that axis to zero.
Losing keyboard focus (clicking another window) stops all movement.

Feedback, top to bottom:
    Keyboard   ACTIVE (green) when keys reach this window
    buttons    light up while their key or mouse button is held
    request    what the keys ask for right now -- shown even while blocked
    sent       what actually goes to the radio after all safety gates
    last key   the last R / F / H / Space action and its result

Only ASCII on the buttons: the arrow glyphs are missing from Consolas on
Windows and rendered as empty boxes.
"""

import tkinter as tk

BG, PANEL, FG, DIM = "#0d0d1a", "#16213e", "#e6e6e6", "#7a8599"
GREEN, YELLOW, RED, CYAN = "#33dd77", "#ffcc33", "#ff4455", "#00d4ff"
BTN, BTN_ACTIVE, BTN_HELD = "#26365a", "#33508a", "#2f7de1"
FONT = ("Consolas", 10)
FONT_B = ("Consolas", 10, "bold")

_STATE_COLORS = {"NO_BRIDGE": RED, "STANDBY": YELLOW, "GATE_ON": CYAN, "ACTIVE": GREEN}
_KEYS = {"w": ("fwd", 1), "s": ("fwd", -1), "d": ("right", 1), "a": ("right", -1),
         "e": ("yaw", 1), "q": ("yaw", -1)}
# (text, row, column, axis, value) -- axis None = STOP
_PAD = [("Q  yaw <", 1, 0, "yaw", -1), ("W  fwd", 1, 1, "fwd", 1), ("yaw >  E", 1, 2, "yaw", 1),
        ("A  left", 2, 0, "right", -1), ("STOP", 2, 1, None, 0), ("right  D", 2, 2, "right", 1),
        ("S  back", 3, 1, "fwd", -1)]
_AXIS_WORDS = {("fwd", 1): "forward", ("fwd", -1): "back", ("right", 1): "right",
               ("right", -1): "left", ("yaw", 1): "yaw right", ("yaw", -1): "yaw left"}


class CommandPanel(tk.Toplevel):
    def __init__(self, master, controller, on_close=None):
        super().__init__(master)
        self.title("Laptop Control")
        self.configure(bg=BG)
        self.resizable(False, False)
        self.ctrl = controller
        self._on_close_cb = on_close
        self._held = {}                  # axis -> value from keys/buttons
        self._keys_down = set()          # letters currently held, for display
        self._buttons = {}               # (axis, value) -> Button
        self._has_focus = False
        self._event_job = None
        self._job = None

        self._build()
        # Keys are bound on the toplevel: its bindtag is on every child too,
        # so they work whichever widget inside the window has focus. All
        # widgets have takefocus=0, so Space can never toggle the checkbox.
        for k in _KEYS:
            for key in (k, k.upper()):
                self.bind(f"<KeyPress-{key}>", self._key_down)
                self.bind(f"<KeyRelease-{key}>", self._key_up)
        self.bind("<space>", lambda e: self._key_action("Space", self._stop_all, "all movement stopped"))
        for key, delta in (("r", +0.5), ("R", +0.5), ("f", -0.5), ("F", -0.5)):
            self.bind(f"<KeyPress-{key}>", lambda e, d=delta: self._nudge(d))
        self.bind("<KeyPress-h>", lambda e: self._toggle_hold(from_key=True))
        self.bind("<KeyPress-H>", lambda e: self._toggle_hold(from_key=True))
        self.bind("<FocusIn>", lambda e: self.after_idle(self._check_focus))
        self.bind("<FocusOut>", lambda e: self.after(50, self._check_focus))
        # A click anywhere in the window gives the keyboard back to it.
        self.bind("<ButtonRelease-1>", lambda e: self.focus_set(), add="+")
        self.protocol("WM_DELETE_WINDOW", self.close)
        self.after(100, self.focus_force)
        self._refresh()

    # ── layout ─────────────────────────────────────────────────────────────
    def _lbl(self, parent, text="", fg=FG, font=FONT, **kw):
        return tk.Label(parent, text=text, fg=fg, bg=parent["bg"], font=font, **kw)

    def _btn(self, parent, text, width, command=None):
        return tk.Button(parent, text=text, width=width, font=FONT_B, bg=BTN, fg=FG,
                         activebackground=BTN_ACTIVE, activeforeground=FG, relief="flat",
                         takefocus=0, command=command)

    def _build(self):
        top = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        top.pack(fill="x", padx=8, pady=(8, 4))
        self.enabled_var = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="Laptop control", variable=self.enabled_var,
                       command=self._on_enable, fg=FG, bg=PANEL, selectcolor=BG,
                       activebackground=PANEL, activeforeground=FG, takefocus=0,
                       font=FONT_B).pack(side="left")
        self.bench_lbl = self._lbl(top, "", RED, FONT_B)
        self.bench_lbl.pack(side="left", padx=8)
        self.state_lbl = self._lbl(top, "NO_BRIDGE", RED, FONT_B)
        self.state_lbl.pack(side="right")

        self.block_lbl = self._lbl(self, "", YELLOW, FONT, justify="left",
                                   anchor="w", wraplength=420)
        self.block_lbl.pack(fill="x", padx=12)

        # keyboard status
        kb = tk.Frame(self, bg=PANEL, padx=10, pady=4)
        kb.pack(fill="x", padx=8, pady=(4, 0))
        self.kb_lbl = self._lbl(kb, "", YELLOW, FONT_B, anchor="w")
        self.kb_lbl.pack(fill="x")
        self._lbl(kb, "W/S fwd-back  A/D left-right  Q/E yaw  Space stop  "
                      "R/F alt +/-  H hold", DIM, anchor="w").pack(fill="x")

        # movement pad
        mv = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        mv.pack(fill="x", padx=8, pady=4)
        self._lbl(mv, "Move (hold key or button)", DIM).grid(row=0, column=0, columnspan=4, sticky="w")
        for text, r, c, axis, val in _PAD:
            b = self._btn(mv, text, 9)
            b.grid(row=r, column=c, padx=2, pady=2)
            if axis is None:
                b.configure(command=lambda: self._key_action("STOP", self._stop_all,
                                                             "all movement stopped"),
                            bg="#5a2630", activebackground="#7a3340")
            else:
                self._buttons[(axis, val)] = b
                b.bind("<ButtonPress-1>", lambda e, a=axis, v=val: self._hold(a, v))
                b.bind("<ButtonRelease-1>", lambda e, a=axis: self._release(a))
        self.speed = tk.Scale(mv, from_=2, to=25, orient="horizontal", label="Stick %",
                              length=140, bg=PANEL, fg=FG, highlightthickness=0,
                              troughcolor=BG, font=FONT, takefocus=0, command=self._on_speed)
        self.speed.set(10)
        self.speed.grid(row=1, column=3, rowspan=3, padx=(16, 0))
        self.req_lbl = self._lbl(mv, "", FG, anchor="w")
        self.req_lbl.grid(row=4, column=0, columnspan=4, sticky="w", pady=(6, 0))
        self.cmd_lbl = self._lbl(mv, "", DIM, anchor="w")
        self.cmd_lbl.grid(row=5, column=0, columnspan=4, sticky="w")

        # altitude hold
        ah = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        ah.pack(fill="x", padx=8, pady=4)
        self.hold_btn = self._btn(ah, "Altitude hold: OFF (H)", 22, command=self._toggle_hold)
        self.hold_btn.grid(row=0, column=0, rowspan=2)
        tk.Button(ah, text="+0.5 m (R)", font=FONT, bg=BTN, fg=FG, relief="flat", takefocus=0,
                  command=lambda: self._nudge(+0.5)).grid(row=0, column=1, padx=6)
        tk.Button(ah, text="-0.5 m (F)", font=FONT, bg=BTN, fg=FG, relief="flat", takefocus=0,
                  command=lambda: self._nudge(-0.5)).grid(row=1, column=1, padx=6)
        self.alt_lbl = self._lbl(ah, "", FG, justify="left")
        self.alt_lbl.grid(row=0, column=2, rowspan=2, sticky="w")

        # gimbal
        gb = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        gb.pack(fill="x", padx=8, pady=4)
        self.tilt = tk.Scale(gb, from_=100, to=-100, orient="vertical", label="Tilt %",
                             length=110, bg=PANEL, fg=FG, highlightthickness=0,
                             troughcolor=BG, font=FONT, takefocus=0, command=self._on_tilt)
        self.tilt.set(0)
        self.tilt.pack(side="left")
        bx = tk.Frame(gb, bg=PANEL)
        bx.pack(side="left", padx=10)
        for text, val in (("Level", 0), ("Down ~45 deg*", -50), ("Down*", -100)):
            tk.Button(bx, text=text, width=14, font=FONT, bg=BTN, fg=FG, relief="flat",
                      takefocus=0, command=lambda v=val: self.tilt.set(v)).pack(pady=2)
        self._lbl(bx, "* depends on servo setup", DIM).pack()

        # last key action + radio echo
        self.event_lbl = self._lbl(self, "", CYAN, FONT_B, anchor="w")
        self.event_lbl.pack(fill="x", padx=12, pady=(2, 0))
        self.radio_lbl = self._lbl(self, "", DIM, justify="left", anchor="w")
        self.radio_lbl.pack(fill="x", padx=12, pady=(2, 8))

    # ── input handling ─────────────────────────────────────────────────────
    def _key_down(self, e):
        k = e.keysym.lower()
        self._keys_down.add(k)
        axis, val = _KEYS[k]
        self._hold(axis, val)

    def _key_up(self, e):
        k = e.keysym.lower()
        self._keys_down.discard(k)
        self._release(_KEYS[k][0])

    def _hold(self, axis, val):
        self._held[axis] = val
        self._push_move()

    def _release(self, axis):
        self._held.pop(axis, None)
        self._push_move()

    def _push_move(self):
        self.ctrl.ui_heartbeat()
        self.ctrl.set_move(self._held.get("fwd", 0), self._held.get("right", 0),
                           self._held.get("yaw", 0))
        self._paint_buttons()
        self._paint_request()

    def _stop_all(self):
        self._held.clear()
        self._keys_down.clear()
        self.ctrl.stop_move()
        self._paint_buttons()
        self._paint_request()

    def _check_focus(self):
        """Keyboard focus is in this window (any of its widgets)?"""
        try:
            w = self.focus_get()
        except (KeyError, tk.TclError):      # focus in a destroyed / foreign widget
            w = None
        has = w is not None and w.winfo_toplevel() is self
        if self._has_focus and not has:
            self._stop_all()                 # safety: no hidden held keys
        self._has_focus = has
        self._paint_keyboard()

    def _key_action(self, key, fn, text):
        fn()
        self._flash(f"{key}: {text}")

    def _nudge(self, delta):
        s = self.ctrl.get_status()
        self.ctrl.nudge_altitude(delta)
        if s.get("hold_engaged"):
            self._flash(f"{'R' if delta > 0 else 'F'}: altitude target {delta:+.1f} m")
        else:
            self._flash(f"{'R' if delta > 0 else 'F'}: ignored -- altitude hold is OFF", YELLOW)

    def _flash(self, text, color=CYAN):
        self.event_lbl.configure(text="last key  " + text, fg=color)
        if self._event_job is not None:
            self.after_cancel(self._event_job)
        self._event_job = self.after(2500, lambda: self.event_lbl.configure(text=""))

    def _on_enable(self):
        self._stop_all()
        self.ctrl.set_enabled(self.enabled_var.get())
        self.focus_set()

    def _on_speed(self, v):
        self.ctrl.set_speed(float(v), yaw_pct=min(30.0, float(v) * 1.5))
        self._paint_request()

    def _on_tilt(self, v):
        self.ctrl.set_gimbal_tilt(float(v))

    def _toggle_hold(self, from_key=False):
        st = self.ctrl.get_status()
        want = not st.get("hold_engaged", False)
        self.ctrl.request_altitude_hold(want)
        if from_key:
            self._flash(f"H: altitude hold {'requested' if want else 'released'}")

    # ── painting ───────────────────────────────────────────────────────────
    def _paint_buttons(self):
        for (axis, val), b in self._buttons.items():
            held = self._held.get(axis) == val
            b.configure(bg=BTN_HELD if held else BTN,
                        relief="sunken" if held else "flat")

    def _paint_keyboard(self):
        if self._has_focus:
            self.kb_lbl.configure(text="Keyboard: ACTIVE -- keys go to this window", fg=GREEN)
        else:
            self.kb_lbl.configure(text="Keyboard: NOT ACTIVE -- click this window first", fg=YELLOW)

    def _paint_request(self):
        try:
            pct = float(self.speed.get())
        except (AttributeError, tk.TclError):
            return
        parts = [_AXIS_WORDS[(a, v)] for a, v in self._held.items() if v]
        keys = "+".join(sorted(k.upper() for k in self._keys_down)) or "-"
        if parts:
            txt = f"request  keys {keys}:  " + ", ".join(parts) + f"  ({pct:.0f} % stick)"
        else:
            txt = f"request  keys {keys}:  hover (no movement)"
        st = self.ctrl.get_status()
        if parts and not st.get("sticks_ok", False):
            txt += "  -- BLOCKED, see reasons above"
            self.req_lbl.configure(text=txt, fg=YELLOW)
        else:
            self.req_lbl.configure(text=txt, fg=GREEN if parts else FG)

    # ── refresh ────────────────────────────────────────────────────────────
    def _refresh(self):
        # Heartbeat: if this Tk thread freezes, the controller stops honouring
        # held movement keys after UI_TIMEOUT_S (see uplink_controller.py).
        self.ctrl.ui_heartbeat()
        s = self.ctrl.get_status()
        state = s.get("state", "NO_BRIDGE")
        self.state_lbl.configure(text=f"{state}  RTT {s.get('rtt_ms', -1):.0f} ms",
                                 fg=_STATE_COLORS.get(state, RED))
        self.bench_lbl.configure(text="BENCH (props off!)" if s.get("bench") else "")
        if self.enabled_var.get() != bool(s.get("enabled", False)):
            self.enabled_var.set(bool(s.get("enabled", False)))

        notes = list(s.get("blockers", []))
        if s.get("enabled") and not s.get("gate_on", False):
            notes.append("pilot switch OFF -- radio ignores laptop sticks/throttle")
        if s.get("hold_reason"):
            notes.append(f"altitude hold: {s['hold_reason']}")
        self.block_lbl.configure(text="\n".join("- " + n for n in notes))

        self._check_focus()
        self._paint_request()
        self.cmd_lbl.configure(text=(
            f"sent     roll {s.get('cmd_roll_pct', 0):+5.1f}%  pitch {s.get('cmd_pitch_pct', 0):+5.1f}%"
            f"  yaw {s.get('cmd_yaw_pct', 0):+5.1f}%   radio: "
            f"{'APPLIED' if s.get('lua_sticks_active') else 'not applied'}"))

        engaged = s.get("hold_engaged", False)
        self.hold_btn.configure(text=f"Altitude hold: {'ON' if engaged else 'OFF'} (H)",
                                bg="#1f6b3a" if engaged else BTN)
        alt = s.get("altitude_m")
        alt_txt = "--" if alt is None else f"{alt:6.2f} m ({s.get('altitude_age_ms', -1)} ms)"
        hold_txt = (f"target {s.get('hold_target_m', 0):.2f} -> {s.get('hold_request_m', 0):.2f} m\n"
                    f"thr {s.get('hold_output_us', 0):.0f} us (hover {s.get('hold_hover_us', 0):.0f})"
                    f"  radio delta {s.get('lua_throttle_delta_us', 0):+.0f} us"
                    + ("  STALE" if s.get("hold_stale") else "")) if engaged else ""
        self.alt_lbl.configure(text=f"altitude {alt_txt}\n{hold_txt}")

        ch = s.get("radio_channels_us", [])
        self.radio_lbl.configure(text=(
            "radio out  " + "  ".join(f"CH{i + 1} {v:.0f}" for i, v in enumerate(ch[:8]))
            if ch else "radio out  --") + f"\nLua period {s.get('script_period_ms', 0)} ms,"
            f" frames {s.get('frames_sent', 0)}, write errors {s.get('write_errors', 0)},"
            f" laptop loop max {s.get('loop_max_ms', 0):.0f} ms")
        self._job = self.after(100, self._refresh)

    def close(self):
        self._stop_all()
        self.ctrl.request_altitude_hold(False)
        self.ctrl.set_enabled(False)
        for job in (self._job, self._event_job):
            if job is not None:
                self.after_cancel(job)
        if self._on_close_cb:
            self._on_close_cb()
        self.destroy()
