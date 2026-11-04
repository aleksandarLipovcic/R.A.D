"""
command_panel.py  --  "Laptop Control" window of the cockpit
============================================================
Top-level Tk window driving UplinkController. Opened from the toolbar
(🕹 Laptop Control). Tk thread only; reads controller.get_status() at 10 Hz.

Keys (window must have focus -- losing focus stops all movement):
    W / S      forward / back            A / D   left / right
    Q / E      yaw left / right          Space   stop all movement
    R / F      altitude target +0.5 / -0.5 m (while holding)
    H          altitude hold on / off
Movement is press-and-hold: releasing the key returns that axis to zero.
"""

import tkinter as tk

BG, PANEL, FG, DIM = "#0d0d1a", "#16213e", "#e6e6e6", "#7a8599"
GREEN, YELLOW, RED, CYAN = "#33dd77", "#ffcc33", "#ff4455", "#00d4ff"
FONT = ("Consolas", 10)
FONT_B = ("Consolas", 10, "bold")

_STATE_COLORS = {"NO_BRIDGE": RED, "STANDBY": YELLOW, "GATE_ON": CYAN, "ACTIVE": GREEN}
_KEYS = {"w": ("fwd", 1), "s": ("fwd", -1), "d": ("right", 1), "a": ("right", -1),
         "e": ("yaw", 1), "q": ("yaw", -1)}


class CommandPanel(tk.Toplevel):
    def __init__(self, master, controller, on_close=None):
        super().__init__(master)
        self.title("Laptop Control")
        self.configure(bg=BG)
        self.resizable(False, False)
        self.ctrl = controller
        self._on_close_cb = on_close
        self._held = {}                  # axis -> value from keys/buttons
        self._job = None

        self._build()
        for k in _KEYS:
            self.bind(f"<KeyPress-{k}>", self._key_down)
            self.bind(f"<KeyRelease-{k}>", self._key_up)
            self.bind(f"<KeyPress-{k.upper()}>", self._key_down)
            self.bind(f"<KeyRelease-{k.upper()}>", self._key_up)
        self.bind("<space>", lambda e: self._stop_all())
        self.bind("<KeyPress-r>", lambda e: self.ctrl.nudge_altitude(+0.5))
        self.bind("<KeyPress-f>", lambda e: self.ctrl.nudge_altitude(-0.5))
        self.bind("<KeyPress-h>", lambda e: self._toggle_hold())
        self.bind("<FocusOut>", lambda e: self._stop_all())
        self.protocol("WM_DELETE_WINDOW", self.close)
        self._refresh()

    # ── layout ─────────────────────────────────────────────────────────────
    def _lbl(self, parent, text="", fg=FG, font=FONT, **kw):
        return tk.Label(parent, text=text, fg=fg, bg=parent["bg"], font=font, **kw)

    def _build(self):
        top = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        top.pack(fill="x", padx=8, pady=(8, 4))
        self.enabled_var = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="Laptop control", variable=self.enabled_var,
                       command=self._on_enable, fg=FG, bg=PANEL, selectcolor=BG,
                       activebackground=PANEL, activeforeground=FG,
                       font=FONT_B).pack(side="left")
        self.bench_lbl = self._lbl(top, "", RED, FONT_B)
        self.bench_lbl.pack(side="left", padx=8)
        self.state_lbl = self._lbl(top, "NO_BRIDGE", RED, FONT_B)
        self.state_lbl.pack(side="right")

        self.block_lbl = self._lbl(self, "", YELLOW, FONT, justify="left",
                                   anchor="w", wraplength=380)
        self.block_lbl.pack(fill="x", padx=12)

        # movement pad
        mv = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        mv.pack(fill="x", padx=8, pady=4)
        self._lbl(mv, "Move (hold)", DIM).grid(row=0, column=0, columnspan=5, sticky="w")
        pad = [("⟲ Q", 1, 0, "yaw", -1), ("▲ W", 1, 2, "fwd", 1), ("⟳ E", 1, 4, "yaw", 1),
               ("◀ A", 2, 1, "right", -1), ("■", 2, 2, None, 0), ("▶ D", 2, 3, "right", 1),
               ("▼ S", 3, 2, "fwd", -1)]
        for text, r, c, axis, val in pad:
            b = tk.Button(mv, text=text, width=5, font=FONT_B, bg="#26365a", fg=FG,
                          activebackground="#33508a", relief="flat")
            b.grid(row=r, column=c, padx=2, pady=2)
            if axis is None:
                b.configure(command=self._stop_all)
            else:
                b.bind("<ButtonPress-1>", lambda e, a=axis, v=val: self._hold(a, v))
                b.bind("<ButtonRelease-1>", lambda e, a=axis: self._release(a))
        self.speed = tk.Scale(mv, from_=2, to=25, orient="horizontal", label="Stick %",
                              length=150, bg=PANEL, fg=FG, highlightthickness=0,
                              troughcolor=BG, font=FONT, command=self._on_speed)
        self.speed.set(10)
        self.speed.grid(row=1, column=5, rowspan=3, padx=(16, 0))
        self.cmd_lbl = self._lbl(mv, "", DIM)
        self.cmd_lbl.grid(row=4, column=0, columnspan=6, sticky="w", pady=(4, 0))

        # altitude hold
        ah = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        ah.pack(fill="x", padx=8, pady=4)
        self.hold_btn = tk.Button(ah, text="Altitude hold: OFF", width=20, font=FONT_B,
                                  bg="#26365a", fg=FG, relief="flat", command=self._toggle_hold)
        self.hold_btn.grid(row=0, column=0, rowspan=2)
        tk.Button(ah, text="+0.5 m (R)", font=FONT, bg="#26365a", fg=FG, relief="flat",
                  command=lambda: self.ctrl.nudge_altitude(+0.5)).grid(row=0, column=1, padx=6)
        tk.Button(ah, text="-0.5 m (F)", font=FONT, bg="#26365a", fg=FG, relief="flat",
                  command=lambda: self.ctrl.nudge_altitude(-0.5)).grid(row=1, column=1, padx=6)
        self.alt_lbl = self._lbl(ah, "", FG, justify="left")
        self.alt_lbl.grid(row=0, column=2, rowspan=2, sticky="w")

        # gimbal
        gb = tk.Frame(self, bg=PANEL, padx=10, pady=8)
        gb.pack(fill="x", padx=8, pady=4)
        self.tilt = tk.Scale(gb, from_=100, to=-100, orient="vertical", label="Tilt %",
                             length=110, bg=PANEL, fg=FG, highlightthickness=0,
                             troughcolor=BG, font=FONT, command=self._on_tilt)
        self.tilt.set(0)
        self.tilt.pack(side="left")
        bx = tk.Frame(gb, bg=PANEL)
        bx.pack(side="left", padx=10)
        for text, val in (("Level", 0), ("Down 45°*", -50), ("Down*", -100)):
            tk.Button(bx, text=text, width=10, font=FONT, bg="#26365a", fg=FG, relief="flat",
                      command=lambda v=val: self.tilt.set(v)).pack(pady=2)
        self._lbl(bx, "* depends on servo setup", DIM).pack()

        # radio echo
        self.radio_lbl = self._lbl(self, "", DIM, justify="left", anchor="w")
        self.radio_lbl.pack(fill="x", padx=12, pady=(2, 8))

    # ── input handling ─────────────────────────────────────────────────────
    def _key_down(self, e):
        axis, val = _KEYS[e.keysym.lower()]
        self._hold(axis, val)

    def _key_up(self, e):
        self._release(_KEYS[e.keysym.lower()][0])

    def _hold(self, axis, val):
        self._held[axis] = val
        self._push_move()

    def _release(self, axis):
        self._held.pop(axis, None)
        self._push_move()

    def _push_move(self):
        self.ctrl.set_move(self._held.get("fwd", 0), self._held.get("right", 0),
                           self._held.get("yaw", 0))

    def _stop_all(self):
        self._held.clear()
        self.ctrl.stop_move()

    def _on_enable(self):
        self._stop_all()
        self.ctrl.set_enabled(self.enabled_var.get())

    def _on_speed(self, v):
        self.ctrl.set_speed(float(v), yaw_pct=min(30.0, float(v) * 1.5))

    def _on_tilt(self, v):
        self.ctrl.set_gimbal_tilt(float(v))

    def _toggle_hold(self):
        st = self.ctrl.get_status()
        self.ctrl.request_altitude_hold(not st.get("hold_engaged", False))

    # ── refresh ────────────────────────────────────────────────────────────
    def _refresh(self):
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
        self.block_lbl.configure(text="\n".join("• " + n for n in notes))

        self.cmd_lbl.configure(text=(
            f"sent  roll {s.get('cmd_roll_pct', 0):+5.1f}%  pitch {s.get('cmd_pitch_pct', 0):+5.1f}%"
            f"  yaw {s.get('cmd_yaw_pct', 0):+5.1f}%   radio: "
            f"{'APPLIED' if s.get('lua_sticks_active') else 'not applied'}"))

        engaged = s.get("hold_engaged", False)
        self.hold_btn.configure(text=f"Altitude hold: {'ON' if engaged else 'OFF'} (H)",
                                bg="#1f6b3a" if engaged else "#26365a")
        alt = s.get("altitude_m")
        alt_txt = "--" if alt is None else f"{alt:6.2f} m ({s.get('altitude_age_ms', -1)} ms)"
        hold_txt = (f"target {s.get('hold_target_m', 0):.2f} → {s.get('hold_request_m', 0):.2f} m\n"
                    f"thr {s.get('hold_output_us', 0):.0f} us (hover {s.get('hold_hover_us', 0):.0f})"
                    f"  radio Δ {s.get('lua_throttle_delta_us', 0):+.0f} us"
                    + ("  STALE" if s.get("hold_stale") else "")) if engaged else ""
        self.alt_lbl.configure(text=f"altitude {alt_txt}\n{hold_txt}")

        ch = s.get("radio_channels_us", [])
        self.radio_lbl.configure(text=(
            "radio out  " + "  ".join(f"CH{i + 1} {v:.0f}" for i, v in enumerate(ch[:8]))
            if ch else "radio out  --") + f"\nLua period {s.get('script_period_ms', 0)} ms,"
            f" frames {s.get('frames_sent', 0)}, write errors {s.get('write_errors', 0)}")
        self._job = self.after(100, self._refresh)

    def close(self):
        self._stop_all()
        self.ctrl.request_altitude_hold(False)
        self.ctrl.set_enabled(False)
        if self._job is not None:
            self.after_cancel(self._job)
        if self._on_close_cb:
            self._on_close_cb()
        self.destroy()
