"""
dual_layer.py  —  two faces per instrument panel: USB cable and ELRS radio
===========================================================================

Each DraggablePanel that needs a different layout on the radio link holds a
DualLayerPanel instead of a single widget:

    ┌ DraggablePanel "imu" (same position, size, saved layout) ──┐
    │  DualLayerPanel                                             │
    │    ├─ USB face    IMUWidget          (full MSP data set)    │
    │    └─ radio face  RadioAttitudePanel (what CRSF delivers)   │
    └─────────────────────────────────────────────────────────────┘

Only one face is gridded at a time; the other is grid_remove()d, keeps its
state, and costs nothing to redraw. Switching is a grid swap — instant.

LayerSwitch decides WHEN to switch, for all panels at once, from the
TelemetryWorker's active source:
  • a new source must be stable for hold_s (default 1.0 s) before the
    faces flip — a link that flickers between USB and radio does not make
    the whole window jump back and forth;
  • "NONE" (no live telemetry) never switches — the faces stay as they are
    and keep showing the last known values;
  • the operator never has to reselect anything: position, size, z-order
    and visibility of every panel belong to the DraggablePanel, which is
    untouched.

Pure Tk (main thread only) + a pure, headless-testable LayerSwitch.
"""

import tkinter as tk

import link_mode

# How long a new telemetry source must stay active before the panels switch.
LAYER_HOLD_S = 1.0


class LayerSwitch:
    """Debounced USB/radio layer decision. No Tk — unit-testable."""

    def __init__(self, hold_s: float = LAYER_HOLD_S):
        self.hold_s = hold_s
        self.radio = False          # current committed layer
        self._candidate = None      # layer waiting out the hold time
        self._since = 0.0

    def observe(self, active_source: str, now: float) -> bool:
        """
        Feed the worker's active source ('USB' / 'ELRS' / 'NONE') once per
        UI tick. Returns True on the tick the committed layer changes.
        """
        want = {link_mode.RADIO: True, link_mode.USB: False}.get(active_source)
        if want is None or want == self.radio:
            self._candidate = None
            return False
        if self._candidate != want:
            self._candidate = want
            self._since = now
            return False
        if now - self._since >= self.hold_s:
            self.radio = want
            self._candidate = None
            return True
        return False

    @property
    def pending(self) -> bool:
        """True while a switch is waiting out the hold time."""
        return self._candidate is not None


class DualLayerPanel(tk.Frame):
    """
    Container for one USB face and one radio face in the same cell.

        panel = DualLayerPanel(parent)
        usb   = IMUWidget(panel, ...)
        radio = RadioAttitudePanel(panel)
        panel.set_faces(usb, "update_ui", radio, "update_radio")
        panel.feed(ui_data)            # every UI tick
        panel.show_radio(True/False)   # from LayerSwitch
    """

    def __init__(self, parent, **kwargs):
        kwargs.setdefault("bg", parent.cget("bg"))
        super().__init__(parent, **kwargs)
        self.rowconfigure(0, weight=1)
        self.columnconfigure(0, weight=1)
        self.usb = None
        self.radio = None
        self._usb_update = None
        self._radio_update = None
        self._radio_active = False

    def set_faces(self, usb, usb_update: str, radio, radio_update: str):
        self.usb, self._usb_update = usb, usb_update
        self.radio, self._radio_update = radio, radio_update
        for face in (usb, radio):
            face.grid(row=0, column=0, sticky="nsew")
        self.radio.grid_remove()
        self._radio_active = False

    @property
    def radio_active(self) -> bool:
        return self._radio_active

    @property
    def active(self):
        return self.radio if self._radio_active else self.usb

    def show_radio(self, radio: bool):
        if radio == self._radio_active:
            return
        self._radio_active = radio
        (self.usb if radio else self.radio).grid_remove()
        self.active.grid()

    def feed(self, data: dict):
        """Update the visible face; let the hidden one keep minimal state."""
        if self._radio_active:
            getattr(self.radio, self._radio_update)(data)
            hidden = self.usb
        else:
            getattr(self.usb, self._usb_update)(data)
            hidden = self.radio
        track = getattr(hidden, "on_hidden_data", None)
        if track is not None:
            track(data)
