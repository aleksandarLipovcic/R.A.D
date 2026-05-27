#!/usr/bin/env python3
"""
Magnetometer Calibration Diagnostic Tool
For: XFlight Hobby F405 V3 / Betaflight 4.5.3 / QMC5883L via NEO-M10-0-10
Protocol: MSP (MultiWii Serial Protocol) over USB

Requirements:
    pip install pyserial

Usage:
    python mag_calibration_tool.py
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import serial
import serial.tools.list_ports
import threading
import time
import math
import struct
import json
import os
from datetime import datetime
from collections import defaultdict


# ---------------------------------------------------------------------------
# MSP Protocol constants
# ---------------------------------------------------------------------------
MSP_HEADER      = b'$M<'
MSP_RAW_IMU     = 102   # returns acc(3), gyro(3), mag(3) as int16 * 9 words
MSP_STATUS      = 101
MSP_ATTITUDE    = 108   # roll, pitch, yaw as int16

SAMPLE_RATE_HZ  = 10    # how many samples per second to request
SAMPLE_DURATION = 3.0   # seconds to collect per heading

HEADINGS = [
    ("North",     0,   "🧭 Point the NOSE of the drone toward NORTH"),
    ("East",      90,  "🧭 Point the NOSE of the drone toward EAST"),
    ("South",     180, "🧭 Point the NOSE of the drone toward SOUTH"),
    ("West",      270, "🧭 Point the NOSE of the drone toward WEST"),
]

HEADING_COLORS = {
    "North": "#2563eb",
    "East":  "#d97706",
    "South": "#dc2626",
    "West":  "#7c3aed",
}


# ---------------------------------------------------------------------------
# MSP communication helpers
# ---------------------------------------------------------------------------

def build_msp_request(cmd: int) -> bytes:
    """Build a minimal MSP request packet (no payload)."""
    size = 0
    chk = size ^ cmd
    return MSP_HEADER + bytes([size, cmd, chk])


def parse_msp_response(data: bytes):
    """
    Parse an MSP response packet.
    Returns (cmd, payload_bytes) or raises ValueError.
    """
    if len(data) < 6:
        raise ValueError("Packet too short")
    if data[0:3] != b'$M>':
        raise ValueError(f"Bad header: {data[0:3]!r}")
    size = data[3]
    cmd  = data[4]
    payload = data[5:5 + size]
    chk = size ^ cmd
    for b in payload:
        chk ^= b
    if chk != data[5 + size]:
        raise ValueError("Checksum mismatch")
    return cmd, payload


def read_msp_packet(ser: serial.Serial, timeout: float = 0.5) -> tuple:
    """Read one MSP packet from the serial port. Returns (cmd, payload)."""
    ser.timeout = timeout
    header = b''
    deadline = time.time() + timeout
    while time.time() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        header += byte
        if len(header) > 3:
            header = header[-3:]
        if header == b'$M>':
            break
    else:
        raise TimeoutError("MSP header not received")

    size_byte = ser.read(1)
    if not size_byte:
        raise TimeoutError("MSP size byte missing")
    size = size_byte[0]

    rest = ser.read(2 + size)   # cmd + payload + checksum
    if len(rest) < 2 + size:
        raise TimeoutError("MSP packet truncated")

    full = b'$M>' + size_byte + rest
    return parse_msp_response(full)


def get_raw_imu(ser: serial.Serial):
    """
    Request MSP_RAW_IMU and return (mag_x, mag_y, mag_z).
    Betaflight packs: acc[3] gyro[3] mag[3] as little-endian int16.
    """
    ser.write(build_msp_request(MSP_RAW_IMU))
    cmd, payload = read_msp_packet(ser)
    if cmd != MSP_RAW_IMU:
        raise ValueError(f"Unexpected MSP cmd {cmd}")
    if len(payload) < 18:
        raise ValueError(f"Payload too short: {len(payload)} bytes")
    words = struct.unpack('<9h', payload[:18])
    # words[0..2]=acc, [3..5]=gyro, [6..8]=mag
    return words[6], words[7], words[8]


def get_attitude(ser: serial.Serial):
    """
    Request MSP_ATTITUDE and return (roll_deg, pitch_deg, yaw_deg).
    Betaflight sends roll*10, pitch*10, yaw (whole degrees).
    """
    ser.write(build_msp_request(MSP_ATTITUDE))
    cmd, payload = read_msp_packet(ser)
    if cmd != MSP_ATTITUDE:
        raise ValueError(f"Unexpected MSP cmd {cmd}")
    if len(payload) < 6:
        raise ValueError(f"Payload too short: {len(payload)} bytes")
    roll_raw, pitch_raw, yaw = struct.unpack('<3h', payload[:6])
    return roll_raw / 10.0, pitch_raw / 10.0, float(yaw)


# ---------------------------------------------------------------------------
# Analysis helpers
# ---------------------------------------------------------------------------

def heading_from_mag(mx, my):
    """Compute compass heading in degrees [0, 360) from raw mag X/Y."""
    h = math.degrees(math.atan2(my, mx))
    return h % 360


def angle_diff(a, b):
    """Signed angular difference a - b in [-180, 180]."""
    d = (a - b + 180) % 360 - 180
    return d


def analyze_samples(samples_by_heading):
    """
    Returns a dict with analysis results per heading and overall stats.
    samples_by_heading: {label: [(mx, my, mz, yaw), ...]}
    """
    results = {}
    for label, expected_deg, _ in HEADINGS:
        samples = samples_by_heading.get(label, [])
        if not samples:
            continue
        mag_headings = [heading_from_mag(s[0], s[1]) for s in samples]
        yaws         = [s[3] for s in samples if s[3] is not None]
        raw_xs = [s[0] for s in samples]
        raw_ys = [s[1] for s in samples]
        raw_zs = [s[2] for s in samples]

        avg_mag_h = sum(mag_headings) / len(mag_headings)
        std_mag_h = math.sqrt(sum((h - avg_mag_h)**2 for h in mag_headings) / len(mag_headings))
        avg_yaw   = sum(yaws) / len(yaws) if yaws else None

        mag_err   = angle_diff(avg_mag_h, expected_deg)
        yaw_err   = angle_diff(avg_yaw, expected_deg) if avg_yaw is not None else None

        results[label] = {
            "expected_deg":  expected_deg,
            "n_samples":     len(samples),
            "avg_mag_h":     avg_mag_h,
            "std_mag_h":     std_mag_h,
            "avg_yaw":       avg_yaw,
            "mag_error":     mag_err,
            "yaw_error":     yaw_err,
            "raw_x_mean":    sum(raw_xs) / len(raw_xs),
            "raw_y_mean":    sum(raw_ys) / len(raw_ys),
            "raw_z_mean":    sum(raw_zs) / len(raw_zs),
            "raw_x_std":     math.sqrt(sum((x - sum(raw_xs)/len(raw_xs))**2 for x in raw_xs) / len(raw_xs)),
            "raw_y_std":     math.sqrt(sum((y - sum(raw_ys)/len(raw_ys))**2 for y in raw_ys) / len(raw_ys)),
        }
    return results


def diagnose(analysis):
    """Return a list of human-readable diagnostic strings."""
    diag = []
    errors = [abs(v["mag_error"]) for v in analysis.values()]
    yaw_errs = [abs(v["yaw_error"]) for v in analysis.values() if v["yaw_error"] is not None]

    if not errors:
        return ["No data to analyse."]

    max_err = max(errors)
    avg_err = sum(errors) / len(errors)

    if max_err > 45:
        diag.append("⛔  SEVERE offset (>45°): Hard-iron distortion is very likely. "
                    "Magnetic objects (motors, ESCs, battery wires) may be too close to the sensor. "
                    "Run Betaflight's full mag calibration routine first.")
    elif max_err > 20:
        diag.append("⚠️  Significant offset (>20°): Hard-iron or soft-iron distortion present. "
                    "Betaflight mag calibration strongly recommended.")
    elif max_err > 10:
        diag.append("ℹ️  Moderate offset (>10°): Within tolerable range for GPS-assisted modes, "
                    "but consider running mag calibration.")
    else:
        diag.append("✅  Low angular error (<10°): Magnetometer is reading well.")

    # Check consistency across headings
    if max_err - min(errors) > 20:
        diag.append("⚠️  Errors are inconsistent across headings — suggests soft-iron distortion "
                    "(elliptical rather than circular response). "
                    "This requires a full 3D calibration spin, not just alignment.")

    # Check noise level
    high_noise = [lbl for lbl, v in analysis.items() if v["std_mag_h"] > 5]
    if high_noise:
        diag.append(f"⚠️  High noise on headings: {', '.join(high_noise)} (σ > 5°). "
                    "Could be vibration, nearby electronics, or WiFi interference from the laptop.")

    # Indoor environment warning
    diag.append("ℹ️  Indoor / near-laptop environment: metal structures and electronics "
                "can add 10–40° of hard-iron bias. Repeat this test outdoors to isolate "
                "permanent vs. environmental errors.")

    if yaw_errs:
        avg_yaw_err = sum(yaw_errs) / len(yaw_errs)
        if avg_yaw_err < avg_err - 5:
            diag.append("ℹ️  Betaflight's fused yaw (attitude) is more accurate than raw mag heading — "
                        "sensor fusion with gyro is partially compensating. "
                        "Check that mag_declination is set correctly in Betaflight.")

    # Alignment check
    if max_err > 25:
        diag.append("🔧  Check ALIGN_MAG in Betaflight (Configurator → Configuration → Board alignment). "
                    "If the GPS module is rotated relative to the FC, set the correct orientation "
                    "(e.g. CW90FLIP) so axes map correctly.")

    return diag


# ---------------------------------------------------------------------------
# Main GUI Application
# ---------------------------------------------------------------------------

class MagCalApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Magnetometer Calibration Tool — QMC5883L / Betaflight")
        self.geometry("780x620")
        self.resizable(True, True)
        self.configure(bg="#f8f9fa")

        self.serial_port: serial.Serial | None = None
        self.samples_by_heading = defaultdict(list)
        self.current_step = 0          # which heading we're on
        self.collecting = False
        self.live_thread: threading.Thread | None = None
        self.stop_flag = threading.Event()
        self.live_yaw = tk.StringVar(value="--")
        self.live_heading = tk.StringVar(value="--")
        self.live_raw = tk.StringVar(value="--")

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self):
        style = ttk.Style(self)
        style.theme_use("clam")
        style.configure("TButton", padding=6, font=("Helvetica", 10))
        style.configure("Big.TButton", padding=10, font=("Helvetica", 12, "bold"))
        style.configure("TLabel", background="#f8f9fa", font=("Helvetica", 10))
        style.configure("Header.TLabel", font=("Helvetica", 14, "bold"), background="#f8f9fa")
        style.configure("Sub.TLabel", font=("Helvetica", 10), background="#f8f9fa",
                        foreground="#555")

        # ---- Top bar: connection ----
        conn_frame = tk.Frame(self, bg="#1e293b", pady=8)
        conn_frame.pack(fill=tk.X)

        tk.Label(conn_frame, text="Serial port:", bg="#1e293b", fg="white",
                 font=("Helvetica", 10)).pack(side=tk.LEFT, padx=(12, 4))

        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=22,
                                        state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=4)

        ttk.Button(conn_frame, text="⟳ Refresh", command=self._refresh_ports).pack(
            side=tk.LEFT, padx=4)
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=4)

        self.conn_status = tk.Label(conn_frame, text="● Disconnected", bg="#1e293b",
                                     fg="#ef4444", font=("Helvetica", 10, "bold"))
        self.conn_status.pack(side=tk.LEFT, padx=8)

        # ---- Live readout strip ----
        live_frame = tk.Frame(self, bg="#e2e8f0", pady=6)
        live_frame.pack(fill=tk.X)

        for label_text, var in [
            ("FC Yaw:", self.live_yaw),
            ("Mag Heading:", self.live_heading),
            ("Raw X/Y/Z:", self.live_raw),
        ]:
            tk.Label(live_frame, text=label_text, bg="#e2e8f0",
                     font=("Helvetica", 9, "bold"), fg="#334155").pack(side=tk.LEFT, padx=(12, 2))
            tk.Label(live_frame, textvariable=var, bg="#e2e8f0",
                     font=("Helvetica", 9), fg="#1e293b", width=16,
                     anchor="w").pack(side=tk.LEFT, padx=(0, 8))

        # ---- Notebook ----
        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=8)

        self._build_calibration_tab()
        self._build_results_tab()
        self._build_log_tab()

        # ---- Bottom status bar ----
        self.status_var = tk.StringVar(value="Connect to the flight controller to begin.")
        status_bar = tk.Label(self, textvariable=self.status_var, bg="#cbd5e1",
                               anchor="w", font=("Helvetica", 9), padx=8)
        status_bar.pack(fill=tk.X, side=tk.BOTTOM)

        self._refresh_ports()

    def _build_calibration_tab(self):
        tab = tk.Frame(self.notebook, bg="#f8f9fa")
        self.notebook.add(tab, text="  Calibration  ")

        # Progress bar at top
        prog_frame = tk.Frame(tab, bg="#f8f9fa", pady=10)
        prog_frame.pack(fill=tk.X, padx=16)
        tk.Label(prog_frame, text="Progress:", bg="#f8f9fa",
                 font=("Helvetica", 10, "bold")).pack(side=tk.LEFT)
        self.progress_bar = ttk.Progressbar(prog_frame, maximum=len(HEADINGS),
                                              length=400, mode="determinate")
        self.progress_bar.pack(side=tk.LEFT, padx=8, expand=True, fill=tk.X)
        self.progress_label = tk.Label(prog_frame, text="0 / 4", bg="#f8f9fa",
                                        font=("Helvetica", 10))
        self.progress_label.pack(side=tk.LEFT)

        # Step card
        self.step_card = tk.Frame(tab, bg="white", relief=tk.RIDGE, bd=1)
        self.step_card.pack(fill=tk.BOTH, expand=False, padx=16, pady=8)

        self.heading_label = tk.Label(self.step_card, text="",
                                       font=("Helvetica", 20, "bold"), bg="white",
                                       fg="#1e293b")
        self.heading_label.pack(pady=(18, 4))

        self.instruction_label = tk.Label(self.step_card, text="",
                                           font=("Helvetica", 11), bg="white",
                                           fg="#475569", wraplength=600, justify="center")
        self.instruction_label.pack(pady=(0, 8))

        self.collect_btn = ttk.Button(self.step_card, text="▶  Collect samples",
                                       style="Big.TButton", command=self._start_collection,
                                       state="disabled")
        self.collect_btn.pack(pady=8)

        self.collection_progress = ttk.Progressbar(self.step_card, maximum=100,
                                                    length=460, mode="determinate")
        self.collection_progress.pack(pady=4)
        self.collection_label = tk.Label(self.step_card, text="", bg="white",
                                          font=("Helvetica", 9), fg="#64748b")
        self.collection_label.pack(pady=(0, 12))

        # Sample count indicators
        count_frame = tk.Frame(tab, bg="#f8f9fa", pady=6)
        count_frame.pack(fill=tk.X, padx=16)
        self.count_labels = {}
        for label, _, _ in HEADINGS:
            col = HEADING_COLORS.get(label, "#333")
            f = tk.Frame(count_frame, bg="white", relief=tk.GROOVE, bd=1, padx=8, pady=4)
            f.pack(side=tk.LEFT, padx=4)
            tk.Label(f, text=label, font=("Helvetica", 9, "bold"),
                     fg=col, bg="white").pack()
            lbl = tk.Label(f, text="0 samples", font=("Helvetica", 9),
                           fg="#64748b", bg="white")
            lbl.pack()
            self.count_labels[label] = lbl

        # Action buttons
        btn_frame = tk.Frame(tab, bg="#f8f9fa")
        btn_frame.pack(pady=8)
        ttk.Button(btn_frame, text="Reset all data", command=self._reset).pack(
            side=tk.LEFT, padx=6)
        self.save_btn = ttk.Button(btn_frame, text="💾 Save results (JSON)",
                                    command=self._save_results, state="disabled")
        self.save_btn.pack(side=tk.LEFT, padx=6)
        self.analyse_btn = ttk.Button(btn_frame, text="📊 Analyse →",
                                       command=self._show_analysis, state="disabled")
        self.analyse_btn.pack(side=tk.LEFT, padx=6)

        self._update_step_ui()

    def _build_results_tab(self):
        tab = tk.Frame(self.notebook, bg="#f8f9fa")
        self.notebook.add(tab, text="  Results  ")
        self.results_text = scrolledtext.ScrolledText(
            tab, font=("Courier", 10), bg="white", relief=tk.FLAT,
            padx=10, pady=10, wrap=tk.WORD)
        self.results_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        self.results_text.insert(tk.END, "Analysis will appear here after all headings are sampled.\n")
        self.results_text.configure(state="disabled")

    def _build_log_tab(self):
        tab = tk.Frame(self.notebook, bg="#f8f9fa")
        self.notebook.add(tab, text="  Log  ")
        self.log_text = scrolledtext.ScrolledText(
            tab, font=("Courier", 9), bg="#0f172a", fg="#94a3b8",
            relief=tk.FLAT, padx=10, pady=10, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)
        ttk.Button(tab, text="Clear log", command=lambda: self._clear_log()).pack(
            side=tk.RIGHT, padx=8, pady=4)

    # ------------------------------------------------------------------
    # Port management
    # ------------------------------------------------------------------

    def _refresh_ports(self):
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_combo["values"] = ports
        if ports:
            self.port_var.set(ports[0])
        else:
            self.port_var.set("")

    def _toggle_connect(self):
        if self.serial_port and self.serial_port.is_open:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "No serial port selected.")
            return
        try:
            self.serial_port = serial.Serial(port, baudrate=115200, timeout=0.5)
            time.sleep(0.3)
            # Quick sanity check
            self.serial_port.write(build_msp_request(MSP_STATUS))
            time.sleep(0.1)
            self.serial_port.flushInput()
        except Exception as e:
            messagebox.showerror("Connection failed", str(e))
            self.serial_port = None
            return

        self.conn_status.config(text="● Connected", fg="#22c55e")
        self.connect_btn.config(text="Disconnect")
        self.collect_btn.config(state="normal")
        self._log(f"Connected to {port} at 115200 baud")
        self._set_status(f"Connected to {port}. Position drone for first heading.")
        self._start_live_thread()

    def _disconnect(self):
        self.stop_flag.set()
        if self.live_thread:
            self.live_thread.join(timeout=2)
        self.stop_flag.clear()
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.conn_status.config(text="● Disconnected", fg="#ef4444")
        self.connect_btn.config(text="Connect")
        self.collect_btn.config(state="disabled")
        self.live_yaw.set("--")
        self.live_heading.set("--")
        self.live_raw.set("--")
        self._log("Disconnected.")

    # ------------------------------------------------------------------
    # Live readout thread
    # ------------------------------------------------------------------

    def _start_live_thread(self):
        self.stop_flag.clear()
        self.live_thread = threading.Thread(target=self._live_loop, daemon=True)
        self.live_thread.start()

    def _live_loop(self):
        while not self.stop_flag.is_set():
            if not self.serial_port or not self.serial_port.is_open:
                break
            if self.collecting:
                # collection thread handles the serial port; skip here
                time.sleep(0.1)
                continue
            try:
                mx, my, mz = get_raw_imu(self.serial_port)
                h = heading_from_mag(mx, my)
                self.live_raw.set(f"{mx}, {my}, {mz}")
                self.live_heading.set(f"{h:.1f}°")
            except Exception:
                pass

            try:
                _, _, yaw = get_attitude(self.serial_port)
                self.live_yaw.set(f"{yaw:.1f}°")
            except Exception:
                pass

            time.sleep(0.2)

    # ------------------------------------------------------------------
    # Calibration steps
    # ------------------------------------------------------------------

    def _update_step_ui(self):
        if self.current_step >= len(HEADINGS):
            self.heading_label.config(text="✅ All headings sampled!", fg="#16a34a")
            self.instruction_label.config(
                text="All four cardinal directions have been recorded.\n"
                     "Click 'Analyse →' to see the diagnostic report.")
            self.collect_btn.config(state="disabled", text="Done")
            if all(len(self.samples_by_heading[lbl]) > 0 for lbl, _, _ in HEADINGS):
                self.analyse_btn.config(state="normal")
                self.save_btn.config(state="normal")
            return

        label, _, instruction = HEADINGS[self.current_step]
        color = HEADING_COLORS.get(label, "#1e293b")
        self.heading_label.config(text=f"Step {self.current_step + 1} of {len(HEADINGS)}: {label}",
                                   fg=color)
        self.instruction_label.config(text=instruction + "\n\nHold the drone steady, then click 'Collect samples'.")
        self.step_card.config(highlightbackground=color)
        self.progress_bar["value"] = self.current_step
        self.progress_label.config(text=f"{self.current_step} / {len(HEADINGS)}")

    def _start_collection(self):
        if not self.serial_port or not self.serial_port.is_open:
            messagebox.showerror("Error", "Not connected.")
            return
        if self.collecting:
            return
        self.collecting = True
        self.collect_btn.config(state="disabled", text="Collecting…")
        self.collection_progress["value"] = 0

        label = HEADINGS[self.current_step][0]
        t = threading.Thread(target=self._collect_thread, args=(label,), daemon=True)
        t.start()

    def _collect_thread(self, label: str):
        samples = []
        n_target = int(SAMPLE_RATE_HZ * SAMPLE_DURATION)
        interval = 1.0 / SAMPLE_RATE_HZ
        self._log(f"Collecting {n_target} samples for heading: {label}")

        for i in range(n_target):
            if self.stop_flag.is_set():
                break
            t0 = time.time()
            try:
                mx, my, mz = get_raw_imu(self.serial_port)
                try:
                    _, _, yaw = get_attitude(self.serial_port)
                except Exception:
                    yaw = None
                samples.append((mx, my, mz, yaw))
            except Exception as e:
                self._log(f"  Sample {i} error: {e}")

            pct = int((i + 1) / n_target * 100)
            self.after(0, lambda p=pct, ii=i+1: self._update_collection_progress(p, ii, n_target))
            elapsed = time.time() - t0
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)

        self.samples_by_heading[label].extend(samples)
        self._log(f"  Collected {len(samples)} samples for {label}. "
                  f"Avg mag heading: {sum(heading_from_mag(s[0], s[1]) for s in samples) / max(1, len(samples)):.1f}°")

        self.after(0, lambda: self._collection_done(label))

    def _update_collection_progress(self, pct, i, n):
        self.collection_progress["value"] = pct
        self.collection_label.config(text=f"Sampling… {i}/{n}")

    def _collection_done(self, label: str):
        self.collecting = False
        self.collection_label.config(text=f"✓ {len(self.samples_by_heading[label])} samples collected")
        self.count_labels[label].config(
            text=f"{len(self.samples_by_heading[label])} samples",
            fg="#16a34a", font=("Helvetica", 9, "bold"))
        self.current_step += 1
        self._update_step_ui()
        if self.current_step < len(HEADINGS):
            self.collect_btn.config(state="normal", text="▶  Collect samples")
        self._set_status(f"✓ {label} done. "
                         + (f"Next: {HEADINGS[self.current_step][0]}" if self.current_step < len(HEADINGS) else "All done!"))

    def _reset(self):
        if not messagebox.askyesno("Reset", "Clear all collected samples and start over?"):
            return
        self.samples_by_heading.clear()
        self.current_step = 0
        self.collecting = False
        for lbl in self.count_labels:
            self.count_labels[lbl].config(text="0 samples", fg="#64748b",
                                           font=("Helvetica", 9))
        self.collection_progress["value"] = 0
        self.collection_label.config(text="")
        self.collect_btn.config(state="normal" if self.serial_port else "disabled",
                                text="▶  Collect samples")
        self.analyse_btn.config(state="disabled")
        self.save_btn.config(state="disabled")
        self._update_step_ui()
        self._set_status("Reset. Position drone for North.")

    # ------------------------------------------------------------------
    # Analysis & output
    # ------------------------------------------------------------------

    def _show_analysis(self):
        analysis = analyze_samples(self.samples_by_heading)
        diag = diagnose(analysis)

        lines = []
        lines.append("=" * 68)
        lines.append("  MAGNETOMETER CALIBRATION DIAGNOSTIC REPORT")
        lines.append(f"  Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        lines.append("=" * 68)
        lines.append("")
        lines.append("  SENSOR:      QMC5883L (NEO-M10-0-10 GPS module)")
        lines.append("  CONTROLLER:  XFlight Hobby F405 V3 / Betaflight 4.5.3")
        lines.append("  ENVIRONMENT: Indoor (results may include external interference)")
        lines.append("")

        lines.append("┌─────────────────────────────────────────────────────────────────┐")
        lines.append("│  PER-HEADING RESULTS                                            │")
        lines.append("├──────────┬──────────┬──────────┬──────────┬──────────┬──────────┤")
        lines.append("│ Heading  │ Expected │ Mag avg  │ Mag err  │ FC yaw   │ Yaw err  │")
        lines.append("├──────────┼──────────┼──────────┼──────────┼──────────┼──────────┤")
        for label, expected_deg, _ in HEADINGS:
            r = analysis.get(label)
            if not r:
                lines.append(f"│ {label:<8} │ {'NO DATA':>8} │         │          │          │          │")
                continue
            yaw_s   = f"{r['avg_yaw']:.1f}°" if r['avg_yaw'] is not None else "N/A"
            yerr_s  = f"{r['yaw_error']:+.1f}°" if r['yaw_error'] is not None else "N/A"
            lines.append(
                f"│ {label:<8} │ {expected_deg:>7}° │ {r['avg_mag_h']:>7.1f}° │ "
                f"{r['mag_error']:>+7.1f}° │ {yaw_s:>8} │ {yerr_s:>8} │"
            )
        lines.append("└──────────┴──────────┴──────────┴──────────┴──────────┴──────────┘")
        lines.append("")

        lines.append("  RAW SENSOR STATISTICS")
        lines.append("  ─────────────────────────────────────────────────────────────────")
        lines.append(f"  {'Heading':<10}  {'X mean':>8}  {'X σ':>6}  {'Y mean':>8}  {'Y σ':>6}  {'Z mean':>8}  {'N':>4}")
        for label, _, _ in HEADINGS:
            r = analysis.get(label)
            if not r:
                continue
            lines.append(
                f"  {label:<10}  {r['raw_x_mean']:>8.1f}  {r['raw_x_std']:>6.1f}"
                f"  {r['raw_y_mean']:>8.1f}  {r['raw_y_std']:>6.1f}"
                f"  {r['raw_z_mean']:>8.1f}  {r['n_samples']:>4}"
            )
        lines.append("")

        lines.append("  DIAGNOSTICS")
        lines.append("  ─────────────────────────────────────────────────────────────────")
        for d in diag:
            # word-wrap at 64 chars
            import textwrap
            wrapped = textwrap.fill(d, width=64, subsequent_indent="     ")
            lines.append(f"  {wrapped}")
            lines.append("")

        lines.append("  RECOMMENDED NEXT STEPS")
        lines.append("  ─────────────────────────────────────────────────────────────────")
        lines.append("  1. Run Betaflight Configurator → Calibrate Magnetometer (spin")
        lines.append("     the drone through all orientations for 30 s outdoors).")
        lines.append("  2. Set GPS > Mag Declination in Configurator for your location.")
        lines.append("     Banja Luka, BA declination ≈ +4.3° East (verify at")
        lines.append("     ngdc.noaa.gov/geomag/calculators/magcalc.shtml).")
        lines.append("  3. In CLI: set align_mag = CW0FLIP (or match your module's")
        lines.append("     physical orientation relative to the FC arrow).")
        lines.append("  4. Re-run this test outdoors, away from metal and electronics.")
        lines.append("=" * 68)

        report = "\n".join(lines)

        self.results_text.configure(state="normal")
        self.results_text.delete("1.0", tk.END)
        self.results_text.insert(tk.END, report)
        self.results_text.configure(state="disabled")

        self.notebook.select(1)   # switch to Results tab
        self._log("Analysis complete.")

    def _save_results(self):
        analysis = analyze_samples(self.samples_by_heading)
        diag = diagnose(analysis)
        payload = {
            "timestamp": datetime.now().isoformat(),
            "sensor": "QMC5883L",
            "fc": "XFlight Hobby F405 V3",
            "firmware": "Betaflight 4.5.3",
            "environment": "indoor",
            "analysis": analysis,
            "diagnostics": diag,
            "raw_samples": {k: v for k, v in self.samples_by_heading.items()},
        }
        fname = f"mag_cal_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        path = os.path.join(os.path.expanduser("~"), fname)
        try:
            with open(path, "w") as f:
                json.dump(payload, f, indent=2, default=float)
            messagebox.showinfo("Saved", f"Results saved to:\n{path}")
            self._log(f"Results saved: {path}")
        except Exception as e:
            messagebox.showerror("Save failed", str(e))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _log(self, msg: str):
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.after(0, lambda: self._append_log(f"[{ts}] {msg}\n"))

    def _append_log(self, text: str):
        self.log_text.configure(state="normal")
        self.log_text.insert(tk.END, text)
        self.log_text.see(tk.END)
        self.log_text.configure(state="disabled")

    def _clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", tk.END)
        self.log_text.configure(state="disabled")

    def _set_status(self, msg: str):
        self.status_var.set(msg)

    def on_closing(self):
        self.stop_flag.set()
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.destroy()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    app = MagCalApp()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()