"""
telemetry_worker.py  —  Background Telemetry Worker
=====================================================
Runs in a daemon thread.  Polls DroneBackend.DroneLink, builds the
ui_data dict, and puts finished frames onto a bounded Queue for the
main Tkinter thread to consume.

Rules enforced here
-------------------
  • NO tkinter imports or calls — ever.
  • NO widget references.
  • The only shared state with the UI thread is:
      - self._queue    (thread-safe queue.Queue)
      - self._running  (threading.Event — set to stop)
      - self._connected (threading.Event — readable from UI for status)

The main thread reads:
    worker.get_frame()        → dict | None   (non-blocking)
    worker.is_connected       → bool (property, reads an Event)
    worker.last_error         → str | None
    worker.get_link_status()  → dict  (USB + ELRS radio link status, always
                                       available — even when no frames flow)

Two telemetry sources (ELRS update)
-----------------------------------
  • USB  — DroneBackend.DroneLink  (MSP over the USB-C cable, full data set)
  • ELRS — DroneBackend.CrsfLink   (CRSF telemetry via the RadioMaster Pocket,
                                    USB-VCP set to "Telem Mirror")
Every poll picks the best source (see _pick_source()):
  1. USB  if the cable link is healthy
  2. ELRS if radio telemetry is OK or DEGRADED
  3. USB  if the cable is connected but momentarily unhealthy
  4. none → no frame is produced, is_connected goes False
The frame dict keeps every existing key, plus "link_source" ("USB"/"ELRS")
and a few ELRS-only keys. Data that CRSF cannot carry (raw IMU, raw mag,
motors, RC channels, satellite list, HDOP, arming flags) is zero/empty on
the ELRS path — the "available_*" keys say which.

get_active_state() is the thread-safe way for the OSD / DetectionLink
trampolines (called from C++ threads) to get the same state the UI shows.
"""

import time
import threading
import queue
from typing import Optional


# ── Tuning constants ──────────────────────────────────────────────────────────
POLL_HZ        = 60          # backend poll rate (frames per second)
_POLL_INTERVAL = 1.0 / POLL_HZ
_QUEUE_DEPTH   = 3           # max buffered frames; older ones are dropped
RECONNECT_S    = 2.0         # seconds between reconnect attempts (legacy, unused)
_IDLE_S        = 0.1         # poll interval while no source is live
_STATUS_HZ     = 10          # link-status refresh rate while frames flow

# CrsfLink status strings that mean "drone telemetry is arriving"
RADIO_LIVE = ("TELEMETRY_OK", "DEGRADED")


class TelemetryWorker:
    """
    Background worker that continuously reads from DroneBackend and
    delivers ui_data dicts to the Tk thread via a Queue.

    Usage
    -----
        worker = TelemetryWorker(hub)
        worker.start()

        # In Tk update loop:
        frame = worker.get_frame()   # returns None if nothing new
        if frame:
            ...feed widgets...

        # On shutdown:
        worker.stop()
    """

    def __init__(self, hub, radio=None) -> None:
        self._hub          = hub
        self._radio        = radio          # DroneBackend.CrsfLink or None
        self._queue: queue.Queue = queue.Queue(maxsize=_QUEUE_DEPTH)

        # Latest link status for the toolbar indicator — replaced as a whole
        # dict (atomic reference swap), so the Tk thread can read it lock-free.
        self._link_status: dict = _empty_link_status(radio is not None)
        self._running      = threading.Event()
        self._connected    = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self.last_error: Optional[str] = None

        # Yaw trim is written only from the main thread via set_yaw_trim(),
        # and read inside the worker thread.  A lock keeps the read atomic.
        self._yaw_trim_lock = threading.Lock()
        self._yaw_trim      = 0.0

        # Expose last-seen values so the Tk thread can read them for the
        # heading-trim calculation without needing a full frame.
        self._state_lock         = threading.Lock()
        self._last_raw_yaw       = 0.0
        self._last_mag_heading   = 0.0
        self._last_mag_valid     = False

    # =========================================================================
    # Public API (called from the main / Tk thread)
    # =========================================================================

    def start(self) -> None:
        """Start the background polling thread."""
        self._running.set()
        self._thread = threading.Thread(
            target=self._run, name="TelemetryWorker", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        """Signal the worker to stop and wait for it to exit cleanly."""
        self._running.clear()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)

    def get_frame(self) -> Optional[dict]:
        """
        Non-blocking. Returns the most recent ui_data dict or None.
        Drains any stale frames so the UI always gets the freshest data.
        """
        latest = None
        while True:
            try:
                latest = self._queue.get_nowait()
            except queue.Empty:
                break
        return latest

    @property
    def is_connected(self) -> bool:
        return self._connected.is_set()

    def set_yaw_trim(self, trim: float) -> None:
        """Thread-safe write from Tk thread."""
        with self._yaw_trim_lock:
            self._yaw_trim = trim

    def get_yaw_trim(self) -> float:
        with self._yaw_trim_lock:
            return self._yaw_trim

    def get_last_state_snapshot(self) -> tuple:
        """Returns (raw_yaw, mag_heading, mag_valid) — for trim calculation."""
        with self._state_lock:
            return self._last_raw_yaw, self._last_mag_heading, self._last_mag_valid

    def get_link_status(self) -> dict:
        """
        USB + ELRS link status for the toolbar (RadioLinkIndicator and the
        USB status label). Refreshed by the worker thread even while no
        telemetry frames are produced, so the pilot always sees why.
        """
        return self._link_status

    def get_active_state(self):
        """
        Thread-safe. The DroneState of the currently best live source, or
        None if neither USB nor ELRS has live telemetry. Used by the OSD and
        DetectionLink trampolines, which run on C++ threads.
        """
        try:
            source, state, _ = self._pick_source()
        except Exception:
            return None
        return state if source is not None else None

    # =========================================================================
    # Source selection  —  USB (DroneLink) vs ELRS (CrsfLink)
    # =========================================================================

    def _pick_source(self):
        """
        Returns (source, state, radio_state):
          source      "USB" / "ELRS" / None (no live telemetry)
          state       DroneState to display (None only if nothing exists)
          radio_state latest CrsfLink state (or None if no radio object),
                      always returned so the indicator can show radio status
                      even while USB is the active source.
        Both get_latest_state() calls are mutex-protected C++ snapshots.
        """
        radio_state = self._radio.get_latest_state() if self._radio is not None else None
        radio_live = (radio_state is not None and
                      radio_state.radio.status_str in RADIO_LIVE)

        usb_state = None
        if self._hub.is_connected():
            usb_state = self._hub.get_latest_state()
            if getattr(usb_state, "link_healthy", False):
                return "USB", usb_state, radio_state

        if radio_live:
            return "ELRS", radio_state, radio_state
        if usb_state is not None:
            return "USB", usb_state, radio_state     # cable in, momentarily unhealthy
        return None, radio_state, radio_state

    # =========================================================================
    # Worker thread
    # =========================================================================

    def _run(self) -> None:
        last_status_t = 0.0
        while self._running.is_set():
            try:
                source, state, radio_state = self._pick_source()

                now = time.monotonic()
                if source is None or now - last_status_t >= 1.0 / _STATUS_HZ:
                    self._link_status = self._build_link_status(source, radio_state)
                    last_status_t = now

                if source is None:
                    # Nothing live: no frames, but keep refreshing the status
                    # quickly so the indicator reacts within ~100 ms.
                    self._connected.clear()
                    time.sleep(_IDLE_S)
                    continue

                ui_data = self._build_ui_data(state, source)

                # Update the shared state snapshot (lock is brief)
                with self._state_lock:
                    self._last_raw_yaw     = float(state.yaw)
                    self._last_mag_heading = getattr(state, "mag_heading_deg", 0.0)
                    self._last_mag_valid   = getattr(state, "mag_valid", False)

                # Drop the oldest frame if the Tk thread is behind
                if self._queue.full():
                    try:
                        self._queue.get_nowait()
                    except queue.Empty:
                        pass
                self._queue.put_nowait(ui_data)

                self._connected.set()
                self.last_error = None

            except Exception as exc:
                self.last_error = str(exc)
                self._connected.clear()
                # Brief pause so we don't spin-burn the CPU on repeated errors
                time.sleep(0.1)
                continue

            time.sleep(_POLL_INTERVAL)

    # =========================================================================
    # Pure data builder  —  NO Tk, NO widget refs, NO side-effects
    # =========================================================================

    def _build_link_status(self, source, radio_state) -> dict:
        """Flat status dict for the toolbar — no Tk, no widget refs."""
        st = _empty_link_status(self._radio is not None)
        st["active_source"] = source or "NONE"

        usb_up = self._hub.is_connected()
        st["usb_connected"] = usb_up
        if usb_up:
            try:
                st["usb_healthy"] = bool(self._hub.get_latest_state().link_healthy)
            except Exception:
                st["usb_healthy"] = False

        if radio_state is not None:
            r = radio_state.radio
            st.update({
                "radio_status":       r.status_str,
                "radio_port":         r.port_name,
                "link_stats_valid":   r.link_stats_valid,
                "uplink_lq":          int(r.uplink_lq),
                "uplink_rssi1_dbm":   int(r.uplink_rssi1_dbm),
                "uplink_rssi2_dbm":   int(r.uplink_rssi2_dbm),
                "uplink_snr":         int(r.uplink_snr),
                "downlink_lq":        int(r.downlink_lq),
                "downlink_rssi_dbm":  int(r.downlink_rssi_dbm),
                "tx_power_mw":        int(r.tx_power_mw),
                "age_attitude_ms":    int(r.attitude_age_ms),
                "age_gps_ms":         int(r.gps_age_ms),
                "age_battery_ms":     int(r.battery_age_ms),
                "age_flight_mode_ms": int(r.flight_mode_age_ms),
                "age_last_frame_ms":  int(r.ms_since_last_frame),
                "rate_attitude_hz":   float(r.attitude_hz),
                "rate_gps_hz":        float(r.gps_hz),
                "rate_battery_hz":    float(r.battery_hz),
                "rate_total_hz":      float(r.total_frame_hz),
                "reconnect_count":    int(r.reconnect_count),
                "link_lost_count":    int(r.link_lost_count),
                "crc_errors":         int(r.crc_errors),
                "radio_armed":        bool(radio_state.armed),
            })
        return st

    def _build_ui_data(self, state, source: str = "USB") -> dict:
        """
        Transforms a raw DroneBackend state object into the flat ui_data
        dict consumed by all widgets.  This is the only place that touches
        the backend state struct — keeping it here means the Tk thread never
        waits on backend parsing.
        """
        with self._yaw_trim_lock:
            yaw_trim = self._yaw_trim

        raw_yaw   = float(state.yaw)
        mag_hdg   = getattr(state, "mag_heading_deg", 0.0)
        mag_valid = getattr(state, "mag_valid", False)
        trimmed_yaw = (raw_yaw - yaw_trim + 360) % 360

        # ── GPS sub-object ────────────────────────────────────────────────────
        gps = getattr(state, "gps", None)
        if gps is not None:
            gps_hdop_raw  = getattr(gps, "hdop", 9999)
            gps_hdop_real = gps_hdop_raw / 100.0 if gps_hdop_raw != 9999 else 99.0

            sv_list_raw   = list(state.sv_list)
            sv_info_valid = getattr(state, "sv_info_valid", False)
            sv_source     = getattr(state, "sv_source", "")

            sv_list = [
                {
                    "gnss_id": getattr(sv, "gnss_name",  "?"),
                    "sv_id":   getattr(sv, "svid",        0),
                    "cno":     getattr(sv, "cno",         0),
                    "used":    getattr(sv, "used",        False),
                    "quality": getattr(sv, "quality",     0),
                    "status":  getattr(sv, "status_str",  "idle"),
                    "elev":    getattr(sv, "elev",        0),
                    "azim":    getattr(sv, "azim",        0),
                }
                for sv in sv_list_raw
            ]

            heartbeat = getattr(gps, "heartbeat",
                        getattr(gps, "gps_heartbeat", None))

            gps_data = {
                "gps_fix_type":         getattr(gps, "fix_type",         0),
                "gps_num_sat":          getattr(gps, "num_sat",           0),
                "gps_hdop":             gps_hdop_real,
                "gps_latitude":         getattr(gps, "latitude",          0.0),
                "gps_longitude":        getattr(gps, "longitude",         0.0),
                "gps_altitude_m":       float(getattr(gps, "altitude_m",  0)),
                "gps_ground_speed_cms": getattr(gps, "ground_speed_cms",  0),
                "gps_ground_course":    getattr(gps, "ground_course",     0),
                "gps_dist_to_home_m":   float(getattr(gps, "dist_to_home_m", 0)),
                "gps_bearing_to_home":  getattr(gps, "bearing_to_home",  0),
                "gps_heartbeat":        heartbeat,
                "gps_raw_valid":        getattr(gps, "raw_valid",         False),
                "gps_comp_valid":       getattr(gps, "comp_valid",        False),
                "gps_position_usable":  getattr(gps, "position_usable",   False),
                "gps_sv_list":          sv_list,
                "gps_sv_info_valid":    sv_info_valid,
                "gps_sv_source":        sv_source,
                "gps_num_sats":         getattr(gps, "num_sat",           0),
                "gps_fix":              getattr(gps, "position_usable",   False),
            }
        else:
            gps_data = {
                "gps_fix_type": 0, "gps_num_sat": 0, "gps_hdop": 99.0,
                "gps_latitude": 0.0, "gps_longitude": 0.0,
                "gps_altitude_m": 0.0, "gps_ground_speed_cms": 0,
                "gps_ground_course": 0, "gps_dist_to_home_m": 0.0,
                "gps_bearing_to_home": 0, "gps_heartbeat": None,
                "gps_raw_valid": False, "gps_comp_valid": False,
                "gps_position_usable": False,
                "gps_sv_list": [], "gps_sv_info_valid": False,
                "gps_sv_source": "",
                "gps_num_sats": 0,
                "gps_fix": False,
            }

        # ── RC and motor helpers ──────────────────────────────────────────────
        rc_ch = list(state.rc_channels)
        rc_n  = int(state.rc_channel_count)
        mot   = list(state.motor_values)

        def _rc(i):
            return int(rc_ch[i]) if rc_n > i else 0

        def _mot(i):
            return int(mot[i]) if len(mot) > i else 0

        _rssi = int(state.rssi)
        rc_link_quality = -1 if _rssi == 0 else int(_rssi * 100 / 255)

        # ── ELRS extras ───────────────────────────────────────────────────────
        is_elrs = (source == "ELRS")
        radio = getattr(state, "radio", None) if is_elrs else None
        if radio is not None and radio.link_stats_valid:
            rc_link_quality = int(radio.uplink_lq)   # real ELRS LQ %, 0 = lost
        elrs_data = {
            "link_source": source,
            # What this source can deliver. Widgets for USB-only data can
            # show "USB only" instead of zeros when these are False.
            "available_raw_imu":      not is_elrs,
            "available_raw_mag":      not is_elrs,
            "available_motors":       not is_elrs,
            "available_rc_channels":  not is_elrs,
            "available_sat_list":     not is_elrs,
            "available_hdop":         not is_elrs,
            "available_arming_flags": not is_elrs,
            "available_cpu_load":     not is_elrs,
            # CRSF only says "arming blocked yes/no" (reasons need USB)
            "arming_blocked": bool(radio.arming_blocked) if radio is not None else False,
            "gps_waiting":    bool(radio.gps_waiting) if radio is not None else False,
            "altitude_source": radio.altitude_source if radio is not None else "BARO",
            # Per-instrument data age in ms (-1 = never). Only meaningful on
            # ELRS; on USB everything is refreshed every poll → 0.
            "age_attitude_ms": int(radio.attitude_age_ms) if radio is not None else 0,
            "age_gps_ms":      int(radio.gps_age_ms) if radio is not None else 0,
            "age_battery_ms":  int(radio.battery_age_ms) if radio is not None else 0,
        }

        try:
            bat_state_str = state.battery_state.name
        except Exception:
            bat_state_str = "INIT"

        sensor_status = getattr(state, "sensor_status", 0)

        return {
            # ── IMU ───────────────────────────────────────────────────────────
            "ax": state.ax, "ay": state.ay, "az": state.az,
            "gx": state.gx, "gy": state.gy, "gz": state.gz,
            "roll":  state.roll  / 10.0,
            "pitch": state.pitch / 10.0,
            "yaw":   trimmed_yaw,

            # ── link health (consumed by Tk thread for status label) ───────────
            "link_healthy": getattr(state, "link_healthy", True),

            # ── Battery ───────────────────────────────────────────────────────
            "battery_voltage":      state.battery_voltage,
            "battery_current":      state.battery_current,
            "battery_mah_drawn":    state.battery_mah_drawn,
            "battery_cell_count":   state.battery_cell_count,
            "battery_capacity_mah": state.battery_capacity_mah,
            "battery_percentage":   state.battery_percentage,
            "battery_state":        bat_state_str,

            # ── Motors ────────────────────────────────────────────────────────
            "motor_1_us": _mot(0),
            "motor_2_us": _mot(1),
            "motor_3_us": _mot(2),
            "motor_4_us": _mot(3),
            "motor_5_us": _mot(4),
            "motor_6_us": _mot(5),
            "motor_7_us": _mot(6),
            "motor_8_us": _mot(7),
            "motor_count": int(state.motor_count),

            # ── RC channels ───────────────────────────────────────────────────
            "rc_channel_count": rc_n,
            "rc_roll":          _rc(0),
            "rc_pitch":         _rc(1),
            "rc_throttle":      _rc(2),
            "rc_yaw":           _rc(3),
            "rc_arm":           _rc(4),
            "rc_aux1":          _rc(5),
            "rc_aux2":          _rc(6),
            "rc_aux3":          _rc(7),

            # ── RC link quality ───────────────────────────────────────────────
            "rc_link_quality": rc_link_quality,

            # ── Misc telemetry ────────────────────────────────────────────────
            "rssi":        _rssi,
            "rtt_ms":      state.last_rtt_ms,
            "fc_cycle_ms": state.fc_cycle_ms,

            # ── Baro ──────────────────────────────────────────────────────────
            "baro_altitude_cm":      getattr(state, "baro_altitude_cm",      0),
            "baro_vario_cm_per_sec": getattr(state, "baro_vario_cm_per_sec", 0),
            "baro_valid":            getattr(state, "baro_valid",             False),

            # ── Magnetometer ──────────────────────────────────────────────────
            "mag_x":                     getattr(state, "mag_x",             0),
            "mag_y":                     getattr(state, "mag_y",             0),
            "mag_z":                     getattr(state, "mag_z",             0),
            "mag_heading_deg":           mag_hdg,
            "mag_valid":                 mag_valid,
            "mag_cal_active":            getattr(state, "mag_cal_active",            False),
            "mag_cal_seconds_remaining": getattr(state, "mag_cal_seconds_remaining", 0),
            "acc_cal_active":            getattr(state, "acc_cal_active",            False),
            "acc_cal_seconds_remaining": getattr(state, "acc_cal_seconds_remaining", 0),

            # ── FC status ─────────────────────────────────────────────────────
            "armed":             getattr(state, "armed",             False),
            "flight_mode_flags": getattr(state, "flight_mode_flags", 0),
            "flight_mode_name":  getattr(state, "flight_mode_name",  "ACRO [DISARMED]"),
            "sensor_status":     sensor_status,
            "sensor_acc_present":         bool(sensor_status & (1 << 0)),
            "sensor_baro_present":        bool(sensor_status & (1 << 1)),
            "sensor_mag_present":         bool(sensor_status & (1 << 2)),
            "sensor_gps_present":         bool(sensor_status & (1 << 3)),
            "sensor_rangefinder_present": bool(sensor_status & (1 << 4)),
            "sensor_gyro_present":        bool(sensor_status & (1 << 5)),
            "i2c_error_count":  getattr(state, "i2c_error_count",  0),
            "cpu_load_percent": getattr(state, "cpu_load_percent", 0),
            "pid_profile":      getattr(state, "pid_profile",      0),

            # ── Arming diagnostics ────────────────────────────────────────────
            "arming_disable_flags": getattr(state, "arming_disable_flags", 0),
            "arming_disable_str":   getattr(state, "arming_disable_str",   ""),

            # ── GPS ───────────────────────────────────────────────────────────
            **gps_data,

            # ── Source + ELRS extras ──────────────────────────────────────────
            **elrs_data,
        }


def _empty_link_status(radio_present: bool) -> dict:
    """Defaults for get_link_status() before the worker's first pass."""
    return {
        "active_source": "NONE",
        "usb_connected": False,
        "usb_healthy": False,
        "radio_present": radio_present,
        "radio_status": "NO_RADIO" if radio_present else "DISABLED",
        "radio_port": "",
        "link_stats_valid": False,
        "uplink_lq": 0, "uplink_rssi1_dbm": 0, "uplink_rssi2_dbm": 0, "uplink_snr": 0,
        "downlink_lq": 0, "downlink_rssi_dbm": 0, "tx_power_mw": 0,
        "age_attitude_ms": -1, "age_gps_ms": -1, "age_battery_ms": -1,
        "age_flight_mode_ms": -1, "age_last_frame_ms": -1,
        "rate_attitude_hz": 0.0, "rate_gps_hz": 0.0, "rate_battery_hz": 0.0,
        "rate_total_hz": 0.0,
        "reconnect_count": 0, "link_lost_count": 0, "crc_errors": 0,
        "radio_armed": False,
    }