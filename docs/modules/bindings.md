# `Bindings.cpp` — the `DroneBackend` Python module

**File:** `Bindings.cpp`
**Defines:** `PYBIND11_MODULE(DroneBackend, m)`

This is the single place where every C++ class, struct, and enum described
in the other module docs gets exposed to Python. It owns no logic of its
own beyond field renaming (C++ `camelCase` → Python `snake_case`) and a
handful of small lambdas/trampolines for things pybind11 can't wrap
directly (numpy conversion, `to_dict()` convenience, etc.). Read this page
alongside the class it wraps — it doesn't repeat *why* a field exists, only
*how* it's reached from Python.

## GPS types

| Python | C++ | Notes |
|---|---|---|
| `DroneBackend.GPSReading` | `GPSReading` | read-only fields: `fix_type`, `num_sat`, `latitude`, `longitude`, `altitude_m`, `ground_speed_cms`, `ground_course`, `hdop`, `dist_to_home_m`, `bearing_to_home`, `gps_heartbeat`, `raw_valid`, `comp_valid`, `position_usable` |
| `DroneBackend.SVInfoEntry` | `SVInfoEntry` | one satellite channel: `chn`, `svid`, `flags`, `quality`, `cno`, `elev`, `azim`, `gnss_id`, `gnss_name`, `status_str`, `used` |
| `DroneBackend.NavStatus` | `NavStatus` | `fix_type`, `gps_flags`, `fix_ok`, `dgps_used`, `map_flags`, `hw_status`, `valid` |
| `DroneBackend.GPSConfig` | `GPSConfig` | read/write, constructible from Python (`py::init<>()`): `constellations`, `update_rate_hz`, `protocol`, `sbas_enabled`, `elevation_mask_deg`, `signal_mask_dbhz` — build one of these and pass it to `DroneLink.apply_gps_config()` |
| `DroneBackend.GPSConfigResult` | `GPSConfigResult` | read-only result of `apply_gps_config()`: `gnss_ack`, `rate_ack`, `protocol_ack`, `save_ack`, `overall_ok`, `error_detail` |
| `DroneBackend.GPSConstellationFlags` | `GPSConstellationFlags` | bound with `py::arithmetic()` so flags can be OR'd together from Python the same as in C++ |
| `DroneBackend.GPSUpdateRate` | `GPSUpdateRate` | plain enum |
| `DroneBackend.GPSProtocol` | `GPSProtocol` | plain enum |
| `DroneBackend.BatteryState` | `BatteryState` | plain enum: `OK`/`WARNING`/`CRITICAL`/`NOT_PRESENT`/`INIT` |

## `RadioLinkStatus` / `RadioLinkStats`

ELRS link types, filled only by `CrsfLink` (see
[crsflink.md](crsflink.md#link-status--radiolinkstatus)).

| Python | C++ | Notes |
|---|---|---|
| `DroneBackend.RadioLinkStatus` | `RadioLinkStatus` | enum, exported to module scope: `NO_RADIO`, `WAITING`, `TELEMETRY_OK`, `DEGRADED`, `TELEMETRY_LOST` |
| `DroneBackend.RadioLinkStats` | `RadioLinkStats` | read-only. **Status:** `status`, `status_str`, `port_name`. **Link statistics:** `link_stats_valid`, `uplink_rssi1_dbm`, `uplink_rssi2_dbm`, `uplink_lq`, `uplink_snr`, `active_antenna`, `rf_mode_index`, `tx_power_mw`, `downlink_rssi_dbm`, `downlink_lq`, `downlink_snr`. **Ages (ms, −1 = never):** `ms_since_last_frame`, `attitude_age_ms`, `gps_age_ms`, `battery_age_ms`, `flight_mode_age_ms`, `baro_age_ms`, `link_stats_age_ms`. **Rates (Hz):** `attitude_hz`, `gps_hz`, `battery_hz`, `flight_mode_hz`, `baro_hz`, `vario_hz`, `link_stats_hz`, `total_frame_hz`. **Counters:** `frames_total`, `crc_errors`, `reconnect_count`, `link_lost_count`. **CRSF hints:** `raw_flight_mode`, `arming_blocked`, `gps_waiting`. **Home:** `home_set`, `home_lat`, `home_lon`, `home_alt_m`. **Altitude:** `altitude_source` (`"BARO"`/`"GPS"`/`"NONE"`) |
| `DroneBackend.SerialPortInfo` | `SerialPortInfo` | read-only: `port`, `friendly_name`, `bus_description` (USB product string), `vid`, `pid`; has a readable `__repr__`. See [serial-port-scan.md](serial-port-scan.md) |

## `DroneState` / `DroneLink` / `IMUSensor`

`DroneBackend.DroneState` exposes essentially every field described in
[dronelink.md](dronelink.md#dronestate--the-shared-snapshot) as
`def_readonly`, `snake_case`-named — IMU raw counts (`ax`…`gz`), attitude
(`roll`/`pitch`/`yaw`), power (`battery_voltage`, `battery_current`,
`battery_mah_drawn`, `rssi`), extended battery detail
(`battery_cell_count`, `battery_capacity_mah`, `battery_percentage`,
`battery_state`), barometer (`baro_altitude_cm`, `baro_vario_cm_per_sec`,
`baro_valid`), magnetometer (`mag_x/y/z`, `mag_heading_deg`, `mag_valid`),
calibration progress (`mag_cal_active`, `mag_cal_seconds_remaining`,
`acc_cal_active`, `acc_cal_seconds_remaining`), FC status (`armed`,
`flight_mode_flags`, `flight_mode_name`, `sensor_status`,
`i2c_error_count`, `cpu_load_percent`, `pid_profile`), arming diagnostics
(`arming_disable_flags`, `arming_disable_str`), motor/RC counts
(`motor_count`, `rc_channel_count` — the underlying arrays are accessed
through `to_dict()`, see below), nested GPS objects (`gps`, `sv_list`,
`sv_info_valid`, `sv_source`, `nav_status`), link diagnostics
(`last_rtt_ms`, `fc_cycle_ms`, `link_healthy`, `packet_count`), and the
telemetry source (`link_source`, `radio`).

A `.def("to_dict", [](const DroneState& s) {...})` lambda is provided as a
convenience so Python code can pull a plain `dict` snapshot (including the
fixed-size C arrays like `motor_values`/`rc_channels`, which aren't
directly exposable as `def_readonly` array fields) instead of touching
every attribute individually.

### `DroneState` additions for the radio link

- `link_source`: `"USB"` (DroneLink, MSP) or `"ELRS"` (CrsfLink, CRSF).
- `radio`: a `RadioLinkStats`, meaningful only when
  `link_source == "ELRS"`.

`to_dict()` now builds the dict first and then appends these keys:

| Key(s) | Meaning |
|---|---|
| `link_source` | as above |
| `available_raw_imu`, `available_raw_mag`, `available_motors`, `available_rc_channels`, `available_sat_list`, `available_hdop`, `available_arming_flags`, `available_cpu_load` | `True` on USB, `False` on ELRS. Widgets should show "USB only" instead of zeros |
| `radio_status`, `radio_status_int`, `radio_port` | link status string / int, Pocket COM port |
| `elrs_link_stats_valid`, `elrs_uplink_lq`, `elrs_uplink_rssi1_dbm`, `elrs_uplink_rssi2_dbm`, `elrs_uplink_snr`, `elrs_active_antenna`, `elrs_rf_mode_index`, `elrs_tx_power_mw`, `elrs_downlink_rssi_dbm`, `elrs_downlink_lq`, `elrs_downlink_snr` | ELRS `LINK_STATISTICS` |
| `age_last_frame_ms`, `age_attitude_ms`, `age_gps_ms`, `age_battery_ms`, `age_flight_mode_ms`, `age_baro_ms`, `age_link_stats_ms` | data ages, −1 = never |
| `rate_attitude_hz`, `rate_gps_hz`, `rate_battery_hz`, `rate_flight_mode_hz`, `rate_baro_hz`, `rate_vario_hz`, `rate_link_stats_hz`, `rate_total_hz` | measured update rates |
| `radio_frames_total`, `radio_crc_errors`, `radio_reconnect_count`, `radio_link_lost_count` | counters |
| `raw_flight_mode`, `arming_blocked`, `gps_waiting`, `home_set`, `home_lat`, `home_lon`, `altitude_source` | CRSF hints and laptop-side home point |

`rc_link_quality` in `to_dict()` is the real ELRS uplink LQ % when
`link_source == "ELRS"` and link statistics are valid. Otherwise the old
rssi-based value is used.

`DroneBackend.DroneLink`:

| Python method | Wraps |
|---|---|
| `DroneLink()` | `py::init<>()` |
| `connect(port_name)` | `DroneLink::connect` |
| `disconnect()` | `DroneLink::disconnect` |
| `is_connected()` | `DroneLink::isConnected` |
| `get_latest_state()` | `DroneLink::getLatestState` — returns a `DroneState` copy |
| `set_poll_interval_ms(ms)` | `DroneLink::setPollIntervalMs` |
| `set_gps_uart_index(idx)` | `DroneLink::setGpsUartIndex` |
| `start_mag_calibration()` | `DroneLink::startMagCalibration` |
| `start_acc_calibration()` | `DroneLink::startAccCalibration` |
| `apply_gps_config(cfg)` | `DroneLink::applyGPSConfig` — takes a `GPSConfig`, returns `GPSConfigResult` |
| `set_fail_injection(active)` | `DroneLink::setFailInjection` — **test-only**, see [dronelink.md](dronelink.md#test--fault-injection-hooks) |

`DroneBackend.IMUSensor`:

| Python method | Wraps |
|---|---|
| `IMUSensor(hub)` | `py::init<DroneLink*>()`, `hub` is a positional/keyword arg |
| `get_raw_data()` | lambda around `IMUSensor::getRawData()`, converts `IMUData` to a Python-friendly return |
| `get_scaled_data()` | lambda around `IMUSensor::getScaledData()`, converts `IMUScaled` similarly |

## `CrsfLink`

`DroneBackend.CrsfLink`, the ELRS telemetry link (see
[crsflink.md](crsflink.md)). Every blocking call releases the GIL
(`py::call_guard<py::gil_scoped_release>`) so the Tk UI keeps running.

| Python method | Wraps |
|---|---|
| `CrsfLink()` | `py::init<>()` |
| `start_auto()` | `startAuto` — non-blocking, auto-detect + auto-reconnect. **Preferred.** *(releases GIL)* |
| `connect_auto(timeout_ms=3000)` | `connectAuto` — blocking scan *(releases GIL)* |
| `connect(port_name)` | `connect` — fixed port, re-opened after unplug *(releases GIL)* |
| `disconnect()` | `disconnect` *(releases GIL)* |
| `is_connected()` | `isConnected` — Pocket's port open; says nothing about the drone |
| `is_running()` | `isRunning` |
| `get_latest_state()` | `getLatestState` — `DroneState` copy with `link_source == "ELRS"` |
| `get_status()` | `getStatus` → `RadioLinkStatus` |
| `get_port_name()` | `getPortName` |
| `get_last_scan_report()` | `getLastScanReport` — text log of the last port scan |
| `set_excluded_ports(ports)` | `setExcludedPorts` |
| `set_port_name_hints(hints)` | `setPortNameHints` |
| `set_scan_interval_ms(ms)`, `set_probe_ms(ms)`, `set_assert_dtr(on)` | scanner tuning |
| `set_lost_timeout_ms(ms)`, `set_degraded_lq(lq_percent)`, `set_instrument_stale_ms(ms)` | status thresholds |
| `set_yaw_signed(signed)`, `set_cell_count(cells)`, `set_cell_voltage_thresholds(warning_v, critical_v)`, `set_battery_capacity_mah(mah)`, `set_min_sats_for_fix(sats)`, `set_gps_altitude_fallback(on)` | mapper configuration |
| `CrsfLink.enumerate_ports()` (static) | `EnumerateSerialPorts()` |

## `VideoLink`

`DroneBackend.CaptureDeviceInfo` — `index`, `name` (both read-only).

`DroneBackend.OsdAnchor` — enum mirroring `VideoLink::OsdAnchor`
(`TopLeft` … `BottomRight`, `Custom`).

`DroneBackend.OsdElementLayout` — read-only: `enabled`, `anchor`,
`margin_x`, `margin_y`, `custom_fx`, `custom_fy`.

`DroneBackend.VideoLink`:

| Python method | Wraps | Notes |
|---|---|---|
| `VideoLink()` | `py::init<>()` | |
| `connect_auto()` | `connectAuto` | |
| `connect(device_index)` | `connect` | |
| `disconnect()` | `disconnect` | Releases the GIL: it joins the capture thread, which calls the Python OSD telemetry provider |
| `is_connected()` | `isConnected` | |
| `get_latest_frame()` | lambda: `matToNumpy(v.getLatestFrame())` | The one place a `cv::Mat` is converted to a numpy array for Python — used for anything **other** than the live feed itself (e.g. recording), since the live feed bypasses Python via `attach_to_window` |
| `get_frame_count()` | `getFrameCount` | |
| `get_measured_fps()` | `getMeasuredFps` | |
| `get_device_name()` | `getDeviceName` | |
| `set_preferred_resolution(w, h)` | `setPreferredResolution` | |
| `attach_to_window(parent_hwnd, x, y, w, h)` | `attachToWindow` | `parent_hwnd` is the integer from Tk's `widget.winfo_id()` |
| `resize_window(w, h)` | `resizeWindow` | |
| `detach_window()` | `detachWindow` | |
| `is_window_attached()` | `isWindowAttached` | |
| `raise_window()` / `lower_window()` / `show_window()` / `hide_window()` | `raiseWindow`/`lowerWindow`/`showWindow`/`hideWindow` | Must stay bound — see [videolink.md](videolink.md#native-direct-to-window-rendering) for why an unbound one silently breaks Z-order sync |
| `set_osd_overlay_enabled(enabled)` / `is_osd_overlay_enabled()` | `setOsdOverlayEnabled`/`isOsdOverlayEnabled` | |
| `set_osd_locked(locked)` / `is_osd_locked()` | `setOsdLocked`/`isOsdLocked` | |
| `get_osd_element_ids()` | `getOsdElementIds` | |
| `set_osd_element_enabled(id, enabled)` | `setOsdElementEnabled` | |
| `set_osd_element_anchor(id, anchor, margin_x, margin_y)` | `setOsdElementAnchor` | |
| `set_osd_element_custom_position(id, fx, fy)` | `setOsdElementCustomPosition` | |
| `get_osd_element_layout(id)` | `getOsdElementLayout` → `OsdElementLayout` | |
| `set_telemetry_provider(callable)` | `setTelemetryProvider` | Python callable returning something convertible to `TelemetrySnapshot` |
| `reset_flight_timer()` | `resetFlightTimer` | |

## `TelemetrySnapshot`

`DroneBackend.TelemetrySnapshot` — constructible from Python
(`py::init<>()`), all fields **read/write** (`def_readwrite`, not
readonly, since Python code builds these to feed into
`set_telemetry_provider`): `valid`, `latitude`, `longitude`, `altitude_m`,
`heading_deg`, `roll_deg`, `pitch_deg`, `gimbal_pan_deg`,
`gimbal_tilt_deg`, `battery_voltage`, `battery_percentage`, `rssi`,
`gps_fix_type`, `gps_num_sat`, `home_distance_m`, `ground_speed_ms`,
`armed`.

## `DetectionLink`

`DroneBackend.DetectionRecord` — read-only mirror of the C++ struct (see
[detectionlink.md](detectionlink.md#telemetrysnapshot-and-detectionrecord)):
`id`, `timestamp_ms`, `class_name`, `confidence`, `track_id`, `sightings`, `best_confidence`, `uncertainty_m`, `bbox_x/y/w/h`,
`latitude`, `longitude`, `georeferenced`, `range_method`, `distance_m`,
`bearing_deg`, `screenshot_path`, `telemetry` (nested `TelemetrySnapshot`).

`DroneBackend.DetectionLink`:

| Python method | Wraps |
|---|---|
| `DetectionLink()` | `py::init<>()` |
| `set_video_link_source(video_link)` | `setVideoLinkSource` |
| `set_telemetry_provider(callable)` | `setTelemetryProvider` |
| `set_model_path(path)` | `setModelPath` |
| `set_screenshot_dir(dir)` | `setScreenshotDir` |
| `set_horizontal_fov_deg(fov)` | `setHorizontalFovDeg` |
| `set_detection_interval_ms(ms)` | `setDetectionIntervalMs` |
| `set_confidence_threshold(t)` | `setConfidenceThreshold` |
| `set_class_confidence_threshold(name, t)` / `clear_class_confidence_thresholds()` | `setClassConfidenceThreshold` / `clearClassConfidenceThresholds` |
| `set_input_size(size)` | `setInputSize` |
| `set_use_cuda(enabled)` / `is_using_cuda()` | `setUseCuda`/`isUsingCuda` |
| `set_use_cuda_fp16(enabled)` / `is_using_cuda_fp16()` | `setUseCudaFp16`/`isUsingCudaFp16` |
| `set_tiling(enabled)` / `is_tiling()` / `is_tiling_active()` | `setTiling`/`isTiling`/`isTilingActive` |
| `set_tiling_budget_ms(ms)` | `setTilingBudgetMs` |
| `set_max_duty_cycle(d)` | `setMaxDutyCycle` |
| `set_known_object_width(class_name, width_m)` | `setKnownObjectWidth` |
| `clear_known_object_widths()` | `clearKnownObjectWidths` |
| `set_min_ground_ray_component(v)` | `setMinGroundRayComponent` |
| `set_triangulation_min_baseline_m(m)` | `setTriangulationMinBaselineM` |
| `set_triangulation_min_bearing_spread_deg(deg)` | `setTriangulationMinBearingSpreadDeg` |
| `set_track_iou_threshold(v)` | `setTrackIouThreshold` |
| `set_track_max_missed_passes(n)` | `setTrackMaxMissedPasses` |
| `set_track_lost_memory_ms(ms)` | `setTrackLostMemoryMs` |
| `set_track_low_confidence(c)` | `setTrackLowConfidence` |
| `set_track_confirm_hits(n)` | `setTrackConfirmHits` |
| `set_track_reid_radius_m(m)` | `setTrackReidRadiusM` |
| `set_track_reid_window_ms(ms)` | `setTrackReidWindowMs` |
| `set_track_camera_motion_compensation(on)` | `setTrackCameraMotionCompensation` |
| `set_track_move_threshold_m(m)` | `setTrackMoveThresholdM` |
| `set_track_refresh_interval_ms(ms)` | `setTrackRefreshIntervalMs` |
| `start()` / `stop()` / `is_running()` | `start`/`stop`/`isRunning` — `stop()` releases the GIL (it joins the inference thread, which calls the Python telemetry provider) |
| `get_detection_count()` | `getDetectionCount` |
| `get_last_pass_duration_ms()` | `getLastPassDurationMs` |
| `get_last_pass_timestamp_ms()` | `getLastPassTimestampMs` |
| `get_all_records()` | `getAllRecords` → `list[DetectionRecord]` |
| `get_records_since(since_id)` | `getRecordsSince` → `list[DetectionRecord]` |
| `get_frame_index_since(since_ms)` | `getFrameIndexSince` → `list[FrameIndexEntry]` (`timestamp_ms`, `track_ids`, `class_names`): confirmed objects per detection pass, for the scene search |
| `clear_records()` | `clearRecords` |
| `get_latest_annotated_frame_jpeg()` | lambda wrapping `getLatestAnnotatedFrameJpeg()` — returns raw JPEG bytes; releases the GIL; see [detectionlink.md](detectionlink.md#live-preview-exception) |

## Module-level free functions

| Python | C++ |
|---|---|
| `DroneBackend.enumerate_serial_ports()` | `EnumerateSerialPorts()` → `list[SerialPortInfo]`, with USB product names so the FC and the Pocket can be told apart |
| `DroneBackend.auto_detect_fc(exclude_ports=[], timeout_ms=300)` | `AutoDetectFlightController()` — returns the first port that answers an MSP request, or `"NOT_FOUND"`. Never picks the radio. *(releases GIL)* |
| `DroneBackend.auto_detect_f405()` | `AutoDetectF405()` — **legacy**: scans `COM1`–`COM29`, returns the first port that opens. With the Pocket plugged in this can return the radio; use `auto_detect_fc()` instead |

## Naming convention (for anyone adding a new binding)

Every binding in this file follows the same rename rule, with no
exceptions found in the current source: C++ `camelCase` members/methods
become Python `snake_case`. Struct/class names themselves are kept
identical (`DroneState`, `GPSReading`, `VideoLink`, …). When adding a new
`DroneState`/`DetectionRecord`/etc. field in C++, mirror it here with the
same `snake_case` transliteration so the convention holds across the whole
module.
