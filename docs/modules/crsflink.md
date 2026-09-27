# `CrsfLink` — ELRS radio telemetry link

**Files:** `CrsfLink.h`, `CrsfLink.cpp`
**Depends on:** `CrsfProtocol` (framing + decoders), `CrsfStateMapper`
(frames → `DroneState`), `SerialPortScan` (COM port enumeration), and
`DroneLink.h` (for `DroneState` / `RadioLinkStats` / `RadioLinkStatus`)
**Exposed to Python as:** `DroneBackend.CrsfLink` (see
[bindings.md](bindings.md#crsflink))

## Responsibility

`CrsfLink` is the **second telemetry source** next to `DroneLink`. Where
`DroneLink` polls the flight controller over MSP through the USB-C cable
(bench/ground use), `CrsfLink` listens to CRSF telemetry that comes back
over the ExpressLRS radio link while the drone is flying:

```
Betaflight ──CRSF telemetry──► RP4TD (ELRS RX) ══ELRS 2.4 GHz══► RadioMaster Pocket (TX)
    Pocket EdgeTX:  SYS → Hardware → USB-VCP = "Telem Mirror"
    ──USB CDC──► laptop ──► CrsfLink ──► DroneState ──► cockpit widgets
```

EdgeTX copies every telemetry frame it receives from the ELRS module to its
USB virtual COM port unchanged. That port is output-only, so
**`CrsfLink` only listens.** Nothing is ever written to the Pocket's port,
and nothing can reach the drone this way. Sending commands to the drone is a
separate design proposal: [command-uplink.md](command-uplink.md).

It is built the same way as `DroneLink`: it owns its own worker thread and
its own lifecycle, and it publishes a thread-safe `DroneState` snapshot.
`getLatestState()` returns a copy whose `linkSource` is `"ELRS"` and whose
`radio` block (`RadioLinkStats`) is filled. Python decides which of the two
links feeds the instruments (see
[../frontend/workers.md](../frontend/workers.md#two-telemetry-sources--usb-vs-elrs)).

## Class layout

| Piece | Role |
|---|---|
| `CrsfLink` | Thread, port handling, auto-detect / reconnect, locking |
| `Crsf::StreamParser` | Turns the byte stream into CRC-checked frames ([crsf-protocol.md](crsf-protocol.md)) |
| `CrsfStateMapper` | Applies frames to `DroneState`, computes data ages, rates and link status ([crsf-protocol.md](crsf-protocol.md#crsfstatemapper--frames--dronestate)) |
| `SerialPortScan` | Lists COM ports with their USB product names ([serial-port-scan.md](serial-port-scan.md)) |

`CrsfProtocol` and `CrsfStateMapper` have no Win32 code, threads or I/O,
so they can be unit-tested without a radio. `CrsfLink` holds the only
Windows-specific code in the radio path.

## Connection lifecycle

There are three entry points. All of them call `disconnect()` first, so
calling one again restarts the link:

| Method | Blocking? | Behaviour |
|---|---|---|
| `startAuto()` | no | Starts the worker in **auto mode**: it scans for the Pocket, connects, and reconnects forever after an unplug. **The cockpit uses this one.** |
| `connectAuto(timeoutMs = 3000)` | yes, up to `timeoutMs` | Scans until the Pocket is found, then starts the worker in auto mode. Same contract as `VideoLink::connectAuto`, so call it from a background thread. |
| `connect(portName)` | yes (one open) | **Fixed-port mode**: opens exactly this port. After an unplug it re-opens only this port and scans no others. The worker starts even if the first open fails, so it keeps retrying. |
| `disconnect()` | yes (joins the thread) | Stops the worker, closes the port, marks the status `NO_RADIO`. The last known telemetry values stay in the snapshot. |

`isConnected()` means only that the Pocket's COM port is open. It says
nothing about the drone. Use `getStatus()` for that.

### Port selection — `scanForRadio()`

One pass over `EnumerateSerialPorts()`. Ports whose names match the hints
are tried first. Each port gets one of these results:

1. **Skipped:** the port is in `setExcludedPorts()` (the cockpit puts
   `DroneLink`'s port here), it is a Bluetooth port, or its USB name
   contains `betaflight` / `inav`. The scanner never opens a flight
   controller port.
2. **Accepted even if silent:** the port's USB name matches a hint
   (`setPortNameHints()`, default `edgetx`, `opentx`, `radiomaster`,
   `pocket`). This is so the radio is found while the drone is still
   powered off.
3. **Accepted only with proof:** any other port is kept only if at least 2
   valid CRSF frames arrive during the probe window (`setProbeMs()`, default
   500 ms).

Every scan writes a readable report (`getLastScanReport()`) listing each
port and why it was skipped or selected. Paste this report when debugging
detection.

### Worker loop — `workerLoop()`

```
not connected ──► scan (auto) / re-open (fixed)  every scanIntervalMs (default 1000 ms)
connected     ──► ReadFile (returns after ≤ 20 ms) ──► StreamParser.feed ──► mapper.apply(frame)
              ──► every 1 s: SerialPortExists(port)?   (unplug check)
              ──► every 100 ms: mapper.refresh()        (ages, rates, status)
```

- **Unplug detection:** the port is dropped if `ClearCommError` or
  `ReadFile` fails, or if the COM device no longer appears in
  `QueryDosDevice` (checked once a second). Windows sometimes keeps a dead
  handle "valid", which is why the second check exists. After a drop the
  worker waits 300 ms and then scans again. Each successful re-attach
  increments `reconnectCount`.
- **Wrong-port safety valve:** in auto mode, a port that was accepted
  **only** because it carried CRSF frames (no name match) is released and
  rescanned if it goes silent for 5 s (`WRONG_PORT_SILENCE_MS`). This
  stops the link from holding on to a port that might be the flight
  controller.
- **Last values are kept.** After `TELEMETRY_LOST`, or after the port
  drops, nothing in `DroneState` is cleared. In a search-and-rescue
  mission the last GPS position before a crash or link loss matters most.

### Serial settings

`openPort()` opens the port as `\\.\COMx` at 115200 8N1. USB CDC ignores
the baud rate, but it is set anyway. DTR is asserted by default
(`setAssertDtr(true)`, the same as pyserial) because some CDC firmwares
only start sending once the host raises DTR. Read timeouts are set so that
`ReadFile` returns as soon as any byte arrives, or after 20 ms.

## Link status — `RadioLinkStatus`

`CrsfStateMapper::evaluateStatus()` computes the status from the port state
and from the data ages in `RadioLinkStats`:

| Status | Meaning | Typical cause |
|---|---|---|
| `NO_RADIO` | Pocket's COM port is not open | Pocket unplugged, or its USB mode is not *USB Serial (VCP)* |
| `WAITING` | Port open, but no frame from the flight controller yet | Drone powered off, not bound, or USB-VCP is not set to *Telem Mirror* |
| `TELEMETRY_OK` | FC telemetry is arriving and fresh | — |
| `DEGRADED` | Frames arrive, but uplink LQ is below `degradedLq` **or** attitude is older than `instrumentStaleMs` | Long range, obstacles, antenna orientation |
| `TELEMETRY_LOST` | Telemetry arrived before, but the newest FC frame is older than `lostTimeoutMs`, **or** fresh link statistics report uplink LQ = 0 | Drone out of range, crash, or receiver lost |

"FC telemetry" means only the attitude, battery, flight-mode and GPS
frames. `LINK_STATISTICS` can be generated by the TX module on its own, so
it does not prove the drone is alive. Every transition from `OK` or
`DEGRADED` to `LOST` increments `linkLostCount`.

## Configuration

All setters can be called at any time from any thread.

| Setter | Default | Effect |
|---|---|---|
| `setExcludedPorts(ports)` | `[]` | Ports the scanner never opens (the FC's port) |
| `setPortNameHints(hints)` | `edgetx, opentx, radiomaster, pocket` | USB name fragments that identify the radio |
| `setScanIntervalMs(ms)` | 1000 (min 100) | Delay between scans while disconnected |
| `setProbeMs(ms)` | 500 (min 100) | How long an unnamed port is listened to during a scan |
| `setAssertDtr(on)` | `true` | Raise DTR when the port is opened |
| `setLostTimeoutMs(ms)` | 1500 | No FC telemetry for this long → `TELEMETRY_LOST` |
| `setDegradedLq(pct)` | 70 | Uplink LQ below this → `DEGRADED` |
| `setInstrumentStaleMs(ms)` | 1500 | Attitude older than this → `DEGRADED` |
| `setYawSigned(s)` | `true` | Yaw wire encoding; bench test **IT-ELRS-004** decides which one Betaflight 4.5 uses |
| `setCellCount(n)` | 0 (= auto) | Battery cell count. **Set 4 for the 4S pack on real missions** (see below) |
| `setCellVoltageThresholds(w, c)` | 3.5 V / 3.3 V | Per-cell `WARNING` / `CRITICAL` (Betaflight 4.5 defaults) |
| `setBatteryCapacityMah(mah)` | 0 | Only shown in the UI |
| `setMinSatsForFix(n)` | 5 | CRSF has no fix type or HDOP, so position is "usable" when there are at least this many satellites |
| `setGpsAltitudeFallback(on)` | `true` | If there is no baro frame, drive the altimeter from GPS altitude relative to home |

> **Cell-count auto-detect** uses `ceil(V / 4.35)` on the **first** voltage
> reading. If the radio connects mid-flight to a pack that is already
> sagging, the guess can be one cell low, which makes every battery warning
> wrong. Set the count explicitly for missions. The cockpit does this at
> startup from `RADIO_BATTERY_CELLS` (default 4) in `DroneCockpitUI.py`.

## Threading & locking

| Mutex | Guards |
|---|---|
| `cfgMutex_` | `fixedPort_`, `excluded_`, `hints_`, `lastScanReport_` |
| `stateMutex_` | `state_`, `mapper_` (including its `Config`), `thresholds_`, `portName_` |

Scalar settings (`scanIntervalMs_`, `probeMs_`, `assertDtr_`) and the
`running_` / `portOpen_` flags are `std::atomic`. `getLatestState()` and
`getStatus()` call `mapper_.refresh()` under the lock **before** copying,
so the ages Python reads are exact at the moment of the call, not up to
100 ms old.

## What the ELRS path cannot provide

CRSF telemetry carries far less than MSP. On this link the following
`DroneState` fields stay at zero or empty: raw IMU (`ax`..`gz`), raw
magnetometer (`magX/Y/Z`), motor outputs, RC channels, satellite list,
HDOP, the arming-disable bitmask (only "blocked yes/no" is available), CPU
load, I2C errors and cycle time. `to_dict()` and the Python worker publish
`available_*` flags so widgets can show "USB only" instead of a false zero
(see [bindings.md](bindings.md#dronestate-additions-for-the-radio-link)).
