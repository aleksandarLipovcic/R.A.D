# Command uplink (laptop → drone) — design proposal

> **Status: not implemented.** This page records the requirements, the
> candidate designs and the bench tests that decide between them. Build
> it after the drone → laptop telemetry path has been verified on the real
> hardware (see [crsflink.md](crsflink.md)).

## Requirements

| # | Requirement | Priority |
|---|---|---|
| R1 | Gimbal pan/tilt from the laptop (camera aiming for detection) | high |
| R2 | Emergency actions from the laptop: trigger GPS Rescue (RTH), and a failsafe/land action | high |
| R3 | Simple movement commands ("move forward/left/…", "hold") | later |
| R4 | The pilot's RadioMaster Pocket keeps **final authority** at all times | mandatory |
| R5 | Losing the laptop (crash, USB unplug, app hang) must never affect flight | mandatory |
| R6 | Telemetry to the laptop (the existing `CrsfLink` path) keeps working | mandatory |

## The constraint that shapes everything

Today the Pocket's USB port runs in EdgeTX **USB-VCP = Telem Mirror**
mode. It is **output-only**: EdgeTX copies received telemetry to the
laptop and ignores anything the laptop writes. The laptop has no path to
the drone at all. Any uplink design therefore changes how the laptop is
connected to the Pocket, the drone, or both.

All data to the drone has to travel over the ELRS link the Pocket already
flies with. That link carries:

- **RC channels** at the full packet rate. This is how stick and switch
  positions reach Betaflight.
- **Low-rate "MSP over CRSF"**: CRSF frame types `0x7A` MSP_REQ, `0x7B`
  MSP_RESP and `0x7C` MSP_WRITE. Betaflight uses these for its TX Lua
  scripts (PID and VTX settings from the radio). They are only a few
  frames per second and not suitable for continuous control.

## Candidate designs

### A. EdgeTX Lua bridge on the Pocket → extra RC channels (recommended to test first)

```
laptop ──USB-VCP (mode "LUA")──► Pocket Lua script ──► mixer outputs on AUX channels ══ELRS══► RP4TD ──► Betaflight
                                        │
                                        └──► telemetry back to the laptop (re-encoded by the script)
```

- Set the Pocket's USB-VCP to the Lua serial mode. A Lua script reads
  command packets from the laptop (`serialRead`), and a mixer script
  outputs them as **mix sources**. EdgeTX mixes put those on free AUX
  channels: gimbal pan/tilt, and a "laptop requests RTH" channel.
- Betaflight then treats them like any other switch or channel:
  - **Gimbal:** AUX channel → servo output, or the SimpleBGC controller's
    own RC input.
  - **RTH:** an AUX range in the Modes tab mapped to GPS RESCUE.
- **Authority (R4):** in the EdgeTX mixer, the laptop's channels are only
  used while a Pocket switch ("LAPTOP CTRL") is on, and the pilot's
  switches override them. Turning the switch off returns full control
  instantly, on the radio, without any software involved.
- **Watchdog (R5):** the Lua script falls back to neutral (gimbal centre,
  RTH request off) if no packet has arrived for ~500 ms.
- **Cost:** the VCP can run one mode at a time. Leaving Telem Mirror means
  the script must also send telemetry to the laptop, and `CrsfLink` then
  needs to accept that stream as a second input format.

**Unknowns (bench tests):** UT-UPL-A1 through A4 below. They cover whether
serial Lua is available in mixer scripts on the Pocket's EdgeTX version,
the script cycle time, and how much telemetry a Lua script can read and
forward.

### B. MSP over CRSF for one-shot commands

The Lua script (or a future EdgeTX feature) sends `MSP_WRITE` frames
through the Pocket's ELRS module to Betaflight.

- Good for **rare, discrete** commands, such as changing a setting or
  selecting a PID profile.
- Not suitable for continuous gimbal or movement control: the rate is too
  low and delivery isn't guaranteed.
- Betaflight's **MSP override** (`msp_override_channels_mask` plus the
  MSP OVERRIDE mode on a switch) lets `MSP_SET_RAW_RC` replace selected
  channels while the pilot holds that switch. In principle this is a path
  for R3 (movement), still under the pilot's switch. It needs a bench test
  with Betaflight 4.5 on the F405 (UT-UPL-B1).

### C. Separate link for the gimbal

Drive the SimpleBGC gimbal from its own link, for example a second
receiver channel or the controller's serial API through a companion
radio. This keeps the flight link untouched, but adds hardware. It's only
worth doing if A can't provide the gimbal channels.

## Safety rules (apply to any design)

1. **Never command throttle** from the laptop. Movement (R3) goes through
   Betaflight flight modes and small attitude offsets, never raw stick
   injection with throttle.
2. **Emergency = Betaflight's own modes.** RTH means GPS RESCUE and "land"
   means the configured failsafe. The laptop only *requests* them, and the
   requests use the same Modes-tab switches the pilot has.
3. **Pilot switch gates everything.** With the switch off, laptop input is
   ignored in the radio itself, not in the laptop software.
4. **Watchdog to neutral** within ≤ 500 ms of lost laptop packets.
5. **Visible state.** The cockpit shows whether laptop control is
   enabled, which is only known from telemetry (for example, the flight
   mode), and it logs every command sent.
6. **Rate-limit and range-check** every command before it leaves the
   laptop.

## Proposed software shape (once a design is chosen)

- **`CommandLink` (C++, next to `CrsfLink`)** owns the VCP when it is in
  Lua mode. It sends framed command packets (sequence number, command id,
  value, CRC) at a fixed rate and receives the telemetry stream for
  `CrsfLink`'s parser, so there is one port owner and two logical
  directions.
- **Python API:** `set_gimbal(pan_deg, tilt_deg)`, `request_rth()`,
  `cancel_request()`, `heartbeat()`. Every command is state ("what the
  laptop wants now"), resent periodically, never a one-shot, so a lost
  packet heals on its own.
- **UI:** a "Laptop control" panel. It is disabled on the USB cable link
  and while the pilot switch is off. RTH is a two-step confirm button.

## Bench tests that decide the design

| Test | Question | Pass criterion |
|---|---|---|
| UT-UPL-A1 | Does the Pocket's EdgeTX expose a Lua serial mode on USB-VCP, and can a script read laptop bytes? | Echo test: laptop → script → laptop |
| UT-UPL-A2 | Can a mixer script output values used as mix sources on AUX channels? | AUX value visible in Betaflight Receiver tab |
| UT-UPL-A3 | End-to-end latency, laptop command → Betaflight channel | < 100 ms |
| UT-UPL-A4 | Can the script forward the telemetry `CrsfLink` needs (attitude, GPS, battery, mode, LQ) at usable rates? | Attitude ≥ 5 Hz, GPS/battery ≥ 1 Hz |
| UT-UPL-B1 | Does MSP override work over CRSF on the F405 with Betaflight 4.5? | Overridden AUX follows `MSP_SET_RAW_RC` only while the switch is on |
| UT-UPL-S1 | Pilot switch off, or laptop unplugged mid-command | Channels back to pilot/neutral within 500 ms |
