# `SerialPortScan` — COM port enumeration & FC detection

**Files:** `SerialPortScan.h`, `SerialPortScan.cpp`
**Used by:** `CrsfLink` (radio detection), `DroneCockpitApp` via
`auto_detect_fc()` (flight-controller detection)
**Exposed to Python as:** `DroneBackend.SerialPortInfo`,
`DroneBackend.enumerate_serial_ports()`, `DroneBackend.auto_detect_fc()`
(see [bindings.md](bindings.md#module-level-free-functions))
**Links against:** `setupapi.lib` (pulled in by a `#pragma comment` on
MSVC)

## Why it exists

With the RadioMaster Pocket plugged in, the laptop has **two STM32 USB
serial devices**: the F405 flight controller and the radio. Both usually
report the same VID:PID (`0483:5740`, "STMicro Virtual COM Port"). The old
`AutoDetectF405()` returns the first COM port that opens, and that can now
be the Pocket. This module fixes that problem in two ways:

- it reads each port's **USB product string** so the devices can be told
  apart by name, and
- it offers a flight-controller detector that accepts a port **only if it
  answers an MSP request**, so it can never pick the radio.

## API

```cpp
struct SerialPortInfo {
    std::string port;            // "COM7"
    std::string friendlyName;    // Device Manager name, e.g. "USB Serial Device (COM7)"
    std::string busDescription;  // USB iProduct string, e.g. "Betaflight STM32F405" / "EdgeTX ..."
    uint16_t    vid, pid;
};

std::vector<SerialPortInfo> EnumerateSerialPorts();
bool        PortMatchesHints(const SerialPortInfo&, const std::vector<std::string>& hints);
bool        SerialPortExists(const std::string& portName);
std::string AutoDetectFlightController(const std::vector<std::string>& excludePorts = {},
                                       int timeoutMs = 300);
std::vector<std::string> DefaultRadioPortHints();   // {"edgetx","opentx","radiomaster","pocket"}
```

| Function | What it does |
|---|---|
| `EnumerateSerialPorts()` | Uses SetupAPI to list every **present** device in the *Ports* class (`GUID_DEVCLASS_PORTS`). For each one it reads `PortName` from the device registry key, the friendly name, VID/PID from the hardware ID, and `DEVPKEY_Device_BusReportedDeviceDesc`, which is the USB product string and the useful field. Non-COM entries (LPT) are skipped. The result is sorted by port number. |
| `PortMatchesHints()` | Case-insensitive substring search of each hint in `friendlyName + busDescription`. |
| `SerialPortExists()` | Cheap `QueryDosDevice` check. `CrsfLink` calls it once a second to notice a USB unplug even when the driver keeps the old handle "valid". |
| `AutoDetectFlightController()` | Tries ports that call themselves *Betaflight* first. It skips excluded ports, ports matching the radio hints, and Bluetooth ports. It opens each remaining port at 57600 8N1 with DTR/RTS off, sends `MSP_API_VERSION` (`$M<` len 0, cmd 1), and returns the first port that replies with `$M>` and command 1 within `timeoutMs`. Returns `"NOT_FOUND"` if none does. |

## How the cockpit uses it

- `DroneCockpitApp._auto_connect()` calls `auto_detect_fc(exclude)` on a
  background thread and passes the radio's current port in `exclude`. It
  falls back to `auto_detect_f405()` only on an older `.pyd` without the
  new binding. After a successful USB connect it calls
  `radio.set_excluded_ports([usb_port])`, so the two scanners never fight
  over the same port.
- `CrsfLink::scanForRadio()` uses `EnumerateSerialPorts()` and
  `PortMatchesHints()` to prefer the Pocket by name (see
  [crsflink.md](crsflink.md#port-selection--scanforradio)).

## Cost & caveats

- `AutoDetectFlightController()` blocks for up to `timeoutMs` (300 ms)
  **per candidate port**. Never call it from the Tk thread. The cockpit
  runs it on its own `UsbProbe` thread, and the binding releases the GIL.
- A port that is already open by another process (or by `CrsfLink`) fails
  `CreateFile` and is skipped silently. That is expected, not an error.
- `AutoDetectF405()` (in `DroneLink`) is still bound for backward
  compatibility, but new code should use `auto_detect_fc()`.
