#pragma once
// =============================================================================
// SerialPortScan — COM port enumeration with USB identity, and an
// MSP-verified flight-controller detector.
//
// WHY THIS EXISTS
//   With the Pocket plugged in there are now TWO STM32 USB serial devices on
//   the laptop (F405 FC and the radio), and both typically report the same
//   VID:PID (0483:5740, STMicro Virtual COM Port). The old AutoDetectF405()
//   returns "the first COM port that opens" — that can now be the Pocket.
//
//   • EnumerateSerialPorts() reads each port's USB product string (the
//     "bus reported device description", e.g. "Betaflight STM32F405" or
//     "EdgeTX …") so the two can be told apart by name.
//   • AutoDetectFlightController() only accepts a port that actually answers
//     an MSP request, so it can never pick the radio.
//
// Links against setupapi.lib (pulled in automatically on MSVC via #pragma).
// =============================================================================
#include <string>
#include <vector>
#include <cstdint>

struct SerialPortInfo {
    std::string port;             // "COM7"
    std::string friendlyName;     // Device Manager name, e.g. "USB Serial Device (COM7)"
    std::string busDescription;   // USB iProduct string — the useful one
    uint16_t    vid = 0;
    uint16_t    pid = 0;
};

// All present COM ports, sorted by port number.
std::vector<SerialPortInfo> EnumerateSerialPorts();

// Case-insensitive: does any hint appear in the port's names?
bool PortMatchesHints(const SerialPortInfo& p, const std::vector<std::string>& hints);

// True if the port name still refers to a present device (cheap; used to
// detect a USB unplug even if the driver keeps the handle "valid").
bool SerialPortExists(const std::string& portName);

// Scans ports (skipping `excludePorts` and anything matching `radioHints`),
// sends MSP_API_VERSION and returns the first port that replies "$M>".
// Returns "NOT_FOUND" if none answers. Blocking, ~timeoutMs per port.
std::string AutoDetectFlightController(const std::vector<std::string>& excludePorts = {},
                                       int timeoutMs = 300);

// Default name fragments that identify the radio (case-insensitive).
std::vector<std::string> DefaultRadioPortHints();
