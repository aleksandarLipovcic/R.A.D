#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0601
#endif
#include "SerialPortScan.h"
#include <windows.h>
#include <setupapi.h>
#include <devpropdef.h>
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <chrono>
#include <thread>

#ifdef _MSC_VER
#pragma comment(lib, "setupapi.lib")
#endif

namespace {
    // GUID_DEVCLASS_PORTS {4D36E978-E325-11CE-BFC1-08002BE10318}
    const GUID kPortsClass = { 0x4D36E978, 0xE325, 0x11CE,
        { 0xBF, 0xC1, 0x08, 0x00, 0x2B, 0xE1, 0x03, 0x18 } };

    // DEVPKEY_Device_BusReportedDeviceDesc {540b947e-8b40-45bc-a8a2-6a0b894cbda2}, 4
    const DEVPROPKEY kBusReportedDesc = {
        { 0x540B947E, 0x8B40, 0x45BC, { 0xA8, 0xA2, 0x6A, 0x0B, 0x89, 0x4C, 0xBD, 0xA2 } }, 4 };

    std::string lower(std::string s) {
        std::transform(s.begin(), s.end(), s.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        return s;
    }

    std::string wideToUtf8(const wchar_t* w) {
        if (!w || !*w) return {};
        int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
        std::string s(n > 0 ? n - 1 : 0, '\0');
        if (n > 1) WideCharToMultiByte(CP_UTF8, 0, w, -1, &s[0], n, nullptr, nullptr);
        return s;
    }

    std::string regProp(HDEVINFO set, SP_DEVINFO_DATA& dev, DWORD prop) {
        char buf[512] = {};
        DWORD type = 0;
        if (SetupDiGetDeviceRegistryPropertyA(set, &dev, prop, &type,
                reinterpret_cast<PBYTE>(buf), sizeof(buf) - 1, nullptr))
            return buf;   // REG_MULTI_SZ: first string is enough
        return {};
    }

    uint16_t hexAfter(const std::string& s, const char* key) {
        size_t p = s.find(key);
        if (p == std::string::npos) return 0;
        return static_cast<uint16_t>(std::strtoul(s.substr(p + std::strlen(key), 4).c_str(), nullptr, 16));
    }

    int portNumber(const std::string& p) {
        return (p.size() > 3) ? std::atoi(p.c_str() + 3) : 0;
    }

    bool isExcluded(const std::string& port, const std::vector<std::string>& ex) {
        const std::string lp = lower(port);
        for (const auto& e : ex) if (lower(e) == lp) return true;
        return false;
    }
}

std::vector<std::string> DefaultRadioPortHints() {
    return { "edgetx", "opentx", "radiomaster", "pocket" };
}

bool PortMatchesHints(const SerialPortInfo& p, const std::vector<std::string>& hints) {
    const std::string hay = lower(p.friendlyName + " | " + p.busDescription);
    for (const auto& h : hints)
        if (!h.empty() && hay.find(lower(h)) != std::string::npos) return true;
    return false;
}

bool SerialPortExists(const std::string& portName) {
    char target[512];
    return QueryDosDeviceA(portName.c_str(), target, sizeof(target)) != 0;
}

std::vector<SerialPortInfo> EnumerateSerialPorts() {
    std::vector<SerialPortInfo> out;
    HDEVINFO set = SetupDiGetClassDevsA(&kPortsClass, nullptr, nullptr, DIGCF_PRESENT);
    if (set == INVALID_HANDLE_VALUE) return out;

    SP_DEVINFO_DATA dev = {};
    dev.cbSize = sizeof(dev);
    for (DWORD i = 0; SetupDiEnumDeviceInfo(set, i, &dev); ++i) {
        SerialPortInfo info;

        HKEY key = SetupDiOpenDevRegKey(set, &dev, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);
        if (key != INVALID_HANDLE_VALUE) {
            char name[64] = {};
            DWORD sz = sizeof(name) - 1, type = 0;
            if (RegQueryValueExA(key, "PortName", nullptr, &type,
                                 reinterpret_cast<LPBYTE>(name), &sz) == ERROR_SUCCESS)
                info.port = name;
            RegCloseKey(key);
        }
        if (info.port.rfind("COM", 0) != 0) continue;   // skip LPT etc.

        info.friendlyName = regProp(set, dev, SPDRP_FRIENDLYNAME);
        const std::string hwid = regProp(set, dev, SPDRP_HARDWAREID);
        info.vid = hexAfter(hwid, "VID_");
        info.pid = hexAfter(hwid, "PID_");

        wchar_t desc[256] = {};
        DEVPROPTYPE ptype = 0;
        if (SetupDiGetDevicePropertyW(set, &dev, &kBusReportedDesc, &ptype,
                reinterpret_cast<PBYTE>(desc), sizeof(desc) - sizeof(wchar_t), nullptr, 0))
            info.busDescription = wideToUtf8(desc);

        out.push_back(std::move(info));
    }
    SetupDiDestroyDeviceInfoList(set);

    std::sort(out.begin(), out.end(), [](const SerialPortInfo& a, const SerialPortInfo& b) {
        return portNumber(a.port) < portNumber(b.port);
    });
    return out;
}

// =============================================================================
// AutoDetectFlightController — MSP handshake, never picks the radio
// =============================================================================
std::string AutoDetectFlightController(const std::vector<std::string>& excludePorts, int timeoutMs) {
    auto ports = EnumerateSerialPorts();
    const auto radioHints = DefaultRadioPortHints();

    // Try ports that call themselves Betaflight first.
    std::stable_partition(ports.begin(), ports.end(), [](const SerialPortInfo& p) {
        return lower(p.busDescription + p.friendlyName).find("betaflight") != std::string::npos;
    });

    for (const auto& p : ports) {
        if (isExcluded(p.port, excludePorts)) continue;
        if (PortMatchesHints(p, radioHints)) continue;
        if (lower(p.friendlyName).find("bluetooth") != std::string::npos) continue;

        const std::string path = "\\\\.\\" + p.port;
        HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                               OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (h == INVALID_HANDLE_VALUE) continue;     // busy (e.g. already open) or gone

        DCB dcb = {}; dcb.DCBlength = sizeof(dcb);
        GetCommState(h, &dcb);
        dcb.BaudRate = 57600; dcb.ByteSize = 8; dcb.StopBits = ONESTOPBIT; dcb.Parity = NOPARITY;
        dcb.fDtrControl = DTR_CONTROL_DISABLE; dcb.fRtsControl = RTS_CONTROL_DISABLE;
        SetCommState(h, &dcb);
        COMMTIMEOUTS to = {};
        to.ReadIntervalTimeout = MAXDWORD; to.ReadTotalTimeoutMultiplier = MAXDWORD;
        to.ReadTotalTimeoutConstant = 20; to.WriteTotalTimeoutConstant = 50;
        SetCommTimeouts(h, &to);
        PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

        // MSP_API_VERSION (1), no payload: $ M < len=0 cmd=1 crc=1
        const uint8_t req[] = { '$', 'M', '<', 0x00, 0x01, 0x01 };
        DWORD wr = 0;
        WriteFile(h, req, sizeof(req), &wr, nullptr);

        bool ok = false;
        std::string rx;
        auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
        while (!ok && std::chrono::steady_clock::now() < deadline) {
            char buf[64]; DWORD rd = 0;
            if (ReadFile(h, buf, sizeof(buf), &rd, nullptr) && rd) {
                rx.append(buf, rd);
                size_t pos = rx.find("$M>");
                ok = pos != std::string::npos && rx.size() >= pos + 5 &&
                     static_cast<uint8_t>(rx[pos + 4]) == 0x01;
            }
        }
        CloseHandle(h);
        if (ok) return p.port;
    }
    return "NOT_FOUND";
}
