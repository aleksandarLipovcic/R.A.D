#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "DroneLink.h"
#include "IMUSensor.h"
#include <iostream>
#include <vector>
#include <string>
#include <chrono>

namespace py = pybind11;
using namespace pybind11::literals;

// --- Constructor & Destructor ---
DroneLink::DroneLink() {
    hSerial = INVALID_HANDLE_VALUE;
    connected = false;
}

DroneLink::~DroneLink() {
    disconnect();
}

// --- Connection Logic ---
bool DroneLink::connect(std::string portName) {
    std::string fullPath = "\\\\.\\" + portName;

    hSerial = CreateFileA(fullPath.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, NULL, OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL, NULL);

    if (hSerial == INVALID_HANDLE_VALUE) {
        connected = false;
        return false;
    }

    DCB dcbSerialParams = { 0 };
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return false;
    }

    dcbSerialParams.BaudRate = CBR_115200; // MSP Standard
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return false;
    }

    // Crucial for precise timing: reduce wait times
    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);

    connected = true;
    return true;
}

void DroneLink::disconnect() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
    }
    connected = false;
}

// --- Communication Methods ---
std::vector<uint8_t> DroneLink::sendRequest(uint8_t mspID) {
    if (!connected || hSerial == INVALID_HANDLE_VALUE) return {};

    uint8_t request[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written;
    if (!WriteFile(hSerial, request, sizeof(request), &written, NULL)) return {};

    uint8_t buffer[64];
    DWORD read;
    if (ReadFile(hSerial, buffer, sizeof(buffer), &read, NULL) && read > 5) {
        return std::vector<uint8_t>(buffer, buffer + read);
    }
    return {};
}

TelemetryResult DroneLink::sendRequestWithTiming(uint8_t mspID) {
    if (!connected) return { {}, 0.0 };

    auto start = std::chrono::high_resolution_clock::now();

    uint8_t request[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written;
    WriteFile(hSerial, request, sizeof(request), &written, NULL);

    uint8_t buffer[64];
    DWORD read;
    ReadFile(hSerial, buffer, sizeof(buffer), &read, NULL);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> elapsed = end - start;

    return { std::vector<uint8_t>(buffer, buffer + read), elapsed.count() };
}

// --- Standalone Helper ---
std::string AutoDetectF405() {
    for (int i = 1; i < 30; ++i) {
        std::string portName = "\\\\.\\COM" + std::to_string(i);
        HANDLE testHandle = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
        if (testHandle != INVALID_HANDLE_VALUE) {
            CloseHandle(testHandle);
            return "COM" + std::to_string(i);
        }
    }
    return "NOT_FOUND";
}

// --- The Bridge Module ---
PYBIND11_MODULE(DroneBackend, m) {
    m.doc() = "Project R.A.D Thesis Backend";

    py::class_<DroneLink>(m, "DroneLink")
        .def(py::init<>())
        .def("connect", &DroneLink::connect)
        .def("disconnect", &DroneLink::disconnect);

    py::class_<IMUSensor>(m, "IMUSensor")
        .def(py::init<DroneLink*>())
        // Standard high-speed data
        .def("getRawData", [](IMUSensor& self) {
        auto d = self.getRawData();
        return py::dict("ax"_a = d.accX, "ay"_a = d.accY, "az"_a = d.accZ,
            "gx"_a = d.gyroX, "gy"_a = d.gyroY, "gz"_a = d.gyroZ);
            })
        // Precision Thesis Measurement
        .def("getThesisData", [](IMUSensor& self, DroneLink* hub) {
        // 1. Get IMU Data and measure RTT
        auto imu_res = hub->sendRequestWithTiming(102); // MSP_RAW_IMU

        // 2. Get FC Internal Cycle Time (Requires debug_mode = CYCLETIME set in CLI)
        auto debug_res = hub->sendRequest(254); // MSP_DEBUG

        uint16_t fc_cycle_us = 0;
        if (debug_res.size() >= 7 && debug_res[4] == 254) {
            // Debug 0 index (bytes 5 and 6) contains cycle time
            fc_cycle_us = (debug_res[5] | (debug_res[6] << 8));
        }

        IMUSensor::IMUData d = { 0 };
        if (imu_res.data.size() >= 18 && imu_res.data[4] == 102) {
            d.accX = (imu_res.data[5] | (imu_res.data[6] << 8));
            d.accY = (imu_res.data[7] | (imu_res.data[8] << 8));
            d.accZ = (imu_res.data[9] | (imu_res.data[10] << 8));
            d.gyroX = (imu_res.data[11] | (imu_res.data[12] << 8));
            d.gyroY = (imu_res.data[13] | (imu_res.data[14] << 8));
            d.gyroZ = (imu_res.data[15] | (imu_res.data[16] << 8));
        }

        return py::dict(
            "data"_a = py::dict("ax"_a = d.accX, "ay"_a = d.accY, "az"_a = d.accZ,
                "gx"_a = d.gyroX, "gy"_a = d.gyroY, "gz"_a = d.gyroZ),
            "rtt_ms"_a = imu_res.latencyMs,
            "fc_cycle_ms"_a = fc_cycle_us / 1000.0 // Convert microseconds to ms
        );
            });

    m.def("AutoDetectF405", &AutoDetectF405);
}