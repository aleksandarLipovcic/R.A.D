#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "DroneLink.h"
#include "IMUSensor.h" // Include your new sensor header
#include <iostream>
#include <vector>
#include <string>

namespace py = pybind11;
using namespace pybind11::literals; // Enables the _a suffix for arguments

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

    dcbSerialParams.BaudRate = CBR_115200;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams)) {
        CloseHandle(hSerial);
        return false;
    }

    // Set timeouts to ensure ReadFile doesn't hang forever
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

// --- The "Hub" Method ---
// This is the single pipe that handles all MSP traffic
std::vector<uint8_t> DroneLink::sendRequest(uint8_t mspID) {
    if (!connected || hSerial == INVALID_HANDLE_VALUE) return {};

    // Header: [$, M, <, data_size, message_id, checksum]
    uint8_t request[] = { '$', 'M', '<', 0, mspID, mspID };
    DWORD written;

    if (!WriteFile(hSerial, request, sizeof(request), &written, NULL)) return {};

    uint8_t buffer[64]; // Large enough for most MSP responses
    DWORD read;

    if (ReadFile(hSerial, buffer, sizeof(buffer), &read, NULL) && read > 5) {
        return std::vector<uint8_t>(buffer, buffer + read);
    }
    return {};
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
    m.doc() = "Project R.A.D Modular Backend";

    // 1. Expose the Hub
    py::class_<DroneLink>(m, "DroneLink")
        .def(py::init<>())
        .def("connect", &DroneLink::connect)
        .def("disconnect", &DroneLink::disconnect);

    // 2. Expose the IMU Sensor
    py::class_<IMUSensor>(m, "IMUSensor")
        .def(py::init<DroneLink*>())
        .def("getRawData", [](IMUSensor& self) {
        auto d = self.getRawData();
        // Returning a dictionary makes it very easy to read in Python/YOLO
        return py::dict("ax"_a = d.accX, "ay"_a = d.accY, "az"_a = d.accZ,
            "gx"_a = d.gyroX, "gy"_a = d.gyroY, "gz"_a = d.gyroZ);
            });

    m.def("AutoDetectF405", &AutoDetectF405);
}