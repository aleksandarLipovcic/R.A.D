#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "DroneLink.h"
#include <iostream>
#include <vector>

namespace py = pybind11;

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

// --- Telemetry Methods ---
std::vector<float> DroneLink::getAttitude() {
    if (!connected) return { 0.0f, 0.0f, 0.0f };

    uint8_t request[] = { '$', 'M', '<', 0, 108, 108 };
    DWORD written;
    // Fix for C6031: Check the return value of WriteFile
    if (!WriteFile(hSerial, request, sizeof(request), &written, NULL)) return { 0.0f, 0.0f, 0.0f };

    uint8_t rx[12];
    DWORD read;
    // Fix for C6031: Check the return value of ReadFile
    if (ReadFile(hSerial, rx, 12, &read, NULL) && read >= 11 && rx[4] == 108) {
        int16_t roll = (rx[5] | (rx[6] << 8));
        int16_t pitch = (rx[7] | (rx[8] << 8));
        int16_t yaw = (rx[9] | (rx[10] << 8));
        return { roll / 10.0f, pitch / 10.0f, (float)yaw };
    }
    return { 0.0f, 0.0f, 0.0f };
}

float DroneLink::getBatteryVoltage() {
    if (!connected) return 0.0f;

    uint8_t request[] = { '$', 'M', '<', 0, 110, 110 };
    DWORD written;
    if (!WriteFile(hSerial, request, sizeof(request), &written, NULL)) return -1.0f;

    uint8_t buffer[16];
    DWORD read;
    if (ReadFile(hSerial, buffer, 16, &read, NULL) && read >= 6) {
        return (float)buffer[5] / 10.0f;
    }
    return -1.0f;
}

// --- THE PYBIND11 MODULE ---
// Ensure this name matches your Project Name exactly
PYBIND11_MODULE(DroneBackend, m) {
    m.doc() = "Drone F405 MSP Bridge";

    py::class_<DroneLink>(m, "DroneLink")
        .def(py::init<>())
        .def("connect", &DroneLink::connect)
        .def("disconnect", &DroneLink::disconnect)
        .def("getAttitude", &DroneLink::getAttitude)
        .def("getBatteryVoltage", &DroneLink::getBatteryVoltage);
}