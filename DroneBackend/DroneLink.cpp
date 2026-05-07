#include <windows.h>
#include <string>
#include <vector>
#include <iostream>

std::string AutoDetectF405() {
    char targetPort[10] = "";

    // We check COM ports 1 through 255
    for (int i = 1; i < 256; ++i) {
        std::string portName = "\\\\.\\COM" + std::to_string(i);

        // Try to open the port to see if it exists
        HANDLE hSerial = CreateFileA(portName.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

        if (hSerial != INVALID_HANDLE_VALUE) {
            // Port exists! For your Master's, we can later add a check
            // to verify it's an F405 by sending a small "Identify" MSP packet.
            CloseHandle(hSerial);
            return "COM" + std::to_string(i);
        }
    }
    return "NOT_FOUND";
}