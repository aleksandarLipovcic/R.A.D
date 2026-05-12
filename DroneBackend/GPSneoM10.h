#pragma once
#include <cstdint>
#include <vector>

// ─────────────────────────────────────────────────────────────────────────────
// GPSReading — populated by GPSNeoM10::parseRaw() and GPSNeoM10::parseComp()
//
// Two MSP commands feed this struct:
//   MSP_RAW_GPS  (106) — fix, satellites, lat/lon, alt, speed, ground course
//   MSP_COMP_GPS (107) — distance-to-home, bearing-to-home, GPS heartbeat
//
// All fields are in SI-friendly units; raw ADC / protocol units are converted
// inside the parser so callers never need to know the wire format.
// ─────────────────────────────────────────────────────────────────────────────
struct GPSReading {
    // ── From MSP_RAW_GPS (106) ────────────────────────────────────────────────
    uint8_t  fixType = 0;       // 0 = no fix, 1 = 2D, 2 = 3D
    uint8_t  numSat = 0;       // satellites used in solution
    double   latitude = 0.0;     // decimal degrees  (+ = N, − = S)
    double   longitude = 0.0;     // decimal degrees  (+ = E, − = W)
    uint16_t altitudeM = 0;       // MSL altitude, metres
    uint16_t groundSpeedMs = 0;       // ground speed, cm/s  → divide by 100 for m/s
    uint16_t groundCourse = 0;       // ground course, decidegrees (0–3599)
    uint16_t hdop = 9999;    // horizontal dilution of precision × 100
    // 9999 = unknown; good fix < 200

// ── From MSP_COMP_GPS (107) ───────────────────────────────────────────────
    uint16_t distToHomM = 0;       // distance to home point, metres
    int16_t  bearingToHome = 0;       // bearing to home, degrees (−180 … +180)
    uint8_t  gpsHeartbeat = 0;       // toggles each time FC gets a fresh GPS frame

    // ── Validity flags ────────────────────────────────────────────────────────
    bool rawValid = false;   // true after first successful MSP_RAW_GPS parse
    bool compValid = false;   // true after first successful MSP_COMP_GPS parse
};

// ─────────────────────────────────────────────────────────────────────────────
// GPSNeoM10
//
// Stateless parser class — all methods are static, no internal state.
// Mirrors the BaroBMP280 interface so DroneLink can call it the same way.
//
// Betaflight 4.5.x MSP_RAW_GPS / MSP_COMP_GPS wire format is documented in
// src/main/msp/msp.c  (FC source) and is stable across 4.x releases.
// ─────────────────────────────────────────────────────────────────────────────
class GPSNeoM10 {
public:
    // ── MSP_RAW_GPS (106) ─────────────────────────────────────────────────────
    // Expected frame layout (header bytes [0..4] + payload + checksum):
    //   [0]      '$'
    //   [1]      'M'
    //   [2]      '>'
    //   [3]      payload_size  (should be 16 or 18 bytes depending on BF version)
    //   [4]      0x6A  (106)
    //   [5]      uint8_t   GPS fix type  (0/1/2)
    //   [6]      uint8_t   numSat
    //   [7..10]  int32_t   latitude   × 10 000 000  (little-endian)
    //   [11..14] int32_t   longitude  × 10 000 000  (little-endian)
    //   [15..16] uint16_t  altitude MSL, metres
    //   [17..18] uint16_t  ground speed, cm/s
    //   [19..20] uint16_t  ground course, decidegrees
    //   [21..22] uint16_t  HDOP × 100          (BF ≥ 4.1; absent in older builds)
    //   [last]   checksum
    //
    // Returns true if the buffer is a valid MSP_RAW_GPS response.
    static bool parseRaw(const std::vector<uint8_t>& buf, GPSReading& out);

    // ── MSP_COMP_GPS (107) ────────────────────────────────────────────────────
    // Expected frame layout:
    //   [0]      '$'
    //   [1]      'M'
    //   [2]      '>'
    //   [3]      0x05  payload size = 5 bytes
    //   [4]      0x6B  (107)
    //   [5..6]   uint16_t  distance to home, metres
    //   [7..8]   int16_t   bearing to home, degrees
    //   [9]      uint8_t   GPS heartbeat (toggles each new GPS frame)
    //   [10]     checksum
    //
    // Returns true if the buffer is a valid MSP_COMP_GPS response.
    static bool parseComp(const std::vector<uint8_t>& buf, GPSReading& out);

private:
    // Read little-endian integers from a byte buffer at offset i.
    static int16_t  read16(const std::vector<uint8_t>& b, int i);
    static uint16_t read16u(const std::vector<uint8_t>& b, int i);
    static int32_t  read32(const std::vector<uint8_t>& b, int i);
};