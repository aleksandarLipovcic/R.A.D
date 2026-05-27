# PROJECT R.A.D — IMU Podsistem (Aplikaciona Strana)
**V-Model | Lijeva Strana**

| | |
|---|---|
| **Fakultet** | Elektrotehnički fakultet, Univerzitet u Banjoj Luci |
| **Hardver** | XFlight Hobby F405 V3 — Betaflight 4.5.3 |
| **Obim** | Podsistemski V-Model — Zahtjevi → Implementacija → Verifikacija |
| **Status** | FINALNA v1.3 — Maj 2026 \| V-Model Završen \| Svi Testovi POLOŽENI |
| **Student** | Aleksandar Lipovcić — 2119/25 |

---

## 1. Uvod i Pregled Aplikacione Strane V-Modela

Ovaj dokument obuhvata lijevu stranu V-Modela za IMU podsistem unutar projekta. Dok desna strana pokriva verifikaciju i testiranje, ovaj se fokusira na razvoj i implementaciju: od zahtjeva do konkretnog koda koji obrađuje giroskopske i akcelerometarske podatke.

Aplikacija se sastoji od dva sloja: C++ backend (@ref DroneLink + [IMUSensor](@ref IMUSensor)), koji komunicira s flight kontrolerom putem MSP protokola, i Python frontend (GCS grafički interfejs), koji primljene podatke konvertuje i prikazuje operateru.

### 1.1 Pregled Toka Podataka — MPU-6500 do Prikaza

Ukupni pipeline od fizičkog senzora do widget-a prolazi kroz četiri sloja. Razumijevanje svakog sloja ključno je za analizu kašnjenja i pravilnu dodjelu odgovornosti.

| **Sloj** | **Komponenta** | **Jezik** | **MSP Komanda** | **Odgovornost** |
|---|---|---|---|---|
| L1 | [DroneLink::sendMSP(102)](@ref DroneLink::sendMSP) | C++17 | MSP_RAW_IMU (102) | Slanje zahtjeva, čitanje odgovora, sinhronizacija zaglavlja |
| L2 | [DroneLink::parseIMU()](@ref DroneLink::parseIMU) | C++17 | — | Validacija okvira, ekstrakcija 6×int16 u [DroneState](@ref DroneState) |
| L3 | [IMUSensor::getRawData()](@ref IMUSensor::getRawData) / [IMUSensor::getScaledData()](@ref IMUSensor::getScaledData) | C++17 | — | Thread-safe snapshot, konverzija jedinica (g, °/s) |
| L4 | Python `to_dict()` + `IMUWidget.update_ui()` | Python 3 / Tkinter | — | Evaluacija pragova, bojenje upozorenja, trepćuće ćelije |

> **📌** Svaki sloj ima jasno definisanu granicu odgovornosti. Greška skaliranja ili parsiranja u L2/L3 se direktno manifestuje na L4 prikazu — ovo je iskorišćeno u testovima IT-IMU-001 i IT-IMU-002.

### 1.2 Vremenski Budžet — Od Senzora do Prikaza

| **Faza** | **Min** | **Nom** | **Maks** |
|---|---|---|---|
| MPU-6500 → BF gyro loop (1 kHz) | ~1 ms | ~1 ms | ~2 ms |
| BF MSP handler (priprema odgovora) | ~0.5 ms | ~1 ms | ~2 ms |
| Serial TX: 18 bajta @ 57600 baud | ~4.2 ms | ~4.2 ms | ~5 ms |
| [sendMSP()](@ref DroneLink::sendMSP) read + sync | ~2 ms | ~5 ms | ~15 ms |
| [parseIMU()](@ref DroneLink::parseIMU) + [commitState()](@ref DroneLink::commitState) | <0.1 ms | <0.1 ms | <0.1 ms |
| Python polling + `to_dict()` + `update_ui()` | ~5 ms | ~10 ms | ~30 ms |
| **UKUPNO (end-to-end)** | **~11 ms** | **~20 ms** | **~52 ms** |

> **📌** Zahtjev SRS-IMU-007 propisuje < 200 ms end-to-end kašnjenje. Nominalno kašnjenje od ~20 ms je daleko unutar tog limita. Bottleneck je serial link na 57600 baud, a ne [parseIMU()](@ref DroneLink::parseIMU) ili Python obrada.

---

## 2. Backend Implementacija — C++ Slojevi

### 2.1 DroneLink::communicationLoop() — Upravljanje Petljom Pollinga

Radna nit ([communicationLoop](@ref DroneLink::communicationLoop)) izvodi 12 standardnih MSP upita pri svakom taktu od 10 ms (100 Hz nominalno), plus 2 throttled upita (SV_INFO svaki ~1 s, BATTERY_STATE svaki ~500 ms). MSP_RAW_IMU (komanda 102) je drugi upit u svakom taktu, odmah nakon MSP_STATUS.

#### 2.1.1 Redoslijed Upita po Taktu

| **Br.** | **MSP Komanda** | **Cmd ID** | **Svrha** |
|---|---|---|---|
| 1 | MSP_STATUS | 101 | FC cycle time, flight mode, armed, sensor status |
| 2 | MSP_RAW_IMU | 102 | **Sirovi akcelerometar + giroskop (MPU-6500)** |
| 3 | MSP_ATTITUDE | 108 | Spojeni roll / pitch / yaw (BF AHRS) |
| 4 | MSP_ANALOG | 110 | Napon baterije, struja, mAh, RSSI |
| 5 | MSP_DEBUG | 254 | Magnetometar X/Y/Z (debug_mode=MAG_CALIB) |
| 6 | MSP_ALTITUDE | 109 | BMP280 visina + variometar |
| 7 | MSP_RAW_GPS | 106 | GPS fix, sati, lat/lon, alt, brzina |
| 8–14 | ... | ... | Ostali upiti (COMP_GPS, NAV_STATUS, STATUS_EX, MOTOR, RC, SV_INFO, BATTERY_STATE) |

#### 2.1.2 Kritična Implementaciona Odluka — Uklanjanje PurgeComm

Originalna implementacija pozivala je `PurgeComm(PURGE_RXCLEAR)` na početku svakog [sendMSP()](@ref DroneLink::sendMSP) poziva. Pri 12+ uzastopnih upita na 100 Hz, purge narednog upita brisao je rep odgovora prethodnog upita koji još nije stigao putem serijskog FIFO bafera.

Ovo je posebno pogađalo MSP_RC (komanda 105) koji vraća 16 kanala = 32 bajta = 38-bajtni okvir. Pri 57600 baud, 38 bajta traje ~6.6 ms. Sljedeći [sendMSP()](@ref DroneLink::sendMSP) bio je pokretan unutar tog prozora, uništavajući RC okvir i ostavljajući `rcChannelCount = 0`.

> **📌 Popravka:** uklonjen je bezuslovni pre-poll purge. Petlja čitanja već iscrpljuje tačno `payloadLen+6` bajta (ili istekne). Eventualni zaostali garbage se odbacuje logikom sinhronizacije zaglavlja koja prihvata samo `'$','M','>'` sekvence.

---

### 2.2 DroneLink::sendMSP() — Transport Protokol

Metoda [sendMSP()](@ref DroneLink::sendMSP) implementira MSP v1 request/response ciklus sa sinhronizacijom zaglavlja i zaštitom od zaostalih okvira.

#### 2.2.1 Format MSP Zahtjeva

| **Bajt** | **Vrijednost** | **Opis** |
|---|---|---|
| 0 | `'$'` (0x24) | MSP preambula bajt 1 |
| 1 | `'M'` (0x4D) | MSP preambula bajt 2 |
| 2 | `'<'` (0x3C) | Smjer: Host → FC |
| 3 | `0x00` | `payloadLen` = 0 |
| 4 | `mspID` (npr. 102) | Identifikator komande |
| 5 | `mspID` | Kontrolna suma (XOR 0^mspID = mspID) |

#### 2.2.2 Logika Sinhronizacije Zaglavlja

Nakon slanja zahtjeva, [sendMSP()](@ref DroneLink::sendMSP) sinhronizuje se na `'$','M','>'` preambulu skeniranjem do `SYNC_TRIES` (64) bajta:

```cpp
while (syncTries < SYNC_TRIES) {
    if (!readByte(b0)) return {};
    if (b0 != '$') { ++syncTries; continue; }
    if (!readByte(b1)) return {};
    if (b1 != 'M') { ++syncTries; continue; }
    if (!readByte(b2)) return {};
    if (b2 == '>') break;  // pronađeno zaglavlje
    ++syncTries;
}
```

Provjera ispravnosti: `cmd` bajt mora odgovarati poslanom `mspID`. Neslaganje okida `PurgeComm` i vraća prazan vektor.

---

### 2.3 DroneLink::parseIMU() — Parser MSP Okvira

Parser za MSP_RAW_IMU okvir je minimalan i determinističan. Validira veličinu okvira i `cmd` bajt, zatim izdvaja 6 označenih 16-bitnih integera little-endian redoslijedom.

#### 2.3.1 Format MSP_RAW_IMU Odgovora (Komanda 102)

| **Pomak** | **Veličina** | **Tip** | **Polje** | **Opis** |
|---|---|---|---|---|
| 0 | 1 B | char | `'$'` | MSP preambula bajt 1 |
| 1 | 1 B | char | `'M'` | MSP preambula bajt 2 |
| 2 | 1 B | char | `'>'` | Smjer: FC → Host |
| 3 | 1 B | uint8 | `payloadLen` | Mora biti 12 |
| 4 | 1 B | uint8 | `cmd` | Mora biti 102 |
| 5–6 | 2 B | int16-LE | `ax` | ADC brojač X ose akcelerometra |
| 7–8 | 2 B | int16-LE | `ay` | ADC brojač Y ose akcelerometra |
| 9–10 | 2 B | int16-LE | `az` | ADC brojač Z ose akcelerometra |
| 11–12 | 2 B | int16-LE | `gx` | ADC brojač X ose giroskopa |
| 13–14 | 2 B | int16-LE | `gy` | ADC brojač Y ose giroskopa |
| 15–16 | 2 B | int16-LE | `gz` | ADC brojač Z ose giroskopa |
| 17 | 1 B | uint8 | `checksum` | XOR bajtova [3..16] |

#### 2.3.2 Implementacija parseIMU()

> **📌 Napomena o dizajnu (testabilnost):** [parseIMU()](@ref DroneLink::parseIMU) je deklarisan kao `protected` u `DroneLink.h` (umjesto `private`) isključivo radi testabilnosti. Jedinični testovi (UT-IMU-001 do UT-IMU-004) koriste subklasu `TestableDroneLink` koja nasljeđuje [DroneLink](@ref DroneLink) i poziva [parseIMU()](@ref DroneLink::parseIMU) direktno putem javne metode `callParseIMU()`. Testovi [IMUSensor](@ref IMUSensor) skaliranja (UT-SCALE-*, UT-RAW-001) koriste `MockDroneLink` koji nadjačava [getLatestState()](@ref DroneLink::getLatestState) vraćanjem injektovanog [DroneState](@ref DroneState)-a.

```cpp
bool DroneLink::parseIMU(const std::vector<uint8_t>& buf, DroneState& s) {
    if (buf.size() < 18 || buf[4] != MSP::RAW_IMU) return false;
    auto r16 = [&](int i) -> int16_t {
        return static_cast<int16_t>(buf[i] | (buf[i + 1] << 8)); };
    s.ax = r16(5);  s.ay = r16(7);  s.az = r16(9);
    s.gx = r16(11); s.gy = r16(13); s.gz = r16(15);
    return true;
}
```

> **📌** `buf.size() < 18` pokriva minimalni okvir (zaglavlje 5 + payload 12 + checksum 1 = 18 bajta). MPU-6500 nema interni magnetometar — MSP_RAW_IMU šalje isključivo acc[3] + gyro[3] = 6 × int16 = 12 bajta payload. Detalji ispravke originalnog 24-bajtnog pretpostavka dokumentovani su u DEF-006 (sekcija 6).

---

### 2.4 IMUSensor — Senzorski API Sloj

Klasa [IMUSensor](@ref IMUSensor) pruža thread-safe apstrakcijski sloj iznad [DroneLink::getLatestState()](@ref DroneLink::getLatestState). Implementira konverziju iz ADC brojeva u fizičke jedinice.

#### 2.4.1 Faktori Skaliranja — MPU-6500 na MSP Sloju

| **Osa grupa** | **BF opseg** | **HW osjetljivost** | **MSP-slojna konstanta** | **Izlazna jedinica** |
|---|---|---|---|---|
| Akcelerometar | ±16g | 2048 LSB/g | `ACC_SCALE = 1/2048`* | g (standard gravity) |
| Giroskop | ±2000 °/s | 16.4 LSB/(°/s) | `GYRO_SCALE = 1/16.4` | °/s |

*BF 4.5.x inicijalizuje MPU-6500 na ±16g opseg (INV_FSR_16G), hardware osjetljivost = 2048 LSB/g. Detalji ispravke originalnog `ACC_SCALE = 1/8192` dokumentovani su u DEF-002 (sekcija 6).

#### 2.4.2 getScaledData() Implementacija

```cpp
IMUSensor::IMUScaled IMUSensor::getScaledData() {
    DroneState state = drone->getLatestState();  // thread-safe copy
    IMUScaled d;
    d.accX  = state.ax * ACC_SCALE;   // 1/2048 → g
    d.accY  = state.ay * ACC_SCALE;
    d.accZ  = state.az * ACC_SCALE;
    d.gyroX = state.gx * GYRO_SCALE;  // 1/16.4 → °/s
    d.gyroY = state.gy * GYRO_SCALE;
    d.gyroZ = state.gz * GYRO_SCALE;
    return d;
}
```

---

## 3. Python Frontend — IMUWidget

### 3.1 IMUWidget.update_ui() — Prikaz i Evaluacija Pragova

`IMUWidget.update_ui(data)` prima rječnik iz [DroneState::to_dict()](@ref DroneState::to_dict) i izvodi sljedeće operacije pri svakom pozivu:

- Primjenjuje skaliranje: `ax / 2048` za g, `gx / 16.4` za °/s
- Evaluira prag upozorenja za svaku od 3 gyro ose nezavisno
- Trepće CRIT ćelije sinhrono na 1 Hz (500 ms ON/OFF) koristeći zajednički blink tajmer
- Ispisuje vrijednosti (2 decimale za rotaciju, 3 za G-silu, 1 za ugao)

### 3.2 Mašina Stanja — _gyro_state_label()

`_gyro_state_label()` upravlja SAFE/WARN/CRIT tranzicijama po osi sa histerezom:

| **Stanje** | **Prag** | **Hold tajmer** | **Vizualni prikaz** |
|---|---|---|---|
| SAFE | │rate│ < 30 °/s | — | Zelena |
| WARN | │rate│ ≥ 30 °/s | 2.0 s | Jantarna (puna) |
| CRIT | │rate│ ≥ 100 °/s | 4.0 s | Crvena trepćuća (1 Hz, DO-160) |

---

## 4. Sistemske Karakteristike i Performanse

### 4.1 Efektivna Stopa Pollinga

Pri 57600 baud, svaki bajt traje ~174 μs. MSP_RAW_IMU okvir (18 bajta) zahtijeva ~3.1 ms za serijski prenos. Sa FC kašnjenjem odgovora i jitterom, realna stopa je 3–6 Hz per-command u izolaciji.

> **📌** Radna nit šalje 12 uzastopnih MSP upita po taktu. Ukupno serijalno vrijeme za sve upite (~40 ms) je bottleneck. `POLL_INTERVAL_MS = 10 ms` je nominalni interval, ali stvarno izvršavanje jednog takta traje ~40 ms, što daje efektivnu stopu od ~25 Hz za cijelu petlju. MSP_RAW_IMU se osvježava ~25 puta u sekundi zajedno s ostalim upitima.

### 4.2 Thread Safety Implementacija

[DroneLink](@ref DroneLink) koristi single mutex (`dataMutex`) koji štiti `currentState`. Radna nit upisuje putem [commitState()](@ref DroneLink::commitState), a Python nit čita putem [getLatestState()](@ref DroneLink::getLatestState). Obje operacije su kratke kritične sekcije (copy semantika na POD strukturi).

#### 4.2.1 commitState() i getLatestState()

```cpp
void DroneLink::commitState(const DroneState& s) {
    std::lock_guard<std::mutex> lock(dataMutex);
    currentState = s;   // atomična kopija cijele strukture
}

DroneState DroneLink::getLatestState() {
    std::lock_guard<std::mutex> lock(dataMutex);
    return currentState;  // vraća kopiju — nikad referencu
}
```

> **📌** Radna nit radi sa lokalnom kopijom (`pending`) cijelo vrijeme petlje; [commitState()](@ref DroneLink::commitState) se poziva tek na kraju takta sa kompletiranim stanjem. Ovo minimizira konflikt na mutex-u.

### 4.3 RTT Mjerenje za IMU Upit

Kašnjenje round-trip-time za MSP_RAW_IMU upit mjeri se direktno u [communicationLoop()](@ref DroneLink::communicationLoop), koristeći `high_resolution_clock`:

```cpp
auto t0 = std::chrono::high_resolution_clock::now();
auto buf = sendMSP(MSP::RAW_IMU);
auto t1 = std::chrono::high_resolution_clock::now();
if (parseIMU(buf, pending)) {
    std::chrono::duration<double, std::milli> rtt = t1 - t0;
    pending.lastRttMs = rtt.count();
    anySuccess = true;
}
```

Izmjereni RTT obuhvata: serijska TX kašnjenja + FC handler kašnjenje + serijska RX kašnjenja. Tipično: 2–15 ms. Ovaj podatak je eksportovan u `to_dict()` pod ključem `"rtt_ms"` i prikazan u `IMUWidget`.

### 4.4 Link Health Monitoring

| **Konstanta/Stanje** | **Opis** |
|---|---|
| `FAIL_THRESHOLD = 5` | Broj uzastopnih neuspjeha [parseIMU()](@ref DroneLink::parseIMU) koji okida link unhealthy |
| `consecutiveFails++` | Inkrementira se kada [parseIMU()](@ref DroneLink::parseIMU) vrati `false` |
| `consecutiveFails = 0` | Resetuje se pri prvom uspješnom [parseIMU()](@ref DroneLink::parseIMU) (`anySuccess = true`) |
| `linkHealthy = false` | Setuje se kada `consecutiveFails >= FAIL_THRESHOLD` |
| `setFailInjection(true)` | Test-only: forsira `return {}` iz [sendMSP()](@ref DroneLink::sendMSP), deterministički simulira mrtav link |

> **📌** Sa stvarnom latencijom takta od ~40 ms, `FAIL_THRESHOLD = 5` znači da se link proglašava nezdravim nakon ~200 ms bez uspješnog [parseIMU()](@ref DroneLink::parseIMU) odgovora.

> **📌 Napomena o implementaciji (thread-safety):** `failInjectionActive` je implementiran kao `std::atomic<bool>` u `DroneLink.h`. [sendMSP()](@ref DroneLink::sendMSP) se poziva isključivo iz radne niti ([communicationLoop](@ref DroneLink::communicationLoop)), dok [setFailInjection()](@ref DroneLink::setFailInjection) može biti pozvan iz Python niti (SAT testovi). `std::atomic<bool>` garantuje atomično čitanje/pisanje bez deadlock-a.

---

## 5. Traceability — Zahtjevi do Implementacije

| **SRS ID** | **Fajl** | **Simbol / Lokacija** | **Napomene** |
|---|---|---|---|
| SRS-IMU-001 | `DroneLink.cpp` | [communicationLoop()](@ref DroneLink::communicationLoop) — RAW_IMU poll svaki takt | `POLL_INTERVAL_MS = 10 ms`; stvarna BF senzorska stopa ≥50 Hz unutar BF; MSP link 3–6 Hz efektivno |
| SRS-IMU-002 | `DroneLink.cpp` | [parseIMU()](@ref DroneLink::parseIMU) — `if (buf.size() < 18)` | Odbacuje kratke okvire, vraća `false` |
| SRS-IMU-003 | `DroneLink.cpp` | [parseIMU()](@ref DroneLink::parseIMU) — `r16` lambda na offsetima 5,7,9,11,13,15 | Little-endian ekstrakcija int16 za svih 6 polja |
| SRS-IMU-004a | `IMUSensor.h/cpp` | [getScaledData()](@ref IMUSensor::getScaledData) — `ACC_SCALE = 1.0f / 2048.0f` | Ispravljeno s 1/8192 na 1/2048 (DEF-002) |
| SRS-IMU-004b | `IMUSensor.h/cpp` | [getScaledData()](@ref IMUSensor::getScaledData) — `GYRO_SCALE = 1.0f / 16.4f` | MPU-6500 zadani opseg ±2000°/s |
| SRS-IMU-004c | `IMUWidget.py` | `ACCEL_SCALE=2048`, `GYRO_SCALE=16.4` u `update_ui()` | Widget radi na MSP sirovim int16 iz `to_dict()` |
| SRS-IMU-005 | `IMUWidget.py` | `_gyro_state_label()` — warn threshold 30°/s, hold 2.0s | Nezavisna mašina stanja po osi; žuta boja |
| SRS-IMU-006 | `IMUWidget.py` | `_gyro_state_label()` — crit threshold 100°/s, hold 4.0s | Trepteća crvena; CRIT stanje |
| SRS-IMU-006a | `IMUWidget.py` | Zajednički blink timer — 1 Hz (500ms ON/OFF) | Sve simultane CRIT ćelije trepće sinhrono (DO-160) |
| SRS-IMU-007 | `DroneLink.cpp` + Python | `lastRttMs` mjerenje + Python `update_ui` latencija | Ukupno < 200 ms; nominalno ~20 ms |
| SRS-IMU-008 | `DroneLink.cpp` | [parseIMU()](@ref DroneLink::parseIMU) — `buf[4] != MSP::RAW_IMU` → `return false` | Validacija cmd bajta |
| SRS-IMU-008b | `DroneLink.cpp` | [communicationLoop()](@ref DroneLink::communicationLoop) — try/catch nije potreban (POD) | [parseIMU()](@ref DroneLink::parseIMU) vraća bool; nikad ne baca izuzetak |

---

## 6. Poznati Defekti i Rješenja

| **DEF ID** | **Pogođeni SRS** | **Opis Defekta** | **Ispravka** |
|---|---|---|---|
| DEF-001 | SRS-IMU-001 | Originalni zahtjev "poll interval ≤ 20 ms" miješao je BF senzorsku vrijednost (50 Hz) sa MSP link vrijednošću. Stvarna MSP vrijednost je 3–6 Hz pri 57600 baud. | SRS ažuriran: BF gyro loop ≥50 Hz unutar FC; MSP link je arhitekturalno ograničen i nije SRS kršenje. |
| DEF-002 | SRS-IMU-004a | `ACC_SCALE = 1/8192` pretpostavljao je ±4g hardware opseg. BF 4.5.x inicijalizuje MPU-6500 na ±16g (INV_FSR_16G), gdje hw osjetljivost = 2048 LSB/g. Rezultat: 4× premalo očitanje. | `ACC_SCALE` promijenjeno na 1/2048. `IMUWidget ACCEL_SCALE` usklađen na 2048. |
| DEF-003 | SRS-IMU-002 (RC) | `PurgeComm(PURGE_RXCLEAR)` u [sendMSP()](@ref DroneLink::sendMSP) brisao rep MSP_RC okvira → `rcChannelCount=0`. | Uklonjen bezuslovni pre-poll purge. Sinhronizacija zaglavlja odbacuje zaostale bajte. |
| DEF-004 | SRS-IMU-002 (Batt) | `parseBatteryState()` imao guard `< 17`, ali BF 4.5 šalje okvir od 15 bajta → uvijek `false`. | Guard snižen na `< 15`. |
| DEF-005 | SRS-IMU-005/006 | Fault injection koristio `setPollIntervalMs(0)` što uzrokovalo nedeterministička timing ponašanja. | Dodan [setFailInjection(bool)](@ref DroneLink::setFailInjection) — [sendMSP()](@ref DroneLink::sendMSP) vraća `{}` odmah kad je aktivan. |
| DEF-006 | SRS-IMU-002 | Originalna dokumentacija opisivala je MSP_RAW_IMU kao 24-bajtni okvir. MPU-6500 nema interni magnetometar — BF 4.5.x šalje acc+gyro = 6×int16 = 12 bajta payload; minimalni okvir je 18 bajta. | Guard `buf.size() < 18` ostaje ispravan. Dokumentacija ažurirana. |

---

## 7. Pybind11 Integracija — C++ ↔ Python Interfejs

[DroneBackend](@ref DroneBackend) modul prikazuje C++ klase Python interpreteru putem pybind11.

### 7.1 IMUSensor Binding

| **Python Metoda** | **C++ Implementacija** | **Povratna vrijednost** |
|---|---|---|
| `IMUSensor(hub)` | [IMUSensor::IMUSensor(DroneLink*)](@ref IMUSensor::IMUSensor) | Python objekt koji drži pointer na [DroneLink](@ref DroneLink) |
| `get_raw_data()` | [IMUSensor::getRawData()](@ref IMUSensor::getRawData) | dict: `{accX, accY, accZ, gyroX, gyroY, gyroZ}` — nekalibrišani int16 ADC brojači |
| `get_scaled_data()` | [IMUSensor::getScaledData()](@ref IMUSensor::getScaledData) | dict: `{ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps}` |

### 7.2 DroneLink Binding — IMU Relevantni Pozivi

| **Python Poziv** | **Opis** |
|---|---|
| `DroneLink.connect(port_name)` | Otvara serijski port i pokreće radnu nit; počinje prikupljanje IMU podataka |
| `DroneLink.get_latest_state()` | Vraća thread-safe snapshot [DroneState](@ref DroneState)-a; poziva [getLatestState()](@ref DroneLink::getLatestState) |
| `DroneLink.set_poll_interval_ms(ms)` | Podešava takt radne niti; default 10 ms (100 Hz) |
| `DroneLink.start_acc_calibration()` | Pokreće BF ACC_CAL sekvencu; MSP komanda 205 |
| `DroneLink.set_fail_injection(active)` | Fault injection za testiranje (SAT-IMU-005 Scenario B); **NIKAD u produkcijskom kodu** |
| `DroneState.to_dict()` | Eksportuje kompletno stanje kao Python dict; `IMUWidget` poziva ovo svaki takt |

### 7.3 Tipičan Python Inicijalizacijski Kod

```python
import DroneBackend

link = DroneBackend.DroneLink()
link.connect('COM5')       # otvori serijski port, pokreni radnu nit

imu = DroneBackend.IMUSensor(link)

# Polling u GCS petlji:
state = link.get_latest_state()
data  = state.to_dict()          # eksportuj u Python dict
imu_widget.update_ui(data)       # ažuriraj prikaz
```

---

## 8. MPU-6500 — Opis Senzora i Tehničke Specifikacije

MPU-6500 je 6-osni inercijalni mjerni uređaj (IMU) kompanije InvenSense (TDK), integrisan na XFlight Hobby F405 V3. Kombinuje tro-osni MEMS giroskop i tro-osni MEMS akcelerometar u paketu 3×3×0.9 mm. Komunicira s F405 putem SPI interfejsa na visokim brzinama, što omogućava giroskopsku petlju od 8 kHz unutar Betaflight firmware-a.

### 8.1 Ključne Tehničke Specifikacije

| **Parametar** | **Vrijednost / Opis** |
|---|---|
| **Tip senzora** | 6-osni IMU: 3-osni giroskop + 3-osni akcelerometar (MEMS) |
| **Napon napajanja** | VDD: 1.71–3.6 V; VDDIO: 1.71–3.6 V |
| **Digitalni interfejs** | SPI (do 20 MHz za senzorske registre) i I2C (do 400 kHz) |
| **Giroskop — opseg** | ±250 / ±500 / ±1000 / ±2000 °/s (BF koristi ±2000°/s) |
| **Giroskop — osjetljivost** | 131 / 65.5 / 32.8 / 16.4 LSB/(°/s) |
| **Giroskop — ADC rezolucija** | 16-bitni ADC (int16, opseg −32768 do +32767) |
| **Akcelerometar — opseg** | ±2 / ±4 / ±8 / ±16 g (BF 4.5 koristi ±16g — INV_FSR_16G) |
| **Akcelerometar — osjetljivost** | 16384 / 8192 / 4096 / 2048 LSB/g (za ±2g / ±4g / ±8g / ±16g) |
| **Gyro ODR** | Do 8000 Hz (interno); BF gyro loop konfigurisan na 8 kHz |
| **Akcelerometar ODR** | 1000 Hz |
| **Gyro signal kašnjenje (DLPF isklj.)** | 0.17 ms |
| **Gyro buka (RMS)** | 0.01 °/s/sqrt(Hz) |
| **Gyro Zero-Rate Level offset** | ±20 °/s maks. (tipično ±1 °/s); BF kalibrira ZRL pri pokretanju |
| **Gyro temperaturna osjetljivost** | ±0.1 °/s/°C (ZRL drift po temperaturi) |
| **Potrošnja** | 3.2 mA (gyro + acc aktivni) |
| **Paket / dimenzije** | QFN-24 (3×3×0.9 mm), integrisani 512-bajt FIFO bafer |

### 8.2 Registarsko Mapiranje (SPI DMA burst čitanje)

```
0x3B–0x40: ACCEL_XOUT (6 B)
0x41–0x42: TEMP_OUT  (2 B)   ← Betaflight ne eksportuje kroz MSP_RAW_IMU
0x43–0x48: GYRO_XOUT (6 B)
Ukupno: 14 bajta u jednom burst SPI čitanju
```

MSP_RAW_IMU prenosi samo acc (6 B) i gyro (6 B) — temperatura se ne proslijeđuje GCS-u.

---

## 9. Detaljna Analiza Komunikacije — MPU-6500 do GCS Aplikacije

### 9.1 Sloj 1 — Senzor do Flight Kontrolera (SPI)

MPU-6500 kontinualno mjeri ugaono i linearno ubrzanje putem MEMS struktura. Analogni signal se digitalizuje 16-bitnim ADC-om i smješta u interno FIFO skladište (512 bajta). Kada je novi uzorak spreman, INT pin MPU-6500 aktivira EXTI interrupt na STM32F405.

STM32 ISR pokreće SPI DMA transfer koji čita 14 bajta podataka iz MPU-6500 registara (0x3B–0x48). DMA transfer se obavlja bez CPU intervencije — ovaj ciklus SPI čitanja traje manje od 2 µs pri 20 MHz SPI satu.

### 9.2 Sloj 2 — Betaflight Interna Obrada (FC CPU)

Betaflight primljene sirove int16 vrijednosti obrađuje: (1) kalibracija nulte tačke i skaliranje, (2) DLPF i RPM filter, (3) AHRS fusion algoritam (Mahony ili Madgwick) koji daje apsolutne ugaone procjene roll/pitch/yaw u decidegrees.

**Važno:** MSP_RAW_IMU (komanda 102) eksportuje int16 ADC vrijednosti koje su prošle kroz BF kalibraciju nulte tačke, ali NISU skalirane ni filtrirane od BF-a. Efektivna akcelerometarska osjetljivost na MSP sloju je 2048 LSB/g (DEF-002).

### 9.3 Sloj 3 — MSP Protokol (FC ↔ GCS, UART/USB)

Sekvenca jednog MSP_RAW_IMU ciklusa:
1. [DroneLink::sendMSP(102)](@ref DroneLink::sendMSP) gradi 6-bajtni zahtjev i šalje ga UART portom
2. FC priprema MSP_RAW_IMU odgovor sa 12 bajta payload (acc + gyro, 3 × int16 svaki)
3. FC šalje 18-bajtni odgovor UART-om
4. [DroneLink](@ref DroneLink) readByte() petlja sinhronizuje na `'$','M','>'` preambulu i čita preostalih 15 bajta
5. [parseIMU()](@ref DroneLink::parseIMU) ekstrahuje 6 int16 vrijednosti (acc + gyro)
6. [commitState()](@ref DroneLink::commitState) atomski upisuje u [DroneState](@ref DroneState) uz mutex zaštitu
7. Python thread čita stanje putem [getLatestState()](@ref DroneLink::getLatestState) i poziva `to_dict()`

### 9.4 Sloj 4 — Python Frontend Obrada i Prikaz

`IMUWidget.update_ui(data)` pri svakom pozivu:
- Primjenjuje skaliranje (`ax/2048` za g, `gx/16.4` za °/s)
- Evaluira prag upozorenja za svaku od 3 gyro ose nezavisno (SAFE < 30°/s; WARN 30–100°/s; CRIT > 100°/s)
- CRIT indikatri trepće sinhrono na 1 Hz (500 ms ON/OFF) koristeći zajednički blink tajmer
- Vrijednosti se ispisuju sa 2 decimale za rotaciju, 3 za G-silu, 1 za ugao

### 9.5 Kompletni Komunikacijski Tok — Rezime

| **Sloj** | **Put podataka** | **Protokol/Bus** | **Kašnjenje** |
|---|---|---|---|
| S0 | MPU-6500 MEMS → ADC → FIFO | Interno (analogno) | ~0.17 ms |
| S1 | MPU-6500 FIFO → STM32 DMA | SPI @ 20 MHz | < 2 μs |
| S2 | BF gyro loop, DLPF, AHRS fusion | Interni firmware | 0.12 ms |
| S3 | FC MSP handler → USB CDC TX | MSP v1 / UART 57600 | ~4.2 ms TX |
| S4 | [DroneLink](@ref DroneLink) C++ RX + [parse](@ref DroneLink::parseIMU) | Win32 SERIAL API | 2–15 ms RTT |
| S5 | Pybind11 `to_dict()` → Python | In-process call | < 0.1 ms |
| S6 | `IMUWidget.update_ui()` prikaz | Python / Tkinter | 5–30 ms |

> **📌** Ukupno end-to-end kašnjenje (S0 do S6): nominalno ~20 ms, maksimalno ~52 ms. Bottleneck sistema je serijski UART link na 57600 baud (S3).

---

## 10. DroneCockpitApp — Glavni Prozor i Integracija Widgeta

`DroneCockpitApp` je klasa koja nasljeđuje iz `tk.Tk` i predstavlja korijenski prozor GCS aplikacije. Ona instancira sve instrumente (widgete), kreira plutajući radni prostor (Canvas), pokreće `TelemetryWorker` pozadinsku nit i upravlja petljom osvježavanja UI-a.

### 10.1 Arhitektura Prozora — DraggablePanel i Canvas

Radni prostor je implementiran kao `tk.Canvas` widget koji prekriva cijeli prozor. Svaki instrument se kreira kao instanca klase `DraggablePanel` — Frame widget kreiran na Canvas-u putem `create_window()`. Ovaj pristup omogućava slobodnu poziciju, promjenu veličine i kontrolu Z-redosljeda za svaki panel nezavisno.

`DraggablePanel` pruža:
- Prevlačenje naslova (drag) za premiještanje panela, uz magnetno privlačenje (snap, `SNAP_PX = 14 px`)
- Promjena veličine putem grip kontrola (`MIN_PANEL_W = 120 px`, `MIN_PANEL_H = 60 px`)
- Z-redosljed: klik na panel ga automatski podiže na vrh (`raise_panel / lift()`)
- Kontekstni meni (desni klik) sa opcijama za snap pozicioniranje

### 10.2 TelemetryWorker i UI Refresh Petlja

`TelemetryWorker` je pozadinska nit (daemon thread) koja poziva [DroneBackend](@ref DroneBackend) putem C++ [DroneLink](@ref DroneLink) interfejsa na 60 Hz. Rezultati se stavljaju u thread-safe queue.

Tk main thread se budi svakih `UI_REFRESH_MS = 20 ms` (~50 Hz) i:
1. Provjerava da li je `TelemetryWorker` i dalje aktivan
2. Ispražnjava red uzimajući najsvježiji okvir podataka (`get_frame()`)
3. Distribuira `ui_data` rječnik prema svakom widgetu prema throttle rasporedu
4. Zakazuje sljedeće buđenje putem `root.after()`

---

## 11. Sažetak i Zaključak

### 11.1 Pregled Implementiranih Komponenti

| **Komponenta** | **Status** | **Ključni Zahtjev** |
|---|---|---|
| [DroneLink::sendMSP(102)](@ref DroneLink::sendMSP) | ✅ IMPLEMENTIRANO | Pouzdani transport bez purge-greške (DEF-003) |
| [DroneLink::parseIMU()](@ref DroneLink::parseIMU) | ✅ IMPLEMENTIRANO | SRS-IMU-002, SRS-IMU-003, SRS-IMU-008 |
| [IMUSensor::getScaledData()](@ref IMUSensor::getScaledData) | ✅ IMPLEMENTIRANO (ispravljeno) | SRS-IMU-004a: `ACC_SCALE = 1/2048` (DEF-002) |
| [DroneState](@ref DroneState) IMU polja | ✅ IMPLEMENTIRANO | Thread-safe, mutex-zaštićena dijeljenja memorija |
| [DroneState::to_dict()](@ref DroneState::to_dict) IMU ključevi | ✅ IMPLEMENTIRANO | Svi ax/ay/az/gx/gy/gz ključevi prisutni i ispravni |
| `IMUWidget.update_ui()` | ✅ IMPLEMENTIRANO | SRS-IMU-005/006/006a: upozorenja, tajmeri, treptanje |
| RTT mjerenje (`lastRttMs`) | ✅ IMPLEMENTIRANO | SRS-IMU-007: < 200 ms end-to-end |
| [DroneLink::setFailInjection()](@ref DroneLink::setFailInjection) | ✅ IMPLEMENTIRANO | DEF-005: deterministički za SAT-IMU-005 Scenario B |

---

## 12. Rječnik Termina (Glossary)

| **Termin / Akronim** | **Definicija** |
|---|---|
| `ACC_SCALE` | Faktor skaliranja akcelerometra u C++ sloju: 1/2048 (ispravka DEF-002). Prevodi sirove MSP int16 ADC vrijednosti u g. |
| `ACCEL_SCALE` | Ekvivalentna konstanta u Python `IMUWidget` sloju (vrijednost 2048). |
| **AHRS** | Attitude and Heading Reference System — algoritam fuzije u Betaflight (Mahony ili Madgwick filter). |
| **BF** | Betaflight — open-source firmware za flight kontrolere (verzija 4.5.3). |
| **CDC-ACM** | Communications Device Class – Abstract Control Model. USB klasa koja emulira serijski port. |
| **DLPF** | Digital Low Pass Filter. U BF 4.5.3 na F405 V3 koristi NORMAL konfiguraciju (~256 Hz). |
| **GCS** | Ground Control Station — softverska aplikacija (Python/Tkinter). |
| `GYRO_SCALE` | Faktor skaliranja giroskopa: 1/16.4. Prevodi sirove ADC vrijednosti u °/s. |
| **IMU** | Inertial Measurement Unit — inercijalni mjerni uređaj. U ovom projektu: MPU-6500. |
| **MSP** | MultiWii Serial Protocol — binarni request/response protokol (verzija 1). |
| **POD** | Plain Old Data — [DroneState](@ref DroneState) je POD, može se kopirati s `=` operatorom atomično. |
| **RTT** | Round-Trip Time — ukupno kružno kašnjenje od slanja MSP zahtjeva do primanja odgovora. Tipično: 2–15 ms. |
| **SPI** | Serial Peripheral Interface — sinhroni serijski protokol (SCLK, MOSI, MISO, CS). |
| **ZRL** | Zero-Rate Level — nulti odmak giroskopa. BF kalibrira ZRL pri pokretanju (1000+ uzoraka). |

---

*Elektrotehnički fakultet, Univerzitet u Banjoj Luci | 2025/2026*
