# Komande sa računara preko Pocket-a – uputstvo

Project R.A.D. – upravljanje dronom iz aplikacije kontrolne stanice preko RadioMaster Pocket-a. Pocket ostaje predajnik, a pilot zadržava konačnu kontrolu.

> **Stanje:** kod je napisan i ispitan bez hardvera: 661 C++ provjera, 51 provjera Lua skripte pod simuliranim EdgeTX-om i 224 Python provjere. Na stvarnom Pocket-u i dronu **ništa još nije ispitano**. Prvi let je tek nakon što prođu bench testovi iz poglavlja 5, i to bez propelera.

## 1. Kako radi

```
Kontrolna stanica                       RadioMaster Pocket (EdgeTX 2.12)               Dron
─────────────────                       ───────────────────────────────               ────
CommandPanel (tipke, dugmad)
   │
UplinkController (25 Hz)
 + AltitudeHold (baro → gas)
   │ set_command()
CommandLink (C++) ──CRSF RC okvir──►  USB-VCP = LUA
   ▲                                  radcmd.lua (mix skripta)
   │                                    ├─ izlazi Rol/Pit/Thr/Yaw/Tlt/Lnk
   │                                    ├─ mikser: pomak na CH1–4, gimbal na CH8 ══ELRS══► RP4TD ─► Betaflight
CrsfLink (C++) ◄──CRSF telemetrija────  └─ telemetrija iz senzora + RC + status ◄══ELRS══ telemetrija
   │
TelemetryWorker → instrumenti (kao i do sada)
```

- **Jedan port, dva smjera.** Pocket-ov USB port prelazi iz režima *Telem Mirror* u režim *LUA*. Skripta `radcmd.lua` prima komande i šalje telemetriju nazad, prepakovanu u **iste CRSF okvire** koje je slao Telem Mirror. Zato `CrsfLink` i svi instrumenti rade bez izmjena.
- `CommandLink` nema svoj port. Okvire piše radna nit `CrsfLink`-a, i to tek kad skripta pošalje statusni okvir (`RAD_STATUS`). Ako je Pocket i dalje u režimu Telem Mirror, računar na port ne piše ništa.
- Komanda je standardni CRSF okvir sa kanalima i CRC-om. Isti okvir bi radio i sa EdgeTX režimom *CRSF Trainer preko USB-a*, ako taj režim uđe u zvanično izdanje.

## 2. Šta računar može, a šta ne može

| Može | Ne može (nikad) |
|---|---|
| Naprijed/nazad, lijevo/desno: mali pomak palice, samo u ANGLE režimu, tako da pomak znači nagib (≤ 25 %) | Armiranje i dezarmiranje |
| Okretanje (yaw) do 30 % | Promjenu režima leta |
| Držanje visine: gas iz regulatora, u uskom opsegu oko gasa lebdenja | Failsafe i GPS Rescue |
| Nagib gimbala (radi i bez pilotovog prekidača) | Gas dalje od ±100 µs od položaja pilotove palice |

**Slojevi zaštite** (svaki je dovoljan sam za sebe):

1. **Pilotov prekidač na Pocket-u** (podrazumijevano `SD`). Kad je isključen, skripta na radiju daje nulu za palice i gas, bez obzira na to šta računar šalje.
2. **Watchdog na radiju:** ako 500 ms ne stigne komanda sa *novim* brojačem, izlazi idu na nulu. Isto važi ako se aplikacija zaglavi, ugasi ili se izvuče USB.
3. **Ograničenja na radiju:** pomak palica ±300 (oko 30 %). Gas je *razlika u odnosu na pilotovu palicu*, najviše ±200 (oko 100 µs), i samo dok je palica iznad oko 1200 µs. Kad pilot spusti palicu, gas ide dolje. Kad isključi prekidač, gas skače na položaj palice za najviše 100 µs.
4. **Pilotove palice se uvijek sabiraju** sa komandom računara (ADD u mikseru), pa pilot može ispraviti dron u svakom trenutku.
5. **Na računaru:** glavni prekidač, zakup komande od 300 ms (zaglavljena Python nit ne ostavlja staru komandu aktivnom), ograničenja i brzina promjene, provjera da je dron armiran i u ANGLE režimu, i zapis svake komande u `DroneCockpitUI/logs/uplink_*.csv`.

## 3. Postavljanje Pocket-a

### 3.1 Skripta
1. Pocket priključi na računar i izaberi *USB Storage*.
2. Kopiraj `Firmware/pocket/SCRIPTS/MIXES/radcmd.lua` u `/SCRIPTS/MIXES/` na SD kartici (ime fajla ima najviše 6 znakova).
3. Ako u skripti treba nešto promijeniti (prekidač, granice), mijenja se tabela `CFG` na vrhu fajla.

### 3.2 USB port
`SYS → HARDWARE → USB-VCP` postavi na **LUA**. Telem Mirror od tada ne radi, a telemetriju šalje skripta.
Vraćanje na stari režim: USB-VCP ponovo na *Telem Mirror*. Kokpit tada radi kao ranije, a `CommandLink` ne piše ništa.

### 3.3 Senzori telemetrije
U `MODEL → TELEMETRY` uradi *Discover new sensors* sa uključenim dronom. EdgeTX daje ime **„Alt"** i GPS visini i baro visini. Preimenuj ih:
- GPS visina → `GAlt`
- baro visina → `BAlt`

Provjeri i da ostala imena odgovaraju tabeli `S` u skripti (`Ptch`, `Roll`, `Yaw`, `RxBt`, `Curr`, `Capa`, `Bat%`, `GPS`, `GSpd`, `Hdg`, `Sats`, `FM`, `VSpd`, `1RSS`, `RQly` …).

### 3.4 Model
1. `MODEL → CUSTOM SCRIPTS` → prvi red → `radcmd`. Treba da se pojave izlazi `Rol Pit Thr Yaw Tlt Lnk`.
2. `MODEL → LOGICAL SWITCHES` → **L01**: funkcija `a>x`, izvor `Lnk` (izlaz skripte), vrijednost `50`. L01 je uključen samo kad je komanda svježa i pilotov prekidač uključen.
3. `MODEL → MIXES`. Postojeće linije ostaju. Dodaj:

| Kanal | Nova linija | Težina | Multiplex | Prekidač |
|---|---|---|---|---|
| CH1 (roll) | izvor `Rol` | 100 | ADD | L01 |
| CH2 (pitch) | izvor `Pit` | 100 | ADD | L01 |
| CH3 (gas) | izvor `Thr` | 100 | ADD | L01 |
| CH4 (yaw) | izvor `Yaw` | 100 | ADD | L01 |
| CH8 (AUX4, gimbal) | izvor `Tlt` | 100 | REPLACE | – |

   Prekidač L01 je druga, nezavisna zaštita: skripta i sama daje nulu kad prekidač nije uključen.
4. Na liniji gasa (CH3) ne smije biti krive ni expo-a. Skripta računa razliku u odnosu na sirovu palicu `thr`.
5. **ELRS switch mode → Wide** (ELRS Lua skripta na Pocket-u). U Hybrid režimu AUX kanali imaju samo 6 položaja, pa bi gimbal skakao. U Wide režimu imaju 128 položaja. CH1–4 su u oba režima pune rezolucije.

### 3.5 Pilotov prekidač
Podrazumijevano je `SD` (dolje = uključeno). Ako je SD već zauzet, promijeni `CFG.GATE_SW` i `CFG.GATE_ON` u skripti. Prekidač treba biti lako dostupan, jer je to „vrati mi kontrolu" prekidač.

## 4. Betaflight

- ANGLE režim već postoji na AUX2 (CH6). Računar odbija pomak ako dron nije u ANGLE režimu.
- Gimbal: u **Servos** tabu za servo 0 (pitch) čekiraj **A4** (CH8), pa *Save*. Ako se gimbal kreće u pogrešnu stranu, *Rate* stavi na −100.
- Ostalo se ne mijenja.

## 5. Bench testovi (propeleri SKINUTI)

Alat `DroneCockpitUI/tools/uplink_bench.py` radi bez kokpita (`pip install pyserial`). Port Pocket-a se vidi sa `--list`.

| Test | Postupak | Prolaz |
|---|---|---|
| **UT-UPL-A1** most | `python uplink_bench.py --port COMx` | Stižu `RAD_STATUS`, `fresh=True`, `crc errors 0`, period skripte ≤ 50 ms. Ako ima CRC grešaka u telemetriji, `serialWrite` možda reže nula-bajtove (vidi poglavlje 8). |
| **UT-UPL-A2** kanali | Dron uključen, Betaflight → Receiver tab, pa `--sweep` | Sa prekidačem **isključenim** CH1/CH2 miruju. Sa uključenim kruže ±20 %. Pitch naprijed povećava CH2. |
| **UT-UPL-A3** kašnjenje | FC na USB-u (COMy), `--latency --msp COMy` | Medijana < 100 ms |
| **UT-UPL-A4** telemetrija | Monitor + kokpit preko USB kabla | Stav ≥ 5 Hz, GPS i baterija ≥ 1 Hz. Pitch i roll se slažu sa USB-om (inače `ATT_IN_DEGREES = true`). FM se prikazuje (npr. `STAB`). |
| **UT-UPL-A5** gimbal | `--tilt -50`, pa `--tilt 50` | Gimbal se pomjera i radi i bez prekidača |
| **UT-UPL-S1** prekidač | Tokom `--sweep` isključi prekidač | CH1/CH2 se odmah vraćaju na palice |
| **UT-UPL-S2** kabl | Tokom `--sweep` izvuci USB iz računara | CH1/CH2 na palice za ≤ 500 ms |
| **UT-UPL-S3** pad aplikacije | Tokom `--sweep` ugasi proces (Task Manager) | Isto kao S2 |
| **UT-UPL-S4** opterećenje | 10 min monitor | Nema poruke „Script killed" na Pocket-u, period skripte stabilan |
| **UT-UPL-K1** kokpit | `set COCKPIT_UPLINK_BENCH=1`, kokpit, 🕹 Laptop Control | Stanje `GATE_ON` / `ACTIVE`, RTT se prikazuje, W/A/S/D pomjera CH1/CH2 u Receiver tabu |

## 6. Ispitivanje na stalku sa šipkom (sa propelerima)

Pojačanja regulatora visine (`AltitudeHoldConfig`) su **početne vrijednosti** iz simulacije. Na stvarnom dronu se podešavaju na stalku.

| Test | Postupak | Prolaz |
|---|---|---|
| **ALT-1** preuzimanje | Lebdenje, prekidač uključen, *Altitude hold* | Skok gasa ≤ 10 µs (vidi „radio Δ" u panelu) |
| **ALT-2** korak | `R` (+0,5 m) dva puta | Visina prati cilj bez oscilacija, bez prebačaja većeg od 0,3 m |
| **ALT-3** baterija | 3 min držanja | Visina u granicama ±0,5 m. Integralni dio raste kako baterija pada. |
| **MOV-1** pomaci | W/A/S/D na 5 % | Dron se naginje u pravu stranu, a puštanje tipke ga vraća u nivo |

Simulacija (DroneCockpitUI/tests/unit/test_uplink_python.py) daje: greška lebdenja 0,06 m, pad baterije od 9 % kompenzovan na 0,07 m, korak od +2 m bez trajne greške. Model je jednostavan (bez vjetra, bez kašnjenja motora), pa su ovo samo provjere logike.

## 7. Postupak u letu

1. Poletanje i armiranje kao i obično, **ANGLE** režim.
2. U kokpitu otvori **🕹 Laptop Control** i uključi *Laptop control*.
3. Dovedi dron u lebdenje i drži palicu gasa na tom položaju.
4. Uključi **pilotov prekidač**. Stanje u panelu prelazi u `GATE_ON`.
5. *Altitude hold* (H): gas preuzima regulator od trenutne vrijednosti. Pilot drži palicu gasa blizu lebdenja, jer je ona sidro za ±100 µs.
6. W/A/S/D/Q/E za kretanje (drži tipku), Space za stop, R/F za visinu.
7. **Povratak kontrole:** isključi prekidač. Palice i gas su odmah pilotovi.

Bez GPS zadržavanja pozicije dron u ANGLE režimu polako klizi sa vjetrom. Računar daje nagib, a ne poziciju.

## 8. Pretpostavke koje treba potvrditi na Pocket-u

| Pretpostavka | Gdje se provjerava | Ako ne važi |
|---|---|---|
| `serialRead`/`serialWrite` rade iz mix skripte | A1 | Prebaciti čitanje u function skriptu, pa vrijednosti predavati preko GV |
| `serialWrite` šalje i nula-bajtove | A1 (CRC greške) | Telemetrija se ne može slati u binarnom obliku, pa treba drugo kodiranje |
| `getValue("FM")` vraća tekst | A4 | Bez FM-a kokpit ne vidi ARM ni ANGLE i blokira komande. Rješenje: čitati prekidače ARM/ANGLE u skripti. |
| Ptch/Roll/Yaw su u radijanima | A4 | `ATT_IN_DEGREES = true` |
| Skripta stane u ograničenje CPU-a | S4 | Smanjiti `TLM_PER_RUN` na 1 (izmjereno u simulaciji: 1 400–2 300 Lua instrukcija po pokretanju) |

## 9. Gdje je šta u projektu

| Dio | Fajl |
|---|---|
| Protokol komandi i statusa | `DroneBackend/src/radio/UplinkProtocol.h/.cpp` |
| Stanje komandi, zakup, ograničenja | `DroneBackend/src/radio/CommandLink.h/.cpp` |
| Pisanje na Pocket-ov port | `DroneBackend/src/radio/CrsfLink.h/.cpp` (`set_uplink`) |
| Python veza | `DroneBackend/src/bindings/UplinkBindings.h` (poziva se iz `Bindings.cpp`) |
| Kontroler (25 Hz) | `DroneCockpitUI/workers/uplink_controller.py` |
| Regulator visine | `DroneCockpitUI/control/altitude_hold.py` |
| Prozor *Laptop Control* | `DroneCockpitUI/widgets/command_panel.py` |
| Skripta za Pocket | `Firmware/pocket/SCRIPTS/MIXES/radcmd.lua` |
| Bench alat | `DroneCockpitUI/tools/uplink_bench.py` |
| Testovi | `DroneBackend/tests/test_uplink.cpp`, `DroneCockpitUI/tests/unit/test_uplink_python.py`, `DroneCockpitUI/tests/unit/test_lua_bridge.py` |

**Pokretanje testova:**
```
cd DroneBackend\tests
cl /std:c++17 /EHsc /I..\src\radio test_uplink.cpp ..\src\radio\UplinkProtocol.cpp ..\src\radio\CommandLink.cpp ..\src\radio\CrsfProtocol.cpp
test_uplink.exe                   # 661 provjera

cd ..\..\DroneCockpitUI
pip install lupa
pytest tests/unit                 # sa test_uplink_python i test_lua_bridge
```
(`cl` se pokreće iz *Developer Command Prompt for VS*. Bez `test_uplink.exe` ili bez `lupa` test Lua skripte se preskače.)
