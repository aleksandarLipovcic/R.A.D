# Komande sa računara preko Pocket-a – uputstvo

Project R.A.D. – upravljanje dronom iz aplikacije kontrolne stanice preko RadioMaster Pocket-a. Pocket ostaje predajnik, a pilot zadržava konačnu kontrolu.

> **Stanje:** kod je napisan i ispitan bez hardvera: 676 C++ provjera, 56 provjera Lua skripte pod simuliranim EdgeTX-om, 232 Python provjere komandi i 42 provjere gimbala. Na stvarnom Pocket-u i dronu **ništa još nije ispitano**. Prvi let je tek nakon što prođu bench testovi iz poglavlja 5, i to bez propelera.

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
   │                                    ├─ izlazi Rol/Pit/Thr/Yaw/Tlt/Pan
   │                                    ├─ mikser: pomak na CH1–4, gimbal na CH8/CH9 ══ELRS══► RP4TD ─► Betaflight ─► M5/M6
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
| Gimbal kamere: nagib i okretanje u stepenima (radi i bez pilotovog prekidača) | Gas dalje od ±100 µs od položaja pilotove palice |

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
1. `MODEL → CUSTOM SCRIPTS` → prvi red → `radcmd`. Treba da se pojave izlazi `Rol Pit Thr Yaw Tlt Pan`.
2. `MODEL → MIXES`. Postojeće linije ostaju. Dodaj:

| Kanal | Nova linija | Težina | Multiplex | Prekidač |
|---|---|---|---|---|
| CH1 (roll) | izvor `Rol` | 100 | ADD | `SD↓` |
| CH2 (pitch) | izvor `Pit` | 100 | ADD | `SD↓` |
| CH3 (gas) | izvor `Thr` | 100 | ADD | `SD↓` |
| CH4 (yaw) | izvor `Yaw` | 100 | ADD | `SD↓` |
| CH8 (AUX4, gimbal nagib) | izvor `Tlt` | 100 | REPLACE | – |
| CH9 (AUX5, gimbal pan) | izvor `Pan` | 100 | REPLACE | – |

   Uslov `SD↓` na linijama je druga, nezavisna zaštita: skripta i sama daje nulu kad prekidač nije uključen.
   EdgeTX dozvoljava najviše 6 izlaza po mix skripti, pa je šesti izlaz (ranije `Lnk` i logički prekidač L01) sada gimbal pan. Ako je na modelu ostao L01 sa izvorom `Lnk`, obriši ga, a na linijama CH1–4 umjesto L01 stavi `SD↓`.
3. Na liniji gasa (CH3) ne smije biti krive ni expo-a. Skripta računa razliku u odnosu na sirovu palicu `thr`.
4. **ELRS switch mode → Wide** (ELRS Lua skripta na Pocket-u). U Hybrid režimu AUX kanali imaju samo 6 položaja, pa bi gimbal skakao. U Wide režimu imaju 128 položaja (korak gimbala oko 0,8°). CH1–4 su u oba režima pune rezolucije. CH9 postoji samo ako ELRS šalje bar 12 kanala (Wide šalje CH1–CH12).

### 3.5 Pilotov prekidač
Podrazumijevano je `SD` (dolje = uključeno). Ako je SD već zauzet, promijeni `CFG.GATE_SW` i `CFG.GATE_ON` u skripti. Prekidač treba biti lako dostupan, jer je to „vrati mi kontrolu" prekidač.

## 4. Betaflight i gimbal

- ANGLE režim već postoji na AUX2 (CH6). Računar odbija pomak ako dron nije u ANGLE režimu.
- Ostala podešavanja leta se ne mijenjaju.

### 4.1 Ožičenje gimbala

| Servo | Kabl signala | Betaflight | Radio kanal |
|---|---|---|---|
| Nagib (tilt, servo koji nosi kameru) | **M5** | `resource SERVO 1 A01` → servo 0 = GIMBAL_PITCH | CH8 (AUX4) |
| Pan (servo u osnovi, osnova montirana vertikalno) | **M6** | `resource SERVO 2 A08` → servo 1 = GIMBAL_ROLL | CH9 (AUX5) |

- Napajanje oba servoa (crveni i braon/crni) ide sa step-down pločice od 5 V, ne sa kontrolera.
- **Masa step-down pločice i masa kontrolera moraju biti spojene.** Bez zajedničke mase signal sa M5/M6 nema referencu i servo trza ili ne radi.
- Sa kontrolera ide samo signal (narandžasti/žuti).
- Ako su servoi zamijenjeni (pan na M5), ne treba prespajati: u Servos tabu zamijeni kanale (servo 0 ← A5, servo 1 ← A4), a u kokpitu zamijeni `servo_index` u `GIMBAL_PAN_CAL`/`GIMBAL_TILT_CAL`.

### 4.2 Servos tab

`feature SERVO_TILT` i resursi za M5/M6 su već u `Firmware/betaflight/LISTIC_BF4.5.5_servo_mag_20261103.txt`. U Betaflight Configurator-u, tab **Servos**:

| Servo | MIN | MID | MAX | Rate | Forward |
|---|---|---|---|---|---|
| 0 (nagib, M5) | 1000 | 1500 | 2000 | 100 | **A4** (CH8) |
| 1 (pan, M6) | 1000 | 1500 | 2000 | 100 | **A5** (CH9) |

*Save*, pa u CLI-ju `servo` provjeri da su oba reda upisana.

- MIN/MAX su tvrde granice impulsa. Kad poslije kalibracije (4.4) znaš gdje gimbal udara u mehanički graničnik, stavi MIN/MAX malo unutar tih tačaka. Servo koji gura u graničnik vuče veliku struju i greje se.
- **Rate ne okreće smjer** proslijeđenog kanala. Betaflight kanal sa „Forward" kopira direktno na izlaz, a Rate koristi samo za stabilizaciju (CAMSTAB). Smjer se podešava u kokpitu (`direction` u kalibraciji).
- Mod **CAMSTAB** ne smije biti dodijeljen nijednom prekidaču. On bi pomjerao gimbal prema stavu drona, a kokpit računa uglove u odnosu na ram drona.

### 4.3 Koliki je hod gimbala

Betaflight proslijeđeni kanal šalje na servo bez skaliranja. Sa normalnim granicama radija (±100 %) to je oko 988–2012 µs. MG90S ima oko 10 µs po stepenu, pa je hod **oko 100° po osi**, a ne punih 180° koje nosač dozvoljava. Zato:

- **Nagib:** postavi krilce servoa tako da je kamera na 1500 µs nagnuta oko **45° naniže**. Tada opseg pokriva 0° (gleda naprijed) do 90° (gleda pravo dolje). To je podrazumijevana kalibracija (`center_us = 1050`: impuls na 0°).
- **Pan:** krilce tako da je na 1500 µs kamera u sredini. Opseg je oko ±50°, a u kokpitu je meka granica ±45°.

Ako treba veći hod: u EdgeTX-u uključi *Extended limits* (MODEL SETUP) i na CH8/CH9 stavi limite do 150 %, a u ELRS-u izaberi režim sa punom rezolucijom AUX kanala (npr. „100Hz Full"). Betaflight tada dobija oko 880–2160 µs (oko ±60°). Uz to podigni MIN/MAX u Servos tabu i ponovi kalibraciju.

### 4.4 Kalibracija (bez propelera, dron na USB-u i na radiju)

Alat `uplink_bench.py` šalje impulse direktno u µs i ispisuje šta Betaflight stvarno šalje na M5/M6 (MSP_SERVO):

```
python uplink_bench.py --port COMx --tilt-us 1200 --pan-us 1500 --msp COMy
```

(`COMx` je Pocket, `COMy` kontroler na USB-u.) Za svaku osu:

1. Pošalji jedan impuls (npr. nagib 1200 µs) i izmjeri ugao (uglomjer ili aplikacija nivelacije na telefonu prislonjena uz kameru).
2. Pošalji drugi impuls (npr. 1800 µs) i ponovo izmjeri.
3. Izračunaj:
   `us_per_deg = (us2 − us1) / (ugao2 − ugao1)`, `center_us = us1 − ugao1 · us_per_deg`.
   Ako je `us_per_deg` negativan, upiši ga kao pozitivan, a `direction = -1`.
4. Upiši vrijednosti u `DroneCockpitUI/DroneCockpitUI.py` (`GIMBAL_TILT_CAL`, `GIMBAL_PAN_CAL`), zajedno sa `min_deg`/`max_deg` malo unutar mehaničkih graničnika.

Konvencija uglova (ista koju koristi georeferenciranje):
- **nagib:** 0° = kamera gleda naprijed, +90° = pravo dolje;
- **pan:** sa kamerom okrenutom naniže, **+** = kamera gleda **desno** od drona (gledano odozgo, nos naprijed). Ako za +20° kamera gleda lijevo, promijeni `direction` za pan.

Pan os je kod ovog nosača os duž drona (osnova je vertikalna na nosu), pa je `GIMBAL_MOUNT_AXIS = "roll"`. Kad kamera gleda naprijed (nagib 0°), pan samo rotira sliku oko optičke ose. Kad gleda dolje, pan pomjera pogled lijevo-desno.

### 4.5 Upravljanje i uglovi za georeferenciranje

- U prozoru **🕹 Laptop Control**, okvir *Camera gimbal*: klizači u stepenima, dugmad *Forward / Down 45 / Down 90 / Pan centre*, tasteri **strelice** (nagib/pan po 5°) i **C** (pan u sredinu). Gimbal radi i kad *Laptop control* nije uključen i kad je pilotov prekidač isključen.
- Panel prikazuje zadani i procijenjeni stvarni ugao, izvor procjene i natpis **MOVING** dok se gimbal kreće.
- **📐 Camera Angle → Auto** (podrazumijevano): georeferenciranje uzima procijenjene stvarne uglove. Izvor, po prioritetu:
  1. *USB servo out*: MSP_SERVO, impuls koji Betaflight stvarno šalje (dron na USB kablu);
  2. *radio CH8/CH9*: kanal koji Pocket stvarno šalje (preko Lua mosta);
  3. *command only*: ono što kokpit zadaje.
  Na to se dodaje model brzine servoa (300 °/s, kašnjenje 80 ms). Dok model kaže da se gimbal kreće, detekcija ne dodaje nove pozicije objekata, nego zadržava zadnju (u zapisu: `held (gimbal moving)`).
- **Manual** ostaje za let sa nenapajanim gimbalom ili ručno fiksiranom kamerom.

## 5. Bench testovi (propeleri SKINUTI)

Alat `DroneCockpitUI/tools/uplink_bench.py` radi bez kokpita (`pip install pyserial`). Port Pocket-a se vidi sa `--list`.

| Test | Postupak | Prolaz |
|---|---|---|
| **UT-UPL-A1** most | `python uplink_bench.py --port COMx` | Stižu `RAD_STATUS`, `fresh=True`, `crc errors 0`, period skripte ≤ 50 ms. Ako ima CRC grešaka u telemetriji, `serialWrite` možda reže nula-bajtove (vidi poglavlje 8). |
| **UT-UPL-A2** kanali | Dron uključen, Betaflight → Receiver tab, pa `--sweep` | Sa prekidačem **isključenim** CH1/CH2 miruju. Sa uključenim kruže ±20 %. Pitch naprijed povećava CH2. |
| **UT-UPL-A3** kašnjenje | FC na USB-u (COMy), `--latency --msp COMy` | Medijana < 100 ms |
| **UT-UPL-A4** telemetrija | Monitor + kokpit preko USB kabla | Stav ≥ 5 Hz, GPS i baterija ≥ 1 Hz. Pitch i roll se slažu sa USB-om (inače `ATT_IN_DEGREES = true`). FM se prikazuje (npr. `STAB`). |
| **UT-UPL-A5** gimbal | `--tilt -50`, `--tilt 50`, pa `--pan -50`, `--pan 50` | Nagib (M5) i pan (M6) se pomjeraju, i bez prekidača |
| **UT-GMB-1** kalibracija | Postupak 4.4 za obje ose | Dvije tačke po osi izmjerene, vrijednosti upisane u `GIMBAL_*_CAL` |
| **UT-GMB-2** smjer | Kokpit, *Down 90*, pa pan +20° | Kamera gleda dolje i desno. Ugao na panelu ±3° od izmjerenog |
| **UT-GMB-3** granice | Strelicama do kraja obje ose | Gimbal staje prije mehaničkog graničnika, servo ne zuji u krajnjem položaju |
| **UT-GMB-4** georeferenciranje | Dron na zemlji, nagib 90°, pan +30°, objekat poznatog položaja u kadru | Pin na mapi pada na pravu stranu i u krug nesigurnosti; dok se gimbal okreće, pin miruje |
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
| ELRS Wide šalje CH9 | A5 (pan se pomjera) | Packet rate sa bar 12 kanala, ili gimbal pan na drugi slobodan AUX kanal (promijeniti i Servos tab i `channel_index`) |
| Betaflight odgovara na MSP_SERVO | GMB-1 (alat ispisuje M5/M6) | Bez toga izvor je radio echo; kokpit to sam radi, samo je manje tačno |

## 9. Gdje je šta u projektu

| Dio | Fajl |
|---|---|
| Protokol komandi i statusa | `DroneBackend/src/radio/UplinkProtocol.h/.cpp` |
| Stanje komandi, zakup, ograničenja | `DroneBackend/src/radio/CommandLink.h/.cpp` |
| Pisanje na Pocket-ov port | `DroneBackend/src/radio/CrsfLink.h/.cpp` (`set_uplink`) |
| Python veza | `DroneBackend/src/bindings/UplinkBindings.h` (poziva se iz `Bindings.cpp`) |
| Kontroler (25 Hz) | `DroneCockpitUI/workers/uplink_controller.py` |
| Regulator visine | `DroneCockpitUI/control/altitude_hold.py` |
| Gimbal: kalibracija, model servoa, uglovi | `DroneCockpitUI/control/gimbal.py`, konstante `GIMBAL_*` u `DroneCockpitUI.py` |
| Očitavanje servo izlaza (MSP_SERVO) | `DroneBackend/src/core/DroneLink.h/.cpp` (`servoValues`) |
| Gimbal u georeferenciranju | `DroneBackend/src/detection/DetectionLink.h/.cpp` (`gimbalRollDeg`, `gimbalMoving`) |
| Prozor *Laptop Control* | `DroneCockpitUI/widgets/command_panel.py` |
| Skripta za Pocket | `Firmware/pocket/SCRIPTS/MIXES/radcmd.lua` |
| Bench alat | `DroneCockpitUI/tools/uplink_bench.py` |
| Testovi | `DroneBackend/tests/test_uplink.cpp`, `DroneCockpitUI/tests/unit/test_uplink_python.py`, `DroneCockpitUI/tests/unit/test_lua_bridge.py`, `DroneCockpitUI/tests/unit/test_gimbal.py` |

**Pokretanje testova:**
```
cd DroneBackend\tests
cl /std:c++17 /EHsc /I..\src\radio test_uplink.cpp ..\src\radio\UplinkProtocol.cpp ..\src\radio\CommandLink.cpp ..\src\radio\CrsfProtocol.cpp
test_uplink.exe                   # 676 provjera

cd ..\..\DroneCockpitUI
pip install lupa
pytest tests/unit                 # sa test_uplink_python i test_lua_bridge
```
(`cl` se pokreće iz *Developer Command Prompt for VS*. Bez `test_uplink.exe` Lua skripta se provjerava samo protiv Python dekodera, a bez `lupa` se test Lua skripte preskače. Poslije izmjena u `UplinkProtocol` treba ponovo prevesti `test_uplink.exe`, jer test poredi njegov izlaz sa Python dekoderom.)
