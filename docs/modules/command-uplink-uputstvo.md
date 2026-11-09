# Komande sa računara preko Pocket-a – uputstvo

Project R.A.D. – upravljanje dronom iz aplikacije kontrolne stanice preko RadioMaster Pocket-a. Pocket ostaje predajnik, a pilot zadržava konačnu kontrolu.

> **Stanje:** kod je napisan i ispitan bez hardvera: 678 C++ provjera, 66 provjera Lua skripte pod simuliranim EdgeTX-om, 232 Python provjere komandi, 42 provjere gimbala i 18 testova sigurnosti i slijetanja. Na stvarnom Pocket-u i dronu **ništa još nije ispitano**. Prvi let je tek nakon što prođu bench testovi iz poglavlja 5, i to bez propelera.

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
5. **ANGLE prekidač i pilotove palice na radiju:** skripta propušta pomake samo dok je CH6 u ANGLE položaju i dok pilot ne pomjera palice (pilot override, poglavlje 7.3).
6. **Na računaru:** glavni prekidač, zakup komande od 300 ms (zaglavljena Python nit ne ostavlja staru komandu aktivnom), ograničenja i brzina promjene, provjera da je dron armiran i u ANGLE režimu, zaustavljanje pri gubitku videa ili slaboj vezi, automatsko slijetanje kad nema slike 5 s ili je baterija kritična, odbijanje rada uz nebezbjedan failsafe na dronu, geoograda na dometu prijemnika, upozorenja sa zvukom, i zapis svake komande u `DroneCockpitUI/logs/uplink_*.csv`.
7. **Failsafe na dronu** je zadnji nivo za svaki otkaz. Mora biti podešen kako je opisano u poglavlju 7.2.

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
- Ostala podešavanja leta se ne mijenjaju, osim failsafe-a (poglavlje 7.2, obavezno).

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
| **UT-UPL-A1** most | `python uplink_bench.py --port COMx` | Stižu `RAD_STATUS`, `fresh=True`, `crc errors 0`, period skripte ≤ 50 ms. Ako ima CRC grešaka u telemetriji, `serialWrite` možda reže nula-bajtove (vidi poglavlje 9). |
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
| **ALT-4** gas za slijetanje | Poslije ALT-3: okvir *FC AUTO-LAND* u panelu | „learnt hover … us" i preporuka. Upis u Betaflight po poglavlju 7.2 |

Simulacija (DroneCockpitUI/tests/unit/test_uplink_python.py) daje: greška lebdenja 0,06 m, pad baterije od 9 % kompenzovan na 0,07 m, korak od +2 m bez trajne greške. Model je jednostavan (bez vjetra, bez kašnjenja motora), pa su ovo samo provjere logike.

## 7. Sigurnosni mehanizmi i failsafe

Zaštita ima tri nivoa, i svaki radi bez ostalih:

1. **Dron (Betaflight).** Jedini nivo koji radi i kad nestane sve ostalo (radio, računar, video). Kad se izgubi veza, dron **slijeće** (AUTO-LAND, 7.2).
2. **Radio (`radcmd.lua` na Pocket-u).** Odlučuje šta od komandi kontrolne stanice uopšte izlazi na kanale. Ništa ne izlazi bez pilotovog prekidača, svježe komande i ANGLE prekidača. Pomaci ne izlaze ni dok pilot pomjera palice.
3. **Kontrolna stanica.** Šalje komandu samo kad su ispunjeni svi uslovi. Kad nema slike ili je baterija kritična, sama **spušta dron na zemlju** (7.4). Upozorava operatera trakom ispod alatne trake, zvukom i zapisom.

Kontrolna stanica **nikad nije potrebna** da bi dron sletio. Kad ona otkaže, dron leti po pilotovim palicama. Kad otkaže veza sa dronom, slijeće sam po Betaflight failsafe-u.

Granice su postavljene za radni opseg R.A.D. drona: domet prijemnika oko 5 km i let do 5 km visine. Zaštite djeluju tek na rubu tog opsega ili pri stvarnom otkazu. Kratki prekidi slike i pad kvaliteta veze, koji su na velikoj daljini normalni, ne zaustavljaju rad.

### 7.1 Šta se dešava kada nešto otkaže

| Otkaz | Kako se otkriva | Kontrolna stanica | Radio (`radcmd.lua`) | Dron (Betaflight) |
|---|---|---|---|---|
| **Gubitak FPV videa** (prijemnik, USB hvatač, crna slika) | `VideoLink`: *SignalLost* / *NoVideoInput*, ili nema novog kadra 1 s | Poslije 0,5 s: kretanje 0. Poslije **5 s** bez slike: **automatsko slijetanje** (7.4). Ako se slika vrati prije dodira sa zemljom: slijetanje se prekida, dron drži visinu, operater odlučuje | – | Spušta se po komandi gasa sa računara |
| **Gubitak ELRS veze** dron ↔ Pocket | Telemetrija `TELEMETRY_LOST` | Sve komande blokirane. CRITICAL „DRONE LINK LOST" | Komande ionako ne stižu do drona. EdgeTX javlja *Telemetry lost* | **Failsafe AUTO-LAND**: poslije `failsafe_delay` (1,5 s) dron drži naučeni gas za slijetanje (7.2) |
| Slaba veza (LQ < 30 % duže od 1 s) | `uplink_lq` iz ELRS statistike | Kretanje 0, visina i slijetanje rade dalje. WARNING | – | – |
| Kritična baterija | `battery_state` | Ako računar upravlja: **automatsko slijetanje**. Inače CRITICAL „land now" za pilota | – | Betaflight upozorenje (OSD, beeper) |
| USB kabl Pocket ↔ računar izvučen | `CrsfLink` bez porta, most ne odgovara | `NO_BRIDGE`, CRITICAL ako je *Laptop control* uključen | **Watchdog 500 ms**: izlazi na 0, palice i gas su pilotovi. Haptika i ton | Leti po pilotu |
| Aplikacija se zaledi (Tk nit, dijalog) | UI heartbeat 500 ms | Pritisnute tipke se poništavaju, kretanje 0. Visina i slijetanje rade dalje (druga nit) | – | – |
| Python nit kontrolera stane | Zakup komande 300 ms u `CommandLink`-u | C++ šalje neutralnu komandu | – | – |
| Izuzetak (greška) u kontroleru | `UplinkController._run` | `neutral()`, greška se broji i ispisuje. CRITICAL traka 10 s | Bez nove komande watchdog vraća pilotu | – |
| Proces padne, Windows ga ugasi, računar se isključi | Nema okvira | Zapis u `logs/crash_*.log` (i pad u nativnom kodu, `faulthandler`) | **Watchdog 500 ms** | Leti po pilotu |
| Pogrešno protumačena komanda (oštećen bajt, stari okvir, drugi program na portu) | CRC-8 na svakom okviru, brojač koji se mora mijenjati, verzija protokola, most se mora prvo predstaviti (`RAD_STATUS`) | Pokrenuta može biti samo **jedna** kontrolna stanica (mutex) | Okvir sa lošim CRC-om ili starim brojačem se ignoriše. Granice ±30 % palice, gas ±100 µs od pilotove palice | – |
| Pilot pomjeri palice (preuzima) | Skripta: roll/pitch/yaw > ~30 % | Panel: **PILOT OVERRIDE** | Pomaci sa računara na 0 dok pilot drži palicu i još 0,3 s poslije. Gas (visina, slijetanje) radi dalje | Leti po pilotu |
| ANGLE prekidač (CH6) nije u ANGLE | Skripta, CH6 izvan 1800–2100 µs | Kretanje blokirano | Pomaci i gas na 0 | – |
| Dron dalje od 5 km od kuće | GPS udaljenost i smjer prema kući | Od 4 km WARNING. Iza 5 km dozvoljeno samo kretanje prema kući (±60°), yaw slobodan | – | – |
| Visina iznad plafona | Baro | Cilj držanja visine najviše 5000 m iznad mjesta armiranja | – | – |
| Failsafe na dronu nebezbjedan | `MSP_FAILSAFE_CONFIG` preko USB-a, provjeren prema naučenom gasu lebdenja | *Laptop control* odbijen, CRITICAL traka | – | – |
| Dron viši nego što AUTO-LAND može spustiti | Brzina spuštanja × `failsafe_landing_time` | WARNING sa visinom koju failsafe pokriva | – | Poslije `failsafe_landing_time` Betaflight dezarmira, i to u zraku |
| Windows uspava računar ili isključi USB | – | Spriječeno (`SetThreadExecutionState`) | – | – |

**Pilot override i 0,3 s.** Za vrijeme override-a dron nije bez komandi. Pilotove palice idu na kanale cijelo vrijeme (ADD u mikseru). Na 0 su samo *pomaci sa računara*, a gas sa računara radi dalje. 0,3 s poslije puštanja palice služi samo da računar i pilot ne bi naizmjenično preuzimali komandu dok se palica vraća na sredinu. U ANGLE režimu dron se za to vrijeme sam vraća u nivo. Kad se pomaci vrate, kontroler ih uključuje postepeno (ograničena brzina promjene), bez trzaja. Ranije je ovo bilo 1 s, a sada je `OVERRIDE_HOLD = 30` (0,3 s).

### 7.2 Betaflight failsafe (obavezno prije prvog leta)

**Nalaz u konfiguraciji `LISTIC_BF4.5.5_servo_mag_20261103.txt` (KRITIČNO):** `failsafe_procedure = AUTO-LAND` je dobar izbor dok je GPS isključen, ali sa `failsafe_throttle = 1020` dron bi pri gubitku veze **pao**. AUTO-LAND drži gas `failsafe_throttle` tokom `failsafe_landing_time` i onda dezarmira, a 1020 µs je praktično ugašen motor. Kontrolna stanica to prepoznaje i odbija *Laptop control* (crvena traka „FC FAILSAFE UNSAFE").

**Gas za slijetanje se ne pogađa, nego se uči.** Kontrolna stanica za vrijeme svakog mirnog leta i lebdenja (i na postolju) bilježi gas koji radio stvarno šalje, vertikalnu brzinu iz barometra i napon baterije. Uzorci se uzimaju samo kad je dron u nivou (nagib < 8°) i kad se ne ubrzava. Iz njih računa (`control/landing_model.py`):

- **gas lebdenja** u zavisnosti od napona. Isti potisak traži približno isti proizvod (gas − 1000) × napon, pa se zna koliki je gas lebdenja i sa punom i sa praznom baterijom;
- **nagib**: koliko µs gasa daje 1 m/s spuštanja. Dok nema dovoljno podataka sa penjanjem i spuštanjem, uzima se početna vrijednost 35 µs po m/s. Prvo automatsko slijetanje sa računara izmjeri tačnu vrijednost.

Iz toga se bira **jedan** `failsafe_throttle` koji radi za cijelu bateriju:
- sa **punom** baterijom dron se mora spuštati bar 0,5 m/s, inače bi svježa baterija mogla dizati dron (odlijetanje);
- sa **praznom** baterijom ne brže od 3 m/s;
- cilj je 1 m/s sa punom baterijom.

`failsafe_landing_time` se računa iz najveće visine leta u sesiji i najsporijeg spuštanja, uz 15 % rezerve. Betaflight dozvoljava **najviše 250 s** (`get failsafe_landing_time` u CLI-ju ispisuje dozvoljeni opseg), pa AUTO-LAND sa 1 m/s pokriva oko 250 m. Kad je dron viši od toga, failsafe bi dezarmirao u zraku. Panel to prikazuje, a kontrolna stanica u letu daje upozorenje. Za letove visoko iznad mjesta polijetanja jedini dobar failsafe je **GPS-RESCUE**, kad GPS bude uključen (7.3).

Model se pamti u `DroneCockpitUI/landing_model.json` i prenosi u sljedeći let. Stari podaci postepeno gube težinu, tako da se nova masa (npr. teret) nauči za nekoliko minuta leta.

**Postupak:**
1. Na postolju ili u letu lebdi bar 10–20 s mirno (dron na USB-u ili na radiju, kokpit radi).
2. U *Laptop Control* panelu, okvir *FC AUTO-LAND*: „learnt hover … us" i preporuka, npr. `failsafe_throttle 1371 us, landing_time 60 s: 1.0 m/s full pack, 3.0 m/s empty`.
3. **Copy CLI**, pa sa dronom na USB-u, **dezarmiranim**, zalijepi u Betaflight CLI. Kopiraju se ove linije:
   ```
   set failsafe_procedure = AUTO-LAND
   set failsafe_throttle = <izračunato>
   set failsafe_landing_time = <izračunato>
   save
   ```
4. Ponovo priključi USB dok kokpit radi. U panelu treba da piše „FC failsafe: AUTO-LAND … m/s down now", bez crvene trake.

Kontrolna stanica provjerava i ručno upisanu vrijednost. Ako po modelu dron ne bi silazio (sporije od 0,2 m/s) ili bi padao (brže od 5 m/s), laptop komande su odbijene. Bez naučenog modela provjera je gruba: ispod 1100 µs je sigurno pad. Ostale vrijednosti tada dobijaju samo informaciju „not yet checked", i rad se ne blokira.

**Ostale postavke** (fajl `Firmware/betaflight/LISTIC_failsafe_safety.txt`):
- `failsafe_delay = 15` (faza 1 traje 1,5 s). `rxfail 5 s 2000` (ANGLE) ostaje.
- `rxfail 6 s 2000` uključuje GPS Rescue kutiju (AUX3) već u fazi 1. **Dok je GPS isključen** to treba promijeniti u `rxfail 6 h` (AUX3 zadržava zadnju vrijednost), jer GPS Rescue bez GPS-a ne može raditi.
- `gps_rescue_allow_arming_without_fix = ON` je sada potreban, jer je GPS isključen zbog testova u zatvorenom. Kad GPS bude uključen, vraća se na `OFF` (vidi 7.3).

### 7.3 Kad GPS bude uključen

Kad se pređe na letove napolju sa GPS-om:
- `set failsafe_procedure = GPS-RESCUE`, `set gps_rescue_allow_arming_without_fix = OFF`, `set gps_rescue_min_sats = 10`, a `rxfail 6 s 2000` se vraća;
- `gps_rescue_initial_climb`: sa `CURRENT_ALT` dron se pri spašavanju prvo penje toliko metara. Za visoke letove to treba prilagoditi;
- naučeni `failsafe_throttle` ostaje upisan kao rezerva;
- kontrolna stanica odbija *Laptop control* ako je failsafe GPS-RESCUE, a dron je armiran bez GPS-a. U tom slučaju spašavanje ne bi radilo.
- Ako GPS otkaže tokom spašavanja, Betaflight dezarmira dron.
- Kompas (`gps_rescue_use_mag`) uključiti tek kad je kalibrisan i ne smeta mu struja motora.

### 7.4 Automatsko slijetanje sa kontrolne stanice

Slijetanje koristi isti regulator kao držanje visine. Cilj visine se pomjera naniže zadatom brzinom: **1,5 m/s** dok je dron više od 20 m iznad mjesta armiranja, a niže **0,7 m/s**. Cilj nikad nije više od 2 m ispod stvarne visine, pa se ništa ne „navije" ako je spuštanje sporije.

- **Pokreće se:** tasterom **L** / dugmetom **LAND** u panelu, poslije **5 s bez slike**, ili kod **kritične baterije**. Automatski samo dok računar stvarno upravlja (*Laptop control*, pilotov prekidač, dron armiran). Ako držanje visine nije bilo uključeno, uključuje se samo, od trenutnog gasa.
- **Prekida se:** ponovo **L**, isključenjem pilotovog prekidača (pilot odmah preuzima), ili povratkom slike ako je slijetanje pokrenuo gubitak slike. Poslije prekida dron drži visinu na kojoj je.
- **Dodir sa zemljom:** gas je na donjoj granici, a vertikalna brzina ispod 0,25 m/s duže od 2 s. Tada panel i traka javljaju **LANDED**. Računar drži gas ispod pilotove palice (do 100 µs, nikad iznad), pa svako spuštanje palice smanjuje gas. **Dezarmira pilot.** Računar ne može dezarmirati, i to je namjerno.
- **Ako slijetanje ne može početi** (nema barometra, prekidač isključen…), traka javlja „CANNOT AUTO-LAND … pilot must land". Odbijeni zahtjev se ne pokreće kasnije sam od sebe.
- **Ograničenja:**
  - Gas sa računara je i dalje u pojasu ±100 µs oko pilotove palice. Pilot zato drži palicu gasa blizu lebdenja.
  - Visina se mjeri u odnosu na mjesto armiranja, pa teren na mjestu slijetanja nije poznat. Spori dio počinje 20 m iznad mjesta armiranja. Iznad višeg terena dron može dotaći zemlju i brže, do 1,5 m/s. Senzor udaljenosti od zemlje (rangefinder) bi to riješio.
  - Bez slike dron se spušta tamo gdje jeste i klizi sa vjetrom (u Betaflight 4.5 nema zadržavanja pozicije).

Svako slijetanje sa računara daje i podatke o spuštanju, pa model (7.2) poslije njega zna i nagib, a ne samo gas lebdenja.

### 7.5 Radio

- **RP4TD (ELRS, CRSF):** kad nestane veza, prijemnik prestaje da šalje kanale i Betaflight sam prepoznaje gubitak signala. Na prijemniku se ne podešava „zadnja pozicija" niti zadate vrijednosti kanala.
- **EdgeTX alarmi:** `MODEL → TELEMETRY`. Alarmi *RSSI/RQly low* i *critical* trebaju biti uključeni, kao i obavještenje *Telemetry lost*. Vrijednosti postaviti prema ExpressLRS preporukama za izabrani packet rate.
- **`radcmd.lua`** (tabela `CFG`):
  - `ANGLE_CH = "ch6"`, `ANGLE_MIN = 600`, `ANGLE_MAX = 1200`: pomaci i gas prolaze samo dok je CH6 u ANGLE položaju (1800–2100 µs, isto kao `aux 1 1 1 1800 2100`).
  - `OVERRIDE = 300`, `OVERRIDE_HOLD = 30`: pilotova palica dalja od ~30 % gasi pomake sa računara do 0,3 s poslije puštanja. Gas sa računara ostaje.
  - `ALERTS = true`: haptika i ton kad komande sa računara prestanu dok je prekidač uključen, i kratka haptika na početku override-a.

### 7.6 Kontrolna stanica

- **Traka upozorenja** ispod alatne trake: crvena za CRITICAL, narandžasta za WARNING. Klik prikazuje sva aktivna upozorenja.
- **Zvuk:** svako novo upozorenje (WARNING i jače) daje zvuk, a CRITICAL se ponavlja svakih 10 s dok traje.
- **Zapisi** u `DroneCockpitUI/logs/`:
  - `safety_YYYYMMDD.log`: početak (`+`) i kraj (`-`) svakog upozorenja;
  - `crash_YYYYMMDD.log`: neuhvaćene greške i padovi;
  - `uplink_*.csv`: svaka komanda.
- **Zaštita procesa** (`core/runtime_guard.py`): samo jedna pokrenuta kontrolna stanica, bez uspavljivanja računara i gašenja USB-a, tajmer od 1 ms, prioritet *Above normal*. Nit Pocket-ovog porta (`CrsfLink`) i nit video prijema rade sa prioritetom *Above normal*.
- **Granice** su u `DroneCockpitUI/control/safety.py` i mijenjaju se samo tamo:

| Konstanta | Vrijednost | Značenje |
|---|---|---|
| `GEOFENCE_M` / `RANGE_WARN_M` | 5000 / 4000 m | domet prijemnika: iza toga samo prema kući / upozorenje |
| `ALT_CEILING_M` | 5000 m | najveći cilj visine iznad mjesta armiranja |
| `MIN_LQ_PCT`, `LQ_LOW_S` | 30 %, 1 s | trajno slaba veza zaustavlja kretanje |
| `VIDEO_STOP_S`, `VIDEO_LAND_S` | 0,5 s, 5 s | bez slike: stop kretanja / slijetanje |
| `LAND_FAST_MPS`, `LAND_SLOW_MPS`, `LAND_SLOW_ALT_M` | 1,5 / 0,7 m/s, 20 m | profil slijetanja |

**Bench režim** (`COCKPIT_UPLINK_BENCH=1`) preskače provjere armiranja, ANGLE režima, videa i failsafe-a, i ne pokreće automatsko slijetanje. Taster L radi. U letu se bench režim **nikad** ne koristi.

### 7.7 Bench testovi failsafe-a i slijetanja

| Test | Postupak | Prolaz |
|---|---|---|
| **FS-1** očitavanje | Kokpit radi, dron na USB-u, otvori *Laptop Control* | Sa sadašnjom konfiguracijom: crvena traka „FC FAILSAFE UNSAFE … 1020 us" |
| **FS-2** učenje gasa | Na postolju, sa propelerima, 20 s mirnog lebdenja | Panel: „learnt hover … us", preporuka sa `failsafe_throttle` i `landing_time` |
| **FS-3** upis | Copy CLI → Betaflight CLI (dezarmiran), pa ponovo USB | „FC failsafe: AUTO-LAND … m/s down now", bez crvene trake |
| **FS-4** gubitak veze (na postolju) | Lebdenje na postolju, ugasi Pocket | Poslije ~1,5 s dron se polako spušta (≈ brzina iz preporuke), poslije `landing_time` dezarmira |
| **FS-5** gubitak videa | Bez bench režima, armiran u ANGLE, *Laptop control* i prekidač uključeni, lebdenje sa držanjem visine. Izvuci USB hvatač videa | 0,5 s: pomaci 0. Odbrojavanje „auto-land in … s". Poslije 5 s: AUTO-LANDING. Vrati hvatač prije dodira sa zemljom: slijetanje se prekida, visina se drži |
| **FS-6** slijetanje L | Lebdenje sa držanjem visine, taster L | Kontrolisano spuštanje, LANDED na zemlji, gas ispod pilotove palice. Pilot dezarmira |
| **FS-7** pilot override | Drži W, pomjeri pilotov roll > 30 % | PILOT OVERRIDE, Pocket kratko vibrira, visina se ne mijenja. 0,3 s poslije puštanja W ponovo radi |
| **FS-8** ANGLE prekidač | CH6 u srednji položaj (HORIZON) | „ANGLE switch off on the radio", pomaci 0 |
| **FS-9** pad aplikacije | Tokom FS-6 ugasi proces u Task Manager-u | ≤ 500 ms: gas i palice pilotovi, Pocket vibrira i daje ton |
| **FS-10** druga instanca | Pokreni kokpit dva puta | Druga instanca javlja da kokpit već radi i zatvara se |
| **FS-11** zapisi | Poslije FS-4 … FS-9 | `logs/safety_*.log` ima redove `+`/`-` za svaki događaj |

### 7.8 Propisi

Okvir je EU uredba **2019/947** (pravila letenja) i **2019/945** (zahtjevi za letjelice). Let na 5 km daljine i visine izlazi iz otvorene kategorije (120 m, vizuelni kontakt). Za to treba **specifična kategorija** i odobrenje vlasti, a za akcije traganja i spašavanja često posebna pravila ili izuzeća za državne službe. Kako se to primjenjuje u Bosni i Hercegovini treba provjeriti kod **BHDCA** (Direkcija za civilnu avijaciju BiH). Ovo nije pravni savjet. Sistem nema ugrađene zakonske granice, već samo granice radnog opsega iz 7.6.

### 7.9 Šta još nema

- **Zadržavanje pozicije:** Betaflight 4.5 ga nema. Pri gubitku slike dron se spušta tamo gdje jeste i klizi sa vjetrom. Betaflight 2025.12 donosi *POS HOLD* (GPS i kompas). Sa njim bi slijetanje bez slike moglo držati poziciju.
- **Povratak kući sa računara:** i dalje samo pilotovim prekidačem (AUX3 = GPS Rescue), kad GPS bude uključen.
- **Udaljenost od zemlje:** bez senzora udaljenosti (rangefinder) slijetanje ne zna teren ispod drona.

## 8. Postupak u letu

0. **Prije leta:** dron jednom na USB dok kokpit radi (failsafe se očita), bez crvene trake upozorenja, upisan naučeni gas za slijetanje (7.2), video radi, bench režim isključen.
1. Poletanje i armiranje kao i obično, **ANGLE** režim.
2. U kokpitu otvori **🕹 Laptop Control** i uključi *Laptop control*.
3. Dovedi dron u lebdenje i drži palicu gasa na tom položaju.
4. Uključi **pilotov prekidač**. Stanje u panelu prelazi u `GATE_ON`.
5. *Altitude hold* (H): gas preuzima regulator od trenutne vrijednosti. Pilot drži palicu gasa blizu lebdenja, jer je ona sidro za ±100 µs.
6. W/A/S/D/Q/E za kretanje (drži tipku), Space za stop, R/F za visinu.
7. **Povratak kontrole:** isključi prekidač. Palice i gas su odmah pilotovi. Isto se dešava čim pilot pomjeri palice (pilot override).
8. **Slijetanje:** L (ili dugme LAND). Na zemlji, kad panel javi LANDED, pilot spušta gas i dezarmira.
9. Kad se pojavi crvena traka (video, veza, baterija): računar je već zaustavio kretanje. Bez slike 5 s, ili kod kritične baterije, sam počinje slijetanje. Pilot ga može prekinuti tasterom L ili prekidačem.

Bez GPS zadržavanja pozicije dron u ANGLE režimu polako klizi sa vjetrom. Računar daje nagib, a ne poziciju.

## 9. Pretpostavke koje treba potvrditi na Pocket-u

| Pretpostavka | Gdje se provjerava | Ako ne važi |
|---|---|---|
| `serialRead`/`serialWrite` rade iz mix skripte | A1 | Prebaciti čitanje u function skriptu, pa vrijednosti predavati preko GV |
| `serialWrite` šalje i nula-bajtove | A1 (CRC greške) | Telemetrija se ne može slati u binarnom obliku, pa treba drugo kodiranje |
| `getValue("FM")` vraća tekst | A4 | Bez FM-a kokpit ne vidi ARM ni ANGLE i blokira komande. Rješenje: čitati prekidače ARM/ANGLE u skripti. |
| Ptch/Roll/Yaw su u radijanima | A4 | `ATT_IN_DEGREES = true` |
| Skripta stane u ograničenje CPU-a | S4 | Smanjiti `TLM_PER_RUN` na 1 (izmjereno u simulaciji: 1 400–2 300 Lua instrukcija po pokretanju) |
| ELRS Wide šalje CH9 | A5 (pan se pomjera) | Packet rate sa bar 12 kanala, ili gimbal pan na drugi slobodan AUX kanal (promijeniti i Servos tab i `channel_index`) |
| Betaflight odgovara na MSP_SERVO | GMB-1 (alat ispisuje M5/M6) | Bez toga izvor je radio echo; kokpit to sam radi, samo je manje tačno |

## 10. Gdje je šta u projektu

| Dio | Fajl |
|---|---|
| Protokol komandi i statusa | `DroneBackend/src/radio/UplinkProtocol.h/.cpp` |
| Stanje komandi, zakup, ograničenja | `DroneBackend/src/radio/CommandLink.h/.cpp` |
| Pisanje na Pocket-ov port | `DroneBackend/src/radio/CrsfLink.h/.cpp` (`set_uplink`) |
| Python veza | `DroneBackend/src/bindings/UplinkBindings.h` (poziva se iz `Bindings.cpp`) |
| Kontroler (25 Hz) | `DroneCockpitUI/workers/uplink_controller.py` |
| Sigurnosna pravila i granice | `DroneCockpitUI/control/safety.py` |
| Učenje gasa lebdenja, preporuka za AUTO-LAND | `DroneCockpitUI/control/landing_model.py` (`landing_model.json`) |
| Slijetanje (regulator) | `DroneCockpitUI/control/altitude_hold.py` (`start_landing`) |
| Upozorenja, zvuk, `safety_*.log` | `DroneCockpitUI/core/safety_monitor.py` |
| Zaštita procesa, `crash_*.log` | `DroneCockpitUI/core/runtime_guard.py` |
| Očitavanje failsafe-a (MSP_FAILSAFE_CONFIG) | `DroneBackend/src/core/DroneLink.h/.cpp` (`fsProcedure`, `fsThrottle`) |
| Betaflight failsafe (AUTO-LAND, naučeni gas) | `Firmware/betaflight/LISTIC_failsafe_safety.txt` |
| Regulator visine | `DroneCockpitUI/control/altitude_hold.py` |
| Gimbal: kalibracija, model servoa, uglovi | `DroneCockpitUI/control/gimbal.py`, konstante `GIMBAL_*` u `DroneCockpitUI.py` |
| Očitavanje servo izlaza (MSP_SERVO) | `DroneBackend/src/core/DroneLink.h/.cpp` (`servoValues`) |
| Gimbal u georeferenciranju | `DroneBackend/src/detection/DetectionLink.h/.cpp` (`gimbalRollDeg`, `gimbalMoving`) |
| Prozor *Laptop Control* | `DroneCockpitUI/widgets/command_panel.py` |
| Skripta za Pocket | `Firmware/pocket/SCRIPTS/MIXES/radcmd.lua` |
| Bench alat | `DroneCockpitUI/tools/uplink_bench.py` |
| Testovi | `DroneBackend/tests/test_uplink.cpp`, `DroneCockpitUI/tests/unit/test_uplink_python.py`, `DroneCockpitUI/tests/unit/test_lua_bridge.py`, `DroneCockpitUI/tests/unit/test_gimbal.py`, `DroneCockpitUI/tests/unit/test_safety.py` |

**Pokretanje testova:**
```
cd DroneBackend\tests
cl /std:c++17 /EHsc /I..\src\radio test_uplink.cpp ..\src\radio\UplinkProtocol.cpp ..\src\radio\CommandLink.cpp ..\src\radio\CrsfProtocol.cpp
test_uplink.exe                   # 678 provjera

cd ..\..\DroneCockpitUI
pip install lupa
pytest tests/unit                 # sa test_uplink_python, test_lua_bridge, test_gimbal i test_safety
```
(`cl` se pokreće iz *Developer Command Prompt for VS*. Bez `test_uplink.exe` Lua skripta se provjerava samo protiv Python dekodera, a bez `lupa` se test Lua skripte preskače. Poslije izmjena u `UplinkProtocol` treba ponovo prevesti `test_uplink.exe`, jer test poredi njegov izlaz sa Python dekoderom.)
