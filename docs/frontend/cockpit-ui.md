# Izgled kontrolne stanice: kokpit raspored i vojna tema

Ovaj dokument opisuje vizuelni dio kontrolne stanice: pilotski prozor (`DroneCockpitUI.py`) i prozor kopilota (`widgets/DetectionMapWidget.py`).

## 1. Principi

- **Raspored po prioritetu.** U većem prozoru ima više instrumenata. Kad se prozor smanjuje, ostaje samo ono što je pilotu neophodno za let. Prozor se ne može smanjiti ispod najmanjeg formata.
- **Boje imaju značenje** (konvencija iz avijacije, FAA AC 25-11B, MIL-STD-1472):

| Boja | Značenje | Primjeri |
|---|---|---|
| crvena | upozorenje, odmah reagovati | MASTER WARN, DISARMED, gubitak videa, kritična baterija |
| žuta (amber) | oprez | MASTER CAUT, degradirana veza, upozorenje baterije |
| zelena | normalno, uključeno, bezbjedno | ARMED, režim leta, ispravni senzori |
| cijan | izabrano, informacija | zaglavlja panela, izabrana vrijednost |
| magenta | cilj, navigacija | |
| bijela | izmjerena vrijednost | brojevi na instrumentima |
| zeleno-siva | natpisi, jedinice, skale | |

  Crvena, žuta i zelena se koriste samo za stanje, nikad kao ukras.
- **Tamna podloga** (crno-maslinasta), pa je ekran čitljiv i noću i ne zasljepljuje.
- **Fontovi:** natpisi u *Bahnschrift* (DIN stil, dolazi uz Windows 10/11), brojevi u *Consolas* (fiksna širina, cifre ne „skaču" dok se vrijednost mijenja).

## 2. Kokpit raspored (◧ COCKPIT)

Paneli se slažu sami, prema veličini prozora. Prioritet instrumenata:

| Prioritet | Panel | Zašto |
|---|---|---|
| 1 | FPV video | pilotove oči |
| 1 | ADI | položaj drona; traka sa kursom je već u njemu |
| 1 | ALT / VSI | visina, desno od ADI-ja (kao visinomjer u „basic T" rasporedu) |
| 1 | FC status | ARMED, režim leta, baterija, veza |
| 2 | magnetometar (kurs) | ispod ADI-ja, kao HSI |
| 3 | GPS / mapa | navigacija |
| 4 | pre-flight lista, IMU detalji | sistemski podaci |

Formati (radna površina = prozor bez alatne trake):

| Format | Prikazano | Okvirna veličina prozora |
|---|---|---|
| ESSENTIAL | FPV, ADI, ALT/VSI, FC status | od ~850 × 480 (najmanja dozvoljena) |
| STANDARD | + magnetometar | od ~1170 × 750 |
| EXTENDED | + GPS / mapa | od ~1170 × 790 |
| FULL | + pre-flight lista i IMU | od ~1550 × 790 (npr. 1920 × 1080) |

- Format koji se trenutno koristi piše pored dugmeta ◧. Kad se pređe mišem preko njega, vidi se šta nije prikazano.
- Format se bira tako da instrumenti dobiju udobnu veličinu (`PREF_SIZE`), a ne samo najmanju. Zato prozor koji raste prvo poveća video, pa tek onda doda instrumente.
- Kod granice formata postoji histereza od 24 px, pa se raspored ne „trese" dok se prozor razvlači.
- Panel isključen u meniju *Panels ▾* ostaje isključen, a ostali paneli zauzmu njegov prostor (nema rupa).
- Naslovi panela se skraćuju kad nema mjesta: prikazuje se samo dio prije „—", npr. „ALT / VSI".
- **FREE (✥):** stari način rada (ručno pomjeranje, promjena veličine, snap). Režim se pamti u layout profilu. Postojeći profili bez ovog podatka se otvaraju u COCKPIT režimu, a njihov ručni raspored ostaje sačuvan za FREE režim. Fabrički FREE raspored je sada FULL format na 1600 × 920, bez preklapanja (stari je imao preklopljene panele).

Geometrija je u `core/cockpit_layout.py`: stablo podjela (horizontalna i vertikalna), najmanje i udobne veličine po panelu, testovi u `tests/unit/test_cockpit_layout.py`.

## 3. Alatna traka

Slijeva nadesno: **MASTER WARN / MASTER CAUT**, USB status, ELRS indikator, dugmad, format i ◧.

- **MASTER WARN** (crveno) svijetli dok postoji CRITICAL upozorenje, a **MASTER CAUT** (žuto) dok postoji WARNING. Treperi dok nije potvrđeno. Klik potvrđuje: treptanje i ponavljanje zvuka prestaju dok ne dođe **novo** upozorenje. Potvrda se upisuje u `logs/safety_*.log`. Tekst upozorenja je u traci ispod alatne trake.
- Kad je prozor uzak, traka se skraćuje ovim redom:
  1. dugmad ostaju samo sa ikonom;
  2. nestaju natpisi „Layout: …" i ugao kamere;
  3. nestaje oznaka formata;
  4. nestaju Help, Layouts i Camera Angle;
  5. nestaje Panels.

  Annunciatori, status veze, Detections, Laptop Control i ◧ ostaju uvijek.
- Dugmad za zaključavanje, vraćanje i fabrički raspored vide se samo u FREE režimu.

## 4. Prozor kopilota (Detections & Map)

Prioritet: 1 lista objekata, 2 mapa, 3 živa slika / snimak detekcije, 4 pretraga scena.

- Najmanja veličina prozora je 860 × 520.
- Pretraga scena (redovi „Frames with…", „Quick:" i tabela scena) prikazuje se od 1180 × 720. U manjem prozoru se otvara dugmetom **▸ SCENES**, a otvori se i sama kad se pokrene pretraga.
- Živa slika i snimak detekcije prikazuju se od visine 620. U nižem prozoru se otvaraju dugmetom **▸ LIVE**.
- Ispod širine 1180 dugmad mape se skraćuju (⊙, SAT, TOPO).
- Zaglavlja su u istom stilu kao paneli pilota. Tabele, padajuće liste i klizači imaju istu temu (`theme.apply_ttk`).

## 5. Tema

- `core/theme.py`. Svaka boja u UI-ju prolazi kroz `C("#rrggbb")`:
  - **military** (podrazumijevano): boja se preslikava na paletu po ulozi (crvena ostaje crvena, tamna podloga ostaje tamna, svjetlina se čuva);
  - **classic**: vraća originalne boje.
- Nebo i zemlja na ADI-ju ostaju plavo i smeđe, samo prigušeni.
- Izbor teme: *Layouts ▾ → Color theme*, ili `config/ui_theme.json` (`{"theme": "classic"}`), ili promjenljiva okruženja `RAD_UI_THEME`. Promjena važi od sljedećeg pokretanja.
- Za novi kod koristi se semantička paleta `theme.P` (`WARNING`, `CAUTION`, `NORMAL`, `ADVISORY`, `DATA`, `LABEL`, `BEZEL`…), a ne nove hex vrijednosti.
