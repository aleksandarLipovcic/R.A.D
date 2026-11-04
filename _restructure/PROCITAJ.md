# Reorganizacija projekta R.A.D – jednokratno

Ovaj folder sadrži sve što treba za novu strukturu projekta i ugradnju komandi preko Pocket-a. Ne briši ga dok ne završiš korake ispod.

## Pokretanje

1. **Zatvori Visual Studio.**
2. Commit-uj trenutno stanje (skripta odbija da radi ako postoje necommit-ovane izmjene).
3. U terminalu, u folderu `Project R.A.D`:
   ```
   python _restructure\restructure_project.py --dry-run
   python _restructure\restructure_project.py
   ```
   Prva komanda samo prikazuje plan. Druga premješta fajlove (`git mv`, pa se istorija čuva) i upisuje izmijenjene fajlove iz `payload/`.

## Provjera

1. `git status`: premještanja se vide kao *renamed*.
2. Visual Studio → otvori `Project R.A.D.slnx` → **Rebuild** DroneBackend (Release x64).
3. `cd DroneCockpitUI` pa `pytest`: 90 testova, kao prije reorganizacije plus 3 nova.
4. Pokreni `DroneCockpitUI.py`. Rasporedi panela se čitaju iz `config/`.
5. Ako je sve u redu: `git commit`, pa obriši folder `_restructure`.

## Vraćanje na staro (prije commit-a)

```
python _restructure\restructure_project.py --undo
```
Vraća sve fajlove na stara mjesta i sa starim sadržajem.

## Šta se mijenja

- `DroneBackend/src/{core,sensors,radio,video,detection,bindings}`, plus `tests/` i `scripts/`
- `DroneCockpitUI/{widgets,workers,core,control,config,tools}` i `tests/{unit,integration,system}`
- `Firmware/` (Pocket Lua skripta, Betaflight backup-i)
- `docs/{doxygen,hardware,screenshots}`
- Projektni fajlovi (`.vcxproj`, `.filters`, `.pyproj`, `.slnx`), importi i dokumentacija su usklađeni sa novim putanjama.
- Ugrađene su komande sa računara preko Pocket-a: `CommandLink`, dugme **🕹 Laptop Control**, testovi i uputstvo (`docs/modules/command-uplink-uputstvo.md`).

`NNTraining/` sa podacima i modelima ostaje gdje jeste.
