# Firmware

Files that run on the radio and the flight controller, not on the laptop.

| Path | What |
|---|---|
| `pocket/SCRIPTS/MIXES/radcmd.lua` | EdgeTX mixer script for the RadioMaster Pocket: laptop command bridge (USB-VCP = LUA). Copy to `/SCRIPTS/MIXES/` on the Pocket's SD card. See [docs/modules/command-uplink-uputstvo.md](../docs/modules/command-uplink-uputstvo.md). |
| `betaflight/LISTIC_BF4.5.5_servo_mag_20261103.txt` | Current Betaflight 4.5.5 configuration (OMNIBUSF4SD): gimbal servos on M5/M6, compass. Paste into the CLI after flashing. |
| `betaflight/LISTIC_BF4.5.3_backup_20261103.txt` | Backup before the servo/compass firmware change (4.5.3). |

Firmware builds used: Betaflight 4.5.5 with the build options listed at the
top of the 4.5.5 file; EdgeTX 2.12 on the Pocket.
