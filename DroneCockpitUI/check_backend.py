"""
check_backend.py — Project R.A.D.: does the rebuilt DroneBackend.pyd load,
and does it contain the ELRS radio support (CrsfLink)?

Put this file in the SAME folder as DroneCockpitUI.py and run:
    python check_backend.py

It uses the same .pyd location and DLL folders as DroneCockpitUI.py, so if
this works, the cockpit's import works too. Paste the output into the chat.
"""
import os
import sys

# ── Same path configuration as DroneCockpitUI.py — keep in sync ─────────────
script_dir   = os.path.dirname(os.path.abspath(__file__))
backend_path = os.path.abspath(os.path.join(script_dir, '..', 'x64', 'Release'))
OPENCV_BIN_DIR = r"C:\openCVBuild\build\bin\Release"
CUDA_BIN_DIR   = r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v13.3\bin\x64"
CUDNN_BIN_DIR  = r"C:\Program Files\NVIDIA\CUDNN\v9.24\bin\13.3\x64"

print(f"Python      : {sys.version.split()[0]}  ({sys.executable})")
print(f"Backend dir : {backend_path}")
pyd = os.path.join(backend_path, "DroneBackend.pyd")
if os.path.isfile(pyd):
    import datetime
    t = datetime.datetime.fromtimestamp(os.path.getmtime(pyd))
    print(f".pyd        : found, built {t:%Y-%m-%d %H:%M}")
else:
    print(".pyd        : NOT FOUND there — build Release|x64, or fix backend_path")

sys.path.append(backend_path)
valid = []
for d in (backend_path, OPENCV_BIN_DIR, CUDA_BIN_DIR, CUDNN_BIN_DIR):
    if os.path.isdir(d):
        os.add_dll_directory(d)
        valid.append(d)
    else:
        print(f"DLL dir     : missing {d}")
os.environ["PATH"] = os.pathsep.join(valid) + os.pathsep + os.environ["PATH"]

try:
    import DroneBackend as d
except ImportError as e:
    print(f"\nIMPORT FAILED: {e}")
    sys.exit(1)

print(f"Loaded from : {d.__file__}")
has = {name: hasattr(d, name) for name in
       ("CrsfLink", "RadioLinkStatus", "enumerate_serial_ports", "auto_detect_fc")}
for name, ok in has.items():
    print(f"  {name:24} {'OK' if ok else 'MISSING  <- old .pyd, rebuild Release|x64'}")

if has["enumerate_serial_ports"]:
    print("\nCOM ports:")
    ports = d.enumerate_serial_ports()
    if not ports:
        print("  (none)")
    for p in ports:
        print(f"  {p.port:6} {p.vid:04X}:{p.pid:04X}  usb='{p.bus_description}'  name='{p.friendly_name}'")

if has["CrsfLink"]:
    import time
    print("\nCrsfLink scan (3 s):")
    link = d.CrsfLink()
    found = link.connect_auto(3000)
    print(link.get_last_scan_report().rstrip() or "  (no report)")
    s = link.get_latest_state()
    print(f"  found={found}  port='{link.get_port_name()}'  status={s.radio.status_str}")
    if found:
        time.sleep(2.5)
        s = link.get_latest_state()
        r = s.radio
        print(f"  after 2.5 s: status={r.status_str} frames={r.frames_total} "
              f"LQ={r.uplink_lq}% attitude {r.attitude_hz:.1f} Hz  mode='{s.flight_mode_name}'")
    link.disconnect()
