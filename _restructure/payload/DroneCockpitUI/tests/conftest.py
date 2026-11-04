"""
Shared pytest setup for every suite under DroneCockpitUI/tests/:
puts DroneCockpitUI/ (the app packages) and <repo>/x64/Release|Debug
(DroneBackend.pyd) on sys.path, so tests import e.g.
``from widgets.IMUWidget import IMUWidget`` no matter where pytest is started.

    tests/unit/          fast tests, no hardware (default: plain `pytest`)
    tests/integration/   IT-* tests, need the flight controller on USB
    tests/system/        SAT-* tests, need the flight controller on USB
"""
import os
import sys

_APP = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _p in (os.path.join(_APP, "..", "x64", "Debug"),
           os.path.join(_APP, "..", "x64", "Release"),
           _APP):
    _p = os.path.abspath(_p)
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)
