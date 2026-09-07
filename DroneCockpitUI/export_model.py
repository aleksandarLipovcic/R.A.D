"""
export_model.py — One-off export: training run -> app-relative ONNX model
===========================================================================
Run this ONCE (or whenever you retrain) from anywhere; it writes two files
into DroneCockpitUI/models/, which is what DetectionLink.setModelPath()
should point at -- always as a path relative to the app, never the
absolute NNTraining path on this machine, since that path won't exist on
another machine, after a folder rename, or once you archive the training
run.

Usage (from anywhere):
    pip install ultralytics --break-system-packages   # if not already installed
    python export_model.py

Requires ultralytics (the same package train.py already uses).
"""

from pathlib import Path
from ultralytics import YOLO

# ── Edit these two if your layout differs ──────────────────────────────

# The training run to export. Point this at wherever yolom_main_run
# actually landed on THIS machine -- this script's own location doesn't
# matter, only where it writes its output (below).
RUN_DIR = Path(
    r"C:\Users\TheSilent\Desktop\Project R.A.D\DroneCockpitUI\NNTraining\runs\detect\yolom_main_run"
)
WEIGHTS = RUN_DIR / "weights" / "best.pt"

# Where the app actually looks. This directory ships/lives inside the
# repo next to main.py -- resolve it however your project structure
# does elsewhere; APP_ROOT here assumes this script sits at the repo
# root alongside main.py. Adjust the single line below if it doesn't.
APP_ROOT = Path(__file__).resolve().parent
MODEL_DIR = APP_ROOT / "models"

# MUST match DetectionLink::setInputSize() at runtime. yolom_main_run was
# trained at 960 -- exporting at 960 keeps full training accuracy but
# costs more inference time per pass than 640. If detection passes are
# too slow on CPU, export at 640 instead and call setInputSize(640) to
# match (accuracy trade-off, not a correctness one -- just make the two
# agree).
EXPORT_IMGSZ = 960

# ── Export ───────────────────────────────────────────────────────────

def main():
    if not WEIGHTS.exists():
        raise SystemExit(f"Weights not found: {WEIGHTS}\n"
                          f"Check RUN_DIR/WEIGHTS above against your actual training output.")

    MODEL_DIR.mkdir(parents=True, exist_ok=True)

    model = YOLO(str(WEIGHTS))

    # opset=12 is broadly compatible with OpenCV's ONNX importer across
    # OpenCV versions; simplify=True runs onnx-simplifier so the graph
    # matches the [1, 4+numClasses, N] layout DetectionLink::runInference
    # assumes (Ultralytics' default export layout -- if you ever see
    # nonsensical boxes after swapping models, this shape assumption is
    # the first thing to re-check, per the comment in runInference()).
    exported_path = model.export(format="onnx", imgsz=EXPORT_IMGSZ, opset=12, simplify=True)

    final_onnx = MODEL_DIR / "yolo26m_main.onnx"
    Path(exported_path).replace(final_onnx)

    # DetectionLink::setModelPath() auto-loads a sibling ".names" file
    # with one class name per line, index == line number == the model's
    # own class index order -- model.names is an {index: name} dict in
    # that same order, so this just needs to preserve it.
    names_path = final_onnx.with_suffix(".names")
    with open(names_path, "w") as f:
        for i in sorted(model.names.keys()):
            f.write(f"{model.names[i]}\n")

    print(f"Wrote {final_onnx}")
    print(f"Wrote {names_path}")
    print(f"Classes ({len(model.names)}): {list(model.names.values())}")
    print(f"\nIn main.py (DroneBackend's pybind11 API is snake_case, not "
          f"the C++ method names -- see Bindings.cpp):")
    print(f'    DETECTION_MODEL_PATH = str(APP_ROOT / "models" / "yolo26m_main.onnx")')
    print(f'    DETECTION_INPUT_SIZE = {EXPORT_IMGSZ}   # must match EXPORT_IMGSZ above')
    print(f'    detection_link.set_model_path(DETECTION_MODEL_PATH)')
    print(f'    detection_link.set_input_size(DETECTION_INPUT_SIZE)')


if __name__ == "__main__":
    main()