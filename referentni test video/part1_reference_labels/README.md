# R.A.D validation — part0 + part1 reference labels (hand-labelled)

Reference labels for `rad_fpv_cropped_part0.mp4` (2000 frames) and `rad_fpv_cropped_part1.mp4` (1853 frames), 1342×1074, 60 fps, in your 5-class taxonomy:
0 person, 1 car, 2 large_vehicle, 3 motorcycle, 4 other_vehicle.

These labels were not produced by any detection network. Keyframes every 30 frames (67 keyframes) were labelled by eye. Frames in between were filled by blending a forward and a backward OpenCV CSRT tracker, anchored to linear interpolation between the two keyframes. The two parked cars are placed using the measured camera drift. Treat this as a first pass and confirm it with `label_editor.py`.

## Files
- `labels/part0/part0_NNNNNN.txt`: one YOLO txt per frame, 0-based (`part0_000000.txt` = first frame). Each line is `cls xc yc w h`, normalized; there are no confidence values.
- `part0_reference_boxes.csv`: the same boxes, one row per box, in pixel and normalized coordinates. `object_id` stays the same while an object is in view (c* = vehicle, p* = person).
- `part0_reference_labeled.mp4`: the video with boxes and class names, for viewing only.
- `label_editor.py`: the review/correction tool (key list at the top of the file).
- `render_labels.py`: re-renders the labelled video after you edit.
- `compare_detections.py`: compares your NN's output against these labels.

## Workflow
    pip install opencv-contrib-python numpy
    python label_editor.py --video rad_fpv_cropped_part0.mp4 --labels labels/part0
    # fix / draw / delete, press v on each checked frame, n jumps to the next unchecked one
    python render_labels.py --video rad_fpv_cropped_part0.mp4 --labels labels/part0 --out part0_reference_labeled.mp4
    python compare_detections.py --ref labels/part0 --pred <your_nn_labels> --pred-frame-base 1 --out cmp_part0

Your NN run should use `save_txt=True, save_conf=True`. Ultralytics numbers video label files from 1, which is why the command uses `--pred-frame-base 1`.

## Labelling decisions to check
- **Vans:** both white vans (c14 at about 26–30 s, c15 at about 26–33 s) are labelled large_vehicle, following your van→large_vehicle mapping. Every other vehicle is a car. There are no motorcycles or bicycles in part0.
- **Hard cases:** objects are labelled while partly hidden (entering behind the tree on the right, or cut off by the balcony ledge and frame edges) and dropped once they are essentially invisible. Check these frames first.
- **Pedestrians:** the far-sidewalk walkers are only 10–20 px wide, so their boxes are the least precise.

## part1 (added)
- `labels/part1/part1_NNNNNN.txt`, `part1_reference_boxes.csv`, `part1_reference_labeled.mp4`: same formats as part0; frames 0-based within part1.
- Object IDs continue from part0 (c1/c2 parked, p3/p4, c16 carry over from the end of part0).
- **Cyclist (b1, other_vehicle)** rides left along the far sidewalk from ~5 s to ~17 s. It is small and blurred; check its boxes first.
- **Van c18** (~12–17 s) is labelled large_vehicle. All other vehicles are cars. No motorcycles.
- p6 is a pedestrian standing/walking slowly at the rear of the left parked car (~22 s to end); p3 goes behind the parked car around 16 s and is dropped there.

    python label_editor.py --video rad_fpv_cropped_part1.mp4 --labels labels/part1
