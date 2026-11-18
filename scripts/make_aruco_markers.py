#!/usr/bin/env python3
"""Generate printable ArUco markers matching misty_params.yaml.

    python3 scripts/make_aruco_markers.py            # ids from misty_nav mission
    python3 scripts/make_aruco_markers.py 1 2 3 4    # specific ids

Writes markers/aruco_<id>.png. When printing, set the IMAGE WIDTH to the
printed total width (the script prints it), so the black square comes out
exactly marker_size_m wide. Mount markers flat, facing the direction Misty
approaches from; measure the printed square and set marker_size_m to it.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
cfg = yaml.safe_load(open(ROOT / 'src/misty_ros/config/misty_params.yaml'))
sensors = cfg['misty_sensors']['ros__parameters']
ids = [int(a) for a in sys.argv[1:]] or cfg['misty_nav']['ros__parameters']['mission']
size_m = float(sensors['marker_size_m'])
dictionary = cv2.aruco.getPredefinedDictionary(int(sensors['aruco_dictionary']))
dpi = 300
px = int(round(size_m / 0.0254 * dpi))
out = ROOT / 'markers'
out.mkdir(exist_ok=True)
for i in ids:
    img = cv2.aruco.generateImageMarker(dictionary, i, px)
    border = px // 4                                     # white quiet zone around the marker
    img = cv2.copyMakeBorder(img, border, border + 60, border, border, cv2.BORDER_CONSTANT, value=255)
    cv2.putText(img, f'ArUco id {i}  dict {sensors["aruco_dictionary"]}  {size_m * 100:.0f} cm',
                (border, img.shape[0] - 25), cv2.FONT_HERSHEY_SIMPLEX, 1.2, 0, 2)
    path = out / f'aruco_{i}.png'
    cv2.imwrite(str(path), img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
    total_cm = img.shape[1] / px * size_m * 100
    print(f'{path}: print {total_cm:.1f} cm wide -> black square {size_m * 100:.1f} cm')
