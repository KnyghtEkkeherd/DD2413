#!/usr/bin/env python3
"""Generate printable ArUco markers matching misty_params.yaml.

    python3 scripts/make_aruco_markers.py            # ids in marker_map + mission (misty_params.yaml)
    python3 scripts/make_aruco_markers.py 1 2 3 4    # specific ids

Writes markers/aruco_markers.pdf (one A4 page per marker, true size) and
markers/aruco_<id>.png (300 dpi). Print the PDF at 100 % / "actual size", not
"fit to page": the black square (and the line above it) is marker_size_m wide. Mount markers flat, facing the direction Misty
approaches from; measure the printed square and set marker_size_m to it.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
cfg = yaml.safe_load(open(ROOT / 'src/misty_ros/config/misty_params.yaml'))
sensors = cfg['misty_sensors']['ros__parameters']
if sys.argv[1:]:
    ids = [int(a) for a in sys.argv[1:]]
else:   # every marker the robot needs: the measured marker_map plus the mission route
    marker_map = json.loads((cfg.get('/**') or {}).get('ros__parameters', {}).get('marker_map', '{}'))
    ids = sorted({int(i) for i in marker_map} | {int(i) for i in cfg['misty_nav']['ros__parameters']['mission']})
size_m = float(sensors['marker_size_m'])
dictionary = cv2.aruco.getPredefinedDictionary(int(sensors['aruco_dictionary']))
dpi = 300
px = int(round(size_m / 0.0254 * dpi))
cm = lambda c: int(round(c / 2.54 * dpi))
page_w, page_h = cm(21.0), cm(29.7)                      # A4 portrait
if px + 2 * cm(1.5) > page_w:                            # too big for A4: page = marker + 3 cm margins
    page_w = page_h = px + 2 * cm(3.0)
out = ROOT / 'markers'
out.mkdir(exist_ok=True)
pages = []
for i in ids:
    page = np.full((page_h, page_w), 255, np.uint8)
    x0, y0 = (page_w - px) // 2, (page_h - px) // 2
    page[y0:y0 + px, x0:x0 + px] = cv2.aruco.generateImageMarker(dictionary, i, px)
    label = f'ArUco id {i}   dict {sensors["aruco_dictionary"]}   black square {size_m * 100:.1f} cm   print at 100 %'
    cv2.putText(page, label, (x0, y0 + px + cm(1.2)), cv2.FONT_HERSHEY_SIMPLEX, 1.6, 0, 3)
    cv2.line(page, (x0, y0 - cm(1.0)), (x0 + px, y0 - cm(1.0)), 0, 2)       # ruler: measure = square width
    img = Image.fromarray(page)
    img.save(out / f'aruco_{i}.png', dpi=(dpi, dpi))
    pages.append(img)
pdf = out / 'aruco_markers.pdf'
pages[0].save(pdf, save_all=True, append_images=pages[1:], resolution=dpi)
print(f'{pdf}: {len(pages)} page(s), ids {list(ids)}, {page_w / dpi * 2.54:.1f} x {page_h / dpi * 2.54:.1f} cm pages')
print(f'Print at 100 % / "actual size" (NOT fit to page): the black square and the line above it are {size_m * 100:.1f} cm.')
