#!/usr/bin/env python3
"""Compose assets/shoes.svg for the warehouse demo from the golf shoe artwork.

Source: ~/Downloads/golfShoeModel_sensorbedOverlay.svg
  - _Image1/_Image2: sensor-bed flex circuits (transparent PNGs)
  - _Image3: top-down shoes render (drawn on top at 0.75 opacity)
  - 16 red pads (8/foot, right foot explicit, left foot = mirrored group)

Output:
  - assets/shoes.svg with pads id'd pad-{left,right}-{0..7} in anatomical
    order (0 heel .. 7 hallux) + a #cop-layer overlay (trail pool + dot)
  - pad centers printed as a JS const for app.js
"""
import re, sys, os

SRC = os.path.expanduser("~/Downloads/golfShoeModel_sensorbedOverlay.svg")
OUT = os.path.expanduser("~/Documents/GitHub/warehouse-ergonomics-demo/assets/shoes.svg")

svg = open(SRC).read()

# ---- embedded images ----
imgs = dict(re.findall(r'<image id="(_Image\d)"[^>]*xlink:href="(data:image/png;base64,[^"]+)"', svg))
assert set(imgs) == {"_Image1", "_Image2", "_Image3"}, imgs.keys()

# ---- sensor bed group transforms (verbatim) ----
bed_r = re.search(r'<g id="sensorBed_right" transform="([^"]+)">', svg).group(1)
bed_l = re.search(r'<g id="sensorBed_left" transform="([^"]+)">', svg).group(1)

# ---- pad groups: first 8 = right foot; left foot repeats them under a mirror matrix ----
pads = re.findall(r'<g transform="matrix\(([^)]+)\)">\s*<path d="([^"]+)"', svg)
assert len(pads) == 16
right_pads = pads[:8]
mirror = re.search(r'<g transform="matrix\((-0\.999244[^)]+)\)">', svg).group(1)
M = [float(v) for v in mirror.split(",")]

# source file order -> anatomical order (0 heel, 1 mid-med, 2 mid-lat,
# 3 ball-med, 4 ball-ctr, 5 ball-lat, 6 forefoot-lat, 7 hallux)
ANAT = [4, 5, 6, 1, 2, 3, 7, 0]

def apply(m, x, y):
    a, b, c, d, e, f = m
    return a * x + c * y + e, b * x + d * y + f

centers_r, centers_l = [], []
pad_els_r, pad_els_l = [], []
for anat_i, src_i in enumerate(ANAT):
    mat, d = right_pads[src_i]
    mvals = [float(v) for v in mat.split(",")]
    cx, cy = apply(mvals, 722, 233)          # path bbox center
    lx, ly = apply(M, cx, cy)
    centers_r.append((cx, cy))
    centers_l.append((lx, ly))
    pad_els_r.append(f'    <g transform="matrix({mat})"><path id="pad-right-{anat_i}" class="pad" d="{d}"/></g>')
    pad_els_l.append(f'    <g transform="matrix({mat})"><path id="pad-left-{anat_i}" class="pad" d="{d}"/></g>')

# ---- COP zone: bounding box of all pad centers, padded ----
all_c = centers_r + centers_l
xs = [c[0] for c in all_c]; ys = [c[1] for c in all_c]
PAD = 55
rect = (min(xs) - PAD, min(ys) - PAD, (max(xs) - min(xs)) + 2 * PAD, (max(ys) - min(ys)) + 2 * PAD)

TRAIL_N = 40
trail = "\n".join(
    f'    <circle class="cop-trail" cx="-999" cy="-999" r="{4 + 6 * (i / TRAIL_N):.1f}" opacity="0"/>'
    for i in range(TRAIL_N)
)

out = f'''<svg viewBox="160 80 840 940" xmlns="http://www.w3.org/2000/svg" xmlns:xlink="http://www.w3.org/1999/xlink" xml:space="preserve" style="fill-rule:evenodd;clip-rule:evenodd;">
  <style>
    .pad {{ fill: #0e1330; opacity: 0.16; }}
    .cop-trail {{ fill: #00b894; }}
    #cop-dot {{ fill: #00d4aa; stroke: rgba(6,37,29,0.55); stroke-width: 3; }}
    #cop-zone {{ fill: none; stroke: rgba(26,31,71,0.28); stroke-width: 2; stroke-dasharray: 8 8; }}
    .cop-caption {{ font: 600 24px -apple-system, sans-serif; fill: rgba(26,31,71,0.45); letter-spacing: 0.08em; }}
  </style>
  <g id="sensorBed_right" transform="{bed_r}">
    <use xlink:href="#_Image1" x="0" y="0" width="332px" height="910px"/>
  </g>
  <g id="sensorBed_left" transform="{bed_l}">
    <use xlink:href="#_Image2" x="0" y="0" width="332px" height="910px"/>
  </g>
  <g id="pads-right">
{chr(10).join(pad_els_r)}
  </g>
  <g id="pads-left" transform="matrix({mirror})">
{chr(10).join(pad_els_l)}
  </g>
  <use xlink:href="#_Image3" x="0" y="0" width="1154px" height="1074px" opacity="0.75"/>
  <g id="cop-layer">
    <rect id="cop-zone" x="{rect[0]:.0f}" y="{rect[1]:.0f}" width="{rect[2]:.0f}" height="{rect[3]:.0f}" rx="26"/>
    <text class="cop-caption" x="{rect[0]:.0f}" y="{rect[1] - 16:.0f}">CENTER OF PRESSURE</text>
{trail}
    <circle id="cop-dot" cx="-999" cy="-999" r="14"/>
  </g>
  <defs>
    <image id="_Image1" width="332px" height="910px" xlink:href="{imgs['_Image1']}"/>
    <image id="_Image2" width="332px" height="910px" xlink:href="{imgs['_Image2']}"/>
    <image id="_Image3" width="1154px" height="1074px" xlink:href="{imgs['_Image3']}"/>
  </defs>
</svg>
'''

os.makedirs(os.path.dirname(OUT), exist_ok=True)
open(OUT, "w").write(out)
print(f"wrote {OUT} ({len(out)//1024} KB)")
print(f"COP_RECT: x={rect[0]:.0f} y={rect[1]:.0f} w={rect[2]:.0f} h={rect[3]:.0f}")
names = ["heel", "midMed", "midLat", "ballMed", "ballCtr", "ballLat", "foreLat", "hallux"]
print("  // pad centers in shoes.svg viewBox coords, anatomical order heel->hallux")
print("  const PAD_POS = {")
print("    left:  [" + ", ".join(f"[{x:.0f},{y:.0f}]" for x, y in centers_l) + "],")
print("    right: [" + ", ".join(f"[{x:.0f},{y:.0f}]" for x, y in centers_r) + "],")
print("  }; // " + " ".join(f"{i}:{n}" for i, n in enumerate(names)))
