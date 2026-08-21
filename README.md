# Warehouse Ergonomics Demo

Live browser demo on the [Brilliant Wear JavaScript SDK](https://github.com/brilliantsole/BrilliantWear-JavaScript-SDK):
**footsteps · balance · center of pressure · lifting posture** for warehouse / box-moving scenarios.
Built for enterprise evaluations (systems-integrator and logistics conversations).

## What it shows
- **Center of pressure & balance** — a **3D stance** (the golf app's `RecordingStance3D`, ported to vanilla three.js: the same translucent shoe GLBs + sensor-bed art) showing each insole's **live heading / relative orientation**, per-pad pressure glow, per-foot + combined center of pressure, with the L/R load split + stability score below it
- **Footstep map** — waltz-chart-style numbered L/R footprints with a 10-second fade, direction from IMU yaw (insole-only; no external positioning SDK)
- **Lifting posture** — trunk flexion/lean/twist from a Sense unit on the upper back; optional second unit on the pelvis distinguishes hip-hinge (good) from stooped-back (risky) lifts
- **Per-insole + combined center of pressure** — a COP ring on each insole plus the combined both-feet COP with a sway trail
- **Haptic coaching** — insoles buzz the moment posture enters the red zone (on real captures)
- **Session metrics** — steps, cadence, asymmetry, lift counts (good vs risky), time-in-red

This is a **showcase + review** app. It has two modes and does **not** capture data itself —
**recording lives in the BrilliantWear portal** (Session Recording → the "Warehouse ergonomics" preset):

- **▶ Simulate** — a full walking + good/bad-lift loop with no hardware. For quick demos.
- **⤓ Load Recording** — open a session captured in the portal and replay the **entire dashboard
  scrubbed in sync with its video**, the golf-demo review experience (see below).

## Loading a recording

Export a session from the portal (its raw sensor **data JSON** + the **webcam video**), then
**⤓ Load Recording** and pick both files (select them together, or the JSON then the video).

The app re-derives the whole warehouse dashboard from the raw streams — per-insole & combined COP,
individual sensor pressure, trunk flexion / hip-hinge-vs-stoop, footstep map, step & lift counts —
and plays it back **synced to the video**: scrub or play the clip and the dashboard follows. With no
video, a slider scrubs the data. Everything runs locally in the browser; nothing is uploaded.

Two recording JSON shapes are accepted:

1. **Portal export** (from the portal's `GET /recordings/:id/export?format=json`, or the detail page's
   Download JSON) — flat, timestamped rows: `recording.devices[]` + `scalar[]` (`{time, device_id,
   sensor_type, x/y/z/w}`) + `pressure[]` (`{time, device_id, normalized_center_x/y, normalized_sum,
   sensors:[{position, normalizedValue, …}]}`) + `events[]`. Device role comes from each device's
   `placement` (`left foot`/`right foot` → feet, `upper back` → torso, `pelvis` → pelvis). Pressure is
   already normalized so it maps straight in; posture/heading need `gameRotation` (or `rotation`) in
   `scalar`, which requires recording with the **Warehouse ergonomics preset** (the default preset
   streams only acceleration/gyroscope → no orientation).
2. **SDK-nested** (`devices[].sensorData[]` with `rawValue` arrays + `positions` + `dataRate`).

Video sync uses `video.syncOffsetMs` from the JSON if present, else 0. (The portal keeps `syncOffsetMs`
on the video track, not in the JSON export — align with the scrubber if the downloaded clip looks off.)

## Vision foot positions (experimental)

By default the two shoes sit at a **fixed** stance and the footstep map uses an **estimated** stride.
Load a **foot-track** alongside the recording and toggle **Vision foot positions** to drive real
relative foot placement instead — feet closer/wider, one forward/back (Z), lift height (Y) — in both
the 3D stance and the footstep map (real step length / sway). Insoles still provide orientation,
pressure, COP, and step *timing*; vision provides *where* each foot is. Any low-confidence / occluded
frame falls back to the insole estimate.

Make a foot-track with **`tools/foot-track-extractor.html`**: open it, load the clip, and it runs
MediaPipe **Pose** in the browser (X lateral, Y lift) — optionally sampling the **Depth Anything V3**
server (`ws://localhost:8765`, from the SDK's `examples/depth-anything-v3`) for Z — and downloads a
`*.foottrack.json`. Load that file together with the recording + video.

Foot-track schema (`foot-track/v1`): `{ fps, video:{syncOffsetMs}, frames:[{ t, l:{x,y,z,c}, r:{x,y,z,c} }] }`
— x lateral (m, +right), z forward (m, +away), y lift (m), c confidence 0–1; `t` is video-relative ms.
`POS_SCALE` in `stance3d.js` maps metres → scene units.

**Caveats (prototype):** monocular depth is relative not metric and weakest on Z; a floor calibration
(homography) would make the top-down map metric; the extractor's per-foot depth samples the server's
colormapped JPEG as a proxy — extend the server to return raw depth for accuracy. See the extractor
page's own notes.

## Run it
Serve the folder and open it in Chrome/Edge:

```bash
npx serve .
```

(Serving matters: the shoe artwork is fetched from `assets/shoes.svg`, which `file://` blocks.
Opening `index.html` directly still works, minus the artwork.)

## Tuning notes

The warehouse pipeline (used for both Simulate and replay re-derivation) lives in `app.js`:

- Thresholds (flexion zones, step detection, stride) are in `CFG`/`MAP` at the top.
- **COP orientation** — `COP_FLIP_X` / `COP_FLIP_Y` near `COP_RECT`. If a replayed COP reads mirrored
  front↔back or left↔right vs. the wearer, flip the offending constant (one line).
- **Sensor→pad order** — `SDK_SENSOR_TO_PAD` maps an 8-sensor insole to the artwork pads (anatomical,
  0 heel → 7 hallux); other counts (e.g. 16-sensor beds) are placed by each sensor's `position`.
- A dormant `SDKAdapter` (verified against SDK v0.0.78) is kept as reference / re-enable point for
  live Web Bluetooth, but the app does not connect to hardware — capture is the portal's job.

**3D stance** (`stance3d.js`, ES module, three.js via import-map CDN): loads `assets/golf-shoe-{left,right}.glb`
+ `assets/sensorbed-{left,right}.png` (copied from the portal) and reads `window.WH` (the app's state) each
frame — shoe yaw from `S.footYaw`, pads from `S.sensors`, per-foot COP from `S.copFoot`. Field constants
(`SHOE_MODEL_FOR_SIDE`, `BED_ROTATION_Z`, `PAD_MIRROR_X`, `STANCE_TOE_SIGN`, `SHOE_MODEL_YAW`) carried over
from the golf widget. Per-foot heading = insole `gameRotation` yaw (replay) or a synthesized flare (Simulate).

**2D artwork** (`assets/shoes.svg`, `tools/build_shoes.py`) is now dormant — the `ShoeStage` code is kept
but the 3D stance replaced it in the UI.

## Deploy (GitHub Pages)
Push to a public repo → Settings → Pages → deploy from `main` root. The page is fully static;
the SDK loads from unpkg.

---
© Brilliant Sole, Inc. (dba Brilliant Wear)
