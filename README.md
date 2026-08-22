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

This is a **showcase + review** app. It does **not** capture data itself —
**recording lives in the BrilliantWear portal** (Session Recording → the "Warehouse ergonomics" preset):

- **☰ Recordings** (`recordings.html`) — a gallery of real captures. Each card plays the recording
  through the dashboard (`index.html?rec=<id>`) synced to its video, and offers the raw sensor data
  as **⬇ JSON** / **⬇ CSV**. Driven by `recordings/manifest.json`; add an entry + files to publish a take.
- **▶ Simulate** — a full walking + good/bad-lift loop with no hardware. For quick demos.
- **⤓ Load Recording** — open any portal export locally and replay the **entire dashboard
  scrubbed in sync with its video**, the golf-demo review experience (see below).
- **⬇ JSON / ⬇ CSV** on the dashboard export the loaded recording's raw data (JSON as-is; CSV is a
  flattened long format, one row per sample across scalar + pressure streams).

### Publishing a recording to the gallery
1. Portal → recording → **Download JSON**; **Download MP4** from its video track.
2. Optionally shrink the JSON to the streams the dashboard uses (orientation + pressure) — see
   `tools/` notes; WE 4 went from 28 MB → 5.7 MB this way.
3. Drop both into `recordings/`, add a `manifest.json` entry (`id`, `title`, `json`, `video`,
   `syncOffsetMs` from the portal's video track, `sensors`, `note`), commit, push.

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

## Vision foot positions (automatic)

When a recording has a video, the app runs MediaPipe **Pose** over it **automatically in the
background** (~20 s for a 47 s clip) and switches **Vision foot positions** on when done — no
separate tool. Gallery recordings ship a pre-baked `*.foottrack.json` (referenced by `footTrack` in
the manifest) so they load instantly. `tools/foot-track-extractor.html` still exists for batch /
offline use (with the optional Depth Anything V3 server for Z) and shares the same logic.

**What vision contributes — a deliberate split.** Each source does what it measures well:

| | source |
|---|---|
| orientation, pressure, COP, step *timing* | insoles |
| **step length** (stride) | insoles (Pose's monocular Z is relative and compressed) |
| **step height** | insoles — an unloaded + tilted foot is in the air (WE 4: 5–7 cm, physically right) |
| **stance width** (L/R lateral gap) | vision — applied along the insole-heading path, so the map stays a continuous walk |
| relative front/back (Z) | vision, as a soft cue |

**Vision never supplies lift (Y is always 0 in the track).** This was a hard-won decision from WE 4:
that clip was shot from waist height, close in — the shoulders were cut off (even *below* the hips
in the image at times) and the feet left the bottom of the frame whenever the wearer walked toward
the camera. MediaPipe then *extrapolates* the missing landmarks to plausible in-frame positions and
reports them at **visibility 1.0**, so no geometric or confidence gate can tell a real body reference
from a hallucinated one — every height formula tried (fixed floor row, percentile baseline, hip→heel
scale, torso scale, torso + confidence) pinned lift at its clamp. The insole signal has none of
these problems. Any foot that is out of frame or below `c ≥ 0.6` (`VISION_MIN_CONF`) falls back to
the insole estimate for that moment.

For the best vision tracking on future captures: **full body in frame, camera ~1 m high, ~3 m back,
wearer walking across the view rather than straight at it.**

Foot-track schema (`foot-track/v1`): `{ fps, video:{syncOffsetMs}, bodyVisible, frames:[{ t,
l:{x,y,z,c}, r:{x,y,z,c} }] }` — x lateral (m, +right), z forward (m, +away, relative not metric),
y always 0, c confidence 0–1 (MediaPipe's `visibility`, unknown = 0); `t` is video-relative ms;
`bodyVisible` is an informational fraction of frames with the torso usably in frame. `POS_SCALE` in
`stance3d.js` maps metres → scene units.

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
