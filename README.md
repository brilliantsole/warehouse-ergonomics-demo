# Warehouse Ergonomics Demo

Live browser demo on the [Brilliant Wear JavaScript SDK](https://github.com/brilliantsole/BrilliantWear-JavaScript-SDK):
**footsteps · balance · center of pressure · lifting posture** for warehouse / box-moving scenarios.
Built for enterprise evaluations (systems-integrator and logistics conversations).

## What it shows
- **Center of pressure & balance** — live COP trail and per-sensor pressure over real top-down shoe artwork (the golf shoe model render with its sensor-bed flex-circuit overlay); L/R load split, stability score
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

Accepted data JSON = the SDK/portal recording schema: `devices[].sensorData[]` with pressure
`rawValue` arrays + sensor `positions`, and `gameRotation` (or `rotation`) quaternions, each with
`initialTimestamp` + `dataRate`. Device role comes from `type` (`leftInsole`/`rightInsole`) and
`placement` (`upper back` → torso, `pelvis` → pelvis); generic Sense units otherwise fall back to
first-torso / second-pelvis. Video sync uses `video.syncOffsetMs` from the JSON if present, else 0.

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

**Artwork**: `assets/shoes.svg` (pads id'd `pad-{left,right}-{0..7}`, centers in `PAD_POS`) is composed
from the golf render + sensor-bed overlay (`golfShoeModel_sensorbedOverlay.svg`, in `~/Downloads`);
regenerate with `tools/build_shoes.py` if the source artwork changes.

## Deploy (GitHub Pages)
Push to a public repo → Settings → Pages → deploy from `main` root. The page is fully static;
the SDK loads from unpkg.

---
© Brilliant Sole, Inc. (dba Brilliant Wear)
