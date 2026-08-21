# Warehouse Ergonomics Demo

Live browser demo on the [Brilliant Wear JavaScript SDK](https://github.com/brilliantsole/BrilliantWear-JavaScript-SDK):
**footsteps · balance · center of pressure · lifting posture** for warehouse / box-moving scenarios.
Built for enterprise evaluations (systems-integrator and logistics conversations).

## What it shows
- **Center of pressure & balance** — live COP trail and per-sensor pressure over real top-down shoe artwork (the golf shoe model render with its sensor-bed flex-circuit overlay); L/R load split, stability score
- **Footstep map** — waltz-chart-style numbered L/R footprints with a 10-second fade, direction from IMU yaw (insole-only; no external positioning SDK)
- **Lifting posture** — trunk flexion/lean/twist from a Sense unit on the upper back; optional second unit on the pelvis distinguishes hip-hinge (good) from stooped-back (risky) lifts
- **Haptic coaching** — insoles buzz the moment posture enters the red zone
- **Session metrics** — steps, cadence, asymmetry, lift counts (good vs risky), time-in-red
- **Data collection** — **● Record** a session, then **Save CSV** / **Save JSON** (see below)

## Collecting data

Press **● Record** to start a session, run the scenario (walk + lifts), press **■ Stop**, then export:

- **Save JSON** — every connected device's *raw* sensor stream (pressure raw values + sensor
  positions, gameRotation quaternions) with timestamps and data rates, in the SDK's own recording
  schema. Loads directly into the SDK's `examples/recording` loader/visualizer, and is the file to
  keep for ML (e.g. Edge Impulse lift classification). *(Requires hardware — disabled in Simulate.)*
- **Save CSV** — the ergonomics *metrics* timeline sampled at ~10 Hz: `t_ms, cop_x, cop_y,
  load_l, load_r, flexion_deg, lean_deg, twist_deg, pelvis_flexion_deg, hinge_delta_deg, zone,
  steps, lifts, lifts_good, lifts_bad, stability`. Opens in any spreadsheet; works in Simulate too.

Start/stop each fire a distinct insole buzz, so the wearer feels the session boundaries (and it
doubles as a haptics check). Files download with an ISO-timestamp name; nothing leaves the browser.

## Run it
Serve the folder and open it in Chrome/Edge (Web Bluetooth required for hardware):

```bash
npx serve .
```

(Serving matters: the shoe artwork is fetched from `assets/shoes.svg`, which `file://` blocks.
Opening `index.html` directly still works, minus the artwork.)

**No hardware?** Press **▶ Simulate** — full walking + good/bad lift loop.

## Hardware setup (2–3 devices)
1. **Insoles** (pair) — Connect Insoles → pick left, then right in the Bluetooth chooser
2. **Torso Sense** — band mount between the shoulder blades; stand tall ~1 s after connect (auto-calibrates upright)
3. **Pelvis Sense** *(optional but recommended)* — band at the sacrum; enables hinge-vs-stoop lift classification

## SDK notes (wired against SDK v0.0.78)

All SDK touchpoints live in `SDKAdapter` inside `app.js`, verified against the v0.0.78 build and examples:

- **Insoles** — `BS.DevicePair.insoles` (auto-assigns connected insoles). `new BS.Device(); device.connect()`
  opens the chooser; call once per foot. Combined COP from the pair's `pressure` event
  (`normalizedCenter`); per-side sensors + load from `devicePressure` (`{ pressure, side }`,
  `pressure.sensors[i].normalizedValue` / `.position`, `pressure.normalizedSum`).
- **Sense** (torso/pelvis) — `new BS.Device(); device.connect()`, then the `gameRotation` event
  (`{ gameRotation: quaternion, gameRotationEuler }`). Trunk angles are computed relative to the
  upright quaternion captured in the first ~1 s.
- **Vibration** — `devicePair.triggerVibration([{ type:"waveformEffect", segments:[{ effect }] }])`,
  effects from `BS.VibrationWaveformEffects`.
- **Recording** — `BS.DeviceManager` `deviceConnected` → per-device `sensorData` events, stored in the
  SDK recording schema (see `examples/recording`).

Thresholds (flexion zones, step detection, stride) are in `CFG`/`MAP` at the top.

### Two things to eyeball on first hardware run
1. **COP orientation** — `COP_FLIP_X` / `COP_FLIP_Y` near `COP_RECT`. If the live dot reads mirrored
   front↔back or left↔right vs. the wearer, flip the offending constant (one line). Defaults are a best guess.
2. **Sensor→pad order** — `SDK_SENSOR_TO_PAD` (identity) maps an 8-sensor insole to the artwork pads
   (anatomical, 0 heel → 7 hallux). Insoles reporting a different count (e.g. 16-sensor beds) are placed
   automatically by each sensor's `position`. If an 8-sensor insole lights the wrong pads, reorder this array.

**Artwork**: `assets/shoes.svg` (pads id'd `pad-{left,right}-{0..7}`, centers in `PAD_POS`) is composed
from the golf render + sensor-bed overlay (`golfShoeModel_sensorbedOverlay.svg`, in `~/Downloads`);
regenerate with `tools/build_shoes.py` if the source artwork changes.

## Deploy (GitHub Pages)
Push to a public repo → Settings → Pages → deploy from `main` root. The page is fully static;
the SDK loads from unpkg.

---
© Brilliant Sole, Inc. (dba Brilliant Wear)
