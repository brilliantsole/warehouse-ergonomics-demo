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

## SDK notes (for tuning)
All SDK touchpoints are centralized in `SDKAdapter` inside `app.js`, marked `TODO[SDK]`:
event names (`pressure`, `gameRotation`), payload fields, `setSensorConfiguration` keys, and the vibration call.
If a newer SDK build renames anything, it's a five-minute fix in one place. Thresholds (flexion zones,
step detection, stride length) live in the `CFG`/`MAP` objects at the top.

**Sensor↔pad mapping**: the 16 pads in `assets/shoes.svg` are id'd `pad-{left,right}-{0..7}` in
anatomical order (0 heel → 7 hallux); centers are in `PAD_POS` in `app.js`. If the hardware's
pressure-array order differs, adjust `SDK_SENSOR_TO_PAD` in the pressure handler (`TODO[SDK]`).
`assets/shoes.svg` is composed from the golf shoe render + sensor-bed overlay
(`golfShoeModel_sensorbedOverlay.svg`, in `~/Downloads`); regenerate with `tools/build_shoes.py`
if the source artwork changes.

## Deploy (GitHub Pages)
Push to a public repo → Settings → Pages → deploy from `main` root. The page is fully static;
the SDK loads from unpkg.

---
© Brilliant Sole, Inc. (dba Brilliant Wear)
