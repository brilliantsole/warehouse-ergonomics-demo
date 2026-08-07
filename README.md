# Warehouse Ergonomics Demo

Live browser demo on the [Brilliant Wear JavaScript SDK](https://github.com/brilliantsole/BrilliantWear-JavaScript-SDK):
**footsteps · balance · center of pressure · lifting posture** for warehouse / box-moving scenarios.
Built for enterprise evaluations (systems-integrator and logistics conversations).

## What it shows
- **Center of pressure & balance** — live COP trail, per-sensor foot heatmaps, L/R load split, stability score
- **Footstep map** — waltz-chart-style numbered L/R footprints with a 10-second fade, direction from IMU yaw (insole-only; no external positioning SDK)
- **Lifting posture** — trunk flexion/lean/twist from a Sense unit on the upper back; optional second unit on the pelvis distinguishes hip-hinge (good) from stooped-back (risky) lifts
- **Haptic coaching** — insoles buzz the moment posture enters the red zone
- **Session metrics** — steps, cadence, asymmetry, lift counts (good vs risky), time-in-red

## Run it
Open `index.html` in Chrome/Edge (Web Bluetooth required for hardware), or serve the folder:

```bash
npx serve .
```

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

## Deploy (GitHub Pages)
Push to a public repo → Settings → Pages → deploy from `main` root. The page is fully static;
the SDK loads from unpkg.

---
© Brilliant Sole, Inc. (dba Brilliant Wear)
