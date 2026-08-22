/* Brilliant Wear — Warehouse Ergonomics Demo
   Footsteps · balance · center of pressure · lifting posture, in the browser.
   Hardware path: Brilliant Wear JS SDK (window.BS from unpkg build).
   Every SDK touchpoint lives in SDKAdapter below — if an event or method name
   differs from a newer SDK build, fix it there (marked TODO[SDK]). Simulate
   mode exercises the entire UI with no hardware. */

(() => {
  "use strict";

  // ---------- tunables ----------
  const CFG = {
    sensorRateMs: 20,            // requested sensor period (ms)
    flexionAmber: 20,            // trunk flexion ° → amber
    flexionRed: 45,              // trunk flexion ° → red + haptic
    liftStartDeg: 45,            // flexion crossing that opens a lift
    liftEndDeg: 20,              // return crossing that closes a lift
    stoopPeakDeg: 60,            // no-pelvis fallback: peak above this = stooped
    hingeGoodDelta: 25,          // torso-pelvis delta below this at peak = good hinge
    stepThreshold: 0.55,         // normalized side-load crossing for a step
    stepRefractoryMs: 300,
    hapticCooldownMs: 2000,
    copTrail: 40,               // matches the trail-circle pool in assets/shoes.svg
    stabilityWindowMs: 3000,
  };

  // ---------- state ----------
  const S = {
    sim: false, simT: 0, simPhase: "walk", simPhaseT: 0, simTimer: null,
    baselineTorso: null, baselinePelvis: null, calSamples: [],
    flexion: 0, lean: 0, twist: 0, pelvisFlexion: null,
    inLift: false, liftPeak: 0, liftPeakDelta: 0,
    lifts: 0, liftsGood: 0, liftsBad: 0,
    steps: 0, stepTimes: [], lastStepAt: { left: 0, right: 0 },
    sideLoad: { left: 0.5, right: 0.5 }, sideLoadPrev: { left: 0, right: 0 },
    stepsPerSide: { left: 0, right: 0 },
    cop: { x: 0.5, y: 0.5 }, copTrail: [], copHistory: [],
    copFoot: { left: { x: 0.5, y: 0.5 }, right: { x: 0.5, y: 0.5 } }, // per-insole COP
    footYaw: { left: -10, right: 10 }, // per-insole heading (deg), relative orientation for the 3D stance
    // vision-derived foot POSITION (from Pose + Depth Anything over the recorded clip):
    // x = lateral (m, + = right), z = forward (m, + = away from camera), y = lift (m), c = confidence 0..1
    footPos: { left: { x: -0.15, y: 0, z: 0, c: 0 }, right: { x: 0.15, y: 0, z: 0, c: 0 } },
    useVision: false, // toggle: drive foot POSITION from the vision foot-track vs. fixed stance + estimated stride
    sensors: { left: new Array(8).fill(0), right: new Array(8).fill(0) },
    redSince: null, redTotalMs: 0, lastHapticAt: 0,
    connected: { insoles: false, torso: false, pelvis: false },
  };

  const $ = (id) => document.getElementById(id);
  let CLOCK = null;                 // when set (offline re-derive), now() returns virtual ms
  const now = () => (CLOCK == null ? performance.now() : CLOCK);
  let suppressLog = false;          // silence the on-screen log during offline re-derive
  const Deriving = { on: false, footEvents: [] };

  function log(msg, cls = "") {
    if (suppressLog) return;
    const el = $("log");
    const t = new Date().toLocaleTimeString([], { hour12: false });
    el.innerHTML += `<div><span class="t">${t}</span> <span class="${cls}">${msg}</span></div>`;
    el.scrollTop = el.scrollHeight;
  }

  // ---------- math ----------
  function quatToEuler(q) {
    // returns degrees {pitch, roll, yaw}
    const { x, y, z, w } = q;
    const sinp = 2 * (w * x + y * z);
    const cosp = 1 - 2 * (x * x + y * y);
    const pitch = Math.atan2(sinp, cosp);
    let sinr = 2 * (w * y - z * x);
    sinr = Math.max(-1, Math.min(1, sinr));
    const roll = Math.asin(sinr);
    const siny = 2 * (w * z + x * y);
    const cosy = 1 - 2 * (y * y + z * z);
    const yaw = Math.atan2(siny, cosy);
    const d = 180 / Math.PI;
    return { pitch: pitch * d, roll: roll * d, yaw: yaw * d };
  }
  function quatConj(q) { return { x: -q.x, y: -q.y, z: -q.z, w: q.w }; }
  function quatMul(a, b) {
    return {
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
  }
  function eulerToQuat(pitchDeg, rollDeg, yawDeg) {
    const r = Math.PI / 360; // half, in one go
    const cp = Math.cos(pitchDeg * r), sp = Math.sin(pitchDeg * r);
    const cr = Math.cos(rollDeg * r), sr = Math.sin(rollDeg * r);
    const cy = Math.cos(yawDeg * r), sy = Math.sin(yawDeg * r);
    return {
      w: cp * cr * cy + sp * sr * sy,
      x: sp * cr * cy - cp * sr * sy,
      y: cp * sr * cy + sp * cr * sy,
      z: cp * cr * sy - sp * sr * cy,
    };
  }

  // ---------- posture pipeline ----------
  function onTorsoQuat(q) {
    if (!S.baselineTorso) {
      S.calSamples.push(q);
      if (S.calSamples.length >= 40) {           // ~1s of samples: calibrate upright
        S.baselineTorso = S.calSamples[S.calSamples.length - 1];
        log("Torso calibrated — stand tall when connecting for best results.");
      }
      return;
    }
    const rel = quatMul(quatConj(S.baselineTorso), q);
    const e = quatToEuler(rel);
    // Flexion = total tilt of the segment away from its calibrated upright pose,
    // measured as the angle between the rest "up" axis and the live one. This is
    // axis-agnostic (mount the Sense any way up) and never wraps through ±180°,
    // unlike reading a single Euler pitch — which is what real WE 4 data exposed
    // (the upper-back unit flexes about its roll axis, pitch read 152°).
    S.flexion = tiltDeg(S.baselineTorso, q);
    S.lean = e.roll;
    S.twist = e.yaw;
    if (!S.sim) S.heading = S.twist; // hardware: torso yaw steers the footstep map
    updatePosture();
  }
  function onPelvisQuat(q) {
    if (!S.baselinePelvis) { S.baselinePelvis = q; return; }
    S.pelvisFlexion = tiltDeg(S.baselinePelvis, q);
  }
  // Tilt (deg, 0..180) of a body segment away from its calibrated upright pose.
  // World "up" expressed in the device frame at rest is g = rest⁻¹·(0,1,0); the
  // live quaternion rotates that same device vector to world space as live·g.
  // The angle between world-up and live·g is the tilt — independent of how the
  // Sense is mounted (which device axis points up) and free of Euler wrap.
  function tiltDeg(rest, live) {
    const gDev = rotateVec(quatConj(rest), [0, 1, 0]);   // device-frame "up" at rest
    const gNow = rotateVec(live, gDev);                   // where that axis points now
    const cosT = Math.max(-1, Math.min(1, gNow[1]));      // dot with world up (0,1,0)
    return Math.acos(cosT) * 180 / Math.PI;
  }
  function rotateVec(q, v) {
    // v' = q v q*  (q unit quaternion, v as pure quaternion)
    const { x, y, z, w } = q;
    const [vx, vy, vz] = v;
    const tx = 2 * (y * vz - z * vy), ty = 2 * (z * vx - x * vz), tz = 2 * (x * vy - y * vx);
    return [vx + w * tx + (y * tz - z * ty), vy + w * ty + (z * tx - x * tz), vz + w * tz + (x * ty - y * tx)];
  }

  function updatePosture() {
    const f = S.flexion;
    const zone = f >= CFG.flexionRed ? "red" : f >= CFG.flexionAmber ? "amber" : "green";

    // red-time accounting + haptic coaching
    if (zone === "red") {
      if (!S.redSince) S.redSince = now();
      maybeHapticAlert();
    } else if (S.redSince) {
      S.redTotalMs += now() - S.redSince;
      S.redSince = null;
    }

    // lift detection: open above liftStartDeg, close below liftEndDeg
    if (!S.inLift && f >= CFG.liftStartDeg) {
      S.inLift = true; S.liftPeak = f;
      S.liftPeakDelta = S.pelvisFlexion != null ? Math.abs(f - S.pelvisFlexion) : null;
    } else if (S.inLift) {
      if (f > S.liftPeak) {
        S.liftPeak = f;
        if (S.pelvisFlexion != null) S.liftPeakDelta = Math.abs(f - S.pelvisFlexion);
      }
      if (f <= CFG.liftEndDeg) {
        S.inLift = false;
        S.lifts++;
        const good = S.liftPeakDelta != null
          ? S.liftPeakDelta <= CFG.hingeGoodDelta          // hips hinged with torso → good
          : S.liftPeak < CFG.stoopPeakDeg;                 // fallback: shallow peak → good
        if (good) { S.liftsGood++; log(`Lift #${S.lifts}: good form (peak ${S.liftPeak.toFixed(0)}°)`); }
        else { S.liftsBad++; log(`Lift #${S.lifts}: RISKY — stooped back (peak ${S.liftPeak.toFixed(0)}°)`, "bad"); }
        renderCounters();
      }
    }
    renderPosture(zone);
  }

  function maybeHapticAlert() {
    if (!$("haptic-toggle").checked) return;
    const t = now();
    if (t - S.lastHapticAt < CFG.hapticCooldownMs) return;
    S.lastHapticAt = t;
    log("⚠ posture red — haptic buzz sent to insoles", "warn");
    SDKAdapter.buzzInsoles();
  }

  // ---------- gait pipeline ----------
  function onSideLoad(side, normalized) {
    const prev = S.sideLoadPrev[side];
    S.sideLoadPrev[side] = normalized;
    const t = now();
    if (prev < CFG.stepThreshold && normalized >= CFG.stepThreshold &&
        t - S.lastStepAt[side] > CFG.stepRefractoryMs) {
      S.lastStepAt[side] = t;
      S.steps++; S.stepsPerSide[side]++;
      S.stepTimes.push(t);
      if (S.stepTimes.length > 12) S.stepTimes.shift();
      placeFootprint(side, t);
      renderGait();
    }
  }

  // ---------- footstep map (waltz-chart style, relative positions) ----------
  // Each step advances the walker along the current heading by an estimated
  // stride, with a lateral L/R offset — numbered footprints fade after 10 s.
  // Heading source: insole/torso yaw when available; simulator drives S.heading.
  const MAP = { strideM: 0.62, lateralM: 0.13, windowMs: 10000 };
  S.heading = 0;            // degrees, 0 = up the map
  S.walker = { x: 0, y: 0 };
  S.footprints = [];        // {x, y, side, n, t, headingAtStep}
  S.stepSeq = 0;

  // Each footprint is oriented by ITS OWN insole's yaw (gameRotation-derived
  // S.footYaw), not a shared body heading — so a print faces the way that foot
  // actually points, and walking backwards / turning reads correctly. The
  // walker also advances along that foot's yaw. In Simulate, S.footYaw is only
  // a small toe-out flare, so the simulator's wandering body heading is added.
  function footHeadingDeg(side) {
    const yaw = (S.footYaw && S.footYaw[side]) || 0;
    return S.sim ? S.heading + yaw : yaw;
  }

  function placeFootprint(side, t) {
    const headingDeg = footHeadingDeg(side);
    const h = (headingDeg * Math.PI) / 180;
    // advance along this foot's heading
    S.walker.x += Math.sin(h) * MAP.strideM;
    S.walker.y -= Math.cos(h) * MAP.strideM;
    // perpendicular offset: left foot to the left of the line of travel
    const perp = side === "left" ? -1 : 1;
    const fx = S.walker.x + Math.cos(h) * MAP.lateralM * perp;
    const fy = S.walker.y + Math.sin(h) * MAP.lateralM * perp;
    S.stepSeq++;
    S.footprints.push({ x: fx, y: fy, side, n: S.stepSeq, t, heading: headingDeg });
    while (S.footprints.length && t - S.footprints[0].t > MAP.windowMs) S.footprints.shift();
    if (Deriving.on) {
      const ev = { t, x: fx, y: fy, side, n: S.stepSeq, heading: headingDeg, hasVis: false };
      // bake the real landing position from the vision foot-track (when present) so the
      // footstep map can show measured step length / sway; est coords are the fallback.
      const ftf = sampleFootTrack(t);
      const p = ftf && ftf[side === "left" ? "l" : "r"];
      if (p && p.c > 0) { ev.xVis = p.x; ev.yVis = -(p.z || 0); ev.hasVis = true; }
      Deriving.footEvents.push(ev);
    }
  }

  const mapCtx = $("stepmap").getContext("2d");
  function drawStepMap() {
    const w = mapCtx.canvas.width, h = mapCtx.canvas.height;
    mapCtx.clearRect(0, 0, w, h);
    const t = (S.replayActive && S.clockT != null) ? S.clockT : now();
    const fresh = S.footprints.filter((p) => t - p.t <= MAP.windowMs);
    if (!fresh.length) {
      mapCtx.fillStyle = "rgba(154,163,199,.45)";
      mapCtx.font = "12px sans-serif"; mapCtx.textAlign = "center";
      mapCtx.fillText("walk to draw the map", w / 2, h / 2);
      return;
    }
    // fit recent prints into view with padding
    const xs = fresh.map((p) => p.x), ys = fresh.map((p) => p.y);
    const minX = Math.min(...xs) - 0.5, maxX = Math.max(...xs) + 0.5;
    const minY = Math.min(...ys) - 0.5, maxY = Math.max(...ys) + 0.5;
    const scale = Math.min(w / (maxX - minX), h / (maxY - minY), 90);
    const ox = (w - (maxX - minX) * scale) / 2 - minX * scale;
    const oy = (h - (maxY - minY) * scale) / 2 - minY * scale;
    const P = (p) => [p.x * scale + ox, p.y * scale + oy];

    // connecting arrows in stride order
    for (let i = 1; i < fresh.length; i++) {
      const a = P(fresh[i - 1]), b = P(fresh[i]);
      const age = (t - fresh[i].t) / MAP.windowMs;
      mapCtx.strokeStyle = `rgba(154,163,199,${0.45 * (1 - age)})`;
      mapCtx.lineWidth = 1.5;
      mapCtx.beginPath(); mapCtx.moveTo(a[0], a[1]); mapCtx.lineTo(b[0], b[1]); mapCtx.stroke();
    }
    // footprints
    fresh.forEach((p) => {
      const [x, y] = P(p);
      const age = (t - p.t) / MAP.windowMs;
      const alpha = Math.max(0.08, 1 - age);
      const color = p.side === "left" ? `rgba(0,212,170,${alpha})` : `rgba(111,123,255,${alpha})`;
      mapCtx.save();
      mapCtx.translate(x, y);
      mapCtx.rotate((p.heading * Math.PI) / 180);
      // sole
      mapCtx.fillStyle = color;
      mapCtx.beginPath();
      mapCtx.ellipse(0, 3, 5, 8.5, 0, 0, Math.PI * 2);
      mapCtx.fill();
      // toe
      mapCtx.beginPath();
      mapCtx.ellipse(0, -8, 3.6, 4, 0, 0, Math.PI * 2);
      mapCtx.fill();
      mapCtx.restore();
      // sequence number
      mapCtx.fillStyle = `rgba(238,241,255,${Math.max(0.25, 1 - age)})`;
      mapCtx.font = "600 9px sans-serif"; mapCtx.textAlign = "center";
      mapCtx.fillText(p.n, x + (p.side === "left" ? -12 : 12), y + 3);
    });
  }

  function onCop(x, y) {
    S.cop = { x, y };
    S.copTrail.push({ x, y });
    if (S.copTrail.length > CFG.copTrail) S.copTrail.shift();
    const t = now();
    S.copHistory.push({ t, x, y });
    while (S.copHistory.length && t - S.copHistory[0].t > CFG.stabilityWindowMs) S.copHistory.shift();
  }

  // Per-insole center of pressure (each foot's own COP, 0..1 within that foot).
  function onFootCop(side, x, y) { S.copFoot[side] = { x, y }; }

  function stabilityScore() {
    const h = S.copHistory;
    if (h.length < 10) return null;
    const mx = h.reduce((s, p) => s + p.x, 0) / h.length;
    const my = h.reduce((s, p) => s + p.y, 0) / h.length;
    const rms = Math.sqrt(h.reduce((s, p) => s + (p.x - mx) ** 2 + (p.y - my) ** 2, 0) / h.length);
    return Math.max(0, Math.min(100, Math.round(100 - rms * 420)));
  }

  // ---------- SDK adapter (every BS.* touchpoint lives here) ----------
  const SDKAdapter = {
    devicePair: null, torso: null, pelvis: null,
    pressureSensorCount: { left: 8, right: 8 },

    hasSDK() { return typeof window.BS !== "undefined"; },

    async connectInsoles() {
      if (!this.hasSDK()) { log("SDK not loaded — check network / unpkg.", "bad"); return; }
      try {
        // BS.DevicePair.insoles is a singleton that auto-assigns any connected
        // insole device to its left/right slot (verified against SDK v0.0.78 examples).
        this.devicePair = BS.DevicePair.insoles;

        // Overall pair connection state (both sides) drives the status dot.
        this.devicePair.addEventListener("isConnected", () => {
          setConnected("insoles", this.devicePair.isConnected);
        });

        // Per-side connect: enable pressure streaming on each insole as it joins.
        this.devicePair.addEventListener("deviceIsConnected", (e) => {
          const { device, side, isConnected } = e.message;
          if (!isConnected) return;
          device.setSensorConfiguration({ pressure: CFG.sensorRateMs });
          device.resetPressureRange?.();
          this.pressureSensorCount[side] = device.numberOfPressureSensors ?? 8;
          log(`${side} insole connected — ${this.pressureSensorCount[side]} pressure sensors @ ${CFG.sensorRateMs}ms.`);
        });

        // Combined center-of-pressure across BOTH insoles → the COP dot.
        this.devicePair.addEventListener("pressure", (e) => {
          const p = e.message.pressure;
          if (p?.normalizedCenter) onCop(p.normalizedCenter.x, p.normalizedCenter.y);
        });

        // Per-side pressure → load split, step detection, and the pad heatmap.
        this.devicePair.addEventListener("devicePressure", (e) => {
          const { pressure: p, side } = e.message;
          if (p?.normalizedSum != null) {
            S.sideLoad[side] = p.normalizedSum;
            onSideLoad(side, p.normalizedSum);
          }
          if (Array.isArray(p?.sensors)) mapSensorsToPads(side, p.sensors);
          if (p?.normalizedCenter) onFootCop(side, p.normalizedCenter.x, p.normalizedCenter.y); // per-insole COP
        });

        // Open the Web Bluetooth chooser; the pair grabs whatever insole is picked.
        // Call once per foot (pick left, then click Connect again for right).
        const device = new BS.Device();
        device.connect();
        log("Insoles: Bluetooth chooser opened. Pick one, then click Connect Insoles again for the other foot.");
      } catch (err) {
        log(`Insole connect failed: ${err.message}`, "bad");
      }
    },

    async connectSense(which) {
      if (!this.hasSDK()) { log("SDK not loaded — check network / unpkg.", "bad"); return; }
      try {
        const device = new BS.Device();
        this[which] = device;
        device.addEventListener("isConnected", () => {
          setConnected(which, device.isConnected);
          if (device.isConnected) {
            device.setSensorConfiguration({ gameRotation: CFG.sensorRateMs });
            log(`${which} Sense connected — hold still ~1s to calibrate upright.`);
          }
        });
        // gameRotation streams a magnetometer-fused quaternion (+ Euler) per sample.
        device.addEventListener("gameRotation", (e) => {
          const q = e.message.gameRotation ?? e.message.quaternion;
          if (!q) return;
          which === "torso" ? onTorsoQuat(q) : onPelvisQuat(q);
        });
        device.connect();
        log(`${which} Sense: Bluetooth chooser opened.`);
      } catch (err) {
        log(`${which} connect failed: ${err.message}`, "bad");
      }
    },

    // Fire a single waveform effect on the connected insoles. Verified shape
    // (SDK v0.0.78): segments is top-level under the config; effect from
    // BS.VibrationWaveformEffects (e.g. strongBuzz100, strongClick100, tripleClick100).
    buzz(effect = "strongBuzz100") {
      if (S.sim || !this.devicePair) return; // sim / no hardware: no-op
      try {
        this.devicePair.triggerVibration?.([{ type: "waveformEffect", segments: [{ effect }] }]);
      } catch (err) {
        log(`Vibration call failed (fix in SDKAdapter.buzz): ${err.message}`, "warn");
      }
    },
    buzzInsoles() { this.buzz("strongBuzz100"); },
  };

  function setConnected(which, on) {
    S.connected[which] = on;
    $(`dot-${which === "pelvis" ? "torso" : which}`)?.classList.toggle("on", on || S.connected.torso);
    const btn = $(`btn-${which}`);
    if (btn) { btn.classList.toggle("connected", on); btn.textContent = on ? `● ${btn.textContent.replace(/^[⊙●] /, "")}` : btn.textContent.replace("●", "⊙"); }
    log(`${which} ${on ? "connected" : "disconnected"}`);
  }

  // ---------- simulator ----------
  function simTick(dt) {
    S.simT += dt; S.simPhaseT += dt;
    const phases = { walk: 6000, liftGood: 4200, walk2: 5000, liftBad: 4200 };
    const order = ["walk", "liftGood", "walk2", "liftBad"];
    if (S.simPhaseT > phases[S.simPhase]) {
      const idx = (order.indexOf(S.simPhase) + 1) % order.length;
      S.simPhase = order[idx]; S.simPhaseT = 0;
      if (S.simPhase.startsWith("lift")) log(`Simulating ${S.simPhase === "liftGood" ? "a proper hip-hinge lift" : "a stooped-back lift"}…`);
    }
    const t = S.simT / 1000;

    if (S.simPhase.startsWith("walk")) {
      // wander: gentle turning while walking so the footstep map draws curves
      S.heading += (dt / 1000) * 14 * Math.sin(t / 4);
      // gait: alternating load at ~1.9 Hz, CoP sways with each step
      const phase = Math.sin(2 * Math.PI * 0.95 * t);
      const left = 0.5 + 0.45 * Math.max(0, phase) + rnd(0.02);
      const right = 0.5 + 0.45 * Math.max(0, -phase) + rnd(0.02);
      feedSim(left, right, 0.5 + 0.16 * phase + rnd(0.01), 0.42 + 0.1 * Math.abs(Math.sin(2 * Math.PI * 1.9 * t)) + rnd(0.01), 3 + rnd(2), 0);
    } else {
      const p = S.simPhaseT / phases[S.simPhase];             // 0..1 through the lift
      const bell = Math.sin(Math.PI * Math.min(1, p));        // down-and-up
      const good = S.simPhase === "liftGood";
      const torso = bell * (good ? 52 : 74) + rnd(1.5);
      const pelvis = bell * (good ? 44 : 18) + rnd(1.5);      // hips hinge on good lifts
      const left = 0.5 + 0.06 * Math.sin(t * 3) + rnd(0.02);
      feedSim(left, 1 - left + rnd(0.02), 0.5 + rnd(0.015), 0.62 + 0.12 * bell + rnd(0.01), torso, pelvis);
    }
  }
  function rnd(a) { return (Math.random() - 0.5) * 2 * a; }
  function feedSim(loadL, loadR, copX, copY, torsoFlex, pelvisFlex) {
    onSideLoad("left", Math.min(1, loadL)); onSideLoad("right", Math.min(1, loadR));
    S.sideLoad.left = Math.min(1, loadL); S.sideLoad.right = Math.min(1, loadR);
    // individual sensors: forefoot loads up as trunk flexes (weight shifts to the ball of the foot)
    const fwd = Math.min(1, torsoFlex / 70);
    for (let i = 0; i < 8; i++) {
      const heelBias = (i < 3 ? 1.25 : i > 5 ? 0.85 : 1) * (1 - 0.5 * fwd) + (i > 5 ? 0.8 * fwd : 0);
      S.sensors.left[i] = Math.max(0, Math.min(1, loadL * heelBias * (0.55 + rnd(0.3))));
      S.sensors.right[i] = Math.max(0, Math.min(1, loadR * heelBias * (0.55 + rnd(0.3))));
    }
    // per-insole COP derived from each foot's own sensors; combined = load-weighted blend
    const fcL = footCopFromSensors("left"), fcR = footCopFromSensors("right");
    onFootCop("left", fcL.x, fcL.y); onFootCop("right", fcR.x, fcR.y);
    const wsum = Math.max(0.001, S.sideLoad.left + S.sideLoad.right);
    onCop(
      (S.sideLoad.left * (fcL.x * 0.5) + S.sideLoad.right * (0.5 + fcR.x * 0.5)) / wsum,
      (S.sideLoad.left * fcL.y + S.sideLoad.right * fcR.y) / wsum,
    );
    // per-foot heading: natural toe-out flare + gentle dynamic sway (relative insole orientation)
    const ty = S.simT / 1000;
    S.footYaw.left = -11 + 4 * Math.sin(ty * 1.3) + rnd(1.5);
    S.footYaw.right = 11 + 4 * Math.sin(ty * 1.3 + 0.6) + rnd(1.5);
    if (!S.baselineTorso) S.baselineTorso = eulerToQuat(0, 0, 0);
    if (!S.baselinePelvis) S.baselinePelvis = eulerToQuat(0, 0, 0);
    onTorsoQuat(eulerToQuat(torsoFlex, rnd(3), rnd(4)));
    onPelvisQuat(eulerToQuat(pelvisFlex, 0, 0));
  }

  // ---------- rendering ----------
  const gaugeCtx = $("gauge").getContext("2d");

  // Real shoe artwork: assets/shoes.svg (golf shoe model + sensor-bed overlay).
  // Pads are vector paths id'd pad-{left,right}-{i} in anatomical order:
  // 0:heel 1:midMed 2:midLat 3:ballMed 4:ballCtr 5:ballLat 6:foreLat 7:hallux.
  // The shoe render sits on top at 75% opacity, so pad color glows through it.
  // Pad centers in the svg's viewBox coordinate space (for Zack / COP tuning):
  const PAD_POS = {
    left:  [[327,819], [413,598], [256,616], [447,351], [353,372], [265,404], [315,281], [419,232]],
    right: [[822,822], [744,598], [900,623], [720,350], [813,375], [900,410], [854,285], [752,232]],
  };
  // normalized COP (0..1) maps into this viewBox rect (pad bounds + margin)
  const COP_RECT = { x: 201, y: 177, w: 754, h: 700 };
  // Each insole's own COP maps into that foot's region (pad bounds + margin).
  const FOOT_COP_RECT = {
    left:  { x: 226, y: 192, w: 251, h: 667 },
    right: { x: 690, y: 192, w: 240, h: 670 },
  };
  const PAD_RGB = { left: "0,212,170", right: "111,123,255" }; // matches L/R legend colors
  const PAD_HEX = { left: "#00d4aa", right: "#6f7bff" };

  // COP axis orientation: SDK normalizedCenter vs. this SVG's viewBox (y grows down).
  // Best-guess defaults; if the live dot reads mirrored on hardware, flip the culprit.
  const COP_FLIP_X = false, COP_FLIP_Y = true;
  const fx = (v) => (COP_FLIP_X ? 1 - v : v);
  const fy = (v) => (COP_FLIP_Y ? 1 - v : v);

  // Per-foot pad centers normalized to [0,1] (heel≈y0, toe≈y1) for position-based
  // sensor→pad assignment when an insole streams a sensor count other than 8.
  const PAD_NORM = {};
  for (const side of ["left", "right"]) {
    const xs = PAD_POS[side].map((p) => p[0]), ys = PAD_POS[side].map((p) => p[1]);
    const minX = Math.min(...xs), maxX = Math.max(...xs), minY = Math.min(...ys), maxY = Math.max(...ys);
    PAD_NORM[side] = PAD_POS[side].map(([x, y]) => ({
      x: (x - minX) / (maxX - minX || 1),
      y: 1 - (y - minY) / (maxY - minY || 1), // viewBox toe is small-y → normalize toe→1
    }));
  }
  // Identity for the 8-sensor insole; flip entries here if hardware order differs.
  const SDK_SENSOR_TO_PAD = [0, 1, 2, 3, 4, 5, 6, 7];

  // Fill S.sensors[side] (8 artwork pads) from a live per-sensor array of any length.
  function mapSensorsToPads(side, sensors) {
    const pads = S.sensors[side];
    pads.fill(0);
    if (sensors.length === 8) {
      for (let i = 0; i < 8; i++) pads[SDK_SENSOR_TO_PAD[i]] = sensors[i].normalizedValue ?? sensors[i].value ?? 0;
      return;
    }
    // Non-8 count (e.g. 16-sensor Ukaton beds): assign each sensor to the nearest
    // artwork pad by its normalized position; brightest sensor wins the pad.
    sensors.forEach((s, i) => {
      const v = s.normalizedValue ?? s.value ?? 0;
      let idx;
      if (s.position) {
        let best = Infinity; idx = 0;
        for (let p = 0; p < 8; p++) {
          const dx = s.position.x - PAD_NORM[side][p].x, dy = s.position.y - PAD_NORM[side][p].y;
          const d = dx * dx + dy * dy;
          if (d < best) { best = d; idx = p; }
        }
      } else {
        idx = Math.min(7, Math.floor((i * 8) / sensors.length));
      }
      if (v > pads[idx]) pads[idx] = v;
    });
  }

  // Weighted centroid of a foot's 8 pad values over their normalized positions —
  // the foot's own center of pressure, in the same 0..1 frame as PAD_NORM.
  function footCopFromSensors(side) {
    const vals = S.sensors[side], pos = PAD_NORM[side];
    let sx = 0, sy = 0, sw = 0;
    for (let i = 0; i < 8; i++) { const w = vals[i]; sx += pos[i].x * w; sy += pos[i].y * w; sw += w; }
    if (sw <= 0.001) return { x: 0.5, y: 0.5 };
    return { x: sx / sw, y: sy / sw };
  }

  const ShoeStage = {
    ready: false, pads: { left: [], right: [] }, trail: [], dot: null, footDots: {},
    async init() {
      const frame = $("shoe-frame");
      if (!frame) return;                 // 2D stage replaced by the 3D stance widget
      try {
        const res = await fetch("assets/shoes.svg");
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        frame.innerHTML = await res.text();
        for (const side of ["left", "right"]) {
          this.pads[side] = PAD_POS[side].map((_, i) => frame.querySelector(`#pad-${side}-${i}`));
        }
        this.trail = [...frame.querySelectorAll(".cop-trail")];
        this.dot = frame.querySelector("#cop-dot");
        // Combined COP: white so it reads distinctly from the side-colored per-insole rings.
        if (this.dot) { this.dot.style.fill = "#f4f7ff"; this.dot.style.stroke = "rgba(26,31,71,0.55)"; this.dot.style.strokeWidth = "3"; }
        // Per-insole COP markers: side-colored hollow rings, drawn into the COP layer.
        const layer = frame.querySelector("#cop-layer") || frame.querySelector("svg");
        const NS = "http://www.w3.org/2000/svg";
        for (const side of ["left", "right"]) {
          const ring = document.createElementNS(NS, "circle");
          ring.setAttribute("r", "13");
          ring.setAttribute("fill", PAD_HEX[side]);
          ring.setAttribute("fill-opacity", "0.28");
          ring.setAttribute("stroke", PAD_HEX[side]);
          ring.setAttribute("stroke-width", "3.5");
          ring.setAttribute("cx", "-999"); ring.setAttribute("cy", "-999");
          layer.appendChild(ring);
          this.footDots[side] = ring;
        }
        this.ready = true;
      } catch (err) {
        frame.innerHTML = `<div class="shoe-fallback">Shoe artwork needs an http server (python3 -m http.server) — data still streams below.</div>`;
        log(`Shoe artwork not loaded: ${err.message}`, "warn");
      }
    },
    render() {
      if (!this.ready) return;
      for (const side of ["left", "right"]) {
        const rgb = PAD_RGB[side];
        S.sensors[side].forEach((v, i) => {
          const pad = this.pads[side][i];
          if (!pad) return;
          if (v <= 0.02) {                       // idle: neutral, circuit shows through
            pad.style.fill = "#0e1330"; pad.style.opacity = "0.16";
          } else {
            pad.style.fill = `rgb(${rgb})`;
            pad.style.opacity = (0.14 + 0.86 * Math.min(1, v)).toFixed(3);
          }
        });
      }
      // per-insole COP rings (each within its own foot region)
      for (const side of ["left", "right"]) {
        const r = FOOT_COP_RECT[side], c = S.copFoot[side], ring = this.footDots[side];
        if (!ring) continue;
        ring.setAttribute("cx", (r.x + fx(c.x) * r.w).toFixed(1));
        ring.setAttribute("cy", (r.y + fy(c.y) * r.h).toFixed(1));
      }
      // combined (both-feet) COP dot + sway trail
      this.dot.setAttribute("cx", (COP_RECT.x + fx(S.cop.x) * COP_RECT.w).toFixed(1));
      this.dot.setAttribute("cy", (COP_RECT.y + fy(S.cop.y) * COP_RECT.h).toFixed(1));
      const n = S.copTrail.length;
      this.trail.forEach((c, i) => {
        const p = S.copTrail[n - this.trail.length + i];  // newest point → biggest circle
        if (!p) { c.setAttribute("opacity", "0"); return; }
        c.setAttribute("cx", (COP_RECT.x + fx(p.x) * COP_RECT.w).toFixed(1));
        c.setAttribute("cy", (COP_RECT.y + fy(p.y) * COP_RECT.h).toFixed(1));
        c.setAttribute("opacity", (0.45 * (i / this.trail.length)).toFixed(3));
      });
    },
  };

  function drawGauge() {
    const w = gaugeCtx.canvas.width, h = gaugeCtx.canvas.height;
    const cx = w / 2, cy = h - 14, R = Math.min(w / 2 - 8, h - 26);
    gaugeCtx.clearRect(0, 0, w, h);
    const segs = [
      [0, CFG.flexionAmber, "#00d4aa"],
      [CFG.flexionAmber, CFG.flexionRed, "#ffb020"],
      [CFG.flexionRed, 90, "#ff5468"],
    ];
    segs.forEach(([a, b, color]) => {
      gaugeCtx.beginPath();
      gaugeCtx.strokeStyle = color; gaugeCtx.lineWidth = 12; gaugeCtx.lineCap = "butt";
      gaugeCtx.arc(cx, cy, R, Math.PI + (a / 90) * Math.PI * 0.99, Math.PI + (b / 90) * Math.PI * 0.99);
      gaugeCtx.stroke();
    });
    const ang = Math.PI + (Math.min(90, S.flexion) / 90) * Math.PI * 0.99;
    gaugeCtx.strokeStyle = "#eef1ff"; gaugeCtx.lineWidth = 3;
    gaugeCtx.beginPath();
    gaugeCtx.moveTo(cx, cy);
    gaugeCtx.lineTo(cx + Math.cos(ang) * (R - 14), cy + Math.sin(ang) * (R - 14));
    gaugeCtx.stroke();
    gaugeCtx.fillStyle = "#eef1ff"; gaugeCtx.font = "600 13px sans-serif"; gaugeCtx.textAlign = "center";
    gaugeCtx.fillText(`${S.flexion.toFixed(0)}°`, cx, cy - R / 2);
  }

  function renderPosture(zone) {
    if (Deriving.on) return;
    $("flexion").textContent = `${S.flexion.toFixed(0)}°`;
    $("lean").textContent = `${S.lean.toFixed(0)}°`;
    $("twist").textContent = `${S.twist.toFixed(0)}°`;
    $("hinge").textContent = S.pelvisFlexion != null ? `${Math.abs(S.flexion - S.pelvisFlexion).toFixed(0)}°` : "—";
    $("liftform").textContent = S.inLift ? "lifting…" : "—";
    const pill = $("posture-pill");
    pill.className = `pill ${zone}`;
    pill.textContent = zone === "green" ? "SAFE" : zone === "amber" ? "CAUTION" : "RISKY";
    const red = S.redTotalMs + (S.redSince ? now() - S.redSince : 0);
    $("redtime").textContent = `${(red / 1000).toFixed(1)}s`;
  }

  function renderGait() {
    if (Deriving.on) return;
    $("steps").textContent = S.steps;
    if (S.stepTimes.length >= 4) {
      const span = S.stepTimes[S.stepTimes.length - 1] - S.stepTimes[0];
      const spm = ((S.stepTimes.length - 1) / (span / 1000)) * 60;
      $("cadence").textContent = `${spm.toFixed(0)} spm`;
    }
    const total = S.stepsPerSide.left + S.stepsPerSide.right;
    if (total > 6) {
      const asym = Math.abs(S.stepsPerSide.left - S.stepsPerSide.right) / total * 100;
      $("asym").textContent = `${asym.toFixed(0)}%`;
    }
  }

  function renderCounters() {
    if (Deriving.on) return;
    $("lifts").textContent = S.lifts;
    $("lifts-good").textContent = S.liftsGood;
    $("lifts-bad").textContent = S.liftsBad;
  }

  function renderLoads() {
    const l = S.sideLoad.left, r = S.sideLoad.right;
    const pct = l + r > 0 ? (l / (l + r)) * 100 : 50;
    $("load-left").style.width = `${pct}%`;
    $("load-left-pct").textContent = `${pct.toFixed(0)}%`;
    $("load-right-pct").textContent = `${(100 - pct).toFixed(0)}%`;
    const st = (S.replayActive && S.stabilityOverride != null) ? S.stabilityOverride : stabilityScore();
    $("stability").textContent = st == null ? "—" : `${st}/100`;
  }

  // ---------- session (loaded recording) + replay ----------
  // Capture happens in the BrilliantWear portal. Here we LOAD a recording it
  // produced (raw sensor JSON + the webcam clip) and replay the whole warehouse
  // dashboard scrubbed in sync with the video — the golf-demo review experience.
  const Session = { frames: [], footEvents: [], video: null, durationMs: 0, footTrack: null, raw: null, rawFormat: null, title: "" };
  const videoWrap = () => document.querySelector(".video-wrap");
  let lastReplayMs = 0; // remembered so the vision toggle can re-plot the current moment

  // Sample the loaded vision foot-track at a recording-clock time. Track frame t
  // is video-relative; align via the track's syncOffsetMs. Returns {left,right}
  // {x,y,z,c} or null. Confidence 0 (or missing foot) means "fall back to insoles".
  function sampleFootTrack(sessionMs) {
    const ft = Session.footTrack;
    if (!ft || !ft.frames || !ft.frames.length) return null;
    const vt = sessionMs - (ft.video?.syncOffsetMs || 0);
    const F = ft.frames;
    let lo = 0, hi = F.length - 1, idx = 0;
    while (lo <= hi) { const m = (lo + hi) >> 1; if (F[m].t <= vt) { idx = m; lo = m + 1; } else hi = m - 1; }
    return F[idx];
  }
  function applyFootPos(fr) {
    const put = (side, p) => { S.footPos[side] = p && p.c > 0 ? { x: p.x, y: p.y || 0, z: p.z || 0, c: p.c } : { ...S.footPos[side], c: 0 }; };
    put("left", fr && fr.l); put("right", fr && fr.r);
  }
  function setVisionAvailable(avail) {
    const t = $("vision-toggle");
    if (!t) return;
    t.disabled = !avail;
    t.checked = avail;               // default ON when a foot-track loads
    S.useVision = avail;
    const hint = $("vision-hint");
    if (hint) hint.textContent = avail ? "vision foot-track loaded" : "";
  }

  function resetSession() {
    Object.assign(S, {
      steps: 0, stepTimes: [], stepsPerSide: { left: 0, right: 0 }, lastStepAt: { left: 0, right: 0 }, sideLoadPrev: { left: 0, right: 0 },
      lifts: 0, liftsGood: 0, liftsBad: 0, inLift: false, liftPeak: 0, liftPeakDelta: 0,
      redTotalMs: 0, redSince: null, copTrail: [], copHistory: [],
      baselineTorso: null, baselinePelvis: null, calSamples: [], pelvisFlexion: null,
      footprints: [], stepSeq: 0, walker: { x: 0, y: 0 }, heading: 0,
      sensors: { left: new Array(8).fill(0), right: new Array(8).fill(0) },
    });
  }

  function snapshotFrame(tms) {
    return {
      t: tms,
      cop: { x: +S.cop.x.toFixed(4), y: +S.cop.y.toFixed(4) },
      cl: { x: +S.copFoot.left.x.toFixed(4), y: +S.copFoot.left.y.toFixed(4) },
      cr: { x: +S.copFoot.right.x.toFixed(4), y: +S.copFoot.right.y.toFixed(4) },
      ll: +S.sideLoad.left.toFixed(4), lr: +S.sideLoad.right.toFixed(4),
      fx: +S.flexion.toFixed(1), ln: +S.lean.toFixed(1), tw: +S.twist.toFixed(1),
      pf: S.pelvisFlexion == null ? null : +S.pelvisFlexion.toFixed(1),
      zone: S.flexion >= CFG.flexionRed ? "red" : S.flexion >= CFG.flexionAmber ? "amber" : "green",
      st: S.steps, li: S.lifts, lg: S.liftsGood, lb: S.liftsBad, stab: stabilityScore(),
      sl: S.sensors.left.map((v) => +v.toFixed(3)), sr: S.sensors.right.map((v) => +v.toFixed(3)),
      fyl: +S.footYaw.left.toFixed(1), fyr: +S.footYaw.right.toFixed(1),
    };
  }

  const quatOf = (s) => (s && typeof s === "object" && "w" in s) ? { x: s.x, y: s.y, z: s.z, w: s.w } : null;

  // Re-derive the warehouse dashboard from a portal/SDK raw recording by replaying
  // its streams through the SAME live pipeline under a virtual clock. Returns frame count.
  function deriveFromRecording(rec) {
    const recStart = rec.timestamp ?? 0;
    const dev = Array.isArray(rec.devices) ? rec.devices : [];
    const streams = { footL: null, footR: null, footLgr: null, footRgr: null, torso: null, pelvis: null };
    const senses = [];
    for (const d of dev) {
      const sd = Array.isArray(d.sensorData) ? d.sensorData : [];
      const press = sd.find((s) => s.sensorType === "pressure");
      const gr = sd.find((s) => s.sensorType === "gameRotation") || sd.find((s) => s.sensorType === "rotation");
      const type = d.type || "", place = String(d.placement || "").toLowerCase();
      if (type === "leftInsole" || place === "left foot") { streams.footL = press || streams.footL; streams.footLgr = gr || streams.footLgr; }
      else if (type === "rightInsole" || place === "right foot") { streams.footR = press || streams.footR; streams.footRgr = gr || streams.footRgr; }
      else if (gr) senses.push({ gr, place });
      else if (press && !streams.footL) streams.footL = press;
      else if (press) streams.footR = press;
    }
    for (const s of senses) {
      if (/back|torso|thorax/.test(s.place) && !streams.torso) streams.torso = s.gr;
      else if (/pelvis|sacrum|hip|waist/.test(s.place) && !streams.pelvis) streams.pelvis = s.gr;
    }
    const rest = senses.filter((s) => s.gr !== streams.torso && s.gr !== streams.pelvis);
    if (!streams.torso && rest.length) streams.torso = rest.shift().gr;
    if (!streams.pelvis && rest.length) streams.pelvis = rest.shift().gr;

    const off = (s) => (s.initialTimestamp ?? recStart) - recStart;
    const rate = (s) => s.dataRate || CFG.sensorRateMs;
    const at = (s, t) => { if (!s || !s.data || !s.data.length) return null; let i = Math.floor((t - off(s)) / rate(s)); return s.data[Math.max(0, Math.min(s.data.length - 1, i))]; };
    const endOf = (s) => (s && s.data && s.data.length ? off(s) + s.data.length * rate(s) : 0);
    const duration = Math.max(endOf(streams.footL), endOf(streams.footR), endOf(streams.torso), endOf(streams.pelvis), 1000);

    const ranges = (s) => {
      if (!s || !s.data || !s.data.length) return null;
      const n = (s.data[0] || []).length, mn = new Array(n).fill(Infinity), mx = new Array(n).fill(-Infinity);
      for (const row of s.data) for (let i = 0; i < n; i++) { const v = +row[i] || 0; if (v < mn[i]) mn[i] = v; if (v > mx[i]) mx[i] = v; }
      return { mn, mx };
    };
    const rL = ranges(streams.footL), rR = ranges(streams.footR);
    const sensorObjs = (s, r, t) => {
      const row = at(s, t); if (!row || !r) return null;
      const pos = s.positions || [];
      return row.map((raw, i) => {
        const span = (r.mx[i] - r.mn[i]) || 1;
        return { normalizedValue: Math.max(0, Math.min(1, ((+raw || 0) - r.mn[i]) / span)), position: pos[i] };
      });
    };

    resetSession();
    S.baselineTorso = streams.torso ? quatOf(at(streams.torso, 0)) : eulerToQuat(0, 0, 0);
    S.baselinePelvis = streams.pelvis ? quatOf(at(streams.pelvis, 0)) : null;
    // per-foot heading baselines (relative insole orientation, deg): first-sample yaw
    const gr0 = { left: streams.footLgr ? quatOf(at(streams.footLgr, 0)) : null, right: streams.footRgr ? quatOf(at(streams.footRgr, 0)) : null };
    const footYawAt = (grStream, base, t, flare) => {
      if (!grStream || !base) return flare; // no insole IMU → natural flare
      const q = quatOf(at(grStream, t)); if (!q) return flare;
      const rel = quatMul(quatConj(base), q);
      return flare + quatToEuler(rel).yaw;
    };
    Deriving.on = true; Deriving.footEvents = []; suppressLog = true;
    const frames = []; let lastSnap = -1000; const STEP = 50;
    for (let t = 0; t <= duration; t += STEP) {
      CLOCK = t;
      // per-foot yaw FIRST so a step detected this tick stamps its print with the current orientation
      S.footYaw.left = footYawAt(streams.footLgr, gr0.left, t, -11);
      S.footYaw.right = footYawAt(streams.footRgr, gr0.right, t, 11);
      const sL = sensorObjs(streams.footL, rL, t), sR = sensorObjs(streams.footR, rR, t);
      let loadL = 0.5, loadR = 0.5;
      if (sL) { mapSensorsToPads("left", sL); loadL = sL.reduce((a, b) => a + b.normalizedValue, 0) / (sL.length || 1); }
      if (sR) { mapSensorsToPads("right", sR); loadR = sR.reduce((a, b) => a + b.normalizedValue, 0) / (sR.length || 1); }
      onSideLoad("left", loadL); onSideLoad("right", loadR);
      S.sideLoad.left = loadL; S.sideLoad.right = loadR;
      const fcL = footCopFromSensors("left"), fcR = footCopFromSensors("right");
      onFootCop("left", fcL.x, fcL.y); onFootCop("right", fcR.x, fcR.y);
      const wsum = Math.max(0.001, loadL + loadR);
      onCop((loadL * (fcL.x * 0.5) + loadR * (0.5 + fcR.x * 0.5)) / wsum, (loadL * fcL.y + loadR * fcR.y) / wsum);
      if (streams.torso) { const q = quatOf(at(streams.torso, t)); if (q) onTorsoQuat(q); }
      if (streams.pelvis) { const q = quatOf(at(streams.pelvis, t)); if (q) onPelvisQuat(q); }
      if (streams.torso) S.heading = S.twist;
      if (t - lastSnap >= 100) { lastSnap = t; frames.push(snapshotFrame(t)); }
    }
    CLOCK = null; Deriving.on = false; suppressLog = false;
    Session.frames = frames; Session.footEvents = Deriving.footEvents; Session.durationMs = duration;
    return frames.length;
  }

  // Re-derive the warehouse dashboard from the PORTAL export format (flat, timestamped
  // rows: recording.devices[] + scalar[] + pressure[] + events[]). Pressure rows already
  // carry per-sensor normalizedValue + position + per-foot normalizedCenter, so no
  // auto-range needed. gameRotation (when the recording has it) comes from scalar rows.
  function deriveFromPortalExport(exp) {
    const devs = exp.recording?.devices || [];
    // Two-pass role assignment. Trunk/pelvis come from placement; FEET are split by
    // the device SIDE, not the placement label — placement labels can be stale or
    // duplicated (observed: the right insole still tagged "left foot"), whereas the
    // side is set on the device itself. First assignment of each role wins.
    const role = {};
    const take = (id, r) => { if (r && !Object.values(role).includes(r)) role[id] = r; };
    for (const d of devs) {
      const p = String(d.placement || "").toLowerCase();
      if (/back|torso|thorax/.test(p)) take(d.deviceId, "torso");
      else if (/pelvis|sacrum|hip|waist/.test(p)) take(d.deviceId, "pelvis");
    }
    for (const d of devs) {
      if (role[d.deviceId]) continue;
      const p = String(d.placement || "").toLowerCase();
      const s = String(d.deviceSide || "").toLowerCase();
      if (!(p.includes("foot") || d.deviceType === "insole")) continue;
      take(d.deviceId, s === "right" ? "footR" : s === "left" ? "footL" : (p === "right foot" ? "footR" : p === "left foot" ? "footL" : null));
    }
    const idFor = (want) => Object.keys(role).find((id) => role[id] === want);
    const ids = { footL: idFor("footL"), footR: idFor("footR"), torso: idFor("torso"), pelvis: idFor("pelvis") };

    const ms = (iso) => Date.parse(iso);
    let t0 = Infinity;
    for (const arr of [exp.scalar, exp.pressure]) for (const r of (arr || [])) { const m = ms(r.time); if (m < t0) t0 = m; }
    if (!Number.isFinite(t0)) return 0;

    // group + sort streams by device
    const pByDev = {}, grByDev = {};
    for (const r of (exp.pressure || [])) (pByDev[r.device_id] ||= []).push(r);
    for (const r of (exp.scalar || [])) if (r.sensor_type === "gameRotation" || r.sensor_type === "rotation") (grByDev[r.device_id] ||= []).push(r);
    const prep = (rows) => (rows || []).map((r) => ({ ...r, t: ms(r.time) - t0 })).sort((a, b) => a.t - b.t);
    const P = { footL: prep(pByDev[ids.footL]), footR: prep(pByDev[ids.footR]) };
    const GR = { footL: prep(grByDev[ids.footL]), footR: prep(grByDev[ids.footR]), torso: prep(grByDev[ids.torso]), pelvis: prep(grByDev[ids.pelvis]) };
    const nearest = (rows, t) => { if (!rows || !rows.length) return null; let lo = 0, hi = rows.length - 1, i = 0; while (lo <= hi) { const m = (lo + hi) >> 1; if (rows[m].t <= t) { i = m; lo = m + 1; } else hi = m - 1; } return rows[i]; };
    const duration = (() => { let d = 1000; for (const s of [P.footL, P.footR, GR.torso, GR.pelvis, GR.footL, GR.footR]) if (s && s.length) d = Math.max(d, s[s.length - 1].t); return d; })();

    resetSession();
    const grQuat = (r) => (r && r.w != null ? { x: r.x, y: r.y, z: r.z, w: r.w } : null);
    S.baselineTorso = grQuat(nearest(GR.torso, 0)) || eulerToQuat(0, 0, 0);
    S.baselinePelvis = grQuat(nearest(GR.pelvis, 0)) || null;
    const gr0 = { footL: grQuat(nearest(GR.footL, 0)), footR: grQuat(nearest(GR.footR, 0)) };
    const footYawFrom = (rows, base, t, flare) => { const q = grQuat(nearest(rows, t)); if (!q || !base) return flare; return flare + quatToEuler(quatMul(quatConj(base), q)).yaw; };

    Deriving.on = true; Deriving.footEvents = []; suppressLog = true;
    const frames = []; let lastSnap = -1000; const STEP = 50;
    const doFoot = (side, key) => {
      const row = nearest(P[key], CLOCK);
      if (!row || !Array.isArray(row.sensors)) return 0.5;
      mapSensorsToPads(side, row.sensors); // sensors[i] = {normalizedValue, position}
      const load = row.normalized_sum != null ? row.normalized_sum : row.sensors.reduce((a, s) => a + (s.normalizedValue || 0), 0) / (row.sensors.length || 1);
      if (row.normalized_center_x != null && row.normalized_center_y != null) onFootCop(side, row.normalized_center_x, row.normalized_center_y);
      else { const fc = footCopFromSensors(side); onFootCop(side, fc.x, fc.y); }
      return Math.max(0, Math.min(1, load));
    };
    for (let t = 0; t <= duration; t += STEP) {
      CLOCK = t;
      // per-foot yaw FIRST so a step detected this tick stamps its print with the current orientation
      S.footYaw.left = footYawFrom(GR.footL, gr0.footL, t, -11);
      S.footYaw.right = footYawFrom(GR.footR, gr0.footR, t, 11);
      const loadL = doFoot("left", "footL"), loadR = doFoot("right", "footR");
      onSideLoad("left", loadL); onSideLoad("right", loadR);
      S.sideLoad.left = loadL; S.sideLoad.right = loadR;
      const wsum = Math.max(0.001, loadL + loadR);
      onCop((loadL * (S.copFoot.left.x * 0.5) + loadR * (0.5 + S.copFoot.right.x * 0.5)) / wsum, (loadL * S.copFoot.left.y + loadR * S.copFoot.right.y) / wsum);
      const qt = grQuat(nearest(GR.torso, t)); if (qt) onTorsoQuat(qt);
      const qp = grQuat(nearest(GR.pelvis, t)); if (qp) onPelvisQuat(qp);
      if (qt) S.heading = S.twist;
      if (t - lastSnap >= 100) { lastSnap = t; frames.push(snapshotFrame(t)); }
    }
    CLOCK = null; Deriving.on = false; suppressLog = false;
    Session.frames = frames; Session.footEvents = Deriving.footEvents; Session.durationMs = duration;
    const has = { footL: !!ids.footL, footR: !!ids.footR, torso: !!ids.torso, pelvis: !!ids.pelvis, gameRotation: (GR.torso.length + GR.footL.length + GR.footR.length) > 0 };
    log(`Portal export: feet[${has.footL ? "L" : "–"}${has.footR ? "R" : "–"}] torso:${has.torso ? "y" : "–"} pelvis:${has.pelvis ? "y" : "–"} gameRotation:${has.gameRotation ? "y" : "NO (no posture/heading)"}.`, has.gameRotation ? "" : "warn");
    return frames.length;
  }

  function enterReplay() {
    if (!Session.frames.length) return;
    if (S.sim) { S.sim = false; $("btn-sim").textContent = "▶ Simulate"; $("btn-sim").classList.add("primary"); }
    S.replayActive = true;
    const cam = $("cam"), scrub = $("scrub"), wrap = videoWrap();
    if (Session.video) {
      cam.srcObject = null; cam.src = Session.video.url; cam.controls = true; cam.muted = true;
      wrap?.classList.add("has-video"); scrub.style.display = "none";
      const drive = () => applyReplayAt(cam.currentTime * 1000 + (Session.video.syncOffsetMs || 0));
      cam.ontimeupdate = drive; cam.onseeking = drive; cam.onseeked = drive;
      $("video-mode").textContent = "review — play / scrub the video; the dashboard follows in sync";
      $("video-sync").textContent = `sync offset ${Session.video.syncOffsetMs | 0}ms · ${(Session.durationMs / 1000).toFixed(1)}s`;
      applyReplayAt(Session.video.syncOffsetMs || 0);
      // Webcam clips often open on a near-black frame (exposure still settling), which
      // reads as "no video". Park the playhead on the first bright frame instead.
      const parkOnBrightFrame = () => {
        const c = document.createElement("canvas"); c.width = 32; c.height = 18;
        const ctx = c.getContext("2d");
        const lum = () => { try { ctx.drawImage(cam, 0, 0, 32, 18); const d = ctx.getImageData(0, 0, 32, 18).data; let s = 0; for (let i = 0; i < d.length; i += 4) s += d[i] + d[i + 1] + d[i + 2]; return s / (d.length / 4) / 3; } catch { return 255; } };
        const tryAt = (t) => { if (t > 3) return; cam.onseeked = () => { cam.onseeked = drive; if (lum() < 12) tryAt(t + 0.5); else drive(); }; cam.currentTime = t; };
        if (lum() < 12) tryAt(0.5);
      };
      if (cam.readyState >= 2) parkOnBrightFrame(); else cam.addEventListener("loadeddata", parkOnBrightFrame, { once: true });
    } else {
      wrap?.classList.remove("has-video");
      $("video-empty").textContent = "No video attached — drag the slider to scrub the recorded data.";
      scrub.style.display = "block"; scrub.value = 0;
      scrub.oninput = () => applyReplayAt((scrub.value / 1000) * Session.durationMs);
      $("video-mode").textContent = "review (data only) — drag the slider to scrub";
      $("video-sync").textContent = `${(Session.durationMs / 1000).toFixed(1)}s`;
      applyReplayAt(0);
    }
  }
  function exitReplay() {
    if (!S.replayActive) return;
    S.replayActive = false; S.stabilityOverride = null; S.clockT = null;
    const cam = $("cam"), scrub = $("scrub");
    if (cam) { cam.ontimeupdate = cam.onseeking = cam.onseeked = null; try { cam.pause(); } catch {} cam.removeAttribute("src"); cam.controls = false; cam.load?.(); }
    if (scrub) { scrub.style.display = "none"; scrub.oninput = null; }
    videoWrap()?.classList.remove("has-video");
    $("video-mode").textContent = "idle"; $("video-sync").textContent = "";
  }
  function applyReplayAt(sessionMs) {
    const F = Session.frames; if (!F.length) return;
    lastReplayMs = sessionMs;
    let lo = 0, hi = F.length - 1, idx = 0;
    while (lo <= hi) { const m = (lo + hi) >> 1; if (F[m].t <= sessionMs) { idx = m; lo = m + 1; } else hi = m - 1; }
    const fr = F[idx];
    S.clockT = fr.t;
    applyFootPos(sampleFootTrack(sessionMs)); // vision foot positions for the 3D stance
    S.cop = { x: fr.cop.x, y: fr.cop.y };
    S.copFoot.left = { x: fr.cl.x, y: fr.cl.y }; S.copFoot.right = { x: fr.cr.x, y: fr.cr.y };
    S.sideLoad.left = fr.ll; S.sideLoad.right = fr.lr;
    S.sensors.left = fr.sl.slice(); S.sensors.right = fr.sr.slice();
    S.flexion = fr.fx; S.lean = fr.ln; S.twist = fr.tw; S.pelvisFlexion = fr.pf;
    if (fr.fyl != null) S.footYaw.left = fr.fyl;
    if (fr.fyr != null) S.footYaw.right = fr.fyr;
    S.steps = fr.st; S.lifts = fr.li; S.liftsGood = fr.lg; S.liftsBad = fr.lb;
    S.stabilityOverride = fr.stab;
    S.copTrail = [];
    for (let i = Math.max(0, idx - CFG.copTrail + 1); i <= idx; i++) S.copTrail.push({ x: F[i].cop.x, y: F[i].cop.y });
    const vis = S.useVision;
    S.footprints = Session.footEvents
      .filter((e) => e.t <= fr.t && fr.t - e.t <= MAP.windowMs)
      .map((e) => {
        const useV = vis && e.hasVis;
        return { x: useV ? e.xVis : e.x, y: useV ? e.yVis : e.y, side: e.side, n: e.n, t: e.t, heading: e.heading };
      });
    renderPosture(fr.zone); renderCounters(); renderGait();
  }

  // ---------- recording file loader ----------
  const readJSON = (file) => new Promise((res, rej) => {
    const fr = new FileReader();
    fr.onload = () => { try { res(JSON.parse(fr.result)); } catch (e) { rej(e); } };
    fr.onerror = () => rej(new Error("read error"));
    fr.readAsText(file);
  });
  const isFootTrack = (obj, name) => (obj && (obj.schema === "foot-track/v1" || Array.isArray(obj.frames) && obj.frames[0] && ("l" in obj.frames[0] || "r" in obj.frames[0]))) || /foot-?track/i.test(name || "");

  async function handleFiles(fileList) {
    const files = [...(fileList || [])];
    if (!files.length) return;
    const jsonFiles = files.filter((f) => /\.json$/i.test(f.name) || (f.type || "").includes("json"));
    const videoFile = files.find((f) => (f.type || "").startsWith("video/") || /\.(webm|mp4|mov|m4v)$/i.test(f.name));

    // Classify the JSON file(s): a recording (portal-export OR SDK-nested) + optional foot-track.
    let rec = null, portal = null, track = null;
    for (const jf of jsonFiles) {
      let obj; try { obj = await readJSON(jf); } catch (e) { log(`Bad JSON (${jf.name}): ${e.message}`, "bad"); return; }
      if (isFootTrack(obj, jf.name)) track = obj;
      else if (obj && obj.recording && (Array.isArray(obj.scalar) || Array.isArray(obj.pressure))) portal = obj; // portal /export
      else { rec = obj; if (!track && obj.footTrack) track = obj.footTrack; } // SDK-nested; may embed a foot-track
    }

    if (!rec && !portal && !Session.frames.length && !track) { log("Select the recording's data JSON (and optionally its video + foot-track).", "warn"); return; }

    if (track) Session.footTrack = track; // set BEFORE derive so footstep events can bake vision positions
    let n = Session.frames.length;
    try {
      if (portal) n = deriveFromPortalExport(portal);
      else if (rec) n = deriveFromRecording(rec);
    } catch (e) { log(`Couldn't read that recording: ${e.message}`, "bad"); return; }
    if (!n) { log("No usable sensor data in that file.", "warn"); return; }
    if (portal || rec) { Session.raw = portal || rec; Session.rawFormat = portal ? "portal" : "sdk"; Session.title = portal?.recording?.title || (jsonFiles[0]?.name || "").replace(/\.json$/i, "") || Session.title; }
    setExportAvailable(!!Session.raw);

    if (videoFile) {
      if (Session.video?.url) URL.revokeObjectURL(Session.video.url);
      const off = (rec && rec.video && rec.video.syncOffsetMs) || (portal && portal.video && portal.video.syncOffsetMs) || (track && track.video && track.video.syncOffsetMs) || 0;
      Session.video = { url: URL.createObjectURL(videoFile), syncOffsetMs: off };
    } else if (rec || portal) { Session.video = null; }

    const tf = Session.footTrack ? ` + foot-track (${Session.footTrack.frames.length} frames)` : "";
    log(`Loaded recording — ${n} frames${videoFile ? ` + video (${videoFile.name})` : ""}${tf}.`);
    setVisionAvailable(!!Session.footTrack);
    setTitle(Session.title);
    enterReplay();
  }

  // ---------- gallery / URL loading + raw export ----------
  function stamp() { return new Date().toISOString().replace(/[:.]/g, "-").slice(0, 19); }
  function download(name, text, mime) {
    const url = URL.createObjectURL(new Blob([text], { type: mime }));
    const a = document.createElement("a"); a.href = url; a.download = name; a.style.display = "none";
    document.body.appendChild(a); a.click();
    setTimeout(() => { URL.revokeObjectURL(url); a.remove(); }, 0);
  }
  function setTitle(t) { const el = $("rec-title"); if (el) el.textContent = t ? `— ${t}` : ""; }
  function setExportAvailable(avail) {
    ["btn-json", "btn-csv"].forEach((id) => { const b = $(id); if (b) b.disabled = !avail; });
  }
  const slug = (s) => (s || "recording").replace(/[^\w.-]+/g, "-").replace(/^-+|-+$/g, "") || "recording";

  // Raw-data exports (the loaded recording, unmodified) — JSON as-is; CSV = raw rows.
  function exportRawJSON() {
    if (!Session.raw) { log("Load a recording first.", "warn"); return; }
    download(`${slug(Session.title)}-${stamp()}.json`, JSON.stringify(Session.raw), "application/json");
    log("Saved raw JSON.");
  }
  function exportRawCSV() {
    if (!Session.raw) { log("Load a recording first.", "warn"); return; }
    const csv = rawToCsv(Session.raw, Session.rawFormat);
    if (!csv) { log("No tabular sensor rows to export.", "warn"); return; }
    download(`${slug(Session.title)}-${stamp()}.csv`, csv, "text/csv");
    log("Saved raw CSV.");
  }
  // Flatten to a long CSV: one row per sensor sample (portal scalar+pressure rows;
  // SDK per-device sensorData). Header union across row types; blanks where N/A.
  function rawToCsv(raw, fmt) {
    const rows = [];
    if (fmt === "portal") {
      for (const r of (raw.scalar || [])) rows.push({ time: r.time, device_id: r.device_id, sensor_type: r.sensor_type, x: r.x, y: r.y, z: r.z, w: r.w });
      for (const r of (raw.pressure || [])) rows.push({ time: r.time, device_id: r.device_id, sensor_type: "pressure", normalized_sum: r.normalized_sum, normalized_center_x: r.normalized_center_x, normalized_center_y: r.normalized_center_y });
    } else if (fmt === "sdk") {
      for (const d of (raw.devices || [])) for (const s of (d.sensorData || [])) (s.data || []).forEach((v, i) => {
        const t = (s.initialTimestamp || 0) + i * (s.dataRate || 20);
        if (s.sensorType === "pressure") rows.push({ time: t, device_id: d.id, sensor_type: "pressure", values: JSON.stringify(v) });
        else rows.push({ time: t, device_id: d.id, sensor_type: s.sensorType, x: v?.x, y: v?.y, z: v?.z, w: v?.w });
      });
    }
    if (!rows.length) return "";
    const cols = [...rows.reduce((set, r) => { Object.keys(r).forEach((k) => set.add(k)); return set; }, new Set())];
    const esc = (v) => v == null ? "" : (typeof v === "string" && /[",\n]/.test(v)) ? `"${v.replace(/"/g, '""')}"` : v;
    return [cols.join(","), ...rows.map((r) => cols.map((c) => esc(r[c])).join(","))].join("\n");
  }

  // Load a recording by URL (gallery card / ?rec= deep link): fetch JSON (+ optional
  // video + foot-track) from the site, derive, and replay. `entry` = manifest row.
  async function loadRecordingByUrl(entry) {
    try {
      log(`Loading “${entry.title || entry.id}”…`);
      Session.footTrack = null;
      const raw = await fetch(entry.json).then((r) => { if (!r.ok) throw new Error(`data ${r.status}`); return r.json(); });
      if (entry.footTrack) Session.footTrack = await fetch(entry.footTrack).then((r) => r.ok ? r.json() : null).catch(() => null);
      const isPortal = raw && raw.recording && (Array.isArray(raw.scalar) || Array.isArray(raw.pressure));
      const n = isPortal ? deriveFromPortalExport(raw) : deriveFromRecording(raw);
      if (!n) { log("No usable sensor data.", "warn"); return; }
      Session.raw = raw; Session.rawFormat = isPortal ? "portal" : "sdk"; Session.title = entry.title || raw.recording?.title || entry.id;
      if (Session.video?.url) URL.revokeObjectURL(Session.video.url);
      Session.video = null;
      if (entry.video) {
        const vb = await fetch(entry.video).then((r) => r.ok ? r.blob() : null).catch(() => null);
        if (vb) Session.video = { url: URL.createObjectURL(vb), syncOffsetMs: entry.syncOffsetMs || 0 };
      }
      log(`Loaded “${Session.title}” — ${n} frames${Session.video ? " + video" : ""}.`);
      setVisionAvailable(!!Session.footTrack); setExportAvailable(true); setTitle(Session.title);
      enterReplay();
    } catch (e) { log(`Couldn't load recording: ${e.message}`, "bad"); }
  }

  async function autoLoadFromUrl() {
    const id = new URLSearchParams(location.search).get("rec");
    if (!id) return;
    try {
      const man = await fetch("recordings/manifest.json").then((r) => r.ok ? r.json() : null);
      const entry = man && (man.recordings || []).find((e) => e.id === id);
      if (entry) loadRecordingByUrl(entry);
      else log(`Recording “${id}” not found in the gallery.`, "warn");
    } catch (e) { log(`Gallery manifest unavailable: ${e.message}`, "warn"); }
  }

  // ---------- main loop ----------
  let lastFrame = now();
  function frame() {
    const t = now(), dt = t - lastFrame; lastFrame = t;
    if (S.replayActive) {
      ShoeStage.render(); drawGauge(); renderLoads(); drawStepMap();
      requestAnimationFrame(frame); return;
    }
    S.clockT = null;
    if (S.sim) simTick(dt);
    ShoeStage.render();
    drawGauge(); renderLoads(); drawStepMap();
    requestAnimationFrame(frame);
  }

  // ---------- wiring ----------
  // Two ways to drive the dashboard: ▶ Simulate (no hardware), or Load Recording
  // (replay a session captured in the BrilliantWear portal, synced to its video).
  $("btn-sim").onclick = () => {
    if (S.replayActive) exitReplay();
    S.sim = !S.sim;
    $("btn-sim").textContent = S.sim ? "⏸ Stop Simulation" : "▶ Simulate";
    $("btn-sim").classList.toggle("primary", !S.sim);
    if (S.sim) { resetSession(); renderCounters(); renderGait(); log("Simulation started — walking, then lifting boxes."); }
    else log("Simulation stopped.");
  };
  $("btn-load").onclick = () => $("file-in").click();
  $("file-in").onchange = (e) => { handleFiles(e.target.files); e.target.value = ""; };
  $("btn-json").onclick = () => exportRawJSON();
  $("btn-csv").onclick = () => exportRawCSV();
  $("vision-toggle").onchange = (e) => {
    S.useVision = e.target.checked;
    log(`Foot positions: ${S.useVision ? "vision (Pose + Depth)" : "insole estimate"}.`);
    if (S.replayActive) applyReplayAt(lastReplayMs); // re-plot the footstep map with the chosen source
  };

  // expose read-only state for the 3D stance module (stance3d.js, ES module)
  window.WH = { S, CFG, PAD_NORM };

  ShoeStage.init();
  setExportAvailable(false);
  log("Ready. Press ▶ Simulate to preview, or Load Recording to replay a portal capture synced to its video.");
  frame();
  autoLoadFromUrl(); // ?rec=<id> deep link from the recordings gallery
})();
