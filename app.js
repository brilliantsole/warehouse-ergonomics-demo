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
    copTrail: 60,
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
    sensors: { left: new Array(8).fill(0), right: new Array(8).fill(0) },
    redSince: null, redTotalMs: 0, lastHapticAt: 0,
    connected: { insoles: false, torso: false, pelvis: false },
  };

  const $ = (id) => document.getElementById(id);
  const now = () => performance.now();

  function log(msg, cls = "") {
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
    S.flexion = Math.abs(e.pitch);
    S.lean = e.roll;
    S.twist = e.yaw;
    if (!S.sim) S.heading = S.twist; // hardware: torso yaw steers the footstep map
    updatePosture();
  }
  function onPelvisQuat(q) {
    if (!S.baselinePelvis) { S.baselinePelvis = q; return; }
    const rel = quatMul(quatConj(S.baselinePelvis), q);
    S.pelvisFlexion = Math.abs(quatToEuler(rel).pitch);
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

  function placeFootprint(side, t) {
    const h = (S.heading * Math.PI) / 180;
    // advance along heading
    S.walker.x += Math.sin(h) * MAP.strideM;
    S.walker.y -= Math.cos(h) * MAP.strideM;
    // perpendicular offset: left foot to the left of the line of travel
    const perp = side === "left" ? -1 : 1;
    const fx = S.walker.x + Math.cos(h) * MAP.lateralM * perp;
    const fy = S.walker.y + Math.sin(h) * MAP.lateralM * perp;
    S.stepSeq++;
    S.footprints.push({ x: fx, y: fy, side, n: S.stepSeq, t, heading: S.heading });
    while (S.footprints.length && t - S.footprints[0].t > MAP.windowMs) S.footprints.shift();
  }

  const mapCtx = $("stepmap").getContext("2d");
  function drawStepMap() {
    const w = mapCtx.canvas.width, h = mapCtx.canvas.height;
    mapCtx.clearRect(0, 0, w, h);
    const t = now();
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

    hasSDK() { return typeof window.BS !== "undefined"; },

    async connectInsoles() {
      if (!this.hasSDK()) { log("SDK not loaded — check network / unpkg.", "bad"); return; }
      try {
        // TODO[SDK]: confirm singleton name — examples use BS.DevicePair.insoles
        this.devicePair = BS.DevicePair.insoles || BS.DevicePair.shared;
        this.devicePair.addEventListener("isConnected", (e) => {
          const on = !!(e.message?.isConnected ?? this.devicePair.isConnected);
          setConnected("insoles", on);
        });
        this.devicePair.addEventListener("pressure", (e) => {
          const p = e.message.pressure;
          if (p?.normalizedCenter) onCop(p.normalizedCenter.x, p.normalizedCenter.y);
          // Per-side loads + per-sensor values for the heatmaps.
          // TODO[SDK]: confirm payload fields (sides[], sensors[], normalizedSum).
          const side = e.message.side; // "left" | "right" on per-device events
          if (side && p?.normalizedSum != null) {
            S.sideLoad[side] = p.normalizedSum;
            onSideLoad(side, p.normalizedSum);
          }
          if (side && Array.isArray(p?.sensors)) {
            S.sensors[side] = p.sensors.map((s) => s.normalizedValue ?? s.value ?? 0);
          }
        });
        // Ask each insole to stream pressure
        this.devicePair.setSensorConfiguration({ pressure: CFG.sensorRateMs });
        this.devicePair.resetPressureRange?.();
        // Trigger the browser's Web Bluetooth chooser for each side
        // TODO[SDK]: confirm — examples call toggleConnection() per available device
        (this.devicePair.sides ?? ["left", "right"]).forEach?.(() => {});
        this.devicePair.toggleConnection?.();
        log("Insoles: Web Bluetooth chooser opened. Pick left, then right.");
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
        // TODO[SDK]: confirm event name + payload ("gameRotation" → quaternion)
        device.addEventListener("gameRotation", (e) => {
          const q = e.message.gameRotation ?? e.message.quaternion;
          if (!q) return;
          which === "torso" ? onTorsoQuat(q) : onPelvisQuat(q);
        });
        device.toggleConnection();
      } catch (err) {
        log(`${which} connect failed: ${err.message}`, "bad");
      }
    },

    buzzInsoles() {
      if (S.sim || !this.devicePair) return; // sim: log only
      try {
        // TODO[SDK]: confirm vibration API. Known shape from SDK demos:
        this.devicePair.triggerVibration?.([
          { type: "waveformEffect", waveformEffect: { segments: [{ effect: "strongBuzz100" }] } },
        ]);
      } catch (err) {
        log(`Vibration call failed (fix in SDKAdapter.buzzInsoles): ${err.message}`, "warn");
      }
    },
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
    onCop(copX, copY);
    for (let i = 0; i < 8; i++) {
      const heelBias = i < 3 ? 1.25 : i > 5 ? 0.85 : 1;
      S.sensors.left[i] = Math.max(0, Math.min(1, loadL * heelBias * (0.55 + rnd(0.3))));
      S.sensors.right[i] = Math.max(0, Math.min(1, loadR * heelBias * (0.55 + rnd(0.3))));
    }
    if (!S.baselineTorso) S.baselineTorso = eulerToQuat(0, 0, 0);
    if (!S.baselinePelvis) S.baselinePelvis = eulerToQuat(0, 0, 0);
    onTorsoQuat(eulerToQuat(torsoFlex, rnd(3), rnd(4)));
    onPelvisQuat(eulerToQuat(pelvisFlex, 0, 0));
  }

  // ---------- rendering ----------
  const footL = $("foot-left").getContext("2d");
  const footR = $("foot-right").getContext("2d");
  const copCtx = $("cop").getContext("2d");
  const gaugeCtx = $("gauge").getContext("2d");
  // sensor layout (normalized foot coords), heel → toes
  const SENSOR_POS = [
    [0.5, 0.9], [0.34, 0.82], [0.66, 0.82],       // heel
    [0.38, 0.58], [0.62, 0.58],                   // midfoot
    [0.3, 0.32], [0.52, 0.26], [0.72, 0.3],       // forefoot
  ];

  function drawFoot(ctx, sensors, mirror) {
    const w = ctx.canvas.width, h = ctx.canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.save();
    if (mirror) { ctx.translate(w, 0); ctx.scale(-1, 1); }
    // outline
    ctx.beginPath();
    ctx.ellipse(w * 0.5, h * 0.62, w * 0.3, h * 0.34, 0, 0, Math.PI * 2);
    ctx.ellipse(w * 0.48, h * 0.2, w * 0.26, h * 0.14, -0.15, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(42,48,112,.55)"; ctx.fill();
    ctx.strokeStyle = "#343b7a"; ctx.stroke();
    // sensors
    sensors.forEach((v, i) => {
      const [nx, ny] = SENSOR_POS[i];
      const r = 7 + v * 9;
      const g = ctx.createRadialGradient(w * nx, h * ny, 1, w * nx, h * ny, r);
      g.addColorStop(0, `rgba(0,212,170,${0.25 + v * 0.75})`);
      g.addColorStop(1, "rgba(0,212,170,0)");
      ctx.fillStyle = g;
      ctx.beginPath(); ctx.arc(w * nx, h * ny, r, 0, Math.PI * 2); ctx.fill();
    });
    ctx.restore();
  }

  function drawCop() {
    const w = copCtx.canvas.width, h = copCtx.canvas.height;
    copCtx.clearRect(0, 0, w, h);
    copCtx.strokeStyle = "#343b7a"; copCtx.setLineDash([4, 4]);
    copCtx.strokeRect(10, 10, w - 20, h - 20);
    copCtx.setLineDash([]);
    copCtx.fillStyle = "rgba(154,163,199,.5)";
    copCtx.font = "10px sans-serif";
    copCtx.fillText("center of pressure", 14, 22);
    S.copTrail.forEach((p, i) => {
      const a = i / S.copTrail.length;
      copCtx.fillStyle = `rgba(0,212,170,${a * 0.5})`;
      copCtx.beginPath();
      copCtx.arc(10 + p.x * (w - 20), 10 + p.y * (h - 20), 2 + a * 2, 0, Math.PI * 2);
      copCtx.fill();
    });
    copCtx.fillStyle = "#00d4aa";
    copCtx.beginPath();
    copCtx.arc(10 + S.cop.x * (w - 20), 10 + S.cop.y * (h - 20), 6, 0, Math.PI * 2);
    copCtx.fill();
  }

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
    const st = stabilityScore();
    $("stability").textContent = st == null ? "—" : `${st}/100`;
  }

  // ---------- main loop ----------
  let lastFrame = now();
  function frame() {
    const t = now(), dt = t - lastFrame; lastFrame = t;
    if (S.sim) simTick(dt);
    drawFoot(footL, S.sensors.left, false);
    drawFoot(footR, S.sensors.right, true);
    drawCop(); drawGauge(); renderLoads(); drawStepMap();
    requestAnimationFrame(frame);
  }

  // ---------- wiring ----------
  $("btn-insoles").onclick = () => SDKAdapter.connectInsoles();
  $("btn-torso").onclick = () => SDKAdapter.connectSense("torso");
  $("btn-pelvis").onclick = () => SDKAdapter.connectSense("pelvis");
  $("btn-sim").onclick = () => {
    S.sim = !S.sim;
    $("btn-sim").textContent = S.sim ? "⏸ Stop Simulation" : "▶ Simulate";
    $("btn-sim").classList.toggle("primary", !S.sim);
    if (S.sim) { S.baselineTorso = null; S.baselinePelvisl = null; log("Simulation started — walking, then lifting boxes."); }
    else log("Simulation stopped.");
  };
  $("btn-reset").onclick = () => {
    Object.assign(S, {
      steps: 0, stepTimes: [], stepsPerSide: { left: 0, right: 0 },
      lifts: 0, liftsGood: 0, liftsBad: 0, redTotalMs: 0, redSince: null,
      copTrail: [], copHistory: [], baselineTorso: null, baselinePelvis: null, calSamples: [],
    });
    renderCounters(); renderGait(); log("Session reset.");
  };

  log("Ready. Connect devices, or press Simulate to preview without hardware.");
  if (!SDKAdapter.hasSDK()) log("Note: SDK global not detected — connect buttons will be inert; Simulate works.", "warn");
  frame();
})();
