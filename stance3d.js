/* Brilliant Wear — 3D stance widget for the warehouse demo.
   A vanilla-three.js port of the golf app's RecordingStance3D: the same
   translucent shoe GLBs, sensor-bed art, per-pad pressure glow, per-foot +
   combined center-of-pressure, and per-foot heading (relative insole
   orientation). Driven live from window.WH (the warehouse app's state), so it
   works in Simulate and in loaded-recording replay alike.

   Field-tuned constants (SHOE_MODEL_FOR_SIDE, BED_ROTATION_Z, PAD_MIRROR_X,
   STANCE_TOE_SIGN, SHOE_MODEL_YAW) are carried over verbatim from the golf
   widget so the shoes, beds, and pads read the same way up. */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const ASSET = (f) => `assets/${f}`;
const SHOE_COLOR = 0xd8dde3, SHOE_OPACITY = 0.4;
const STANCE_GAP_WIDTHS = 1.5, BED_LENGTH_FRAC = 0.82, BED_WIDTH_FRAC = 0.6, BED_Y_FRAC = 0.16;
const STANCE_TOE_SIGN = 1;
const SHOE_MODEL_FOR_SIDE = { left: "right", right: "left" }; // GLB file names are split from one model; swap per golf field-verify
const BED_ROTATION_Z = { left: Math.PI, right: Math.PI };
const PAD_MIRROR_X = { left: 1, right: -1 };
const SHOE_MODEL_YAW = Math.PI;
const PAD_RADIUS = 0.7, TRAIL_POINTS = 16;
const POS_SCALE = 70; // metres → scene units (foot-track positions + insole-derived step height)
// Golf mirrors the delta across the sagittal plane (its panel views the stance
// from BEHIND, a mirrored-world convention). This app's scene is a true
// from-above world shared with the footstep map, field-calibrated on WE 6's
// door turn: with the mirror on, the wearer's real clockwise turn rendered
// counter-clockwise. Raw delta yaw = device yaw = −(map heading), which is
// exactly the scene's CCW+ convention — so NO mirror here. (Flips roll
// handedness vs the golf look; foot roll in walking is small.)
const MIRROR_DELTA_YAW_ROLL = false;
// Scene sign of the ABSOLUTE mag-north foot heading (S.footNorthDeg, set when a
// recording carries magnetometer-fused rotation) premultiplied onto the rest pose.
// Map heading H is compass-style (CW+ viewed from above); three.js yaw about +Y
// is CCW+ from above — so scene rest yaw = −H, and the raw (unmirrored) delta
// supplies Δyaw = −ΔH natively. Composed: scene yaw = −H(t) — one consistent
// from-above frame shared with the footstep map.
const NORTH_SIGN = -1;
const COP_COLOR = 0xf59e0b, COMBINED_COP_COLOR = 0x22d3ee;
const PAD_COLOR = new THREE.Color(0x2ab5a0), PAD_HOT = new THREE.Color(0xf43f5e);

const bedParity = (side) => (Math.abs(BED_ROTATION_Z[side]) < 0.001 ? 1 : -1);
const bedLocalX = (px, side, bedWidth) => (px - 0.5) * bedWidth * PAD_MIRROR_X[side] * bedParity(side);
const bedLocalZ = (py, side, bedLength) => -(py - 0.5) * bedLength * STANCE_TOE_SIGN * bedParity(side);
const lerp = (a, b, f) => a + (b - a) * f;

const _color = new THREE.Color();
const _v = new THREE.Vector3();
const _q = new THREE.Quaternion();
const _qN = new THREE.Quaternion();
const _yUp = new THREE.Vector3(0, 1, 0);

function waitForWH() {
  return new Promise((res) => {
    const tick = () => (window.WH && window.WH.S ? res(window.WH) : requestAnimationFrame(tick));
    tick();
  });
}

async function boot() {
  const mount = document.getElementById("stance3d");
  if (!mount) return;
  const WH = await waitForWH();
  const { S, PAD_NORM } = WH;

  const scene = new THREE.Scene();
  scene.background = null;
  const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  const H = 360;
  const width = () => mount.clientWidth || 640;
  renderer.setSize(width(), H);
  mount.appendChild(renderer.domElement);

  const camera = new THREE.PerspectiveCamera(42, width() / H, 0.1, 5000);
  const controls = new OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true; controls.dampingFactor = 0.08;

  scene.add(new THREE.HemisphereLight(0xffffff, 0x9099a5, 1.1));
  const dir = new THREE.DirectionalLight(0xffffff, 0.6); dir.position.set(30, 80, 40); scene.add(dir);
  const grid = new THREE.GridHelper(140, 28, 0x3a4270, 0x2a3060);
  grid.material.transparent = true; grid.material.opacity = 0.5; scene.add(grid);

  const combinedCoP = new THREE.Mesh(new THREE.SphereGeometry(1.1, 20, 16), new THREE.MeshBasicMaterial({ color: COMBINED_COP_COLOR }));
  combinedCoP.visible = false; scene.add(combinedCoP);
  const trail = [];
  const trailGeo = new THREE.SphereGeometry(0.6, 12, 10);
  for (let i = 0; i < TRAIL_POINTS; i++) {
    const m = new THREE.Mesh(trailGeo, new THREE.MeshBasicMaterial({ color: COMBINED_COP_COLOR, transparent: true, opacity: 0 }));
    m.visible = false; scene.add(m); trail.push(m);
  }
  const copHist = []; // {x,z} ground history for the trail

  const gltfLoader = new GLTFLoader(), texLoader = new THREE.TextureLoader();
  const loadShoe = (model) => new Promise((res) => gltfLoader.load(ASSET(`golf-shoe-${model}.glb`), (g) => res(g.scene), undefined, () => res(null)));

  const feet = {};

  function makeFoot(side, gltf, mirrored) {
    const bbox = new THREE.Box3().setFromObject(gltf);
    const size = new THREE.Vector3(); bbox.getSize(size);
    const shoeH = Math.max(size.y, 1e-3);
    const lengthAxisIsX = size.x > size.z;
    const shoeLength = Math.max(size.x, size.z), shoeWidth = Math.min(size.x, size.z);
    const baseYaw = lengthAxisIsX ? Math.PI / 2 : 0;
    const restQuat = new THREE.Quaternion().setFromEuler(new THREE.Euler(0, baseYaw + SHOE_MODEL_YAW, 0));

    gltf.traverse((o) => {
      if (o.isMesh) {
        o.material = new THREE.MeshStandardMaterial({ color: SHOE_COLOR, transparent: true, opacity: SHOE_OPACITY, depthWrite: false, metalness: 0, roughness: 0.85, side: mirrored ? THREE.DoubleSide : THREE.FrontSide });
        o.renderOrder = 2;
      }
    });
    const group = new THREE.Group(); group.add(gltf);

    const bedLength = shoeLength * BED_LENGTH_FRAC, bedWidth = shoeWidth * BED_WIDTH_FRAC, bedY = shoeH * BED_Y_FRAC;
    const bedTex = texLoader.load(ASSET(`sensorbed-${side}.png`)); bedTex.colorSpace = THREE.SRGBColorSpace;
    const bed = new THREE.Mesh(new THREE.PlaneGeometry(bedWidth, bedLength), new THREE.MeshBasicMaterial({ map: bedTex, transparent: true, opacity: 0.95, depthWrite: false }));
    bed.rotation.x = -Math.PI / 2; bed.rotation.z = BED_ROTATION_Z[side]; bed.position.y = bedY; bed.renderOrder = 0;
    group.add(bed);

    // 8 pads at the warehouse artwork's normalized positions.
    const pads = [];
    const padGeo = new THREE.SphereGeometry(PAD_RADIUS, 16, 12);
    const pos = PAD_NORM[side];
    for (let k = 0; k < 8; k++) {
      const mat = new THREE.MeshStandardMaterial({ color: PAD_COLOR, emissive: PAD_COLOR, emissiveIntensity: 0.1, roughness: 0.5 });
      const mesh = new THREE.Mesh(padGeo, mat);
      mesh.position.set(bedLocalX(pos[k].x, side, bedWidth), bedY + 0.4, bedLocalZ(pos[k].y, side, bedLength));
      mesh.renderOrder = 1; group.add(mesh); pads.push(mesh);
    }
    const footCoP = new THREE.Mesh(new THREE.SphereGeometry(0.9, 16, 12), new THREE.MeshBasicMaterial({ color: COP_COLOR }));
    footCoP.renderOrder = 1; footCoP.visible = false; group.add(footCoP);

    const corners = [];
    for (const cx of [bbox.min.x, bbox.max.x]) for (const cy of [bbox.min.y, bbox.max.y]) for (const cz of [bbox.min.z, bbox.max.z]) corners.push(new THREE.Vector3(cx, cy, cz));

    return { side, group, pads, footCoP, restQuat, basePos: new THREE.Vector3(), bedLength, bedWidth, bedY, corners };
  }

  let [leftGltf, rightGltf] = await Promise.all([loadShoe(SHOE_MODEL_FOR_SIDE.left), loadShoe(SHOE_MODEL_FOR_SIDE.right)]);
  let lMirror = false, rMirror = false;
  if (!leftGltf && rightGltf) { leftGltf = rightGltf.clone(true); leftGltf.scale.x = -1; lMirror = true; }
  if (!rightGltf && leftGltf) { rightGltf = leftGltf.clone(true); rightGltf.scale.x = -1; rMirror = true; }
  if (!leftGltf || !rightGltf) { mount.innerHTML = '<div class="stance-fallback">3D shoe models could not load.</div>'; return; }

  feet.left = makeFoot("left", leftGltf, lMirror);
  feet.right = makeFoot("right", rightGltf, rMirror);
  scene.add(feet.left.group); scene.add(feet.right.group);

  const avgWidth = (feet.left.bedWidth / BED_WIDTH_FRAC + feet.right.bedWidth / BED_WIDTH_FRAC) / 2;
  const centerDist = avgWidth * (1 + STANCE_GAP_WIDTHS);
  for (const side of ["left", "right"]) {
    const fr = feet[side];
    fr.group.quaternion.copy(fr.restQuat);
    fr.basePos.set(side === "left" ? -centerDist / 2 : centerDist / 2, 0, 0);
    fr.defaultPos = fr.basePos.clone(); // fixed stance, used when vision positions are off/absent
    fr.group.position.copy(fr.basePos);
  }

  const box = new THREE.Box3(); box.expandByObject(feet.left.group); box.expandByObject(feet.right.group);
  const c = new THREE.Vector3(); box.getCenter(c);
  const sz = new THREE.Vector3(); box.getSize(sz);
  const reach = Math.max(sz.x, sz.z, 20);
  controls.target.set(c.x, 3, c.z);
  camera.position.set(c.x, reach * 1.7 + 24, reach * 1.6 + 20);
  controls.update();

  document.getElementById("stance-loading")?.remove();

  const ro = new ResizeObserver(() => { camera.aspect = width() / H; camera.updateProjectionMatrix(); renderer.setSize(width(), H); });
  ro.observe(mount);

  function footGroundCoP(side, out) {
    const fr = feet[side], c = S.copFoot[side], load = S.sideLoad[side];
    if (!(load > 0)) return 0;
    out.set(bedLocalX(c.x, side, fr.bedWidth), 0, bedLocalZ(c.y, side, fr.bedLength));
    out.applyQuaternion(fr.group.quaternion).add(fr.group.position);
    return load;
  }

  function update() {
    // Crossed-feet guard: if vision claims the left foot is to the RIGHT of the right
    // foot (mirror/noise residue), ignore vision positions this frame — never render
    // the shoes crossed/overlapping.
    const fpL = S.footPos && S.footPos.left, fpR = S.footPos && S.footPos.right;
    const visCrossed = !!(fpL && fpR && fpL.c >= 0.6 && fpR.c >= 0.6 && fpL.x > fpR.x);
    // Position pre-pass: vision placement when confident, else the fixed stance
    // rotated by the wearer's mean absolute heading (the pair turns together —
    // per-foot orbiting would be wrong). Then a minimum-separation guard:
    // monocular Z compresses fore-aft distance, so mid-stride pass-throughs
    // rendered both shoes stacked on one spot — enforce ~a shoe width apart
    // along their (vision-derived) separation line, midpoint preserved.
    for (const side of ["left", "right"]) {
      const fr = feet[side];
      const fp = S.footPos && S.footPos[side];
      if (S.useVision && fp && fp.c >= 0.6 && !visCrossed) {   // matches app.js VISION_MIN_CONF
        fr.basePos.set(fp.x * POS_SCALE, 0, -fp.z * POS_SCALE);
      } else {
        fr.basePos.copy(fr.defaultPos);
        if (S.magMap && S.footYaw) {
          const mL = (S.footYaw.left || 0) * Math.PI / 180, mR = (S.footYaw.right || 0) * Math.PI / 180;
          const mean = Math.atan2((Math.sin(mL) + Math.sin(mR)) / 2, (Math.cos(mL) + Math.cos(mR)) / 2);
          _qN.setFromAxisAngle(_yUp, NORTH_SIGN * mean);
          fr.basePos.applyQuaternion(_qN);
        }
      }
    }
    {
      const bl = feet.left.basePos, br = feet.right.basePos;
      const dx = br.x - bl.x, dz = br.z - bl.z, d = Math.hypot(dx, dz);
      // Floor on the SHOE LENGTH, not width: mid-stride the separation line runs
      // fore-aft along the shoes, where a width-sized gap still overlaps them.
      const minSep = ((feet.left.bedLength + feet.right.bedLength) / BED_LENGTH_FRAC / 2) * 0.85;
      if (d < minSep) {
        const ux = d > 1e-3 ? dx / d : 1, uz = d > 1e-3 ? dz / d : 0;
        const push = (minSep - d) / 2;
        bl.x -= ux * push; bl.z -= uz * push;
        br.x += ux * push; br.z += uz * push;
      }
    }
    for (const side of ["left", "right"]) {
      const fr = feet[side];
      // Orientation: the foot's FULL relative quaternion (inverse(rest)·live) composed
      // under the rest pose — the golf widget's field-verified convention
      // (final = rest · delta). A lone Euler yaw is meaningless near gimbal lock
      // mid-stride on real insoles and read as erratic. MIRROR_DELTA_YAW_ROLL
      // carries over the golf widget's device→scene handedness fix (negate y,z:
      // yaw+roll flip, pitch preserved) so a toed-out foot doesn't render pigeon-toed.
      const fq = S.footQ && S.footQ[side];
      if (fq) {
        _q.set(fq.x, fq.y, fq.z, fq.w);
        if (_q.lengthSq() < 1e-6) _q.identity();
        if (MIRROR_DELTA_YAW_ROLL) _q.set(_q.x, -_q.y, -_q.z, _q.w);
        fr.group.quaternion.copy(fr.restQuat).multiply(_q);
      } else {
        fr.group.quaternion.copy(fr.restQuat);
      }
      // Absolute (true-north) anchor: premultiply this foot's mag rest heading so
      // composed yaw = north + delta — full X/Y/Z orientation vs true north (golf's
      // true-north view), per foot and continuous while the wearer walks and turns.
      const northDeg = (S.footNorthDeg && S.footNorthDeg[side]) || 0;
      if (northDeg) {
        _qN.setFromAxisAngle(_yUp, NORTH_SIGN * northDeg * Math.PI / 180);
        fr.group.quaternion.premultiply(_qN);
      }
      // Position comes from the pre-pass above (vision / rotated stance + minimum
      // separation). Step HEIGHT is insole-only (S.footLift: unloaded + tilted =
      // foot in the air) — vision never supplies lift; see app.js.
      const liftY = (S.footLift && S.footLift[side] ? S.footLift[side] : 0) * POS_SCALE;
      // ground-clamp so a pitched/yawed shoe rests on the grid (plus any lift)
      let minY = Infinity;
      for (const cor of fr.corners) { _v.copy(cor).applyQuaternion(fr.group.quaternion); if (_v.y < minY) minY = _v.y; }
      fr.group.position.copy(fr.basePos); fr.group.position.y = (Number.isFinite(minY) ? -minY : 0) + liftY;
      // pads
      const vals = S.sensors[side];
      for (let k = 0; k < fr.pads.length; k++) {
        const vv = Math.max(0, Math.min(1, vals[k] || 0));
        const mat = fr.pads[k].material;
        mat.emissiveIntensity = 0.08 + vv * 2.2;
        _color.copy(PAD_COLOR).lerp(PAD_HOT, Math.max(0, vv - 0.5) * 2);
        mat.emissive.copy(_color);
        fr.pads[k].scale.setScalar(0.7 + vv * 0.9);
      }
      // per-foot CoP
      const c = S.copFoot[side];
      fr.footCoP.position.set(bedLocalX(c.x, side, fr.bedWidth), fr.bedY + 0.8, bedLocalZ(c.y, side, fr.bedLength));
      fr.footCoP.visible = S.sideLoad[side] > 0.02;
    }
    // combined CoP on the ground + trail
    let wx = 0, wz = 0, wsum = 0;
    for (const side of ["left", "right"]) { const w = footGroundCoP(side, _v); if (w > 0) { wx += _v.x * w; wz += _v.z * w; wsum += w; } }
    if (wsum > 0) {
      combinedCoP.visible = true; combinedCoP.position.set(wx / wsum, 0.2, wz / wsum);
      copHist.push({ x: wx / wsum, z: wz / wsum }); if (copHist.length > TRAIL_POINTS) copHist.shift();
    } else combinedCoP.visible = false;
    for (let i = 0; i < trail.length; i++) {
      const p = copHist[copHist.length - 1 - (trail.length - 1 - i)];
      if (p) { trail[i].position.set(p.x, 0.15, p.z); trail[i].visible = true; trail[i].material.opacity = 0.45 * (i / trail.length); }
      else trail[i].visible = false;
    }
    controls.update();
    renderer.render(scene, camera);
    requestAnimationFrame(update);
  }
  requestAnimationFrame(update);
}

boot().catch((e) => {
  const mount = document.getElementById("stance3d");
  if (mount) mount.innerHTML = `<div class="stance-fallback">3D stance unavailable: ${e.message}</div>`;
});
