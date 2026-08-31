// Game bootstrap and main loop.

import { SDM26 } from "./vehicle/params.js";
import { ControlsPanel } from "./game/controlsPanel.js";
import { loadCarModel } from "./render/glbcar.js";
import { AudioPanel } from "./game/audioPanel.js";
import { Powertrain, loadTorqueCurve } from "./vehicle/powertrain.js";
import { BicycleModel } from "./vehicle/bicycle.js";
import { loadTrack, TRACKS } from "./track/track.js";
import { loadVenue } from "./track/venue.js";
import { Renderer } from "./render/renderer.js";
import { Input } from "./game/input.js";
import { Hud } from "./game/hud.js";
import { EngineAudio } from "./game/audio.js";
import { Timing, fmt } from "./game/timing.js";
import { loadEtc, saveEtc } from "./vehicle/etcMap.js";
import { EtcEditor } from "./game/etcEditor.js";
import { isDesktop, installDesktopBehaviour } from "./game/desktop.js";
import { renderSpecSheet } from "./game/specSheet.js";
import { PARAM_DEFAULTS, readParam, writeParam } from "./vehicle/paramMeta.js";
import { SetupAdjuster } from "./vehicle/setupAdjust.js";

// Chassis footprint used for cone strikes, and the wheel hub positions, both
// derived from the LIVE geometry. Hardcoding them meant stretching the
// wheelbase moved the physics but not the car you see or the box that knocks
// cones over.
import { bodyBoxFor, hubsFor } from "./render/carmesh.js";

/** Geometry that changes the drawn car; a change here forces a mesh rebuild. */
const GEOMETRY_PATHS = ["wheelbaseM", "weightDistFront", "trackFrontM", "trackRearM"];

// `rigid` means the camera is bolted to the chassis, so the cockpit stays
// still relative to the driver's head and the world rolls instead. Chase is
// deliberately not rigid.
const CAMERAS = [
  // `live` means the eye point is read from the parameters every frame rather
  // than captured here, so the eye-height slider actually moves the camera.
  { name: "Cockpit", live: true, pitch: 0, fov: 78, rigid: true },
  { name: "Nose", ahead: 1.35, height: 0.46, pitch: -0.03, fov: 82, rigid: true },
  { name: "Chase", ahead: -4.6, height: 1.85, pitch: -0.14, fov: 70, rigid: false },
];

class Game {
  constructor(dom) {
    this.dom = dom;
    this.renderer = new Renderer(dom.gl);
    this.hud = new Hud(dom.hud);
    this.input = new Input();
    this.audio = new EngineAudio();

    // Controls settings. Mounted here rather than in the boot sequence so the
    // panel and the Input instance share a lifetime -- the panel reads live
    // axis values straight off it, which is the only reliable way to find out
    // which axis a wheel's brake pedal is actually on.
    const controlsRoot = document.getElementById("controls");
    if (controlsRoot) {
      this.controlsPanel = new ControlsPanel(controlsRoot, this.input, () => {
        this.car.steeringServo = this.input.steeringServo();
      });
      // Re-render when plugging a device in changes the detected profile.
      this.input.onProfileChange = () => this.controlsPanel.render();
    }

    const audioRoot = document.getElementById("audioLevels");
    if (audioRoot) this.audioPanel = new AudioPanel(audioRoot, this.audio);

    this.powertrain = null;
    this.car = null;
    this.track = null;
    this.timing = null;

    // Pedal map. Persisted, so a driver's map survives a reload.
    this.etc = loadEtc();
    this.pedal = 0;         // raw accelerator position
    this.plate = 0;         // throttle plate after the ETC map
    this.brakeApplied = 0;  // brake after ABS intervention
    this.etcEditor = new EtcEditor(dom.etcOverlay, {
      getMap: () => this.etc,
      onChange: (map) => { saveEtc(map); dom.etcSummary.textContent = etcSummary(map); },
      getLive: () => ({ pedal: this.pedal, plate: this.plate }),
    });

    // Live setup adjustment from the d-pad. Writes into the same params object
    // the physics holds, so changes land on the next substep.
    this.setup = new SetupAdjuster(SDM26);
    this.clock = 0;

    this.paused = false;
    this.cameraIndex = 0;
    this.assists = { traction: false, abs: false, autoShift: false };
    this.ggTrail = [];
    this.camRoll = 0;
    this.camPitch = 0;
    this.bumpPhase = 0;
    this.spinFront = 0;   // integrated wheel angle, for the visible rims
    this.spinRear = 0;
    this.lastConeSound = 0;

    this.input.onPadChange = (connected, id) => {
      dom.padStatus.textContent = connected ? `Pad: ${shortPadName(id)}` : "Pad: not detected";
      dom.padStatus.classList.toggle("ok", connected);
    };
  }

  async load(trackId) {
    const spec = TRACKS.find((t) => t.id === trackId) ?? TRACKS[0];
    const [curve, track, cadCar] = await Promise.all([
      loadTorqueCurve(),
      spec.kind === "venue" ? loadVenue(spec.url) : loadTrack(spec.url),
      // Optional CAD bodywork. Absent is the normal case, not an error, so
      // this resolves to null rather than rejecting and taking the load with
      // it.
      this.cadCar !== undefined ? Promise.resolve(this.cadCar) : loadCarModel("./data/car.glb"),
    ]);
    this.powertrain = new Powertrain(SDM26, curve);
    this.car = new BicycleModel(SDM26, this.powertrain);
    this.track = track;
    this.timing = new Timing(track);
    this.renderer.setTrack(track);

    // Remembered across track changes so the file is fetched once.
    this.cadCar = cadCar ?? null;
    if (this.cadCar?.error) {
      this.cadStatus = `data/car.glb could not be read: ${this.cadCar.error}`;
      console.warn(this.cadStatus);
      this.cadCar = null;
    } else if (this.cadCar) {
      this.renderer.useCarModel(this.cadCar);
      const st = this.cadCar.stats;
      this.cadStatus = st.problems.length
        ? `CAD model loaded with problems: ${st.problems.join("; ")}`
        : `CAD model: ${st.triangles.toLocaleString()} triangles, ` +
          `${st.materials} materials (${st.generator})`;
      console.info(this.cadStatus);
    } else {
      this.cadStatus = "No data/car.glb — drawing the procedural body.";
    }
    const cadNote = document.getElementById("cadNote");
    if (cadNote) cadNote.textContent = this.cadStatus;

    this.restart();
    return { track, curve };
  }

  restart() {
    const p = this.track.startPose();
    this.car.respawn(p.x, p.y, p.psi, 0);
    this.track.resetCones();
    this.timing.reset();
    this.ggTrail.length = 0;
    this.clock = 0;
    this.started = true;
  }

  /** Put the car back on the centreline where it left the course. */
  recover() {
    const loc = this.track.locate(this.car.X, this.car.Y, this.car.psi);
    const back = this.track.closed
      ? (loc.index - 6 + this.track.center.length) % this.track.center.length
      : Math.max(0, loc.index - 6);
    const p = this.track.poseAt(back);
    this.car.respawn(p.x, p.y, p.psi, 0);
    this.timing.say("RECOVERED", 1.5);
  }

  pose() {
    return { x: this.car.X, y: this.car.Y, psi: this.car.psi };
  }

  update(dt) {
    const inp = this.input.poll();

    // Hand the control profile's steering dynamics to the vehicle model, and
    // tell the input layer what the car's steering actually is so a wheel can
    // be mapped through the real ratio rather than an assumed one.
    this.input.carLockDeg = SDM26.maxSteerDeg;
    this.input.carSteeringRatio = SDM26.steeringRatio;
    this.car.steeringServo = this.input.steeringServo();

    // The pedal map is applied here, once, before anything else looks at
    // "throttle": everything downstream -- traction control, the engine, the
    // audio -- is dealing with plate position, not pedal position.
    this.pedal = inp.throttle;
    this.plate = this.etc.evaluate(inp.throttle);

    // The map editor is modal, and it needs the live pedal above to keep
    // feeding its marker, so it returns after the read and before the rest.
    if (this.etcEditor.isOpen) return;

    if (this.input.edges.mapEditor) { this.openEtcEditor(); return; }
    if (this.input.edges.home) { this.goHome(); return; }
    if (this.input.edges.pause) this.setPaused(!this.paused);
    if (this.paused) return;

    this.clock += dt;

    // ---- d-pad setup changes: left/right pick, up/down move ----
    const e = this.input.edges;
    if (e.setupPrev) this.announceSetup(this.setup.select(-1));
    if (e.setupNext) this.announceSetup(this.setup.select(1));
    if (e.setupUp || e.setupDown) {
      const item = this.setup.nudge(e.setupUp ? 1 : -1, this.input.setupHoldScale, this.clock);
      this.announceSetup(item);
    }

    if (this.input.edges.camera) {
      this.cameraIndex = (this.cameraIndex + 1) % CAMERAS.length;
      this.timing.say(CAMERAS[this.cameraIndex].name.toUpperCase(), 1.2);
    }
    if (this.input.edges.traction) {
      this.assists.traction = !this.assists.traction;
      this.dom.tcToggle.checked = this.assists.traction;
      this.timing.say(`TRACTION CONTROL ${this.assists.traction ? "ON" : "OFF"}`, 1.5);
    }
    if (this.input.edges.restart) this.restart();
    if (this.input.edges.reset) this.recover();

    const pt = this.powertrain;
    const tel = this.car.telemetry;

    // ---- gearbox ----
    if (this.assists.autoShift) {
      if (pt.canShift()) {
        if (pt.engineRpm > pt.optimalUpshiftRpm()) pt.requestUpshift();
        else if (pt.gear > 0 && pt.engineRpm < 5200 && pt.downshiftSafe(this.car.wR)) {
          pt.requestDownshift();
        }
      }
    } else {
      if (this.input.edges.upshift) pt.requestUpshift();
      if (this.input.edges.downshift) {
        // Refuse a downshift that would bounce the engine off the limiter.
        if (pt.downshiftSafe(this.car.wR)) pt.requestDownshift();
        else this.timing.say("MONEY SHIFT BLOCKED", 1.2);
      }
    }

    // ---- driver aids ----
    let throttle = this.plate;
    let brake = inp.brake;
    if (this.assists.traction) {
      const over = tel.kappaR - 0.13;
      if (over > 0) throttle = Math.max(0.1, throttle * (1 - Math.min(0.9, over * 6)));
    }
    if (this.assists.abs) {
      // Release when a wheel is deep into lockup (large negative slip ratio).
      const lockF = -tel.kappaF - 0.16;
      const lockR = -tel.kappaR - 0.16;
      const worst = Math.max(lockF, lockR);
      if (worst > 0) brake = Math.max(0.15, brake * (1 - Math.min(0.85, worst * 5)));
    }

    this.brakeApplied = brake;

    // ---- physics ----
    this.car.step(dt, { steer: inp.steer, throttle, brake });

    // A venue is bounded by a barrier rather than scored on cones: hold the
    // car inside the foot of the banking before anything reads its position.
    if (this.track.constrain) this.track.constrain(this.car);

    // ---- course state ----
    const loc = this.track.locate(this.car.X, this.car.Y, this.car.psi);
    const hits = this.track.strikeCones(this.pose(), bodyBoxFor(SDM26));
    const moving = this.car.speed > 0.6;
    this.timing.update(dt, loc, moving, hits);
    this.loc = loc;

    if (hits > 0) {
      this.audio.coneHit();
      this.input.rumble(0.7, 0.4, 130);
    }

    // Rumble for the things a driver feels: wheelspin, lockup, running wide.
    const spin = Math.max(0, tel.kappaR - 0.2);
    const lock = Math.max(0, -Math.min(tel.kappaF, tel.kappaR) - 0.2);
    const rough = loc.onTrack ? 0 : 0.35;
    const buzz = Math.min(1, spin * 0.8 + lock * 0.8 + rough);
    if (buzz > 0.06 && performance.now() - this.lastConeSound > 90) {
      this.lastConeSound = performance.now();
      this.input.rumble(buzz * 0.3, buzz * 0.65, 100);
    }

    // ---- audio ----
    this.audio.update({
      rpm: pt.engineRpm,
      throttle,
      // Indicated torque, not net: the sound model solves its heat release to
      // reproduce this much combustion work, and an engine idling at zero NET
      // torque is still burning fuel and still audible. This also carries the
      // idle plate, so the idle note comes from the 14% opening the ETC really
      // holds rather than from a closed throttle.
      torqueNm: pt.shiftTimer > 0 ? 0 : pt.indicatedTorque(pt.engineRpm, throttle),
      // Same for the throttle the sound sees.
      throttlePlate: pt.shiftTimer > 0 ? 0 : pt.platePosition(pt.engineRpm, throttle),
      speed: this.car.speed,
      slip: Math.max(tel.utilF, tel.utilR),
      wheelspin: Math.max(0, tel.kappaR - 0.15),
      shifting: pt.shiftTimer > 0,
    }, dt);

    // ---- wheel rotation, so the rims visibly turn and spin ----
    // Wrapped to keep the angle small; at 25 m/s these turn 125 rad/s and the
    // float would lose precision within a few minutes of running.
    const TAU = Math.PI * 2;
    this.spinFront = (this.spinFront + this.car.wF * dt) % TAU;
    this.spinRear = (this.spinRear + this.car.wR * dt) % TAU;

    // ---- chassis attitude, smoothed so it reads as body motion not jitter ----
    // Signs: positive ay is a LEFT turn, and a car leans onto its outside
    // (right) springs, so roll is +ay. Braking is negative ax and the nose
    // dives, so pitch is +ax. Both were inverted before the cockpit existed,
    // which nothing on screen could show.
    const k = Math.min(1, dt * 9);
    this.camRoll += (((tel.ayG * SDM26.rollGradientDegG) * Math.PI) / 180 - this.camRoll) * k;
    this.camPitch += (((tel.axG * SDM26.pitchGradientDegG) * Math.PI) / 180 - this.camPitch) * k;
    this.bumpPhase += dt * (2 + this.car.speed * 0.55);

    // ---- g-g trail ----
    this.ggTrail.push({ ax: tel.axG, ay: tel.ayG });
    if (this.ggTrail.length > 110) this.ggTrail.shift();
  }

  render() {
    const cam = CAMERAS[this.cameraIndex];
    const tel = this.car.telemetry;
    // Surface texture through the seat: tiny, speed-scaled, and it does a lot
    // for the sense of motion two feet off the deck.
    const vib = SDM26.vibrationScale;
    const bump = (Math.sin(this.bumpPhase * 6.1) * 0.0035 * Math.min(1, this.car.speed / 12) +
                  Math.sin(this.bumpPhase * 11.3) * 0.0018 * Math.min(1, this.car.speed / 18)) * vib;

    // Blur the rims in once the wheels turn fast enough for five spokes to
    // alias at 60 Hz (roughly 5 m/s and up).
    const wheelRate = Math.max(Math.abs(this.car.wF), Math.abs(this.car.wR));
    const rimFade = Math.max(0, Math.min(0.85, (wheelRate - 22) / 70));

    this.renderer.fovDeg = cam.fov;
    this.renderer.draw({
      car: {
        x: this.car.X,
        y: this.car.Y,
        psi: this.car.psi,
        rollRad: this.camRoll,
        pitchRad: this.camPitch,
      },
      view: {
        ahead: cam.live ? SDM26.eyeAheadOfCgM : cam.ahead,
        height: cam.live ? SDM26.eyeHeightM : cam.height,
        pitchOffset: cam.pitch,
        rigid: cam.rigid,
      },
      hubs: hubsFor(SDM26),
      wheels: {
        steerRad: this.car.delta,
        steerRatio: SDM26.steeringRatio,
        spinFront: this.spinFront,
        spinRear: this.spinRear,
        rimFade,
      },
      heaveM: bump - Math.abs(tel.axG) * (SDM26.heaveMmG / 1000) * 0.5 * vib,
      fovBoost: Math.min(10, this.car.speed * 0.42),
    });

    const t = this.timing;
    const last = t.laps.length ? t.laps[t.laps.length - 1] : null;
    this.hud.draw({
      track: this.track,
      trackName: this.track.name,
      closed: this.track.closed,
      rpm: this.powertrain.engineRpm,
      revLimit: SDM26.revLimitRpm,
      shiftRpm: this.powertrain.optimalUpshiftRpm(),
      peakTorqueRpm: this.powertrain.peakTorque.rpm,
      peakPowerRpm: this.powertrain.peakPower.rpm,
      gear: this.powertrain.gear + 1,
      shifting: this.powertrain.shiftTimer > 0,
      speedKph: this.car.speed * 3.6,
      pedal: this.pedal,
      plate: this.plate,
      brake: this.brakeApplied,
      etcName: this.etc.name,
      setup: this.setup.state(this.clock),
      lapTimeText: fmt(t.state === "staged" ? 0 : t.lapTime),
      lastLapText: last ? fmt(last.total) : "--.---",
      bestLapText: t.best ? fmt(t.best.total) : "--.---",
      lap: t.lap,
      progressPct: Math.round(((this.loc?.s ?? 0) / this.track.length) * 100),
      cones: t.cones,
      offCourse: t.offCourse,
      penaltyS: t.penaltyS,
      carX: this.car.X,
      carY: this.car.Y,
      carPsi: this.car.psi,
      axG: tel.axG,
      ayG: tel.ayG,
      ggTrail: this.ggTrail,
      balance: tel.balance,
      message: t.message,
      paused: this.paused,
      tractionControl: this.assists.traction,
    });
  }

  announceSetup(item) {
    this.timing.say(`${item.short} ${item.get().toFixed(1)}${item.unit}`, 1.6);
  }

  openEtcEditor() {
    this.setPaused(true);
    this.dom.pauseHint.hidden = true;
    this.etcEditor.open();
  }

  /** Back to the home screen, with the run left paused behind it. */
  goHome() {
    this.setPaused(true);
    this.dom.pauseHint.hidden = true;
    this.dom.menu.hidden = false;
    this.audio.setEnabled(false);
    // The run is still there — offer to go back to it rather than bin it.
    this.dom.startBtn.textContent = "Resume run";
    this.dom.restartBtn.hidden = false;
  }

  setPaused(on) {
    this.paused = on;
    this.dom.pauseHint.hidden = !on;
  }
}

// ------------------------------------------------------------------- boot ---

const dom = {
  gl: document.getElementById("scene"),
  hud: document.getElementById("hud"),
  menu: document.getElementById("menu"),
  startBtn: document.getElementById("start"),
  trackSel: document.getElementById("track"),
  tcToggle: document.getElementById("tc"),
  absToggle: document.getElementById("abs"),
  autoToggle: document.getElementById("auto"),
  audioToggle: document.getElementById("sound"),
  padStatus: document.getElementById("padStatus"),
  pauseHint: document.getElementById("pauseHint"),
  loadNote: document.getElementById("loadNote"),
  specs: document.getElementById("specs"),
  etcOverlay: document.getElementById("etcOverlay"),
  etcSummary: document.getElementById("etcSummary"),
  etcBtn: document.getElementById("etcBtn"),
  vehicle: document.getElementById("vehicle"),
  resetParams: document.getElementById("resetParams"),
  paramNote: document.getElementById("paramNote"),
  restartBtn: document.getElementById("restartBtn"),
};

const PARAM_KEY = "fsae-sim.params";

/** Persist only what differs from as-shipped, so defaults can move later. */
function saveParams() {
  try {
    const diff = {};
    for (const path of Object.keys(PARAM_DEFAULTS)) {
      const v = readParam(path);
      if (Math.abs(v - PARAM_DEFAULTS[path]) > 1e-12) diff[path] = v;
    }
    localStorage.setItem(PARAM_KEY, JSON.stringify(diff));
  } catch { /* ignore */ }
}

function loadParams() {
  try {
    const raw = localStorage.getItem(PARAM_KEY);
    if (!raw) return 0;
    const diff = JSON.parse(raw);
    let n = 0;
    for (const [path, v] of Object.entries(diff)) {
      if (path in PARAM_DEFAULTS && Number.isFinite(v)) { writeParam(path, v); n++; }
    }
    return n;
  } catch { return 0; }
}

function etcSummary(map) {
  const d = map.describe();
  const name = map.name === "custom" ? "Custom" : map.name[0].toUpperCase() + map.name.slice(1);
  return `${name} · ${d.points} points · initial gain ${d.initialGain.toFixed(2)} ` +
         `(${d.character}) · ${d.plateAtFullPedal}% plate at full pedal`;
}

let game;

async function boot() {
  installDesktopBehaviour();
  if (isDesktop) document.body.classList.add("desktop");

  // Restore saved overrides BEFORE the sheet renders, so the sliders come up
  // showing what the car is actually running. PARAM_DEFAULTS was captured at
  // import time, so it still holds the as-shipped values for the reset button.
  const restored = loadParams();
  const onParamChange = (path) => {
    saveParams();
    if (GEOMETRY_PATHS.includes(path)) game?.renderer.rebuildCar(SDM26);
  };
  renderSpecSheet(dom.vehicle, onParamChange);
  if (restored) dom.paramNote.textContent = `${restored} parameter${restored === 1 ? "" : "s"} restored from your last session.`;

  dom.resetParams.addEventListener("click", () => {
    for (const path of Object.keys(PARAM_DEFAULTS)) writeParam(path, PARAM_DEFAULTS[path]);
    saveParams();
    renderSpecSheet(dom.vehicle, onParamChange);
    game?.renderer.rebuildCar(SDM26);
    dom.paramNote.textContent = "All parameters back to as-shipped.";
  });

  try {
    game = new Game(dom);
  } catch (err) {
    dom.loadNote.textContent = String(err.message ?? err);
    dom.loadNote.classList.add("error");
    return;
  }

  dom.startBtn.disabled = true;
  dom.loadNote.textContent = "Loading course and engine data...";
  const { track, curve } = await game.load(dom.trackSel.value);

  const pt = game.powertrain;
  dom.specs.innerHTML = `
    <div><span>Course</span><b>${track.name} &middot; ${track.length.toFixed(0)} m &middot;
      ${track.cones.length} cones</b></div>
    <div><span>Engine</span><b>${pt.peakTorque.torqueNm.toFixed(1)} N&middot;m @
      ${pt.peakTorque.rpm} &middot; ${pt.peakPower.powerKW.toFixed(1)} kW @
      ${pt.peakPower.rpm}</b></div>
    <div><span>Source</span><b>${curve.name}</b></div>`;

  dom.loadNote.textContent = "";
  dom.startBtn.disabled = false;

  dom.trackSel.addEventListener("change", async () => {
    dom.startBtn.disabled = true;
    dom.loadNote.textContent = "Loading course...";
    await game.load(dom.trackSel.value);
    dom.loadNote.textContent = "";
    dom.startBtn.disabled = false;
  });

  const sync = () => {
    game.assists.traction = dom.tcToggle.checked;
    game.assists.abs = dom.absToggle.checked;
    game.assists.autoShift = dom.autoToggle.checked;
    game.audio.setEnabled(dom.audioToggle.checked);
  };
  for (const el of [dom.tcToggle, dom.absToggle, dom.autoToggle, dom.audioToggle]) {
    el.addEventListener("change", sync);
  }

  game.renderer.rebuildCar(SDM26);
  dom.etcSummary.textContent = etcSummary(game.etc);
  dom.etcBtn.addEventListener("click", () => game.etcEditor.open());

  const enterSim = (fresh) => {
    sync();
    game.audio.start();
    dom.menu.hidden = true;
    dom.restartBtn.hidden = false;
    dom.startBtn.textContent = "Resume run";
    if (fresh) game.restart();
    game.setPaused(false);
    dom.gl.focus();
  };

  dom.startBtn.addEventListener("click", () => enterSim(!game.started));
  dom.restartBtn.addEventListener("click", () => enterSim(true));

  // Debug handle: lets you poke at the model from the console, e.g.
  //   __sim.car.telemetry, __sim.powertrain.wotTorque(9000)
  window.__sim = game;

  let last = performance.now();
  const frame = (now) => {
    const dt = Math.min((now - last) / 1000, 0.05);
    last = now;
    if (!dom.menu.hidden) {
      // Still poll so the pad-connected badge is live in the menu.
      game.input.poll();
    } else {
      game.update(dt);
    }
    game.render();
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}

function shortPadName(id) {
  const m = /^([^(]+)/.exec(id || "");
  return (m ? m[1] : id).trim().slice(0, 28) || "connected";
}

boot();
