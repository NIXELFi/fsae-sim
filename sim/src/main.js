// Game bootstrap and main loop.

import { SDM26 } from "./vehicle/params.js";
import { TIRE_INFO } from "./vehicle/tire.js";
import { ControlsPanel } from "./game/controlsPanel.js";
import { loadCarModel, loadWheelModel, loadBodyModel } from "./render/glbcar.js";
import { AudioPanel } from "./game/audioPanel.js";
import { Powertrain, loadTorqueCurve } from "./vehicle/powertrain.js";
import { BicycleModel } from "./vehicle/bicycle.js";
import { loadTrack, TRACKS } from "./track/track.js";
import { loadVenue } from "./track/venue.js";
import { Renderer } from "./render/renderer.js";
import { Input } from "./game/input.js";
import { Hud } from "./game/hud.js";
import { EngineAudio } from "./game/audio.js";
import { Timing, fmt, sectorVerdict, CONE_PENALTY_S, FSAE_OFF_COURSE_PENALTY_S } from "./game/timing.js";
import { keyLabel } from "./game/controlBindings.js";
import { loadEtc, saveEtc } from "./vehicle/etcMap.js";
import { EtcEditor } from "./game/etcEditor.js";
import { isDesktop, installDesktopBehaviour, rigNative, launchOptions, onLaunchOptions, onWindowClose, closeAppWindow, toggleFullscreen, appVersion } from "./game/desktop.js";
import { ForceFeedback } from "./game/forceFeedback.js";
import { NativeCar } from "./vehicle/nativeCar.js";
import { renderSpecSheet } from "./game/specSheet.js";
import { PARAM_DEFAULTS, readParam, writeParam } from "./vehicle/paramMeta.js";
import { SetupAdjuster, ADJUSTABLE_PATHS } from "./vehicle/setupAdjust.js";
import { drawCoursePlan } from "./game/coursePlan.js";
import { Recorder, datumFor } from "./game/recorder.js";

/** Below this many samples the driver never really started, so a dropped
 *  run is not worth mentioning -- they pressed restart on the line. */
const SAMPLE_HZ_FLOOR = 100;

/** How often the car's dash panel is repainted. 30 Hz: a real dash updates
 *  about this fast and nobody can read one that changes quicker. */
const DASH_REFRESH_MS = 1000 / 30;

/**
 * Seconds between the autocross finish line and the end-of-run card.
 *
 * The car is still doing 60 km/h when the clock stops. Long enough to brake
 * and read the time off the dash, short enough that nobody is waiting.
 */
const FINISH_ROLLOUT_S = 2.0;
import { DeltaTimer, referenceFromRun, referenceLapOf } from "./game/delta.js";
import { newRunId, saveRun, runsDirectory, listRuns, loadRun, parseTelemetry } from "./game/runStore.js";
import { Replay } from "./game/replay.js";
import { ReplayPanel, ghostGap } from "./game/replayPanel.js";

// Chassis footprint used for cone strikes, and the wheel hub positions, both
// derived from the LIVE geometry. Hardcoding them meant stretching the
// wheelbase moved the physics but not the car you see or the box that knocks
// cones over.
import { bodyBoxFor, hubsFor } from "./render/carmesh.js";

/** Geometry that changes the drawn car; a change here forces a mesh rebuild. */
const GEOMETRY_PATHS = ["wheelbaseM", "weightDistFront", "trackFrontM", "trackRearM"];

/**
 * Stamped into every recorded run. A lap time only means something next to the
 * build it was set on -- the differential, the torque curve and the steering
 * rack have all moved under the same courses -- so the log says which one.
 *
 * Which is worth nothing if it says the wrong one, and it did: this was a
 * hardcoded "1.0.0" while the app shipped 0.2.0 and then 0.3.0, so every run
 * ever recorded claims a version that has never existed. Now it asks the
 * shell what it actually is at boot, and the literal below is only the
 * browser fallback -- checked against package.json and tauri.conf.json by
 * `tools/validate.js`, so it cannot drift again either.
 */
export let SIM_VERSION = "0.3.0";

/** Ask the shell what build this is; browsers keep the fallback. */
async function resolveSimVersion() {
  const v = await appVersion();
  if (typeof v === "string" && v) SIM_VERSION = v;
}

// `rigid` means the camera is bolted to the chassis, so the cockpit stays
// still relative to the driver's head and the world rolls instead. Chase is
// deliberately not rigid.
// What an imported wheel is scaled to match.
const GEO_FOR_WHEEL = { tireRadius: SDM26.tireRadiusM, rimRadius: 0.127 };
// What an imported body is placed against. The axle stations come from the
// vehicle parameters; bodywork alone cannot say where they are.
const GEO_FOR_BODY = {
  frontAxle: 0.788, rearAxle: -0.742, tireRadius: SDM26.tireRadiusM,
};

// Vertical field of view, degrees.
//
// These used to be enormous -- the cockpit sat at 78 deg vertical, which on a
// 21:9 screen is about 125 deg horizontal. Everything looked far away and the
// edges of the frame were visibly stretched, which is the single biggest
// reason the picture read as a game rather than a car.
//
// The geometrically honest number is the one that makes the image subtend the
// same angle as the real scene: vFov = 2 * atan(screen_height / 2 / distance).
// A 29 in ultrawide (284 mm tall) at 65 cm is 25 deg; a 34 in at 70 cm is
// 28 deg. Nobody drives a single screen that tight -- with no head tracking
// you lose the peripheral view you need to place a cone -- so the cockpit is
// set at 50, roughly where Assetto Corsa's default sits, and the eye-height
// and FOV are both adjustable. Set it to the formula above for a true 1:1.
const CAMERAS = [
  // `live` means the eye point is read from the parameters every frame rather
  // than captured here, so the eye-height slider actually moves the camera.
  { name: "Cockpit", live: true, pitch: 0, fov: 50, rigid: true },
  { name: "Nose", ahead: 1.35, height: 0.46, pitch: -0.03, fov: 55, rigid: true },
  { name: "Chase", ahead: -4.6, height: 1.85, pitch: -0.14, fov: 58, rigid: false },
  // Circles the car rather than following it. The only view that shows the car
  // from anywhere but directly behind, which is what you need to judge the
  // bodywork -- or to check that an imported CAD model is the right shape and
  // the right way round.
  { name: "Walkaround", orbit: true, radius: 3.6, height: 1.05, focusHeight: 0.42,
    fov: 45, rigid: false },
];
const WALKAROUND = CAMERAS[3];

class Game {
  constructor(dom) {
    this.dom = dom;
    this.renderer = new Renderer(dom.gl);
    this.hud = new Hud(dom.hud);
    this.input = new Input();
    this.audio = new EngineAudio();

    // The rig. In the desktop shell the vehicle model, the wheel and the
    // force feedback run natively at 1 kHz; `rigState` is what it reports.
    // In a browser the JS model runs here and the mixer below only feeds
    // the live display in the settings panel.
    this.ffb = new ForceFeedback();
    this.rigState = { running: false, ffbSupported: false, wheelPresent: false, wheelName: "", wheelError: "" };
    this.useNative = false;
    this.rigReady = rigNative.available()
      ? rigNative.start().then((st) => {
          this.rigState = st;
          this.useNative = !!st.running;
          this.controlsPanel?.render();
        })
      : Promise.resolve();

    // Walkaround camera state. Free rather than a fixed orbit: looking at a
    // car means choosing the angle, and the interesting ones -- low at a
    // wheel, down on the floor, level with a wing -- are not on any one circle.
    this.orbitAngle = Math.PI * 0.75;
    this.orbitRadius = 3.6;
    this.orbitHeight = 1.05;
    this.orbitFocus = 0.42;
    this.orbitAuto = true;
    this.wireWalkaround(dom);

    // Controls settings. Mounted here rather than in the boot sequence so the
    // panel and the Input instance share a lifetime -- the panel reads live
    // axis values straight off it, which is the only reliable way to find out
    // which axis a wheel's brake pedal is actually on.
    const controlsRoot = document.getElementById("controls");
    if (controlsRoot) {
      this.controlsPanel = new ControlsPanel(controlsRoot, this.input, () => {
        this.car.steeringServo = this.input.steeringServo();
        this.syncFfb();
        updateSession();
      }, this);
      // Re-render when plugging a device in changes the detected profile.
      this.input.onProfileChange = () => { this.controlsPanel.render(); this.syncFfb(); updateSession(); };
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
      onChange: (map) => { saveEtc(map); dom.etcSummary.innerHTML = etcSummary(map); this.syncFfb(); updateSession(); },
      getLive: () => ({ pedal: this.pedal, plate: this.plate }),
      // Escape closes the editor AND reaches the input layer as a pause
      // edge on the same frame; swallow that one so Done and Escape agree.
      onClose: () => { this.setPaused(false); this.swallowPauseEdge = true; },
    });

    // Live setup adjustment from the d-pad. Writes into the same params object
    // the physics holds, so changes land on the next substep.
    this.setup = new SetupAdjuster(SDM26);
    this.clock = 0;

    this.paused = false;
    /**
     * The end-of-run card is up (autocross only -- see `onRunFinished`).
     *
     * Separate from `paused` because the two look different and offer
     * different things: pause is "I stepped away", finish is "that was the
     * run, here is what it scored". Both stop the car, so `paused` is set as
     * well and this only decides which card is on screen.
     */
    this.finished = false;
    /** Game clock at which the finish card appears; null when not rolling out. */
    this.finishAt = null;
    /** The scored entry the card is reporting. */
    this.finishRun = null;
    /** The save started at the line; resolves to the run id the card can replay. */
    this.finishSave = null;
    this.finishRunId = null;
    /** The sector splits the finished lap was scored on, and the bests it
     *  was driven against. Both captured by the lap hook. */
    this.finishSectors = [];
    this.finishBestBefore = [];
    this.lapSectors = null;
    this.prevBestSectors = null;
    this.bestSectorsBefore = null;
    /** Why nothing was written, when nothing was. */
    this.notSavedNote = null;
    /** Which end-of-run card is on screen; see `showFinishMenu`. */
    this._finishToken = 0;
    this.cameraIndex = 0;
    this.assists = { traction: false, abs: false, autoShift: false };
    this.ggTrail = [];
    this.camRoll = 0;
    this.camPitch = 0;
    // The driver's head, relative to the chassis: outboard lean, fore-and-aft
    // slide, and the eyes leading into the corner. See `headLatMPerG`.
    this.headLat = 0;
    this.headLong = 0;
    this.headYaw = 0;
    this.bumpPhase = 0;
    this.spinFront = 0;   // integrated wheel angle, for the visible rims
    this.spinRear = 0;
    this.lastConeSound = 0;

    // ---- run recording --------------------------------------------------
    // Every drive is logged unless the launcher asked for it not to be. The
    // recorder is created by `restart()`, so a run is exactly one drive from
    // the line to whatever ended it, and `recorderContext` is allocated once
    // and mutated in place -- it is written 100 times a second and a fresh
    // object per sample would be the only garbage this loop makes.
    this.recorder = null;
    // Live delta to a reference lap. Created with the course, because its
    // table is indexed by distance round that course.
    this.deltaTimer = null;
    /** Set when a launcher opened this window with `--replay`: closing the
     *  replay closes the window instead of returning to the launch screen. */
    this.launchedForReplay = false;
    this.recorder = null;
    this.replay = null;      // a Replay while watching a recorded run
    this.ghost = null;       // a second Replay drawn alongside it
    this.replayPanel = null;
    this.recording = true;
    this.driverName = "";
    /** The launcher's account id for the driver, when there was a launcher. */
    this.driverId = null;
    this.sessionLabel = "";
    /** Set when a launcher supplied this session's run settings, so the UI can
     *  say so rather than look like the rig's own settings changed. */
    this.launchedBy = null;
    this.lastSavedRun = null;
    this.saveError = null;
    this._ctx = { assists: this.assists, t: null, car: null };

    this.input.onPadChange = (connected, id) => {
      dom.padStatus.textContent = connected ? shortPadName(id) : "not detected";
      dom.padStatus.classList.toggle("ok", connected);
    };
  }

  /**
   * Mouse and wheel control for the walkaround camera.
   *
   * Bound to the canvas rather than the window so the HUD panels and the
   * settings sliders keep working normally, and only while that camera is
   * selected so a stray drag never moves a driving view.
   */
  wireWalkaround(dom) {
    const canvas = dom.gl.canvas ?? dom.gl;
    // The launch screen shows the walkaround behind its panels, so the drag
    // and wheel work there too.
    const active = () => CAMERAS[this.cameraIndex]?.orbit || !dom.menu.hidden;

    let dragging = false;
    let lastX = 0;
    let lastY = 0;

    canvas.addEventListener("pointerdown", (e) => {
      if (!active()) return;
      dragging = true;
      this.orbitAuto = false;   // taking hold stops the automatic turn
      lastX = e.clientX;
      lastY = e.clientY;
      canvas.setPointerCapture?.(e.pointerId);
    });
    canvas.addEventListener("pointerup", (e) => {
      dragging = false;
      canvas.releasePointerCapture?.(e.pointerId);
    });
    canvas.addEventListener("pointerleave", () => { dragging = false; });
    canvas.addEventListener("pointermove", (e) => {
      if (!dragging || !active()) return;
      this.orbitAngle -= (e.clientX - lastX) * 0.006;
      // Height, not pitch: the camera always looks at the car, so raising the
      // eye is what "look down at it" means here.
      this.orbitHeight = Math.min(3.5, Math.max(0.06,
        this.orbitHeight + (e.clientY - lastY) * 0.006));
      lastX = e.clientX;
      lastY = e.clientY;
    });

    canvas.addEventListener("wheel", (e) => {
      if (!active()) return;
      e.preventDefault();
      // Multiplicative, so it zooms at the same apparent rate close up and far
      // away. Floor at 1.2 m: closer than that and the near plane clips the car.
      this.orbitRadius = Math.min(14, Math.max(1.2,
        this.orbitRadius * Math.exp(e.deltaY * 0.0012)));
    }, { passive: false });

    canvas.addEventListener("dblclick", () => {
      if (!active()) return;
      this.orbitAuto = !this.orbitAuto;
      this.timing?.say(this.orbitAuto ? "WALKAROUND: AUTO" : "WALKAROUND: FREE", 1.2);
    });
  }

  async load(trackId) {
    // Loading a course abandons whatever was being driven on the last one.
    this.endRun("track-changed");
    const spec = TRACKS.find((t) => t.id === trackId) ?? TRACKS[0];
    this.trackId = spec.id;
    const [curve, track, cadCar, cadWheel, cadBody] = await Promise.all([
      loadTorqueCurve(),
      spec.kind === "venue" ? loadVenue(spec.url) : loadTrack(spec.url),
      // Optional CAD bodywork. Absent is the normal case, not an error, so
      // this resolves to null rather than rejecting and taking the load with
      // it.
      this.cadCar !== undefined ? Promise.resolve(this.cadCar) : loadCarModel("./data/car.glb"),
      // A wheel on its own, which is a much easier thing to supply than a
      // whole car and is four of the biggest objects on screen.
      this.cadWheel !== undefined
        ? Promise.resolve(this.cadWheel)
        : loadWheelModel("./data/wheel.glb", GEO_FOR_WHEEL),
      // Bodywork on its own -- what a CFD assembly usually is, and what an STL
      // round trip leaves you with.
      this.cadBody !== undefined
        ? Promise.resolve(this.cadBody)
        : loadBodyModel("./data/body.glb", GEO_FOR_BODY,
                        { offsetM: this.bodyOffsetM ?? 0 }),
    ]);
    await this.rigReady;
    this.powertrain = new Powertrain(SDM26, curve);
    this.car = this.useNative
      ? new NativeCar(SDM26, this.powertrain)
      : new BicycleModel(SDM26, this.powertrain);
    // Natively the gearbox lives in the rig: shift requests, canShift and
    // the rest must go through the car's proxy, which also mirrors the
    // JS instance's state for the HUD and audio. Pointing `powertrain` at
    // the JS object here meant every gear change was written into a state
    // the next snapshot overwrote -- the car never shifted.
    if (this.useNative) this.powertrain = this.car.pt;
    this.track = track;
    if (this.useNative) {
      this.car.pushParams();
      this.car.pushBoundary(track);
      this.syncFfb();
    }
    this.timing = new Timing(track);
    // A new course means a new reference: a time-at-distance table for the
    // autocross means nothing on the endurance loop.
    this.deltaTimer = new DeltaTimer(track.length);
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
      this.cadStatus = "No data/car.glb -- drawing the procedural body.";
    }
    this.cadWheel = cadWheel ?? null;
    if (this.cadWheel?.error) {
      this.wheelStatus = `data/wheel.glb could not be read: ${this.cadWheel.error}`;
      console.warn(this.wheelStatus);
      this.cadWheel = null;
    } else if (this.cadWheel && !this.cadCar) {
      // A whole-car model already brings its own wheels; only use a separate
      // wheel when the body is procedural.
      this.renderer.useWheelModel(this.cadWheel);
      const w = this.cadWheel.stats;
      this.wheelStatus = `CAD wheel: ${w.triangles.toLocaleString()} triangles ` +
        `(${w.tyreTriangles.toLocaleString()} tyre + ${w.rimTriangles.toLocaleString()} rim)`;
      console.info(this.wheelStatus, w.notes);
    } else {
      this.wheelStatus = "";
    }

    this.cadBody = cadBody ?? null;
    if (this.cadBody?.error) {
      this.bodyStatus = `data/body.glb could not be read: ${this.cadBody.error}`;
      console.warn(this.bodyStatus);
      this.cadBody = null;
    } else if (this.cadBody && !this.cadCar) {
      // Trim on the measured wheel bays, set by eye against the model.
      //
      // The bay detection finds the narrowest station in each half of the car,
      // and on this body that lands ahead of the real axle at both ends -- by
      // 200 mm at the front and 100 mm at the rear. The bays are not symmetric
      // about the axle: bodywork is cut away further ahead of a wheel than
      // behind it, to clear the tyre as it steers and to let air out, so the
      // narrowest point sits forward of the hub. The bias differs front to
      // rear because only the front wheels steer.
      //
      // Kept as a named trim rather than folded into the detector: it is a
      // correction someone made by looking, and it should stay visible as one.
      const trim = this.wheelTrimM ?? { front: -0.200, rear: -0.100 };
      const axles = this.cadBody.axles && {
        front: this.cadBody.axles.front + trim.front,
        rear: this.cadBody.axles.rear + trim.rear,
      };
      this.renderer.useBodyModel(this.cadBody.body, axles, {
        front: SDM26.trackFrontM,
        rear: SDM26.trackRearM,
        tireRadius: SDM26.tireRadiusM,
      });
      const s = this.cadBody.stats;
      this.bodyStatus = `CAD body: ${s.triangles.toLocaleString()} triangles, ` +
        `${s.lengthM.toFixed(2)} m long` +
        (s.wheelbaseM
          ? `, wheel bays ${s.wheelbaseM.toFixed(3)} m apart ` +
            `(parameters say ${(SDM26.wheelbaseM).toFixed(3)})`
          : "");
      console.info(this.bodyStatus, s.notes);
      if (s.problems.length) console.warn("body model:", s.problems);
    } else {
      this.bodyStatus = "";
    }

    const cadNote = document.getElementById("cadNote");
    if (cadNote) {
      cadNote.textContent = [this.cadStatus, this.bodyStatus, this.wheelStatus]
        .filter(Boolean).join("  /  ");
    }

    this.restart();
    return { track, curve };
  }

  restart() {
    // Whatever was being driven is over; bank it before the car moves.
    this.endRun("restarted");
    this.hideFinishMenu();
    // The delta's reference rolls on lap completion whether or not anything
    // is being recorded, so its hook is installed here rather than with the
    // recorder's -- a driver with logging off still gets a delta.
    this.installLapHooks();
    const p = this.track.startPose();
    this.car.respawn(p.x, p.y, p.psi, 0);
    this.track.resetCones();
    // A restart is "try again": the reference the driver is chasing stays.
    this.timing.reset({ keepBest: true });
    // A restart is a new lap, not a new session: the reference the driver is
    // chasing survives, exactly as their best time does.
    this.deltaTimer?.reset();
    this.ggTrail.length = 0;
    this.clock = 0;
    this.started = true;
    this.beginRun();
  }

  /**
   * Route lap completions to everything that cares: the run recorder, and the
   * delta timer's reference lap.
   *
   * One listener, because `Timing.onLap` is a single slot and the recorder is
   * only sometimes there.
   */
  installLapHooks() {
    if (!this.timing) return;
    this.timing.onLap = (entry, sectors) => {
      this.recorder?.recordLap(entry, sectors);
      // Kept for the end-of-run card, and it has to be taken HERE: this hook
      // is the last moment the splits exist. `completeLap` clears them for
      // the next lap before it returns, so reading them back in
      // `onRunFinished` -- which runs after `Timing.update` has returned --
      // found an empty array every time.
      this.lapSectors = sectors.slice();
      // ...and the bests as they stood BEFORE this lap folded into them, so
      // a sector can be compared against something other than itself.
      this.prevBestSectors = this.bestSectorsBefore ?? [];
      this.bestSectorsBefore = this.timing.bestSectors.slice();
      // RAW, not the scored total: a cone is a penalty, not a slower lap, and
      // a driver chasing a reference is chasing the driving. And whether the
      // lap counted, because the delta does not read `Timing.best` and would
      // otherwise take a cut lap as the thing to beat.
      const took = this.deltaTimer?.completeLap(entry.raw, { valid: entry.valid !== false });
      if (took) {
        this.timing.say(`REFERENCE  ${fmt(entry.raw)}`, 2);
        // The log has to say what the delta beside it was measured against,
        // and that changes mid-run the moment a quicker lap takes over.
        this.recorder?.setReference(this.deltaTimer.describeReference());
      }
    };
  }

  /** Open a recorder for the drive that is about to start. */
  beginRun() {
    if (!this.recording || !this.track) { this.recorder = null; return; }
    const trackId = this.trackId ?? "autocross";
    this.recorder = new Recorder({
      runId: newRunId(trackId),
      track: trackId,
      trackName: this.track.name,
      trackKind: this.track.kind ?? "course",
      trackLengthM: this.track.length,
      trackClosed: !!this.track.closed,
      trackSectors: Array.from(this.track.sectors ?? []),
      trackSource: this.track.source ?? this.track.provenance ?? null,
      datum: datumFor(trackId),
      driver: this.driverName || "Unknown",
      // Who Helios says this is. Absent when the simulator was opened
      // directly, which is exactly the distinction the leaderboard needs:
      // a name typed into a text box is not an identity.
      driverId: this.driverId || null,
      session: this.sessionLabel || null,
      profile: this.input.settings?.activeId ?? this.input.profile?.id ?? null,
      profileName: this.input.profile?.label ?? this.input.profile?.name ?? null,
      device: this.rigState.wheelName || this.input.nativeName || null,
      physics: this.car?.native ? "native-1khz" : "javascript",
      assists: { ...this.assists },
      setup: snapshotSetup(),
      etc: this.etc?.points ? JSON.parse(JSON.stringify(this.etc.points)) : null,
      simVersion: SIM_VERSION,
      // Whatever the delta is already chasing -- a lap Helios loaded, or the
      // best of the session so far. The lap hook only reports a reference
      // that CHANGES, so without this a run that never beat its reference
      // said it had none.
      reference: this.deltaTimer?.describeReference() ?? null,
    });
  }

  /**
   * Close the current recording and write it out. Safe to call at any time
   * and from anywhere -- a second call is a no-op, and a run too short to
   * mean anything is dropped rather than filed.
   */
  endRun(reason = "ended") {
    const rec = this.recorder;
    this.recorder = null;
    // The lap hook stays: the delta timer needs it even with nothing being
    // recorded, and `installLapHooks` reads `this.recorder` at call time.
    if (!rec || rec.finished) return null;
    rec.finish(reason);
    if (!rec.worthSaving) {
      // Out loud. A run vanishing without explanation is how a driver ends up
      // wondering whether the archive is broken; "no lap completed" is a thing
      // they can do something about.
      if (rec.samples > SAMPLE_HZ_FLOOR) {
        this.timing?.say(`NOT SAVED  ${rec.notSavedReason.toUpperCase()}`, 3);
      }
      return null;
    }
    const runId = rec.meta.runId;
    const manifest = rec.toManifest();
    const csv = rec.toCsv();
    // Fire and forget: the driver is already on to the next thing, and a
    // failed write must not take the game down with it.
    const p = saveRun(runId, manifest, csv)
      .then((res) => {
        this.lastSavedRun = { ...res, stats: manifest.stats, track: manifest.trackName };
        this.saveError = null;
        updateSession();
        refreshRuns();
        return res;
      })
      .catch((err) => {
        console.error("could not save the run", err);
        this.saveError = String(err?.message ?? err);
        updateSession();
        return null;
      });
    this.pendingSave = p;
    return p;
  }

  /**
   * Start or stop the native force feedback to match the active profile. The
   * shell only opens the wheel when a wheel profile with FFB enabled is
   * active, so a pad user never has DirectInput grabbing a device they do not
   * have.
   */
  syncFfb() {
    if (!this.useNative || !this.car?.native) return;
    this.car.pushControls(this.input.profile, this.etc?.points);
    // The command is queued for the rig thread; status read now is the old
    // state, so read again once it has had a tick or two to act on it.
    const refresh = () => rigNative.status().then((st) => { this.rigState = st; this.controlsPanel?.updateStatus(); });
    refresh();
    clearTimeout(this._statusTimer);
    this._statusTimer = setTimeout(refresh, 400);
  }

  /** After any vehicle parameter edit: the rig holds its own copy. */
  pushParams() {
    if (this.car?.native) this.car.pushParams();
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
    this.lastDt = dt;
    // The sampler is ticked at the END of this frame, but the timing, the cone
    // strikes and the shifts all happen before that and all want stamping with
    // the time the frame lands on. Tell the recorder how far ahead the rest of
    // the frame is about to get.
    if (this.recorder) this.recorder.frameDt = dt;
    this.syncNativeInput();
    const inp = this.input.poll();

    // Hand the control profile's steering dynamics to the vehicle model, and
    // tell the input layer what the car's steering actually is so a wheel can
    // be mapped through the real ratio rather than an assumed one.
    this.input.carLockDeg = SDM26.maxSteerDeg;
    this.input.carSteeringRatio = SDM26.steeringRatio;
    // The measured rack: its table and where it stops at the rim.
    this.input.carSteering = SDM26.steering;
    this.input.carRimHalfDeg = SDM26.steering.rimLockDeg;
    // For the keyboard's speed-sensitive lock: last frame's speed is fine.
    this.input.carSpeed = this.car.speed;
    this.input.carYawRateDegS = this.car.telemetry.yawRateDegS;
    this.input.driving = this.driving;
    this.input.carWheelbaseM = SDM26.wheelbaseM;
    this.input.carPeakSlipDeg = TIRE_INFO.peakSlipAngleDeg;
    this.car.steeringServo = this.input.steeringServo();

    // The pedal map is applied here, once, before anything else looks at
    // "throttle": everything downstream -- traction control, the engine, the
    // audio -- is dealing with plate position, not pedal position.
    this.pedal = inp.throttle;
    this.plate = this.etc.evaluate(inp.throttle);

    // The map editor is modal, and it needs the live pedal above to keep
    // feeding its marker, so it returns after the read and before the rest.
    if (this.etcEditor.isOpen) { this.holdNative(); return; }

    if (this.input.edges.mapEditor) { this.openEtcEditor(); return; }
    if (this.input.edges.home) { this.goHome(); return; }
    if (this.input.edges.hudDensity) {
      const d = this.hud.cycleDensity();
      this.timing?.say(`OVERLAY  ${d.toUpperCase()}`, 1.5);
    }
    if (this.input.edges.dashMode) {
      const m = this.hud.cycleDashMode();
      this.dom.dashMode && (this.dom.dashMode.value = m);
      this.timing?.say(`DASH  ${m.toUpperCase()}`, 1.5);
    }
    if (this.input.edges.pause && !this.swallowPauseEdge) this.setPaused(!this.paused);
    this.swallowPauseEdge = false;
    // The pause overlay offers a restart, so it has to work from there.
    if (this.input.edges.restart) { this.restart(); this.setPaused(false); }
    if (this.paused) { this.holdNative(); return; }

    this.clock += dt;

    // ---- d-pad setup changes: left/right pick, up/down move ----
    const e = this.input.edges;
    if (e.setupPrev) this.announceSetup(this.setup.select(-1));
    if (e.setupNext) this.announceSetup(this.setup.select(1));
    if (e.setupUp || e.setupDown) {
      const item = this.setup.nudge(e.setupUp ? 1 : -1, this.input.setupHoldScale, this.clock);
      this.announceSetup(item);
      this.pushParams();
      // A setup change mid-run is part of the run. Without this the manifest
      // carries the values the run STARTED with and quietly disagrees with the
      // telemetry from the moment the driver touches the d-pad -- which is
      // exactly when somebody will be trying to work out what changed.
      this.recorder?.event("setup", {
        item: item.short, value: round3(item.get()), unit: item.unit,
      });
    }

    if (this.input.edges.camera) {
      this.cameraIndex = (this.cameraIndex + 1) % CAMERAS.length;
      this.timing.say(CAMERAS[this.cameraIndex].name.toUpperCase(), 1.2);
    }
    if (this.input.edges.traction) {
      this.assists.traction = !this.assists.traction;
      this.dom.tcToggle.checked = this.assists.traction;
      updateSession();
      this.timing.say(`TRACTION CONTROL ${this.assists.traction ? "ON" : "OFF"}`, 1.5);
    }
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
    if (this.car.native && this.car.moneyShiftBlocked) this.timing.say("MONEY SHIFT BLOCKED", 1.2);

    // ---- driver aids ----
    // Natively the rig applies them every millisecond; here only in the
    // browser build, once a frame.
    let throttle = this.plate;
    let brake = inp.brake;
    if (this.car.native) {
      this.car.frame.traction = this.assists.traction;
      this.car.frame.abs = this.assists.abs;
      this.car.frame.autoShift = this.assists.autoShift;
      this.car.frame.ffbEnabled = this.input.profile.forceFeedback?.enabled !== false;
      this.car.frame.offTrack = !!this.loc && !this.loc.onTrack && this.car.speed > 2;
      this.car.frame.rimDeg = this.input.rim.deg;
      this.car.frame.halfLockDeg = this.input.rim.halfLockDeg;
      this.car.frame.launch = !!inp.launch;
    } else {
      // Browser build: the JS powertrain holds the launch state directly.
      this.powertrain.setLaunch(!!inp.launch);
    }
    if (!this.car.native && this.assists.traction) {
      const over = tel.kappaR - 0.13;
      if (over > 0) throttle = Math.max(0.1, throttle * (1 - Math.min(0.9, over * 6)));
    }
    if (!this.car.native && this.assists.abs) {
      // Release when a wheel is deep into lockup (large negative slip ratio).
      const lockF = -tel.kappaF - 0.16;
      const lockR = -tel.kappaR - 0.16;
      const worst = Math.max(lockF, lockR);
      if (worst > 0) brake = Math.max(0.15, brake * (1 - Math.min(0.85, worst * 5)));
    }

    this.brakeApplied = brake;

    // ---- physics ----
    this.car.step(dt, { steer: inp.steer, throttle, brake });
    if (this.car.native) {
      // What the rig actually applied, after its own aids and pedals.
      throttle = this.car.applied.throttle;
      brake = this.car.applied.brake;
      this.plate = throttle;
      this.brakeApplied = brake;
    }

    // A venue is bounded by a barrier rather than scored on cones: hold the
    // car inside the foot of the banking before anything reads its position.
    // The rig applies the same barrier every millisecond itself.
    if (this.track.constrain && !this.car.native) this.track.constrain(this.car);

    // ---- course state ----
    const loc = this.track.locate(this.car.X, this.car.Y, this.car.psi);
    const hits = this.track.strikeCones(this.pose(), bodyBoxFor(SDM26));
    const wasStaged = this.timing.state === "staged";
    const wasRunning = this.timing.state === "running";
    this.timing.update(dt, loc, this.car.speed, hits);
    // Autocross ends at the finish line. The card is arranged in
    // `onRunFinished`; the run itself is banked at the bottom of this frame,
    // after the log has taken the finishing step. A closed course never
    // reaches this.
    const justFinished = wasRunning && this.timing.state === "finished";
    if (justFinished) this.onRunFinished();
    if (this.finishAt != null && this.clock >= this.finishAt) this.showFinishMenu();
    // The green flag IS the line crossing, and it is the only one the recorder
    // cannot infer for itself.
    if (wasStaged && this.timing.state === "running") this.recorder?.markLine();
    this.loc = loc;
    // The delta wants distance round the course and time into THIS lap, and
    // only once the lap is actually running -- staged on the line, every
    // sample would land in the first bin and read as an enormous loss.
    if (this.timing.state === "running") {
      this.deltaTimer?.update(loc.s, this.timing.lapTime);
    }

    if (hits > 0) {
      this.audio.coneHit();
      this.input.rumble(0.7, 0.4, 130);
      if (this.car.native) this.car.frame.coneHits += hits;
    }

    // ---- force feedback ----
    // Natively the rig mixes and drives the wheel itself; its last mix is
    // shown in the settings panel. In a browser the mix is computed here so
    // a pad user can still see what a wheel would feel.
    if (this.car.native) {
      this.ffb.last = this.car.ffb;
    } else {
      this.ffb.update(dt, this.input.profile.forceFeedback, tel, this.input.rim, {
        spin: Math.max(0, tel.kappaR - 0.2) * 2.5,
        lock: Math.max(0, -Math.min(tel.kappaF, tel.kappaR) - 0.2) * 2.5,
        offTrack: !loc.onTrack && this.car.speed > 2,
        coneHit: hits,
      });
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
    // ---- the driver's head, which is not bolted to the chassis ----
    // Slower than the chassis attitude above: a body on a six-point belt is
    // not a spring-mounted mass, it arrives late and settles late. Positive
    // ay is a LEFT turn and the body is thrown to the driver's RIGHT, which
    // is +z in the chassis frame; braking is negative ax and throws the body
    // forward, which is +x.
    const hk = Math.min(1, dt * 6);
    this.headLat += (tel.ayG * SDM26.headLatMPerG - this.headLat) * hk;
    this.headLong += (-tel.axG * SDM26.headLongMPerG - this.headLong) * hk;
    this.headYaw += (((tel.ayG * SDM26.headYawDegPerG) * Math.PI) / 180 - this.headYaw) * hk;
    this.bumpPhase += dt * (2 + this.car.speed * 0.55);

    // ---- g-g trail ----
    this.ggTrail.push({ ax: tel.axG, ay: tel.ayG });
    if (this.ggTrail.length > 110) this.ggTrail.shift();

    // ---- the log ----
    // Last, so every channel is the settled value for this step rather than
    // whatever it held halfway through it.
    if (this.recorder) {
      this.recorder.tick(dt, this.recorderContext(dt, inp, throttle, brake, loc, hits), { last: justFinished });
      // An autocross run ends at the finish line. Bank it HERE, after the
      // tick, and not in `onRunFinished` where it used to be: `endRun` nulls
      // the recorder, so ending the run up there meant the finishing step was
      // never logged -- the beacon `recordLap` had just raised for the line
      // never reached a row, and every autocross file had a lap with a start
      // and no end. The driver has no reason to press anything at the flag,
      // and the run that made the time is the one worth keeping, so it still
      // goes to disk on the frame it finished.
      if (this.timing.state === "finished") {
        const rec = this.recorder;
        this.notSavedNote = rec.worthSaving ? null : rec.notSavedReason;
        this.finishSave = this.endRun("finished");
      }
    }
  }

  /**
   * Everything the recorder's columns read, in one object that is allocated
   * once and rewritten in place. At 100 Hz a fresh object per sample is the
   * only garbage the driving loop would produce.
   */
  recorderContext(dt, inp, throttle, brake, loc, hits) {
    const c = this._ctx;
    const tel = this.car.telemetry;
    const pt = this.powertrain;
    const tm = this.timing;
    c.t = tel;
    c.car = this.car;
    c.speed = this.car.speed;
    c.rpm = pt.engineRpm;
    // Gear is 0-based internally and 1-based to a driver, which is also what
    // the car's real logger writes.
    c.gear = pt.gear + 1;
    c.gearRatio = typeof pt.ratio === "function" ? pt.ratio() : 0;
    c.pedal = this.pedal;
    c.plate = throttle;
    c.brake = brake;
    c.brakeBiasFront = SDM26.brakeBiasFront;
    c.roadWheelDeg = tel.steerDeg ?? (this.car.delta * 180) / Math.PI;
    c.steerInput = inp.steer;
    // What steered, observed rather than declared; the recorder totals it and
    // the leaderboards are separated by it. See `Input.steerSource`.
    c.steerSource = this.input.steerSource;
    c.ffbCommand = this.ffb.last?.command ?? 0;
    c.ffbClipped = !!this.ffb.last?.clipped;
    // The JS model runs two rear wheels through the differential; the native
    // rig reports the axle mean.
    c.wRL = this.car.wRL ?? this.car.wR;
    c.wRR = this.car.wRR ?? this.car.wR;
    c.spinFront = this.spinFront;
    c.spinRear = this.spinRear;
    c.s = loc.s ?? 0;
    c.lateral = loc.lateral ?? 0;
    c.headingErrorDeg = ((loc.headingErrorRad ?? 0) * 180) / Math.PI;
    c.curvature = loc.curvature ?? 0;
    c.onTrack = !!loc.onTrack;
    c.lap = tm.lap;
    c.lapTime = tm.lapTime;
    c.sector = tm.sectorIndex + 1;
    c.cones = tm.cones;
    c.offCourse = tm.offCourse;
    c.penaltyS = tm.penaltyS;
    // The delta the HUD is showing this instant, not a reconstruction.
    const dt2 = this.deltaTimer;
    c.deltaValid = !!(dt2 && dt2.hasReference && dt2.deltaValid);
    c.deltaS = c.deltaValid ? dt2.delta : 0;
    c.assists = this.assists;
    // An excursion is one event, logged where it started, not one per frame.
    if (!c.onTrack && c.speed > 2 && !this._wasOff) {
      this._wasOff = true;
      this.recorder.event("off-course", {
        lap: tm.lap, x: round3(this.car.X), y: round3(this.car.Y), s: round3(c.s),
        lateral: round3(c.lateral),
      });
    } else if (c.onTrack && this._wasOff) {
      this._wasOff = false;
    }
    if (c.gear !== this._lastGear) {
      if (this._lastGear != null) {
        this.recorder.event("shift", { from: this._lastGear, to: c.gear, rpm: Math.round(c.rpm), s: round3(c.s) });
      }
      this._lastGear = c.gear;
    }
    c.launch = !!inp.launch;
    c.clutchSlipRpm = pt.clutchSlipRpm ?? 0;
    c.shifting = (pt.shiftTimer ?? 0) > 0;
    if (hits > 0) {
      this.recorder.event("cone", {
        lap: tm.lap, n: hits, x: round3(this.car.X), y: round3(this.car.Y), s: round3(c.s),
      });
    }
    return c;
  }

  render() {
    // Behind the launch screen the walkaround orbits the car, whatever view
    // the driver last had; the driving camera comes back with the run.
    const cam = this.dom.menu.hidden ? CAMERAS[this.cameraIndex] : WALKAROUND;
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

    // The walkaround camera turns on its own, and stops while the car is
    // moving -- orbiting a car that is driving away is nauseating.
    if (cam.orbit) {
      const dt = this.lastDt ?? 1 / 60;
      const w = this.input.walkaround;
      if (w && (w.turn || w.rise || w.zoom)) {
        this.orbitAuto = false;
        this.orbitAngle += w.turn * 1.4 * dt;
        this.orbitHeight = Math.min(3.5, Math.max(0.06, this.orbitHeight + w.rise * 1.1 * dt));
        this.orbitRadius = Math.min(14, Math.max(1.2, this.orbitRadius * Math.exp(-w.zoom * 1.2 * dt)));
      } else if (this.orbitAuto && this.car.speed < 0.5) {
        this.orbitAngle += 0.22 * dt;
      }
    }

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
        // Head motion rides on the cockpit and nose views, which are bolted to
        // the chassis. The chase and walkaround cameras are not in the car.
        ahead: (cam.live ? SDM26.eyeAheadOfCgM : cam.ahead) + (cam.rigid ? this.headLong : 0),
        height: cam.orbit ? (this.orbitHeight ?? cam.height)
              : cam.live ? SDM26.eyeHeightM : cam.height,
        lateral: cam.rigid ? this.headLat : 0,
        yawOffset: cam.rigid ? this.headYaw : 0,
        pitchOffset: cam.pitch,
        rigid: cam.rigid,
        orbit: cam.orbit,
        // Live, so the walkaround can be moved while looking at the car --
        // which is the entire point of having it.
        radius: this.orbitRadius ?? cam.radius,
        focusHeight: this.orbitFocus ?? cam.focusHeight,
        orbitAngle: this.orbitAngle,
      },
      hubs: hubsFor(SDM26),
      wheels: {
        steerRad: this.car.delta,
        steerRatio: SDM26.steeringRatio,
        spinFront: this.spinFront,
        spinRear: this.spinRear,
        rimFade,
      },
      ghost: this.ghost ? this.ghostPose() : null,
      heaveM: bump - Math.abs(tel.axG) * (SDM26.heaveMmG / 1000) * 0.5 * vib,
      // A little FOV with speed helps the sense of motion; a lot of it is a
      // game trope that undoes the honest framing above, so this is 4 deg at
      // 25 m/s rather than the 10 it used to reach.
      fovBoost: cam.orbit ? 0 : Math.min(4, this.car.speed * 0.16),
    });

    // No HUD over the launch screen: the scene is the backdrop there. Nor
    // over a replay -- the replay overlay is a better instrument panel than
    // the driving HUD, and two sets of lap times on one screen is one too many.
    if (!this.dom.menu.hidden || this.replay) { this.hud.clear(); return; }

    const t = this.timing;
    const last = t.laps.length ? t.laps[t.laps.length - 1] : null;
    // Which dash the driver is actually looking at.
    //
    // From the cockpit the real one is on the scuttle in front of them, and a
    // second copy pasted over the screen is exactly the clutter this module
    // has been trying to get rid of. From every other camera the real one is a
    // postage stamp or behind them, so the overlay is the only dash there is.
    const inCockpit = CAMERAS[this.cameraIndex]?.name === "Cockpit";
    this.hud.overlayDash = !inCockpit;

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
      // After the flag the big clock shows the run that just finished, not 0.
      lapTimeText: fmt(t.state === "staged" ? 0 : t.state === "finished" && last ? last.total : t.lapTime),
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
      delta: this.deltaTimer?.state() ?? null,
      message: t.message,
      paused: this.paused,
      tractionControl: this.assists.traction,
    });

    // And the same layout onto the car's own panel.
    this.paintDash(this.hud.lastState);
  }

  /**
   * Repaint the car's dash panel, at the panel's own rate.
   *
   * Throttled rather than drawn every frame: it is a 768x394 canvas plus a
   * texture upload, nobody can read a dash changing faster than this, and at
   * 144 fps the difference is most of a millisecond a frame.
   */
  paintDash(state) {
    if (!state) return;
    const now = performance.now();
    if (now - (this._dashPainted ?? -1e9) < DASH_REFRESH_MS) return;
    this._dashPainted = now;
    // `chrome: false`: on the car the case, its bezel and its buttons are
    // geometry, so the texture is the DISPLAY only.
    this.renderer.updateDashPanel(
      (ctx, x, y, w, h) => this.hud.drawDash(ctx, x, y, w, h, state, { chrome: false }),
    );
  }

  /**
   * The dash as it read at this instant of a recorded run.
   *
   * A replay clears the HUD -- the replay panel is the interface -- so without
   * this the car's own dash sits frozen on whatever the last live session left
   * on it, which is worse than blank: it is a plausible set of numbers
   * belonging to a different drive. Everything here is read out of the log,
   * including the delta the driver was actually looking at.
   */
  replayDashState() {
    const r = this.replay;
    if (!r) return null;
    const lap = r.lapAt();
    const deltaValid = r.value("sim.delta_valid") > 0.5;
    return {
      trackName: this.track?.name ?? "",
      rpm: r.value("engine.rpm"),
      revLimit: SDM26.revLimitRpm,
      shiftRpm: this.powertrain?.optimalUpshiftRpm?.() ?? SDM26.revLimitRpm * 0.9,
      peakTorqueRpm: this.powertrain?.peakTorque?.rpm ?? 0,
      peakPowerRpm: this.powertrain?.peakPower?.rpm ?? 0,
      gear: Math.round(r.valueAt("engine.gear")),
      shifting: r.valueAt("sim.shifting") > 0.5,
      speedKph: r.value("drivetrain.vehicle_speed"),
      brake: r.value("brake.driver_load") / 100,
      ayG: r.value("imu.lat_g"),
      lap: Math.round(r.valueAt("sim.lap")),
      lapTimeText: fmt(r.value("sim.lap_time_s")),
      lastLapText: lap ? fmt(lap.total) : "--.---",
      bestLapText: r.bestLap ? fmt(r.bestLap.total) : "--.---",
      cones: Math.round(r.valueAt("sim.cones_lap")),
      offCourse: Math.round(r.valueAt("sim.off_course_lap")),
      penaltyS: r.value("sim.penalty_s"),
      tractionControl: r.valueAt("sim.traction_control") > 0.5,
      // The live delta, exactly as logged -- see `sim.delta_valid`.
      delta: { hasReference: deltaValid, delta: deltaValid ? r.value("sim.delta_s") : null },
    };
  }

  announceSetup(item) {
    this.timing.say(`${item.short} ${item.get().toFixed(1)}${item.unit}`, 1.6);
  }

  openEtcEditor() {
    this.setPaused(true);
    this.dom.pauseMenu.hidden = true;
    this.etcEditor.open();
  }

  /** Back to the home screen, with the run left paused behind it. */
  goHome() {
    // The run is still resumable, but it is also finished as far as the log
    // is concerned: a driver who walks away must still find their telemetry.
    // Resuming opens a fresh recording rather than reopening this one.
    this.endRun("left-the-run");
    this.hideFinishMenu();
    this.setPaused(true);
    this.dom.pauseMenu.hidden = true;
    this.dom.menu.hidden = false;
    this.audio.setEnabled(false);
    // The run is still there -- offer to go back to it rather than bin it.
    this.dom.startBtn.textContent = "Resume run";
    this.dom.restartBtn.hidden = false;
  }

  /**
   * Watch a recorded run.
   *
   * The course is loaded first, because a replay is only meaningful against
   * the geometry it was set on -- and because the renderer needs the track it
   * is about to draw the car around. Any live run is banked before the switch.
   */
  async enterReplay(runId, ghostId = null) {
    this.endRun("replay-opened");
    this.hideFinishMenu();
    const { manifest, telemetry } = await loadRun(runId);
    if (manifest.track && manifest.track !== this.trackId) {
      this.dom.trackSel.value = manifest.track;
      await this.load(manifest.track);
      drawCoursePlan(this.dom.coursePlan, this.track);
    }
    const replay = new Replay(manifest, parseTelemetry(telemetry));
    if (!replay.rows) throw new Error("that run has no telemetry in it");

    this.exitReplay({ keepMenu: true });
    this.replay = replay;
    // Nothing is being driven, so nothing should be recorded, the rig must
    // not be holding the car against a model that is no longer stepping, and
    // the cones should show the state the run left them in rather than the
    // last live drive's.
    this.recorder = null;
    this.track.resetCones();
    this.holdNative();

    this.dom.menu.hidden = true;
    this.dom.pauseMenu.hidden = true;
    this.dom.replayOverlay.hidden = false;
    this.cameraIndex = 2; // chase: a replay is watched, not driven
    this.audio.setEnabled(false);
    this.syncPointer();

    this.replayPanel = new ReplayPanel(this.dom.replayOverlay, replay, {
      onExit: () => { this.exitReplay(); },
      onCamera: () => {
        this.cameraIndex = (this.cameraIndex + 1) % CAMERAS.length;
      },
    });
    if (ghostId) await this.loadGhost(ghostId);
    this.applyReplayFrame();
    return replay;
  }

  /**
   * Load the delta's reference lap from a recorded run.
   *
   * Takes the run's best lap, because that is the one worth chasing, and
   * refuses a run set on a different course -- a time-at-distance table means
   * nothing anywhere but the course it was driven on.
   */
  async loadReference(runId) {
    try {
      const { manifest, telemetry } = await loadRun(runId);
      if (manifest.track && this.trackId && manifest.track !== this.trackId) {
        throw new Error(`that run is on ${manifest.trackName ?? manifest.track}, not this course`);
      }
      const best = referenceLapOf(manifest.laps);
      if (!best) throw new Error("that run has no valid lap to chase");
      const tel = parseTelemetry(telemetry);
      const table = referenceFromRun(tel, best, this.track.length);
      if (!table) throw new Error("that lap does not cover the course");
      const label = manifest.driver ? `${manifest.driver}` : "reference";
      this.deltaTimer?.loadReference(table, best.raw, label);
      this.recorder?.setReference(this.deltaTimer?.describeReference());
      this.timing?.say(`REFERENCE  ${label} ${fmt(best.raw)}`, 3);
      return true;
    } catch (err) {
      console.error("could not load the reference lap", err);
      this.timing?.say("REFERENCE LAP NOT LOADED", 3);
      return false;
    }
  }

  /** Put a second recorded run in the scene beside the one being watched. */
  async loadGhost(runId) {
    try {
      const { manifest, telemetry } = await loadRun(runId);
      // A ghost from another course is not a ghost, it is a car driving through
      // the scenery: the pose channels are in that course's own frame. Helios's
      // picker only offers same-course runs, but `--ghost` on the command line
      // and the browser dev loop do not go through it.
      if (manifest.track && this.trackId && manifest.track !== this.trackId) {
        throw new Error(`that run is on ${manifest.trackName ?? manifest.track}, not this course`);
      }
      const g = new Replay(manifest, parseTelemetry(telemetry));
      if (!g.rows) throw new Error("the ghost run has no telemetry");
      this.ghost = g;
      this.replayPanel?.setGhost(g);
    } catch (err) {
      console.error("could not load the ghost", err);
      this.ghost = null;
      this.replayPanel?.setGhost(null);
      this.timing?.say("GHOST NOT LOADED", 2.5);
    }
  }

  /**
   * Close the replay.
   *
   * Where "close" goes depends on how the window got here. Opened from the
   * sim's own Runs tab, it goes back to the launch screen. But when HELIOS
   * launched it with `--replay`, this window exists only to watch that one run
   * -- the user came from the Runs table in another app, and dropping them on
   * a launch screen they never asked for means closing a second window to get
   * back to where they were. So that case closes the window, which puts Helios
   * back in front.
   */
  exitReplay({ keepMenu = false } = {}) {
    if (!this.replay && !this.replayPanel) return;
    if (!keepMenu && this.launchedForReplay) {
      this.closeWindow();
      return;
    }
    this.replay = null;
    this.ghost = null;
    this.replayPanel?.destroy();
    this.replayPanel = null;
    this.dom.replayOverlay.hidden = true;
    if (!keepMenu) {
      this.dom.menu.hidden = false;
      this.started = false;
      this.dom.startBtn.textContent = "Start engine";
      this.dom.restartBtn.hidden = true;
      this.hud.clear();
      refreshRuns();
    }
  }

  /**
   * Ask the shell to close. Falls back to leaving the replay on screen in a
   * browser, where there is no window to close and `window.close()` on a page
   * the script did not open is a no-op.
   */
  closeWindow() {
    if (!isDesktop) {
      this.exitReplay({ keepMenu: true });
      this.dom.menu.hidden = false;
      this.started = false;
      return;
    }
    closeAppWindow();
  }

  /** True while a recorded run is on screen. */
  get replaying() { return !!this.replay; }

  /**
   * One replay frame: advance the playback clock, then write the sampled
   * state into exactly the fields the renderer and the HUD already read. That
   * is the whole trick -- nothing downstream knows it is watching a recording.
   */
  replayFrame(dt) {
    const r = this.replay;
    if (!r) return;
    r.advance(dt);
    this.applyReplayFrame();
    this.replayPanel?.paint(dt);
  }

  applyReplayFrame() {
    const r = this.replay;
    if (!r) return;
    const s = r.readSample();
    const car = this.car;
    car.X = s.x;
    car.Y = s.y;
    car.psi = s.yawRad;
    car.delta = s.steerRad;
    car.u = r.value("sim.vel_long");
    car.v = r.value("sim.vel_lat");
    car.wF = (r.value("drivetrain.wheel_speed_fl") * Math.PI * 2) / 60;
    // The JS model's `wR` is a getter over the two rear wheel states (the
    // differential lives between them), so it has to be written through those;
    // the native car has a plain `wR`.
    const wRL = (r.value("drivetrain.wheel_speed_rl") * Math.PI * 2) / 60;
    const wRR = (r.value("drivetrain.wheel_speed_rr") * Math.PI * 2) / 60;
    if ("wRL" in car) { car.wRL = wRL; car.wRR = wRR; } else { car.wR = (wRL + wRR) / 2; }
    this.spinFront = s.spinFront;
    this.spinRear = s.spinRear;
    this.camRoll = s.rollRad;
    this.camPitch = s.pitchRad;
    // The head is a filter on lateral and longitudinal g, and the log has
    // both: recompute rather than record it, so a change to how the driver's
    // head moves shows up in old runs too.
    const ay = r.value("imu.lat_g");
    const ax = r.value("imu.long_g");
    this.headLat = ay * SDM26.headLatMPerG;
    this.headLong = -ax * SDM26.headLongMPerG;
    this.headYaw = ((ay * SDM26.headYawDegPerG) * Math.PI) / 180;
    this.bumpPhase += (this.lastDt ?? 1 / 60) * (2 + Math.abs(car.u) * 0.55);
    // The car's own dash, from the log rather than from a live session that
    // ended twenty minutes ago.
    this.paintDash(this.replayDashState());

    // The telemetry the HUD reads. Only what it draws is filled in; the rest
    // of the object keeps whatever the last live run left, which nothing on
    // screen looks at during a replay.
    const tel = car.telemetry;
    tel.axG = ax;
    tel.ayG = ay;
    tel.balance = r.value("sim.balance");
    tel.yawRateDegS = r.value("imu.yaw_rate");
    tel.bodySlipDeg = r.value("sim.body_slip_deg");
    tel.utilF = r.value("sim.util_front");
    tel.utilR = r.value("sim.util_rear");

    this.pedal = r.value("engine.aps") / 100;
    this.plate = r.value("engine.tps") / 100;
    this.brakeApplied = r.value("brake.driver_load") / 100;

    // The g-g trail is the last couple of seconds of the run being watched,
    // rebuilt on a seek so scrubbing backwards does not leave a stale smear.
    this.ggTrail.length = 0;
    for (let k = 24; k >= 0; k--) {
      const tt = r.t - k * 0.08;
      if (tt < 0) continue;
      this.ggTrail.push({ ax: r.value("imu.long_g", tt), ay: r.value("imu.lat_g", tt) });
    }
  }

  /**
   * The ghost's pose for the renderer.
   *
   * Placed by TIME into the lap, not by distance. Distance is the right axis
   * for the gap NUMBER -- "how long had each car taken to get here" is the
   * only comparison that survives two different lines -- but it is the wrong
   * one for the car you can see: it pins the ghost alongside you all lap and
   * the only thing a driver learns is that both cars went the same way round.
   * On the clock the ghost pulls away where it was quicker and falls back
   * where it was not, which is the whole reason to draw it.
   */
  ghostPose() {
    const g = this.ghost;
    const r = this.replay;
    if (!g || !r) return null;
    const mine = r.lapAt();
    const theirs = g.bestLap ?? g.laps[0];
    // `lapAt` is null in two places, and the absolute replay clock is wrong in
    // both. During STAGING -- however long the driver sat waiting for green --
    // it put the ghost that many seconds into its own lap, so it drove off
    // while the watched car stood still and then snapped back on the flag. In
    // the one-frame gap BETWEEN laps it jumped to the whole elapsed time of
    // the run, which runs past the ghost's duration and made the ghost vanish
    // for a frame at every lap boundary.
    //
    // Before the first lap the honest answer is "the ghost has not started
    // either"; between laps it is "carry on from where the last lap left off".
    let into;
    if (mine) into = r.t - (mine.startedAtS ?? 0);
    else if (r.laps.length && r.t < (r.laps[0].startedAtS ?? 0)) into = 0;
    else into = this._lastGhostInto ?? 0;
    this._lastGhostInto = into;
    const t = theirs ? (theirs.startedAtS ?? 0) + into : into;
    if (t > g.duration + 0.5) return null;   // the ghost's run has ended
    g.seek(Math.min(t, g.duration));
    const gs = g.readSample();
    // Two cars in the same place at the same instant is not a comparison, it
    // is z-fighting. Below a car's length apart the ghost has nothing to say,
    // so it is not drawn -- which is also exactly the moment the gap readout
    // beside it is telling the driver they are level.
    if (Math.hypot(gs.x - this.car.X, gs.y - this.car.Y) < 2.2) return null;
    return {
      x: gs.x, y: gs.y, psi: gs.yawRad,
      rollRad: gs.rollRad, pitchRad: gs.pitchRad,
      steerRad: gs.steerRad, spinFront: gs.spinFront, spinRear: gs.spinRear,
      color: [0.24, 0.62, 0.95], tint: 0.8,
    };
  }

  /**
   * The car has crossed the autocross finish line.
   *
   * Two things happen, in this order and a couple of seconds apart:
   *
   *   1. The run is banked on THIS frame, at the bottom of `update` once the
   *      log has taken the finishing step. The timed run ended at the line,
   *      so the log ends at the line too -- whatever the car does rolling to
   *      a stop is not part of it, and a driver who then quits out must
   *      still find their telemetry.
   *   2. The card comes up after a short roll-out. Freezing the car the
   *      instant the nose crosses is how you lose the only moment the driver
   *      wanted -- seeing FINISH and their time while still braking. Two
   *      seconds is enough to read it and not enough to get bored.
   */
  onRunFinished() {
    this.finishRun = this.timing.laps[this.timing.laps.length - 1] ?? null;
    // Captured by the lap hook a moment ago; see `installLapHooks`.
    this.finishSectors = this.lapSectors ?? [];
    this.finishBestBefore = this.prevBestSectors ?? [];
    // The save itself -- `finishSave`, `notSavedNote` -- is set at the end of
    // `update`, once the log has its finishing row. Nothing reads either
    // before the card comes up, two seconds from now.
    this.finishSave = null;
    this.notSavedNote = null;
    this.finishAt = this.clock + FINISH_ROLLOUT_S;
  }

  /** The end-of-run card: the score, and every way out of the run. */
  showFinishMenu() {
    this.finishAt = null;
    if (this.finished) return;
    this.finished = true;
    // Which card this is. The save below lands asynchronously, and `finished`
    // alone is true of ANY card -- so a slow write (the runs directory can be
    // a network share) could land while the NEXT run's card was up and point
    // Watch at the previous run.
    const token = ++this._finishToken;
    const d = this.dom;
    const entry = this.finishRun;
    d.finishHead.textContent =
      `${this.track?.name ?? ""}${this.driverName ? `  --  ${this.driverName}` : ""}`;

    // The score, laid out the way the scoring sheet is: what you drove, what
    // it cost, what it counts as.
    const rows = [];
    const row = (k, v, cls = "") => rows.push(
      `<span class="k${cls ? " " + cls : ""}">${k}</span><span class="v">${v}</span>`,
    );
    if (entry) {
      row("Raw time", fmt(entry.raw));
      if (entry.cones > 0) {
        rows.push('<span class="k">Cones</span>');
        rows.push(`<span class="v pen">${entry.cones} x 2.000 = +${(entry.cones * CONE_PENALTY_S).toFixed(3)}</span>`);
      }
      if (entry.off > 0) {
        rows.push('<span class="k">Off course</span>');
        rows.push(`<span class="v pen">${entry.off}</span>`);
      }
      rows.push('<span class="rule"></span>');
      if (entry.valid === false) {
        // No time, and say why rather than showing a number that does not
        // count. The figure the rulebook would have given is worth knowing.
        rows.push('<span class="k total">Scored</span>');
        rows.push('<span class="v total pen">NO TIME - OFF COURSE</span>');
        const fsae = entry.raw + entry.cones * CONE_PENALTY_S
          + entry.off * FSAE_OFF_COURSE_PENALTY_S;
        rows.push('<span class="k">Under FSAE +20s</span>');
        rows.push(`<span class="v">${fmt(fsae)}</span>`);
      } else {
        rows.push('<span class="k total">Scored</span>');
        rows.push(`<span class="v total">${fmt(entry.total)}</span>`);
      }

      // The sectors this run was scored on, against the best each has ever
      // been driven. For a team working out where a lap went, this is the
      // most useful thing on the screen -- and it was being computed and then
      // thrown away the moment the card appeared.
      const splits = this.finishSectors ?? [];
      if (splits.length > 1) {
        rows.push('<span class="rule"></span>');
        for (let i = 0; i < splits.length; i++) {
          const v = splits[i];
          if (v == null) {
            rows.push(`<span class="k">S${i + 1}</span><span class="v">--.---</span>`);
            continue;
          }
          // Against the best as it stood BEFORE this lap: a sector that just
          // set the best is its own reference and would always read +0.000.
          // And "best" only on a lap that counted -- see `sectorVerdict`.
          const verdict = sectorVerdict(v, this.finishBestBefore[i], entry.valid !== false);
          let tag;
          if (verdict.best) tag = '<span class="v" style="color:var(--gold)">best</span>';
          else if (verdict.delta == null) tag = '<span class="v"></span>';
          else if (verdict.delta > 0) tag = `<span class="v pen">+${verdict.delta.toFixed(3)}</span>`;
          else tag = `<span class="v">${verdict.delta.toFixed(3)}</span>`;
          rows.push(`<span class="k">S${i + 1}  ${fmt(v)}</span>${tag}`);
        }
      }
      const best = this.timing.best;
      if (best && best !== entry) {
        row("Best this session", fmt(best.total));
      }
    } else {
      row("Result", "not scored");
    }
    d.finishStats.innerHTML = rows.join("");

    // The replay is only offered once the run is actually on disk, and only
    // when there was something worth saving -- a lap that was never written
    // cannot be watched, and a button that errors is worse than no button.
    d.finishWatch.hidden = true;
    const save = this.finishSave;
    if (save) {
      save.then((res) => {
        if (res?.runId && this._finishToken === token) {
          this.finishRunId = res.runId;
          d.finishWatch.hidden = false;
        }
      }).catch(() => { /* the save already reported itself */ });
    } else if (entry) {
      // There was a lap but nothing was written. Say so on the card rather
      // than leaving a button quietly missing.
      rows.push('<span class="k">Replay</span>');
      rows.push(`<span class="v">${this.notSavedNote ?? "not saved"}</span>`);
      d.finishStats.innerHTML = rows.join("");
    }

    d.finishMenu.hidden = false;
    this.setPaused(true);
    // So Enter and Space do the obvious thing, and Tab walks the card.
    d.finishAgain.focus();
  }

  hideFinishMenu() {
    this.finishAt = null;
    if (!this.finished) return;
    this.finished = false;
    this.finishRunId = null;
    this.dom.finishMenu.hidden = true;
  }

  /**
   * Hand the rig's latest reading of the wheel to the input layer.
   *
   * This used to live inside `update`, which only runs while somebody is
   * DRIVING -- and the settings panel is on the home screen, where it does
   * not. On a desktop rig the wheel is acquired natively, so the Gamepad API
   * cannot see it at all and `Input.pad()` has nothing but this to go on.
   * The result was a bindings panel that could not see the wheel: the axis
   * monitor read "(no device)", sweeping a pedal detected nothing, and
   * clicking a device cell waited for a button press that could never
   * arrive. Every control on a wheel was unbindable, which is the one place
   * rebinding actually matters -- a wheel's mapping is never standard.
   *
   * Worse when the driver HAD driven and come back: `nativeDevice` then held
   * a frozen snapshot from the last frame of the run, so the monitor showed
   * plausible unchanging numbers rather than an honest "nothing here".
   */
  syncNativeInput() {
    if (!this.car?.native) return;
    this.input.nativeDevice = this.car.device;
    if (this.rigState.wheelName) this.input.nativeName = this.rigState.wheelName;
    // Everything the rig is reading, base first. What the settings panel uses
    // to tell a driver their pedals are on a device of their own.
    this.input.nativeDeviceNames = this.rigState.deviceNames ?? [];
  }

  /** Keep the rig informed while nothing is being driven. */
  holdNative() {
    if (!this.car?.native) return;
    this.car.hold();
    // The panel is live while the game is not, so the device reading has to
    // keep coming even though nothing is being driven.
    this.syncNativeInput();
  }

  /**
   * Write the driver's actual bindings into the cards' key chips.
   *
   * The chips used to be typed into the HTML, which was fine while the keys
   * were fixed and became a lie the moment they were rebindable: a card that
   * says "Esc" to somebody who moved pause to a paddle is worse than a card
   * with no hint at all. `data-key` on a chip names the action; the rest is
   * whatever that action is bound to now.
   */
  syncMenuKeys() {
    const K = this.input?.profile?.keys ?? {};
    for (const el of document.querySelectorAll("[data-key]")) {
      const codes = K[el.dataset.key] ?? [];
      // Only the first: the chip is a reminder, not the binding table.
      el.textContent = codes.length ? keyLabel(codes[0]) : "";
    }
  }

  setPaused(on) {
    this.paused = on;
    if (on) this.syncMenuKeys();
    // Leaving the pause also leaves the finish card: Esc means "back to the
    // car" from either of them, and the run is over either way.
    if (!on) this.hideFinishMenu();
    this.dom.pauseMenu.hidden = !on || this.finished;
    // A paused engine is silent, not frozen at the last operating point.
    this.audio.setRunning(!on);
    this.input.driving = this.driving;
    this.syncPointer();
  }

  /**
   * Mouse steering wants the pointer captured while driving (movementX keeps
   * coming at the window edge, and the cursor is out of the way) and
   * released the moment a menu is up.
   */
  syncPointer() {
    const want = this.driving && !!this.input.profile.mouse?.enabled;
    const locked = document.pointerLockElement === this.dom.gl;
    try {
      if (want && !locked) this.dom.gl.requestPointerLock?.();
      else if (!want && locked) document.exitPointerLock?.();
    } catch { /* not available (some webviews); movementX still works unlocked */ }
    this.dom.gl.style.cursor = want ? "none" : "";
  }

  /** True while the driver is actually in the run (not menu, pause, editor). */
  get driving() {
    if (this.replay) return false;
    return this.started && this.dom.menu.hidden && !this.paused && !this.etcEditor.isOpen;
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
  dashMode: document.getElementById("dashMode"),
  padStatus: document.getElementById("padStatus"),
  pauseMenu: document.getElementById("pauseMenu"),
  loadNote: document.getElementById("loadNote"),
  specs: document.getElementById("specs"),
  etcOverlay: document.getElementById("etcOverlay"),
  replayOverlay: document.getElementById("replayOverlay"),
  etcSummary: document.getElementById("etcSummary"),
  etcBtn: document.getElementById("etcBtn"),
  vehicle: document.getElementById("vehicle"),
  resetParams: document.getElementById("resetParams"),
  paramNote: document.getElementById("paramNote"),
  restartBtn: document.getElementById("restartBtn"),
  finishMenu: document.getElementById("finishMenu"),
  finishHead: document.getElementById("finishHead"),
  finishStats: document.getElementById("finishStats"),
  finishAgain: document.getElementById("finishAgain"),
  finishWatch: document.getElementById("finishWatch"),
  coursePlan: document.getElementById("coursePlan"),
  sCourse: document.getElementById("sCourse"),
  sCar: document.getElementById("sCar"),
  sAids: document.getElementById("sAids"),
  sEtc: document.getElementById("sEtc"),
  sControls: document.getElementById("sControls"),
  carBadge: document.getElementById("carBadge"),
  paramBadge: document.getElementById("paramBadge"),
  driverName: document.getElementById("driverName"),
  sessionName: document.getElementById("sessionName"),
  recordToggle: document.getElementById("recordToggle"),
  runsDir: document.getElementById("runsDir"),
  runsList: document.getElementById("runsList"),
  runsBadge: document.getElementById("runsBadge"),
  sDriver: document.getElementById("sDriver"),
  sRecording: document.getElementById("sRecording"),
};

// ---- who is driving, remembered between launches ------------------------
const DRIVER_KEY = "fsae-sim.driver";
const SESSION_KEY = "fsae-sim.session";
const RECORD_KEY = "fsae-sim.record";

export function saveDriver(name) {
  try { localStorage.setItem(DRIVER_KEY, name); } catch { /* ignore */ }
}
function loadDriver() {
  try { return localStorage.getItem(DRIVER_KEY) ?? ""; } catch { return ""; }
}
function saveSessionLabel(label) {
  try { localStorage.setItem(SESSION_KEY, label); } catch { /* ignore */ }
}
function loadSessionLabel() {
  try { return localStorage.getItem(SESSION_KEY) ?? ""; } catch { return ""; }
}
function saveRecordPref(on) {
  try { localStorage.setItem(RECORD_KEY, on ? "1" : "0"); } catch { /* ignore */ }
}
function loadRecordPref() {
  try { return localStorage.getItem(RECORD_KEY) !== "0"; } catch { return true; }
}

/** Tabs on the launch screen. Every pane stays in the DOM; only one shows. */
function wireTabs() {
  const tabs = [...document.querySelectorAll(".tab")];
  const panes = [...document.querySelectorAll(".pane")];
  const show = (name) => {
    for (const t of tabs) t.setAttribute("aria-selected", t.dataset.pane === name ? "true" : "false");
    for (const p of panes) p.classList.toggle("active", p.dataset.pane === name);
    try { localStorage.setItem("fsae-sim.tab", name); } catch { /* ignore */ }
  };
  for (const t of tabs) t.addEventListener("click", () => show(t.dataset.pane));
  let saved = null;
  try { saved = localStorage.getItem("fsae-sim.tab"); } catch { /* ignore */ }
  if (saved && tabs.some((t) => t.dataset.pane === saved)) show(saved);
}

/**
 * The session card: what the run will be, in one glance, beside Start.
 * Called after anything that changes it -- course load, an aid toggle, a
 * parameter edit, a control-profile change.
 */
function updateSession() {
  if (!game) return;
  const esc = (v) => String(v).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));

  const t = game.track;
  if (t && dom.sCourse) {
    const detail = t.kind === "venue"
      ? `${(t.length / 1000).toFixed(2)} km oval, free roam`
      : `${t.length.toFixed(0)} m, ${t.cones.length} cones${t.closed ? ", lapped" : ", single run"}`;
    dom.sCourse.innerHTML = `<b>${esc(t.name)}</b><small>${esc(detail)}</small>`;
  }

  let changed = 0;
  for (const path of Object.keys(PARAM_DEFAULTS)) {
    if (Math.abs(readParam(path) - PARAM_DEFAULTS[path]) > 1e-12) changed++;
  }
  const pt = game.powertrain;
  const engine = pt ? `${pt.peakPower.powerKW.toFixed(0)} kW at ${pt.peakPower.rpm} rpm` : "";
  dom.sCar.innerHTML = `<b>SDM26</b><small>${esc(engine)}${changed
    ? ` <span class="changed">${changed} parameter${changed === 1 ? "" : "s"} changed</span>` : ""}</small>`;
  if (dom.paramBadge) dom.paramBadge.textContent = changed ? String(changed) : "";

  const aids = [];
  if (dom.tcToggle.checked) aids.push("TC");
  if (dom.absToggle.checked) aids.push("ABS");
  if (dom.autoToggle.checked) aids.push("auto gearbox");
  dom.sAids.innerHTML = aids.length
    ? `<b>${esc(aids.join(", "))}</b>`
    : `<span class="off">none</span>`;
  if (dom.carBadge) dom.carBadge.textContent = aids.length ? String(aids.length) : "";

  if (game.etc) {
    const d = game.etc.describe();
    const name = game.etc.name === "custom" ? "Custom" : game.etc.name[0].toUpperCase() + game.etc.name.slice(1);
    dom.sEtc.innerHTML = `<b>${esc(name)}</b><small>${esc(d.character)}, ${d.plateAtFullPedal}% plate at full pedal</small>`;
  }

  const profile = game.input?.settings?.active?.();
  if (profile) dom.sControls.innerHTML = `<b>${esc(profile.label)}</b>`;

  if (dom.sDriver) {
    const from = game.launchedBy ? `<small class="changed">set by the launcher, this session only</small>` : "";
    dom.sDriver.innerHTML = game.driverName
      ? `<b>${esc(game.driverName)}</b>${game.sessionLabel ? `<small>${esc(game.sessionLabel)}</small>` : ""}${from}`
      : `<span class="off">unnamed</span>`;
  }
  if (dom.sRecording) {
    if (!game.recording) {
      dom.sRecording.innerHTML = `<span class="off">off</span>`;
    } else if (game.saveError) {
      dom.sRecording.innerHTML = `<b class="changed">could not save</b><small>${esc(game.saveError)}</small>`;
    } else if (game.lastSavedRun) {
      const best = game.lastSavedRun.stats?.bestLapS;
      dom.sRecording.innerHTML = `<b>on</b><small>last run saved${best != null ? `, best ${fmt(best)}` : ""}</small>`;
    } else {
      dom.sRecording.innerHTML = `<b>on</b><small>100 Hz, full telemetry</small>`;
    }
  }
}

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
  return `<b>${name}</b>, ${d.points} points. Initial gain ${d.initialGain.toFixed(2)} ` +
         `(${d.character}), ${d.plateAtFullPedal}% plate at full pedal.`;
}

let game;

async function boot() {
  installDesktopBehaviour();
  if (isDesktop) document.body.classList.add("desktop");
  // Before anything can open a recorder, so no run is stamped with the
  // fallback when the shell could have said.
  await resolveSimVersion();
  wireTabs();

  // Restore saved overrides BEFORE the sheet renders, so the sliders come up
  // showing what the car is actually running. PARAM_DEFAULTS was captured at
  // import time, so it still holds the as-shipped values for the reset button.
  const restored = loadParams();
  const onParamChange = (path) => {
    saveParams();
    game?.pushParams();
    if (GEOMETRY_PATHS.includes(path)) game?.renderer.rebuildCar(SDM26);
    updateSession();
  };
  renderSpecSheet(dom.vehicle, onParamChange);
  if (restored) dom.paramNote.textContent = `${restored} parameter${restored === 1 ? "" : "s"} restored from your last session.`;

  dom.resetParams.addEventListener("click", () => {
    for (const path of Object.keys(PARAM_DEFAULTS)) writeParam(path, PARAM_DEFAULTS[path]);
    saveParams();
    renderSpecSheet(dom.vehicle, onParamChange);
    game?.pushParams();
    game?.renderer.rebuildCar(SDM26);
    dom.paramNote.textContent = "All parameters back to as-shipped.";
    updateSession();
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
  let track, curve;
  try {
    ({ track, curve } = await game.load(dom.trackSel.value));
  } catch (err) {
    // A missing data file used to hang here with the button greyed out and
    // nothing said. Say what is missing; the course selector retries.
    console.error(err);
    dom.loadNote.textContent = `Could not load the course or engine data: ${err.message ?? err}`;
    dom.loadNote.classList.add("error");
    return;
  }
  dom.loadNote.classList.remove("error");

  const showCourse = (track, curve) => {
    const pt = game.powertrain;
    const geometry = track.kind === "venue"
      ? `<em>${(track.length / 1000).toFixed(2)} km</em> oval, <em>${track.width.toFixed(1)} m</em> wide, infield and apron driveable`
      : `<em>${track.length.toFixed(0)} m</em>, <em>${track.cones.length}</em> cones, <em>${track.width.toFixed(1)} m</em> wide, ${track.closed ? "lapped" : "single run"}`;
    dom.specs.innerHTML = `
      <div><span>Layout</span><b>${geometry}</b></div>
      <div><span>Engine</span><b><em>${pt.peakTorque.torqueNm.toFixed(1)} N.m</em> at ${pt.peakTorque.rpm} rpm,
        <em>${pt.peakPower.powerKW.toFixed(1)} kW</em> at ${pt.peakPower.rpm} rpm</b></div>
      <div><span>Source</span><b>${curve.name}</b></div>`;
    drawCoursePlan(dom.coursePlan, track);
    updateSession();
  };
  showCourse(track, curve);

  dom.loadNote.textContent = "";
  dom.startBtn.disabled = false;

  let loadSeq = 0;
  const loadCourse = async () => {
    const seq = ++loadSeq;
    dom.startBtn.disabled = true;
    dom.loadNote.textContent = "Loading course...";
    dom.loadNote.classList.remove("error");
    try {
      const loaded = await game.load(dom.trackSel.value);
      if (seq !== loadSeq) return; // a later change won
      showCourse(loaded.track, loaded.curve);
      dom.loadNote.textContent = "";
    } catch (err) {
      if (seq !== loadSeq) return;
      console.error(err);
      dom.loadNote.textContent = `Could not load that course: ${err.message ?? err}`;
      dom.loadNote.classList.add("error");
      return false;
    }
    dom.startBtn.disabled = false;
    return true;
  };
  dom.trackSel.addEventListener("change", loadCourse);

  // Losing the window (alt-tab, minimise, another app grabbing focus) pauses
  // the run: keyboard state is already cleared on blur, but the rig would
  // keep the last throttle and the engine would hold its note.
  const holdOnLeave = () => { if (game.driving) game.setPaused(true); game.holdNative(); };
  window.addEventListener("blur", holdOnLeave);
  document.addEventListener("visibilitychange", () => { if (document.hidden) holdOnLeave(); });
  // Closing the window banks whatever was being driven: a driver who alt-F4s
  // out of a good lap should still find it in Helios. That means holding the
  // close until the write has actually landed, which is what
  // `onWindowClose` is for -- `beforeunload` starts the IPC and returns, and
  // the webview then tears down underneath it.
  onWindowClose(async () => {
    game.endRun("window-closed");
    if (rigNative.available()) await rigNative.stop().catch(() => {});
    await Promise.resolve(game.pendingSave).catch(() => {});
  });
  // A reload is not a close request, and in a browser there is no close
  // request at all. Best effort in both: the rig gets told to let go of the
  // wheel before the new page grabs it, and a run in progress is at least
  // attempted.
  window.addEventListener("beforeunload", () => {
    game.endRun("window-closed");
    if (rigNative.available()) rigNative.stop();
  });
  window.addEventListener("resize", () => { if (game?.track) drawCoursePlan(dom.coursePlan, game.track); });

  const sync = () => {
    game.assists.traction = dom.tcToggle.checked;
    game.assists.abs = dom.absToggle.checked;
    game.assists.autoShift = dom.autoToggle.checked;
    game.audio.setEnabled(dom.audioToggle.checked);
    updateSession();
  };
  // A control profile can ask for driver aids on by default -- the keyboard
  // does, because its pedals are switches. Applied once at boot from whatever
  // profile is active; the toggles stay the driver's after that.
  {
    const d = game.input.profile?.assistDefaults;
    if (d) {
      if (d.traction != null) dom.tcToggle.checked = d.traction;
      if (d.abs != null) dom.absToggle.checked = d.abs;
    }
  }
  for (const el of [dom.tcToggle, dom.absToggle, dom.autoToggle, dom.audioToggle]) {
    el.addEventListener("change", sync);
  }

  game.renderer.rebuildCar(SDM26);
  dom.etcSummary.innerHTML = etcSummary(game.etc);
  updateSession();
  dom.etcBtn.addEventListener("click", () => game.etcEditor.open());

  // ---- the Runs tab -----------------------------------------------------
  game.driverName = loadDriver();
  game.sessionLabel = loadSessionLabel();
  game.recording = loadRecordPref();
  if (dom.driverName) {
    dom.driverName.value = game.driverName;
    dom.driverName.addEventListener("input", () => {
      game.driverName = dom.driverName.value.trim().slice(0, 64);
      // Typing here IS the rig's own setting, so this one sticks -- and it
      // takes the run back from the launcher, identity included: a name typed
      // into a box is not the person Helios signed in.
      saveDriver(game.driverName);
      game.driverId = null;
      game.launchedBy = null;
      updateSession();
    });
  }
  if (dom.sessionName) {
    dom.sessionName.value = game.sessionLabel;
    dom.sessionName.addEventListener("input", () => {
      game.sessionLabel = dom.sessionName.value.trim().slice(0, 96);
      saveSessionLabel(game.sessionLabel);
      updateSession();
    });
  }
  if (dom.recordToggle) {
    dom.recordToggle.checked = game.recording;
    dom.recordToggle.addEventListener("change", () => {
      game.recording = dom.recordToggle.checked;
      saveRecordPref(game.recording);
      updateSession();
    });
  }
  runsDirectory().then((d) => { if (dom.runsDir) dom.runsDir.textContent = d; })
    .catch(() => { if (dom.runsDir) dom.runsDir.textContent = "unavailable"; });
  refreshRuns();

  const enterSim = (fresh) => {
    game.exitReplay({ keepMenu: true });
    // Before the recording opens, so the manifest carries the aids the driver
    // is actually about to drive with.
    sync();
    game.audio.start();
    dom.menu.hidden = true;
    dom.restartBtn.hidden = false;
    dom.startBtn.textContent = "Resume run";
    if (fresh) {
      game.restart();
    } else if (!game.recorder || game.recorder.samples === 0) {
      // `load()` places the car by calling restart(), which opens a recording
      // -- at boot, before the driver has typed their name, chosen a control
      // profile or touched the driver aids. The Start button then resumes
      // rather than restarts (the car is already on the line), so without
      // this the FIRST run of every session is filed as "Unknown" with the
      // app's boot-time settings. A recording that has not sampled anything
      // has nothing to lose, so it is simply re-opened here.
      game.beginRun();
    }
    game.setPaused(false);
    // The keypress that got here is still DOWN.
    //
    // Escape on the home screen starts the run synchronously inside the
    // keydown handler, and `Input` has already put "Escape" into `keys`. The
    // next frame is the first one to run `update()`, so it sees the key as a
    // fresh edge and pauses the run the driver just entered. Same swallow the
    // throttle-map editor uses on the way out (see `openEtcEditor`).
    game.swallowPauseEdge = true;
    dom.gl.focus();
  };

  // Which dash the driver wants. Remembered per machine by the HUD itself, so
  // the select just reflects and sets it.
  if (dom.dashMode) {
    dom.dashMode.value = game.hud.dashMode;
    dom.dashMode.addEventListener("change", () => {
      game.hud.setDashMode(dom.dashMode.value);
      dom.gl.focus();
    });
  }

  dom.startBtn.addEventListener("click", () => enterSim(!game.started));
  dom.restartBtn.addEventListener("click", () => enterSim(true));

  // The pause menu. The keys still work (input.js); these are the same
  // actions for a mouse.
  document.getElementById("pauseResume").addEventListener("click", () => { game.setPaused(false); dom.gl.focus(); });
  document.getElementById("pauseRestart").addEventListener("click", () => { game.restart(); game.setPaused(false); dom.gl.focus(); });
  document.getElementById("pauseHome").addEventListener("click", () => game.goHome());
  const quitToShell = () => {
    const w = window.__TAURI__?.window;
    const win = w?.getCurrentWindow?.() ?? w?.getCurrent?.();
    win?.close?.();
  };
  const quitBtn = document.getElementById("pauseQuit");
  if (isDesktop) {
    quitBtn.hidden = false;
    quitBtn.addEventListener("click", quitToShell);
  }

  // ---- the end-of-run card ----
  // Every item is also a key, and both go through the same handlers so the
  // mouse and the keyboard can never drift apart.
  const finishQuit = document.getElementById("finishQuit");
  const finish = {
    again: () => { game.restart(); game.setPaused(false); dom.gl.focus(); },
    // `openReplay`, not `enterReplay`: it is the wrapper that reports a run
    // that will not load. Going straight to `enterReplay` left a frozen car,
    // no card, and an unhandled rejection in the console.
    watch: () => { if (game.finishRunId) void openReplay(game.finishRunId); },
    roll: () => { game.setPaused(false); dom.gl.focus(); },
    home: () => game.goHome(),
    quit: quitToShell,
  };
  dom.finishAgain.addEventListener("click", finish.again);
  dom.finishWatch.addEventListener("click", finish.watch);
  document.getElementById("finishRoll").addEventListener("click", finish.roll);
  document.getElementById("finishHome").addEventListener("click", finish.home);
  if (isDesktop) {
    finishQuit.hidden = false;
    finishQuit.addEventListener("click", finish.quit);
  }
  // Registered before the replay and menu handlers below and it stops the
  // event, so nothing else acts on a press aimed at this card. Backspace, H
  // and Esc are deliberately NOT here: they are the game's own keys and
  // already do exactly these things through `input.js`.
  addEventListener("keydown", (e) => {
    if (!game?.finished) return;
    if (/^(INPUT|TEXTAREA|SELECT)$/.test(e.target?.tagName ?? "")) return;
    // Enter is here as well as being the focused button's own default: click
    // the scene behind the card (the canvas takes focus) and the button no
    // longer has it, while the card still says Enter.
    const act = e.code === "Enter" || e.code === "NumpadEnter" ? finish.again
      : e.code === "KeyW" ? (game.finishRunId ? finish.watch : null)
      : e.code === "KeyQ" ? (isDesktop ? finish.quit : null)
      : null;
    if (!act) return;
    e.preventDefault();
    e.stopPropagation();
    act();
  }, true);
  // ---- replay transport keys ----
  // Registered ahead of the menu's Esc handler and it returns early, so the
  // two never both act on one press.
  addEventListener("keydown", (e) => {
    if (!game?.replaying) return;
    if (/^(INPUT|TEXTAREA|SELECT)$/.test(e.target?.tagName ?? "")) return;
    const r = game.replay;
    const shift = e.shiftKey;
    switch (e.code) {
      case "Space": e.preventDefault(); r.toggle(); break;
      case "ArrowLeft": e.preventDefault(); r.nudge(shift ? -0.1 : -1); break;
      case "ArrowRight": e.preventDefault(); r.nudge(shift ? 0.1 : 1); break;
      case "ArrowUp": e.preventDefault(); r.setRate(r.rate * 2); break;
      case "ArrowDown": e.preventDefault(); r.setRate(r.rate / 2); break;
      case "Home": e.preventDefault(); r.seek(0); break;
      case "End": e.preventDefault(); r.seek(r.duration); break;
      case "KeyC": game.cameraIndex = (game.cameraIndex + 1) % CAMERAS.length; break;
      case "KeyL": if (r.bestLap) r.seekLap(r.bestLap.lap); break;
      case "KeyT": e.preventDefault(); game.replayPanel?.toggleTraces(); break;
      case "Tab": e.preventDefault(); game.replayPanel?.toggleBare(); break;
      case "Escape": e.preventDefault(); game.exitReplay(); break;
      default: return;
    }
    game.applyReplayFrame();
    game.replayPanel?.paint(0, true);
  }, true);

  // Esc on the home screen with a run behind it goes back to the run, the
  // way Esc from the run comes here.
  addEventListener("keydown", (e) => {
    if (e.code === "Escape" && !game.replaying && !dom.menu.hidden && game.started && !game.etcEditor.isOpen &&
        !/^(INPUT|TEXTAREA|SELECT)$/.test(e.target?.tagName ?? "")) {
      enterSim(false);
    }
  });
  // Clicking the scene while driving re-captures the pointer after the
  // browser released it (Esc under pointer lock releases it silently).
  dom.gl.addEventListener("click", () => game.syncPointer());
  document.addEventListener("pointerlockchange", () => {
    // Under pointer lock the browser consumes Esc to release the lock and
    // the page never sees the key. So a lock that goes away while we are
    // driving is the driver pressing Esc: open the menu.
    if (document.pointerLockElement !== dom.gl && game.driving && game.input.profile.mouse?.enabled) {
      game.setPaused(true);
    }
  });

  // ---- launch requests: `?track=mis&profile=wheel&autostart=1` in a browser,
  // `fsae-sim --track mis --profile wheel --autostart` on the desktop, and the
  // same again if a running app is launched a second time (Helios, a shortcut).
  const onOff = (v) => (v == null ? undefined : /^(1|on|true|yes)$/i.test(String(v)) ? true : /^(0|off|false|no)$/i.test(String(v)) ? false : undefined);
  const applyLaunch = async (o) => {
    if (!o) return;
    if (o.track && TRACKS.some((t) => t.id === o.track) && o.track !== dom.trackSel.value) {
      dom.trackSel.value = o.track;
      if (!(await loadCourse())) return;
    }
    // ---- what a launcher may decide, and what it may not --------------
    //
    // Helios owns the RUN: who is driving, on which course, with which aids,
    // and whether it is logged. The simulator owns the RIG: the control
    // profile's mapping and force feedback, the pedal calibration, the
    // throttle map, the vehicle parameters, the camera. See docs/SETTINGS.md.
    //
    // So everything below is applied to this session and NOT written to the
    // rig's own saved settings. That is not a detail: `setProfile` pins the
    // profile and switches off device auto-detection, and saving the driver
    // name left the next person at the rig filing runs under somebody else's
    // name.
    if (o.profile && game.input.settings.ids().includes(o.profile)) {
      game.input.useProfileForSession(o.profile);
    }
    if (o.driver) game.driverName = String(o.driver).slice(0, 64);
    if (o.driverId) game.driverId = String(o.driverId).slice(0, 64);
    if (o.session) game.sessionLabel = String(o.session).slice(0, 96);
    if (o.noRecord != null) game.recording = !o.noRecord;
    if (o.driver || o.session || o.profile || o.track || o.noRecord != null) {
      game.launchedBy = "launcher";
    }
    if (o.traction != null) dom.tcToggle.checked = o.traction;
    if (o.abs != null) dom.absToggle.checked = o.abs;
    if (o.autoShift != null) dom.autoToggle.checked = o.autoShift;
    if (o.fullscreen) toggleFullscreen();
    updateSession();
    // A launcher can open straight into a recorded run instead of a drive.
    // That is how Helios's "Watch replay" works: it starts (or re-focuses)
    // the sim with the run it wants on the command line.
    if (o.replay) {
      // Remember that this window exists to watch a run. Closing the replay
      // then closes the window rather than dropping the user on a launch
      // screen they never asked for -- they came from Helios's Runs table and
      // that is where "Close" should put them back.
      game.launchedForReplay = true;
      await openReplay(o.replay, o.ghost ?? null);
      return;
    }
    // A lap to chase, loaded before the drive starts so the delta is live
    // from the first corner rather than from lap two.
    if (o.reference) await game.loadReference(o.reference);
    // An autostart from a launcher is not a user gesture; the audio context
    // may open suspended and resumes on the first key or button.
    if (o.autostart) enterSim(true);
  };
  const fromQuery = () => {
    const q = new URLSearchParams(location.search);
    if (![...q.keys()].length) return null;
    return {
      track: q.get("track"), profile: q.get("profile"),
      traction: onOff(q.get("tc")), abs: onOff(q.get("abs")), autoShift: onOff(q.get("auto")),
      driver: q.get("driver"), driverId: q.get("driverId"), session: q.get("session"),
      noRecord: onOff(q.get("record")) === false ? true : onOff(q.get("norecord")),
      replay: q.get("replay"), ghost: q.get("ghost"), reference: q.get("reference"),
      autostart: onOff(q.get("autostart")) === true, fullscreen: onOff(q.get("fullscreen")) === true,
    };
  };
  await applyLaunch({ ...(fromQuery() ?? {}), ...((isDesktop && (await launchOptions())) || {}) });
  onLaunchOptions((o) => { applyLaunch(o); });

  // Debug handle: lets you poke at the model from the console, e.g.
  //   __sim.car.telemetry, __sim.powertrain.wotTorque(9000)
  window.__sim = game;

  let last = performance.now();
  let frameErrors = 0;
  const frame = (now) => {
    const dt = Math.min((now - last) / 1000, 0.05);
    last = now;
    try {
      if (game.replaying) {
        // A replay steps the playback clock, not the physics. The rig is held
        // so the wheel stays quiet and the native model does not run away
        // while nothing is feeding it.
        game.input.driving = false;
        game.input.poll();
        game.holdNative();
        game.replayFrame(dt);
      } else if (!dom.menu.hidden) {
        // Still poll so the pad-connected badge is live in the menu.
        game.input.driving = false;
        game.input.poll();
        game.holdNative();
      } else {
        game.update(dt);
      }
      game.render();
    } catch (err) {
      // A throw here used to end the loop: a frozen frame with no message.
      // Report the first one loudly, then keep going; the rig's watchdog and
      // the pause both rely on this loop running.
      if (frameErrors++ === 0) {
        console.error("frame error", err);
        dom.loadNote.textContent = `Something went wrong in the game loop: ${err.message ?? err}`;
        dom.loadNote.classList.add("error");
      }
      if (frameErrors > 300) return; // hopeless; stop burning the CPU
    }
    requestAnimationFrame(frame);
  };
  requestAnimationFrame(frame);
}

/**
 * Redraw the recorded-runs list on the launch screen.
 *
 * Deliberately reads the manifests rather than keeping a cached index: a run
 * directory shared with Helios can gain runs from another machine, and a list
 * that quietly went stale would be worse than one that costs a directory read.
 */
async function refreshRuns() {
  const host = dom.runsList;
  if (!host) return;
  let runs = [];
  try {
    runs = await listRuns(60);
  } catch (err) {
    host.innerHTML = `<p class="note error">Could not read recorded runs: ${escHtml(String(err?.message ?? err))}</p>`;
    return;
  }
  if (dom.runsBadge) dom.runsBadge.textContent = runs.length ? String(runs.length) : "";
  if (!runs.length) {
    host.innerHTML = `<p class="note">${isDesktop
      ? "Nothing recorded yet. Drive a run and it appears here."
      : "In a browser a finished run downloads instead of being filed. Drop it into Helios."}</p>`;
    return;
  }
  host.innerHTML = "";
  for (const { runId, manifest: m } of runs) {
    const st = m.stats ?? {};
    const best = st.bestLapS;
    const row = document.createElement("div");
    row.className = "run-row";
    const when = m.startedAt ? new Date(m.startedAt) : null;
    const whenText = when && !isNaN(when) ? when.toLocaleString(undefined,
      { month: "short", day: "numeric", hour: "2-digit", minute: "2-digit" }) : runId;
    row.innerHTML =
      `<div class="who"><b>${escHtml(m.driver || "Unknown")}</b> &middot; ${escHtml(m.trackName || m.track || "?")}</div>` +
      `<div class="right"><span class="time">${best != null ? fmt(best) : "--.---"}</span>` +
      `<button class="secondary" data-replay="${escHtml(runId)}">Replay</button></div>` +
      `<div class="meta">${escHtml(whenText)} &middot; ${st.laps ?? 0} lap${st.laps === 1 ? "" : "s"}` +
      ` &middot; ${(st.durationS ?? 0).toFixed(1)} s` +
      ` &middot; ${st.totalCones ?? 0} cone${st.totalCones === 1 ? "" : "s"}` +
      `${st.peakLatG ? ` &middot; ${st.peakLatG.toFixed(2)} g peak` : ""}` +
      `${m.session ? ` &middot; ${escHtml(m.session)}` : ""}</div>`;
    host.appendChild(row);
  }
  host.querySelectorAll("button[data-replay]").forEach((b) => {
    b.addEventListener("click", () => openReplay(b.dataset.replay));
  });
}

/**
 * Open a replay, reporting anywhere it can fail. Called from the Runs tab, a
 * `--replay` launch, and the `replay=` query string in the browser build.
 */
async function openReplay(runId, ghostId = null) {
  if (!game || !runId) return;
  try {
    dom.loadNote.classList.remove("error");
    dom.loadNote.textContent = "Loading run...";
    await game.enterReplay(runId, ghostId);
    dom.loadNote.textContent = "";
  } catch (err) {
    console.error("could not open the replay", err);
    game.exitReplay();
    dom.loadNote.textContent = `Could not open that run: ${err?.message ?? err}`;
    dom.loadNote.classList.add("error");
  }
}

function escHtml(v) {
  return String(v).replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
}

function round3(v) {
  return Number.isFinite(v) ? Math.round(v * 1000) / 1000 : 0;
}

/**
 * The live vehicle parameters, as the run was driven with them.
 *
 * The spec sheet and the d-pad setup adjuster both write straight into the
 * shared SDM26 object, so a run's times are only interpretable next to the
 * numbers that were in force. `PARAM_DEFAULTS` is the same list the setup UI
 * exposes, which is exactly the set a driver can have moved.
 */
function snapshotSetup() {
  const out = {};
  for (const path of Object.keys(PARAM_DEFAULTS)) {
    try {
      const v = readParam(path);
      if (typeof v === "number" && Number.isFinite(v)) out[path] = v;
    } catch { /* a parameter this build no longer has */ }
  }
  // The d-pad's two items are NOT in PARAM_DEFAULTS -- that list comes from the
  // spec sheet's editable rows, and roll distribution and brake bias are
  // adjusted from a different surface. They are also the only two a driver
  // changes from inside the car, which makes them the likeliest to differ
  // between two runs and the worst two to be missing from the record.
  for (const path of ADJUSTABLE_PATHS) {
    const v = readParam(path);
    if (typeof v === "number" && Number.isFinite(v)) out[path] = v;
  }
  return out;
}

function shortPadName(id) {
  const m = /^([^(]+)/.exec(id || "");
  return (m ? m[1] : id).trim().slice(0, 28) || "connected";
}

boot();
