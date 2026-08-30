# How the Simulation Figures Out Where a Missile Is Going

*An end-to-end walkthrough of ballistic missile tracking in this codebase, written for someone with no radar engineering background. Every step links to the actual code so you can follow along.*

---

## The Big Picture

Imagine you're standing in a dark field and someone launches a bottle rocket somewhere far away. You can't see the whole flight — you only get quick flashes of where it is at certain moments. From those flashes, you have to answer three questions:

1. **Where is it right now?** (a *detection*)
2. **Where is it heading and how fast?** (a *track*)
3. **Where will it land?** (a *trajectory prediction*)

The simulation does exactly this — and it's forbidden from cheating. The defense side never gets to peek at the missile's "true" position. Everything it knows must come from simulated radar measurements. This document walks through the whole pipeline, stage by stage.

```
MISSILE FLIES (ground truth — hidden from defense)
        │
        ▼
RADAR "SEES" IT ──────► raw detections (noisy: bearing + range + altitude)
        │
        ▼
DETECTIONS → TRACKS ──► 3+ sightings required, quality-scored
        │
        ▼
VELOCITY ESTIMATE ────► speed, heading, climb/dive rate (EKF or least-squares)
        │
        ▼
SENSOR FUSION ────────► combine all radars' tracks into one "best" track
        │
        ▼
CONVERGED TRAJECTORY ─► fit a parabola through altitude history →
                        launch point, landing point, apogee, flight time
        │
        ▼
FIRE CONTROL ─────────► the yellow X marker, interceptor launch solutions
```

---

## Step 0: What the Missile Is Actually Doing (the hidden truth)

Before we can understand how the *defense* reconstructs a missile's path, we need to know what the *real* path looks like — the thing the sensors are trying to discover.

Ballistic missiles work like a thrown football: after the boost (engine burn) finishes, they're just coasting under gravity. The simulation models the full flight as a simple, smooth curve:

- **Ground path:** a straight-ish line (a "great circle") from launch site to target. The missile covers this distance at a *constant* ground speed.
- **Altitude:** a perfect parabola in time. It rises smoothly to a peak (the *apogee*) at the halfway point, then descends symmetrically.

The math for the altitude is at `src/simulation/physics.rs:148`:

```rust
// Parabola: h(t) = 4 * max_h * t * (1 - t)
// This gives 0 at t=0 and t=1, max at t=0.5
4.0 * self.max_altitude_km * t * (1 - t)
```

Here `t` is the fraction of the flight that's complete (0 at launch, 1 at impact). If you've ever graphed `y = x(1−x)`, you've seen this shape — an arch. A 1000 km-range missile in this sim peaks around 330 km up and flies for about 650 seconds.

The missile's position is advanced along this curve every simulation tick by `update_missile()` at `src/simulation/engine.rs:2821`, which calls `position_at()` at `src/simulation/physics.rs:131`.

**Key fact:** the defense code never calls these functions to *decide* anything. It can only reconstruct what they do — from noisy radar data. The only place ground truth is read is to physically move the missile. (This rule is documented in `AGENTS.md` — "Defense uses only sensor-detected data, not ground truth.")

---

## Step 1: The Radar Sees Something (Detections)

Every frame, the simulation asks each radar: "Do you detect anything this scan?"

The entry point is `DetectionSystem::update()` at `src/simulation/detection.rs:1274`. For each radar and each active missile, the sim computes a **probability of detection** — because real radars aren't magic; detection gets harder with distance, small targets, and bad weather.

The full physics-inspired formula lives in `calculate_detection_probability()` at `src/simulation/detection.rs:3282`. Three factors multiply together:

| Factor | What it models | Code |
|---|---|---|
| Range | Real radars obey the "inverse fourth power" law — doubling the distance makes the echo 16× weaker. | `1.0 / (1.0 + range_ratio.powi(2))` (~line 3295) |
| RCS (radar cross-section) | How big/shiny the target looks to radar. Small stealthy targets are harder. | `10.0_f64.powf(rcs_advantage_db / 20.0)` (~line 3312) |
| Atmosphere | The radar beam loses energy passing through air (worse at long range, low altitude). | `calculate_atmospheric_attenuation(...)` (~line 3319) |

A random roll decides whether the radar actually reports the target this scan. And sometimes radars report things that *aren't* there — false alarms happen at a 5% per-scan rate (`BASE_FALSE_ALARM_RATE` at `src/simulation/detection.rs:1293`).

When a detection happens, the radar reports three noisy numbers: **bearing** (compass direction to the target), **range** (distance), and **altitude**. These are the raw ingredients — the same three numbers a real fire-control radar measures. The formal measurement type is `RadarMeasurement` at `src/simulation/ekf.rs:38` (range, azimuth, elevation).

From bearing + range, the target's ground position is computed with trigonometry in `calculate_position_from_bearing_range()` — walk `range` kilometers along `bearing` degrees from the radar's position, and you've located the target (on the map's curved surface).

> **High-school analogy:** You're in a dark field and you hear a bottle rocket. You point at it (bearing) and estimate how far the sound is coming from (range). That's a detection.

---

## Step 2: From Detections to Tracks

One flash of light doesn't tell you much. Is it a missile? A plane? A false alarm? Real systems — and this simulation — demand **multiple sightings before believing anything**.

### Track creation (the "3 sightings" rule)

Detections become tracks in the block starting at `src/simulation/detection.rs:2100`. The rules:

1. **Quality gate:** a detection must score above a quality threshold (0.3 normally, relaxed to 0.2 if another sensor already tracks the same target — that's "network-aided acquisition," letting radars hand off targets to each other; see the `network_boost` logic at `src/simulation/detection.rs:2271-2296`).
2. **History:** each track keeps a timestamped list of every position measurement — `position_history`, capped at 100 entries (`MAX_POSITION_HISTORY` at `src/simulation/detection.rs:78`, truncation at lines 180-181).
3. **Quality score:** every new detection *raises* the track's `track_quality` (0.0 to 1.0); every second *without* an update lowers it (`src/simulation/detection.rs:2110`). If quality hits 0, the track is deleted (line 2114). So a track is a living thing: feed it measurements and it thrives; go blind and it dies in ~10 seconds.

### The gate that matters most

A track is only trusted enough to *act on* once it has several measurements. Fire control requires **10 measurements** before it will launch an interceptor, plus a quality ≥ 0.6, plus a velocity confidence ≥ 0.55 — all enforced in `calculate_intercept_solution_from_track()` at `src/simulation/engine.rs:2189` (gates at lines 2280-2296). And the converged trajectory (Step 5) requires at least 3 measurements before it will even attempt a prediction (`MIN_MEASUREMENTS` at `src/simulation/detection.rs:2674`).

> **High-school analogy:** One flash could be anything. Three flashes in a row, all moving the same direction — now you believe there's a rocket, and roughly where it's going.

---

## Step 3: Estimating Velocity (speed, heading, climb rate)

Once a track has a few position fixes in its history, the sim can compute **how fast the target is moving and in which direction**. This is `update_velocity_estimate()` at `src/simulation/detection.rs:285`.

Two methods, in order of preference:

### Method A: The Extended Kalman Filter (EKF)

The EKF (`src/simulation/ekf.rs`) is the fancy, professional way to do this. Here's the intuition:

> You don't just remember *where* the target is — you remember what you *believe* about where it is, including your uncertainty. Each new radar measurement updates that belief, but only as much as you trust it. Noisy measurement? It nudges your belief slightly. Clean measurement? It pulls harder.

The filter keeps two things per track: an **estimated state** (position + velocity) and an **uncertainty** (how blurry your estimate is). Every new measurement:

1. **Predict:** assuming the target keeps doing what it's doing (with gravity pulling it down), predict where the next measurement should be.
2. **Compare:** see how far off the radar's actual measurement was.
3. **Correct:** blend prediction and measurement based on which one you trust more.

The result — velocity (ground speed, heading, vertical rate) with a *confidence score* — is extracted at `src/simulation/detection.rs:296-300`:

```rust
let (ground_speed, heading, vertical_rate) = ekf.get_velocity();
```

### Method B: Weighted least-squares (the fallback)

If the EKF isn't available, the sim does what you'd do with graph paper: it takes pairs of position measurements, computes `distance ÷ time` for each pair, and takes a weighted average — recent measurements and high-quality ones count more (`src/simulation/detection.rs:340-396`). It also sanity-checks the results (a ballistic missile shouldn't be going faster than ~7 km/s horizontally — line 344), throwing out physically impossible values.

Either way, the output is a `VelocityEstimate` (struct at `src/simulation/detection.rs:68`):

```rust
pub struct VelocityEstimate {
    pub ground_speed_km_s: f64,   // e.g., 2.2 km/s
    pub heading_deg: f64,         // e.g., 78° (east-northeast)
    pub vertical_rate_km_s: f64,  // e.g., -0.5 (descending)
    pub confidence: f64,          // 0.0-1.0
}
```

> **High-school analogy:** You saw the rocket at point A at 8:00:00 and point B at 8:00:05. Divide the distance between A and B by 5 seconds — that's its speed. The direction from A to B is its heading.

---

## Step 4: Sensor Fusion (combining all the radars)

One radar gives you one opinion. Two radars give you two slightly *different* opinions — because both are noisy. Which do you trust?

`get_fused_track()` at `src/simulation/detection.rs:2409` merges all sensor tracks for the same target into a single **fused track**:

- **Position:** a *quality-weighted average*. A high-quality track (0.9) pulls the average 9× harder than a low-quality one (0.1). (`src/simulation/detection.rs:2440-2446`)
- **Velocity:** same idea, weighted by each estimate's confidence.
- **Fused quality:** more sensors watching the same target = more confidence (`calculate_fused_quality()` at `src/simulation/detection.rs:2875`).
- **Uncertainty radius:** how big a circle of "we're not sure exactly, but it's in here" to draw — computed from quality, staleness (how long since the last update), and sensor count (`calculate_uncertainty_radius()` at `src/simulation/detection.rs:2895`). Fresh, multi-sensor tracks can shrink uncertainty to ~5 km.

The fused track also carries a **measurement count** — how many raw radar sightings went into it — which is what fire control checks against the "10 measurements minimum" rule.

> **High-school analogy:** Three friends each guess where the rocket is. One was standing close and has binoculars (weight: high); one was squinting into the sun (weight: low). You don't average their guesses equally — you trust the binocular friend more.

---

## Step 5: The Converged Trajectory (the pièce de résistance)

This is where raw tracking becomes *understanding*: "the missile launched *here*, will land *there*, peaks at *this* altitude, total flight time is *this many seconds*."

The result is a `ConvergedTrajectory` (struct at `src/simulation/detection.rs:459`) containing origin, target, apogee, range, flight time, and confidence. It's what draws the yellow X impact marker and what interceptor launch solutions are computed from.

### The key insight that makes it work

Remember Step 0: the missile's altitude is a **parabola in time** — `h(t) = 4A·τ(1−τ)`. That formula is a *quadratic* in time. Which means: **if you fit a quadratic curve through the missile's altitude measurements, you recover the whole trajectory exactly.**

Think about it:

- The fitted parabola equals zero at two times: **launch time** and **impact time** (a parabola crosses zero at its two roots — that's algebra class!).
- The peak of the fitted parabola is the **apogee**.
- Total flight time = impact time − launch time.

All of this comes out of one curve-fit through noisy altitude readings. No magic, no peeking at the missile's true plan — just math on measurements.

### How the fit works

`fit_altitude_quadratic()` at `src/simulation/detection.rs:3402` performs a **least-squares fit** — it finds the parabola that minimizes the total squared error against all the altitude samples collected so far. (Least-squares is "find the curve that misses all your points by the least total amount" — the same math behind drawing a trend line through scatter points, but for a curve.)

Requirements before it will even try (`src/simulation/detection.rs:3402-3410`):

- At least **6 altitude samples** (`MIN_POINTS`)
- Spread across at least **4 seconds** of flight (`MIN_SPAN_SEC`) — three points from one instant would fit anything, which is useless

The normal-equations solution (solving the least-squares system by determinants, Cramer's rule) runs through line ~3460, and the fit quality is measured as an **RMS error** — how far off the parabola is from the actual points, on average.

From the fitted parabola, the roots (launch/impact times) and peak (apogee) are extracted by the `AltitudeFit` helpers just above, at `src/simulation/detection.rs:3340-3380`.

### Converting the fit into a full trajectory

The refinement happens in `refine_converged_trajectory()` at `src/simulation/detection.rs:2664`. Given the fit:

- **Impact point:** the missile flies at *constant ground speed* (Step 0). So from its current position and heading, project forward: `landing point = current position + heading × speed × time remaining`. (`calculate_position_from_bearing_range` does the geometry.)
- **Launch point:** same idea, projected *backward*.

### Stability: why the yellow X stops jumping around

Early in tracking, each new measurement shifts the estimate a lot (few points = noisy fit). The sim handles this with two mechanisms:

1. **A running weighted average** — each new estimate is blended into the accumulated one with a weight that grows with fit quality and time span. As more samples arrive, each new one moves the needle less and less. The marker *converges* instead of jittering. (`src/simulation/detection.rs:2770-2830`, the `total_weight`/`sample_count` blending.)

2. **Outlier dampening** — if a single new estimate suddenly claims a landing point 400+ km from the running average, its weight is cut 10×, so one wild radar glitch can't yank the marker. (`src/simulation/detection.rs:2775-2779`.)

3. **A measurement gate** — the estimate is *only* recomputed when a genuinely new radar measurement arrives. Between measurements, nothing moves — no amount of screen-redrawing jitters the marker. (The `last_meas_ts` check at `src/simulation/detection.rs:2686-2690`.)

There's also a **fallback estimator** — `parabola_state_from_altitude()` at `src/simulation/detection.rs:3473` — used before enough altitude history exists for the full fit. It takes the current altitude and climb rate and solves (with a couple of refinement iterations) for a self-consistent (apogee, flight-progress) pair assuming the same parabolic profile. It's deliberately blended in with low weight, since it's built on less data.

> **High-school analogy:** Imagine plotting the rocket's height over time on graph paper. Six or more points in, you can sketch the smooth curve through them. Extend the curve down to the ground on both sides — that's where it launched and where it will land. The more points you plot, the less your sketch changes with each new dot.

---

## Step 6: What the Defense Does With It

Once a converged trajectory exists, the defense can finally *act* — all through one accessor, `DetectionSystem::get_converged_trajectory()` at `src/simulation/detection.rs:2637`, which hands back the trajectory plus its estimated total flight time.

1. **The yellow X impact marker** (UI) reads `converged.target` from the fused track. This is why it now appears within seconds of detection, settles near the real impact point, and freezes instead of wandering.

2. **Interceptor launch solutions** — `calculate_intercept_solution_from_track()` at `src/simulation/engine.rs:2189` scans *future positions along the converged trajectory* and finds the earliest point the interceptor can reach **before** the missile does, inside its altitude/range envelope. If the sensor chain can't produce a converged trajectory, there is *no launch* — by design (see `AGENTS.md`, "Fire Control Architecture").

3. **Mid-course guidance** — the same projection steers in-flight interceptors onto the missile's predicted path.

4. **Doctrine timing (shoot-look-shoot)** — time-to-impact for "do we have time for a second shot after assessing the first?" comes from the same estimate.

Everything — the marker, the launches, the guidance — traces back to Step 5's parabola fit on radar data.

---

## Summary Table

| Stage | Question answered | Key code |
|---|---|---|
| Missile physics | Where *is* it, truly? (hidden from defense) | `physics.rs:131` `position_at`, `physics.rs:148` altitude parabola, `engine.rs:2821` `update_missile` |
| Detection | Is something there? (probabilistic, noisy) | `detection.rs:1274` `update`, `detection.rs:3282` `calculate_detection_probability` |
| Track | Where is it *probably* right now? | `detection.rs:2100-2345` track update/creation, 3-measurement minimum |
| Velocity | How fast, which way, climbing or diving? | `detection.rs:285` `update_velocity_estimate` (EKF preferred, least-squares fallback) |
| Fusion | Combine all radars' opinions | `detection.rs:2409` `get_fused_track`, uncertainty at `detection.rs:2895` |
| Converged trajectory | Where did it launch, where will it land? | `detection.rs:2664` `refine_converged_trajectory`, `detection.rs:3402` quadratic fit |
| Fire control | Launch? Where do we shoot? | `engine.rs:2189` solution scan, gated on 10 measurements + quality thresholds |

---

## Why It's Built This Way

- **No peeking:** every defense decision must survive on sensor data alone. If the sensor chain fails, the defense *fails* — which is the point. The system's realism comes from the difficulty.
- **The parabola fit is exact, not approximate:** because the simulation's missiles fly a mathematically perfect parabola (constant ground speed + quadratic altitude), the quadratic fit recovers the *true* trajectory from measurements — limited only by radar noise, which the weighted averaging smooths out.
- **Layered skepticism:** 1 detection ≠ a threat. 3 measurements ≠ a trajectory. 10 measurements ≠ a firing solution. Each layer demands more evidence before committing to the next.
- **Convergence, not jitter:** weighted averaging + a measurement gate mean every displayed estimate gets *more confident over time*, like a detective narrowing suspects — the opposite of the omniscient "exact answer from frame one" that less rigorous sims would give you.