# How Interceptors Find, Chase, and Kill Incoming Missiles

*The companion to `ballistic-track-explainer.md` (how the defense figures out where a missile is going). This document covers what happens next: deciding to launch, flying the interceptor, and determining hit or miss — with links to the actual code.*

---

## The Big Picture

Once the defense knows a missile's trajectory (see the tracking explainer), the clock is ticking. The missile will land in minutes. The defense must:

1. **Decide to shoot** — is the target real, is the shot physically possible, is it in range? (*launch*)
2. **Plan the meeting point** — find a spot in the sky where the interceptor and missile arrive at the *same time*. (*fire control solution*)
3. **Fly there** — boost, coast, and correct course along the way. (*guidance*)
4. **See the target** — the interceptor's own seeker must lock on for the final seconds. (*terminal homing*)
5. **Collide** — hit-to-kill means literally hitting a bullet with a bullet. (*resolution*)
6. **Assess and retry** — if it missed, figure that out and shoot again. (*kill assessment, doctrine*)

```
THREAT DETECTED & TRACKED (see tracking explainer)
        │
        ▼
LAUNCH DECISION ─────► track quality ≥ 0.6, 10+ measurements, in fire-control range,
        │               closure feasible, shots remaining, no shots already in flight
        ▼
FIRE CONTROL SOLUTION ► scan the predicted trajectory for the EARLIEST point where
        │               the interceptor arrives BEFORE the missile, inside the envelope
        ▼
BOOST → COAST ───────► rocket accelerates to max speed, then coasts (drag-exact)
        │
        ▼
MID-COURSE GUIDANCE ─► every few seconds: re-project the target from the latest
        │               radar track, correct the interceptor's aim point
        ▼
TERMINAL PHASE ──────► seeker acquires (range + gimbal gates), then
        │               Proportional Navigation / Lambert steering
        ▼
RESOLUTION ──────────► inside kill radius = HIT; passed CPA = MISS;
        │               1.2× planned time with no contact = timeout MISS
        ▼
KILL ASSESSMENT ─────► 3-second confirmation delay; confirmed miss →
                        follow-up shot queued (Shoot-Look-Shoot)
```

**The golden rule** (from `AGENTS.md`): the defense decides everything from *sensor data only*. If the radar chain can't produce a firing solution, the launch is refused — the interceptor never gets to peek at the missile's true position to fix its aim. The only exception is the seeker: once it's physically close enough to "see" the target itself, it tracks what it sees — that's a real sensor, doing its real job.

---

## Step 1: The Launch Decision (Should We Shoot?)

The whole decision process lives in `launch_interceptors_at_threats()` at `src/simulation/engine.rs:1837`. It runs every simulation tick. Before any interceptor can launch, a stack of gates must all pass:

### Gate A: Is the track trustworthy?

From the tracking pipeline, the fused track must be "fire control quality":

- **Quality ≥ 0.6** — the track is scored 0.0–1.0 based on detection probability and update freshness (`engine.rs:2280-2284`)
- **10+ measurements** — enough radar sightings to be sure this isn't a false alarm (`engine.rs:2289`)
- **Staleness < 5 s** — the last radar update was recent; a stale track means the missile could be anywhere (`engine.rs:2284`)
- **Velocity confidence ≥ 0.55** — we trust our speed/heading estimate enough to predict a meeting point (`engine.rs:2298-2299`)

> **High-school analogy:** Before you swing at a pitch, you watch it long enough to be sure it's a real pitch — not the catcher faking a throw. One glimpse isn't enough; you track it out of the pitcher's hand.

### Gate B: Are we already shooting at it?

The engine tracks how many interceptors are in flight per target and how many shots have been fired total (`engine.rs:1950-1972`):

- **Something already in flight?** Don't stack more shots yet — wait for the outcome.
- **Total shots ≥ `max_shots_per_target` (default 4)?** Stop wasting interceptors on this target.

### Gate C: Is the target in reach?

- The fused track position must be inside the unit's **fire-control range** — detection range × a multiplier (`engine.rs:1957-1971`). Measured to the *sensor track's* position, never the missile's true position.
- The computed intercept point must be inside the interceptor's **altitude envelope** — e.g., THAAD can't engage anything above 150 km; Iron Dome can't reach anything above ~10 km (`engine.rs:2359-2373`, envelope check at `engine.rs:2382-2385`).

### Gate D: Can we physically make the shot? (closure feasibility)

Even if the intercept point is inside the "paper" envelope, the interceptor must actually be able to get there in time. The check at `engine.rs:2010-2031`:

```
required average speed = distance to intercept point ÷ time until the missile gets there
```

If that exceeds the interceptor's max velocity by more than 20%, the launch is **refused** with an `[ENGAGE REJECT]` log line. This single check organically stops silly matchups — an 0.7 km/s Iron Dome Tamir against a 2–3 km/s MRBM warhead — without any hardcoded "who can fight what" table. The geometry simply doesn't close.

---

## Step 2: The Fire Control Solution (Where Do We Meet It?)

This is `calculate_intercept_solution_from_track()` at `engine.rs:2255`, which hands off to `calculate_intercept_from_converged_trajectory()` at `engine.rs:2325`.

### The core idea: a time-and-place rendezvous

An interceptor can't chase a missile from behind — the missile moves 2–7 km/s. Instead, both parties are aimed at a single **meeting point**, chosen so both arrive at the same moment:

1. Rebuild the missile's predicted path from the **converged trajectory** — the radar-derived estimate of launch point, landing point, apogee, and flight time (from the tracking pipeline). The path is reconstructed with `BallisticTrajectory::with_params()` (`src/simulation/physics.rs:109`), which reproduces the same parabolic-altitude, constant-ground-speed curve the missile actually flies.

2. Figure out where the missile is *along* that path right now — its progress fraction — using the fused track position (`engine.rs:2344-2350`).

3. Scan **60 candidate future positions** along the path (`engine.rs:2374-2412`). For each candidate:
   - Is it inside the altitude envelope? (THAAD's 150 km ceiling, etc.)
   - Is it inside the platform's engagement range?
   - **Will the interceptor arrive before or with the missile?** Interceptor arrival time comes from `calculate_flight_time()` (`engine.rs:2416`); missile arrival time comes from the trajectory profile. The interceptor must arrive **no later than 0.5 s after** the missile — no exceptions.
   - If all pass: *this* is the meeting point. The scan takes the **earliest** feasible point, which preserves time for follow-up shots if this one misses.

### The single source of truth for "how long to get there"

Interceptor arrival time is computed by `InterceptorKinematics::time_to_cover_distance()` at `src/simulation/entities.rs:723`. It accounts for:

- **The boost phase:** the rocket starts at 0 and accelerates (e.g., SM-3: 15 g for 30 s to reach 4.5 km/s). During boost, distance = the physics-1 formula `v₀t + ½at²` — solved as a quadratic for time.
- **The coast phase:** after burnout, constant max speed (with a 12% drag allowance for low-altitude systems — see Step 3).
- **Already-in-flight corrections:** for guidance updates mid-flight, the function starts from the interceptor's *current* velocity, not from zero.

Everything — launch planning, mid-course corrections, intercept-time recalculations — calls this one function. That wasn't always true: before the interceptor overhaul, three different parts of the code used three different timing formulas that disagreed by 1–81 seconds, and every engagement missed. (The history is documented in `AGENTS.md`, "Fire Control Architecture.")

> **High-school analogy:** A friend is walking across campus on a straight path. You're on a bike at a known speed. You don't chase them — you pick a spot on their path you can reach at the same time they do, and you both arrive together.

---

## Step 3: Flight — Boost, Coast, and Drag

The interceptor is created at `engine.rs:2189` (`Interceptor::new`) with its launch time, aim point, and predicted intercept time. Then `update_interceptors()` at `engine.rs:706` advances every in-flight interceptor every tick.

### The three flight phases

Defined by `InterceptorPhase` (`entities.rs:573`) and computed by `phase_at_progress()` (`entities.rs:789`):

| Phase | What happens | Roughly |
|---|---|---|
| **Boost** | Rocket motor burning; accelerating at `boost_acceleration_g` | First 3–170 s depending on system |
| **Coast** | Motor spent; flying at constant max velocity | Most of the flight |
| **Terminal** | Last 30% of the flight; seeker active, sharp maneuvers allowed | Final seconds |

### Drag is real (and cumulative)

Endo-atmospheric interceptors (Patriot, THAAD, Iron Dome, David's Sling) fly through air below 100 km, and air slows them down — *a lot* near the ground. `update_kinematics()` at `entities.rs:992` integrates drag every tick:

```rust
// Drag deceleration: a = ρv²/(2β)
let drag_decel_m_s2 = (rho * v_m_s * v_m_s) / (2.0 * ballistic_coef);
```

The formula is standard aerodynamics: deceleration grows with air density (ρ, from the 1976 US Standard Atmosphere model) and the *square* of velocity, and shrinks with the interceptor's ballistic coefficient β (how "slippery" it is — mass ÷ drag area). Crucially, this is **cumulative**: each tick subtracts from the *previous actual velocity*, so speed loss compounds over the flight exactly as it does in real life. `distance_traveled_km` (field at `entities.rs:824`, integration at `entities.rs:1035`) tracks the true path length.

Exo-atmospheric systems (GBI, SM-3, Arrow 3) spend their coast above the atmosphere, where there's no air — so no drag applies, matching real physics.

> **High-school analogy:** Throw a tennis ball versus a paper airplane at the same speed. The airplane slows down much faster — that's drag. Now imagine the airplane losing a *little* speed every single instant, each loss making the next bit of flight slower. That's cumulative drag integration.

---

## Step 4: Mid-Course Guidance (Correcting Along the Way)

The launch solution was computed from the best radar estimate *at launch time*. But the estimate keeps improving as more radar measurements arrive — so the aim point gets refined in flight. This is the **mid-course guidance pass** at `engine.rs:755`.

Every tick, for each in-flight interceptor (outside terminal phase):

1. Get the latest fused radar track for the target.
2. Get the **latest converged trajectory** — if the radar fit has improved since launch, the predicted path is better now (`engine.rs:830`).
3. Re-solve the rendezvous: find where the missile *will be* when the interceptor can get there, using the same `time_to_cover_distance()` timing as everything else (`engine.rs:875`).
4. If the new aim point differs from the current one by more than 0.5 km, update the interceptor's `target_position` and recompute its intercept time. You'll see `[GUIDANCE UPDATE]` log lines when this happens (log statement at `engine.rs:1032`).

The effect: the interceptor flies toward the *best current estimate* of the meeting point, not the possibly-stale launch-time guess. But it never sees the missile's true position — only the radar's story about it. (If no converged trajectory exists, the guidance update is skipped: no sensor data, no correction. Realism rule again.)

---

## Step 5: Terminal Homing — The Seeker Takes Over

In the last 30% of the flight, the interceptor's own **seeker** (its onboard mini-radar) tries to find the target. This is the only phase where the interceptor can see the missile directly — because it's finally close enough for its own sensor, not because the sim is cheating.

### Acquisition: proving you see it

The seeker must pass three tests to "acquire" (`engine.rs:1154-1174`):

1. **Gimbal limit:** the seeker is bolted to the interceptor and can only look so far to the side. The angle between the interceptor's flight direction and the target (the *off-boresight angle*) must be within the gimbal limit — e.g., 20° for SM-3. If the target is out the side window, the seeker physically can't see it.
2. **Range gate:** the target must be within `seeker_range_km × 1.5` (e.g., SM-3's 80 km published seeker range → 120 km limit; the 1.5× margin accounts for closing geometry). Before this gate existed, seekers "acquired" at 190 km — twice the real range.
3. **Acquisition delay:** even once in view, the seeker needs **0.5 s** of continuous tracking to confirm lock — it doesn't snap on instantly.

Lock can also be *lost*: if the target drifts out of the gimbal cone (×1.2 tolerance for hysteresis) or out of seeker range, the seeker drops it.

### Steering laws: PN and Lambert

Once locked, how does the interceptor steer for the kill? Two laws, matched to the flight regime (`engine.rs:1340-1420`):

**Proportional Navigation (PN)** — used endo-atmospherically with seeker lock (`engine.rs:1384-1410`). The idea is elegant: don't chase the target; watch the line between you and it. If that line-of-sight (LOS) is *rotating*, you're on a collision course to miss — so turn to cancel the rotation. The commanded turn rate is:

```
heading_rate = N × LOS_rate        (N = navigation constant, typically 4)
```

The LOS rate is measured every tick (`engine.rs:1425-1442`), and the commanded turn is clamped by the interceptor's maneuvering limit — max g's ÷ velocity, from each system's `terminal_maneuver_g` config (Patriot: 50 g — very agile; SM-3: 25 g).

**Lambert guidance** — used exo-atmospherically (above ~100 km, `engine.rs:1364-1377`), solving the classic astrodynamics problem "given where I am and where I want to be, what constant-gravity arc connects them?" via `solve_lambert()` (`src/simulation/physics.rs:726`).

> **High-school analogy (PN):** You're running to catch a football thrown to you. You don't run at where the ball *is* — you watch it. If the ball seems to slide to the left in your vision, you adjust left to stop the slide. When the ball stops moving in your view, it's coming straight at you. That's PN.

### CPA: knowing when it's over

Hit-to-kill interceptors **cannot turn around** — no fuel, too fast, wrong direction. So the code watches for the *closest point of approach* (CPA): the moment distance-to-target stops shrinking and starts growing. If you've passed CPA and you're not inside the kill radius, you missed — full stop.

To avoid false triggers from momentary wobbles (a sharp PN turn can nudge distance up for a split second), CPA needs **10 consecutive ticks** of increasing distance to confirm (`engine.rs:1303-1325`, `CPA_HYSTERESIS_FRAMES` at line 1316). Once confirmed, the interceptor stops steering (`engine.rs:1354`) — it just flies straight, like a real miss would.

---

## Step 6: Resolution — Hit or Miss?

`resolve_intercepts()` at `engine.rs:2444` runs every tick and decides the outcome. The rules, in order:

| Condition | Outcome | Code |
|---|---|---|
| 3D distance to target ≤ `kill_radius_km` | **HIT** — deterministic, no dice roll | `engine.rs:2653-2668` |
| Passed CPA (confirmed by hysteresis) and outside kill radius | **MISS** (CPA miss) | `engine.rs:2620-2633` |
| In seeker range, ≥90% through planned flight, but not yet in kill radius | Keep homing — still closing | `engine.rs:2670` |
| Still homing but ≥120% of planned flight time elapsed | **MISS** (timeout) | `engine.rs:2682` |
| Not even in seeker range and ≥130% of planned time | **MISS** (way past) | `engine.rs:2697` |

### The kill radius

"Hit-to-kill" means the interceptor's body must physically strike the warhead — at closing speeds of several km/s, a graze destroys both. So kill radii are deliberately tiny, from config:

- SM-3 Block IIA: **0.1 km** (`config/interceptors/sm3_block_iia.toml:51` — "must physically collide ~100 m")
- 40N6: **0.05 km** (fragmentation warhead, ~50 m lethal radius — the exception to hit-to-kill)
- Tamir (Iron Dome): **0.03 km** (proximity-fuzed small warhead)

Because the radius is so small, the sim uses physics **sub-stepping** near intercept: when any interceptor is within 50 km of its target, the engine runs up to 100 tiny physics steps per frame (`config/simulation.toml`, `[physics]` section). At 4 km/s closure, a normal frame is ~67 meters of travel — bigger than the kill radius — so without sub-stepping, intercepts would be missed by timing luck alone.

### Deterministic kills, probability for planning

The hit itself is **deterministic**: inside the kill radius = hit, every time. But a *Probability of Kill (Pk)* is still computed and stored — not to decide this hit, but to guide planning (should we fire a second shot at this target in parallel?). Pk uses a **weighted log-odds model** (`calculate_pk_weighted()` at `engine.rs:170`): seven factors — timing sync, track quality, prediction error, countermeasures, closure speed, aspect angle, energy state — each penalize the base probability by a weighted amount, then the result converts back from log-odds space. The weights live in `config/simulation.toml` under `[pk_weights]` (timing and track quality weighted highest at 2.0 each).

---

## Step 7: Kill Assessment and Follow-Up Shots (Doctrine)

Real defenses never assume a hit. Radar needs time to confirm the target actually vanished. That's modeled by `KillAssessment` (`engine.rs:104-127`): a **3-second delay** (`ASSESSMENT_DELAY_SEC` at line 119) between the intercept event and the verdict.

### Shoot-Look-Shoot vs Shoot-Shoot-Look

The core doctrine question: fire one interceptor, *wait to see if it killed*, then fire another — or fire the whole volley at once?

- **SLS (shoot, look, shoot)** is efficient — don't waste interceptors on a dead target. But it costs time: the missile keeps falling while you wait. It's only chosen when there's time to spare *and* the track is high-confidence (`engine.rs:2060-2075`: total SLS time must fit before impact, and track confidence ≥ `SLS_CONFIDENCE_THRESHOLD` = 0.75 at `engine.rs:2073`).
- **SSL (shoot, shoot, look)** is for low confidence or short timelines: fire the salvo immediately (`engine.rs:2081-2090`, salvo size scaled by confidence, spread by `salvo_delay` seconds).
- After a **confirmed miss** (assessment complete, target still flying), the target is queued for follow-up fire in `resolve_intercepts()` (`engine.rs:2760-2806`) — subject to the same gates as any launch, and the same `max_shots_per_target` cap.

### Why a target can survive all four shots

A target that dodges the geometry (typically a high-crossing-angle shot where the seeker's gimbal can't hold it) will legitimately miss repeatedly. The system fires its shots, misses honestly, and stops at the cap — that's the correct behavior, not a bug. Crossing-geometry misses are the real-world reason some engagements fail even with good interceptors.

---

## The Realism Rules That Shape All of This

From `AGENTS.md`, "Fire Control Architecture" — the invariants an agent (or you) must not break:

1. **Sensor-derived only.** Launch solutions, guidance, doctrine timing, Pk factors, terminal lead — *all* project the target through `DetectionSystem::get_converged_trajectory()`. There is deliberately no ground-truth fallback: if the sensor chain fails, the launch is refused.
2. **One timing function.** `InterceptorKinematics::time_to_cover_distance()` is the single source of truth for arrival-time math. Never reintroduce ad-hoc `distance ÷ max_velocity` shortcuts — that's exactly the bug class that made every engagement miss before the overhaul.
3. **Cumulative drag.** `update_kinematics(dt)` integrates drag from the *previous actual velocity*; `distance_traveled_km` is the true path length.
4. **One CPA tracker.** The guidance-side hysteresis tracker owns `passed_cpa`; `resolve_intercepts` only consumes the flag.
5. **`max_shots_per_target` (4) caps total shots; `salvo_size` (2) is per-salvo only.**
6. **Crossing misses are realistic.** Don't "fix" them with aim-point cheating — that's the omniscience you're supposed to avoid.

---

## Summary Table

| Stage | Question answered | Key code |
|---|---|---|
| Launch decision | Should we shoot? | `engine.rs:1837` `launch_interceptors_at_threats`; quality gates `engine.rs:2280-2300`; closure check `engine.rs:2010-2031` |
| Fire control solution | Where and when do we meet? | `engine.rs:2255` + `engine.rs:2325`; scan `engine.rs:2374-2412` |
| Arrival-time math | How long to get there? | `entities.rs:723` `time_to_cover_distance` (single source of truth) |
| Flight kinematics | Boost, coast, drag | `entities.rs:672` `velocity_at_time`, `entities.rs:992` `update_kinematics`, phases `entities.rs:789` |
| Mid-course guidance | Course corrections | `engine.rs:755` guidance pass, re-solve `engine.rs:875` |
| Seeker acquisition | Can I see it? | `engine.rs:1154-1174` (gimbal + range + delay gates) |
| Terminal steering | How do I steer to it? | PN: `engine.rs:1384-1410`; Lambert: `engine.rs:1364`, `physics.rs:726` |
| CPA detection | Is it over? | `engine.rs:1303-1325` (10-frame hysteresis) |
| Resolution | Hit or miss? | `engine.rs:2444` `resolve_intercepts`; kill radius hit `engine.rs:2653`; timeouts `engine.rs:2682,2697` |
| Kill assessment | Did it die? | `engine.rs:104-127` `KillAssessment` (3 s delay) |
| Doctrine | One at a time or volley? | SLS/SSL choice `engine.rs:2062-2090`; follow-up queue `engine.rs:2760-2806` |
| Pk (planning only) | How good is this shot? | `engine.rs:170` `calculate_pk_weighted`; weights in `config/simulation.toml` |

---

## Why It's Built This Way

- **Gates everywhere:** one radar blip can't trigger a launch. The launch needs a mature track, a feasible geometry, free launchers, and budget remaining. Every gate is a place where the system *refuses to act* — because real defenses are full of "no."
- **The rendezvous, not the chase:** everything is organized around computing a meeting point where two independently-moving objects arrive together. That's the actual physics of ballistic missile defense — you get exactly one pass, at several km/s of closure.
- **Sensor-only aiming:** the interceptor flies toward where *radar says* the missile will be — warts and noise included. When the guidance chain is honest, misses happen for honest reasons (stale tracks, crossing geometry, drag surprises), and hits mean the whole sensor-to-shooter chain genuinely worked.
- **Honest failure:** misses are detected (CPA, timeouts), confirmed (3-second assessment), and acted on (follow-up shots) — but never papered over. A target that survives four good shots gets to land, because that's what the physics said.