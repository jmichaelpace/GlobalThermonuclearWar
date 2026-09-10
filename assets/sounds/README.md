# Sound Effects

Pre-recorded sound effects for simulation events. The app loads these at
startup from this directory (relative to the working directory); set
`GTW_SOUNDS_DIR` to override the path for packaged builds.

## Files

| File                  | Event                                                       |
|-----------------------|-------------------------------------------------------------|
| missile_launch.wav    | Hostile missile lifts off (Boost phase transition)          |
| interceptor_launch.wav| Defense interceptor launch                                   |
| intercept_hit.wav     | Successful intercept (kill)                                  |
| intercept_miss.wav    | Interceptor missed its target                                |
| self_destruct.wav     | Interceptor command-destruct (FTS doctrine)                  |
| missile_impact.wav    | Hostile missile reaches its target                            |
| decoy_deployed.wav    | Missile deploys decoys                                        |
| threat_warning.wav    | DEFCON escalation klaxon (sensor-derived threat warning)      |

The loader accepts `.wav`, `.ogg`, or `.mp3` for each name (first match wins,
in that order). Missing files are skipped with a warning — the app runs
without them.

## Placeholders

The committed `.wav` files are procedurally generated placeholders
(`generate_placeholders.py`) so the sim has working audio out of the box.
They are intentionally crude. Replace them with licensed/recorded files of
the same names — no code changes needed.

To regenerate placeholders: `python3 assets/sounds/generate_placeholders.py`
(run from the repo root).

## Attribution

When replacing the placeholders, record attribution/license info here.