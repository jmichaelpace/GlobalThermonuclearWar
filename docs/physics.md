## Physics ##

### Earth Model

The simulation uses the **WGS-84 ellipsoid** (NIMA TR 8350.2, 3rd ed., 2000):

| Constant | Value | Symbol |
|---|---|---|
| Semi-major axis (equatorial radius) | 6378.137 km | a |
| Flattening | 1/298.257223563 | f |
| Semi-minor axis (polar radius) | 6356.752 km | b |
| First eccentricity squared | 0.00669438 | e² |
| Mean radius (IUGG, (2a+b)/3) | 6371.0088 km | R1 |

Distances, azimuths, and dead reckoning are **geodesic** (Vincenty 1975,
"Direct and Inverse Solutions of Geodesics on the Ellipsoid with Application
of Nested Equations", Survey Review 23(176), pp. 88–93):
- `haversine_distance` (name kept for API compatibility) is the Vincenty
  inverse solution; differences from the legacy R=6371 sphere reach ~0.5%
  on long legs depending on azimuth and latitude.
- `calculate_position_from_bearing_range` is the Vincenty direct solution —
  all interceptor dead reckoning, terminal lead, and track projection stay
  round-trip-consistent with the distance/azimuth functions.
- `BallisticTrajectory` caches the origin azimuth at construction; each
  `position_at` lookup is a single direct call.
- Near-antipodal Vincenty non-convergence (not reached at this sim's
  geometries, < 8000 km legs) falls back to a mean-radius great circle.

Geodetic ↔ ECEF conversion is ellipsoidal; the ECEF→geodetic inverse uses
Bowring's non-iterative method (Bowring 1976, sub-millimeter accuracy at
terrestrial and low-orbit altitudes).

The EKF and linear Kalman filter use the WGS-84 curvature radii:
- Meridional M(φ) = a(1−e²)/(1−e²sin²φ)^{3/2} for north-south motion
- Prime vertical N(φ) = a/(1−e²sin²φ)^{1/2} for east-west motion
The EKF state-transition Jacobian includes the d(1/M)/dφ and d(1/N)/dφ
latitude derivatives.

Radar-horizon and line-of-sight geometry use the IUGG mean radius (the
ellipsoidal difference in horizon dip is < 0.3%, below the granularity of
those visibility gates). Map display (Web Mercator viewport, globe
projection, km/degree UI constants) is display-only and stays spherical.

### Gravity

Standard gravity is **9.80665 m/s²** (BIPM standard gravity, 1901),
expressed as 0.00980665 km/s² (shared `physics::G0`). Altitude correction
is inverse-square: g(h) = g₀·(R1/(R1+h))².

### Ballistic Trajectories

- Apogee from burnout conditions via energy conservation (specific orbital
  energy E = v²/2 − μ/r; μ = 398600.4418 km³/s², the WGS-84 gravitational
  parameter). Reference: Bate/Mueller/White, "Fundamentals of Astrodynamics".
- Ground track: WGS-84 geodesic between origin and target.
- Altitude profile: parabolic h(t) = 4·H·t·(1−t).
- Flight phase boundaries: boost < 10% progress, midcourse < 85%, terminal
  thereafter.

### Atmospheric Drag

1976 US Standard Atmosphere (NASA-TM-X-74335), piecewise density model below
100 km; zero density above the Kármán line. Drag deceleration
a = ρv²/(2β) with published ballistic coefficients per vehicle class
(ICBM RV β ≈ 20,000 kg/m², MRBM RV ≈ 6,667, SRBM ≈ 3,333, exo interceptor
KV ≈ 7,000, endo KV ≈ 4,167).

### Lambert Guidance

Universal-variable Lambert solver (Vallado, "Fundamentals of Astrodynamics
and Applications") for exo-atmospheric intercepts above 100 km. Known
limitation: the solver treats ECEF as inertial (no Earth rotation); the
effect is small for interceptor flight times (30–170 s) but is tracked in
audit-plan.md for a future ECI+GMST pass.