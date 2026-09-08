## Platform logic and Intercept geometry

### F2T2EA - Find, Fix, Track, Target, Engage, Assess
This is the kill-chain or workflow every platform adheres to
1. **Find**: An incoming missile has to be detected by a friendly sensor before any interception.
2. **Fix**: A platform uses its own sensors or in conjunction with other friendly sensors to determine its position in space
3. **Track**: A platform uses multiple position fixes from the second step to develop a track of the target's current velocity, vector and trajectory.  Subsequent sensor readings of the target are added to the track to refine the target's track.
4. **Target**: A platform must determine if a target is hostile or not.  If it is determined as hostile, then an intercept solution is started and refined with further sensor focus( often using a targeting radar or mode that gives higher resolution and more frequent information on the target)
5. **Engage**: If an intercept solution is within the envelop of a platform's interceptor, the platform decides on the appropriate single or salvo fire mode
6. **Assess**: If a target has not been destroyed, then a new intercept point should be calculated and another interceptor fired

### Platform logic - Firing logic
- An interceptor should be fired:
  - when the target's trajectory confidence is high
  - when the computed interception point is within the interceptor's performance envelop
  - only inside a TIME-SYNCHRONIZED launch window: the interceptor and the target must arrive at the planned intercept point together (see "Synchronized arrival" below)
- When there will not be time for a follow-up shot, the platform should salvo fire, or shoot-shoot-look
- When there is time for a follow-up shot after an interceptor has confirmed to have missed the target, use a shoot-look-shoot
- When NO time-synchronized solution exists in the interceptor's envelope (e.g., the target is receding and every reachable point on its remaining trajectory is either altitude-illegal or the missile passes it long before/after the interceptor can arrive), fire control HOLDS FIRE. The target is a leaker for that platform — accepted risk, or hand-off to another asset. A chase shot at a receding target is not a valid engagement: ballistic kill vehicles cannot loiter, so "arrive early and wait" is not a real engagement mode.

### Synchronized arrival
- A valid intercept solution requires the interceptor and the missile to arrive at the intercept point at the same moment in time, not merely for the point to be geometrically reachable:
  - Late arrival is forbidden outright (the missile is gone).
  - Early arrival is forbidden beyond tolerance — `max(5 s, 10% of the missile's own travel time to the point)`. The converged trajectory estimate is coarse early in the threat's flight; mid-course guidance can legitimately refine a modestly mistimed point, but an interceptor that beats the missile by minutes can never refine it into a kill.
- Mid-course guidance re-solves must CONVERGE to a synchronized solution to be applied. An unconverged re-solve is discarded — previously the last iteration was applied anyway, deferring the intercept point down-range on every update (a chase pattern), stringing an interceptor along its whole flight toward a point the target reaches long after it.
- ENGAGEMENT BREAK-OFF: once guidance fails to find any synchronized solution for several consecutive update cycles (the geometry is gone — e.g., the target has passed and recedes), fire control commands destruct of the in-flight round (modeled as `SelfDestruct` with `MissReason::BreakOff`, feeding shoot-look-shoot exactly like a miss). Chasing on wastes the round; real doctrine terminates the engagement.

### Post-miss resolution (Assess step)
- Once fire control confirms an interceptor has passed the target's closest point of approach (CPA) outside the kill radius — in ANY flight phase, not just terminal — the round is expended and must not keep flying:
  - Hit-to-kill interceptors (THAAD, PAC-3, SM-3, GBI, Arrow 3, Stunner) cannot turn around; real doctrine is flight termination / command-destruct (FTS) at that point rather than letting a live round fly on uncontrolled. The simulation models this as a transition to `SelfDestruct` with a destruct effect, and records it as a MISS outcome so shoot-look-shoot queues a follow-up.
  - Warhead interceptors (Tamir, 40N6) use proximity fuzes in reality; the sim currently resolves them identically (CPA miss → destruct). A CPA-detonation fragmentation kill model for these types is a possible future realism extension.
- CPA detection uses time-based hysteresis (sole owner: the guidance-side `CpaDivergenceState` tracker):
  - Coast phase: ~1.5x the mid-course guidance update interval — fire control gets one full re-solve opportunity (the intercept point legitimately moves as the track converges) before a miss is declared
  - Terminal phase: ~0.15 s of sustained range opening — prompt resolution
  - Boost phase: no CPA assessment (geometry under thrust is not yet meaningful)
- Resolution timeouts remain as backstops for pathological cases (seeker-range hover at progress ≥ 1.2, definitive far-past at progress ≥ 1.3), but the CPA destruct path is now the primary miss resolution.

### Interception
- A valid intercept point is: 
  - a point in space along the target's trajectory
  - a point in space within the interceptor's performance envelope to reach at the same time the target arrives (time-synchronized, see above)
- A successful interception is defined as the target and interceptor arriving at the intercept point at the same moment in time within a given minimal distance.
- Intercept calculations always use the predicted trajectory from available sensors and not actual or "ground truth" position and trajectory of the target missile
- Fire control radar constantly refines the target's trajectory and the intercept point
- The platform sends mid-course updates to the interceptor when it needs to adjust its vector for a successful interception
- Midcourse updates should be computed by realistic schemes such as Proportional Navigation, Predictive Guidance, or Lambert Guidance
- Prefer Lambert Guidance model for exo-atmospheric interceptions 

