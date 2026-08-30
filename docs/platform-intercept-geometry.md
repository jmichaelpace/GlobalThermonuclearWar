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
- When there will not be time for a follow-up shot, the platform should salvo fire, or shoot-shoot-look
- When there is time for a follow-up shot after an interceptor has confirmed to have missed the target, use a shoot-look-shoot

### Interception
- A valid intercept point is: 
  - a point in space along the target's trajectory
  - a point in space within the interceptor's performance envelope to reach at the same time the target arrives
- A successful interception is defined as the target and interceptor arriving at the intercept point at the same moment in time within a given minimal distance.
- Intercept calculations always use the predicted trajectory from available sensors and not actual or "ground truth" position and trajectory of the target missile
- Fire control radar constantly refines the target's trajectory and the intercept point
- The platform sends mid-course updates to the interceptor when it needs to adjust its vector for a successful interception
- Midcourse updates should be computed by realistic schemes such as Proportional Navigation, Predictive Guidance, or Lambert Guidance
- Prefer Lambert Guidance model for exo-atmospheric interceptions 

