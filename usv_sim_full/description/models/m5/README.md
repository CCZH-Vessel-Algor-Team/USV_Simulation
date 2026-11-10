# M5 identified planar model

This model realizes the M5 first-round strict identification
(`fossen_quadratic_bounded`, M5Experiment simple delivery 2026-11) on the
real ~5.26 m "_20型船" hull mesh. The v1 dynamics entry is
`usv_sim_full/launch/m5_dynamics_validation.launch.py`; the v2 full-stack
entry and its model-specific configuration are installed with this package.

## Full-stack entry (v2)

With a compatible `usv_bringup` and its navigation/perception dependencies
installed (the integration baseline uses bringup develop `32618635`):

```bash
ros2 launch usv_sim_full usv_bringup_system_m5.launch.py
```

This selects `config/system/m5_sim.yaml`, the M5 scene, the `power` bridge,
and the kW PID profile together. It does not depend on task-local YAML or
Docker-specific paths. The original `usv_bringup system.launch.py` default
and the legacy WAM-V profiles are unchanged; use the M5 entry explicitly.
Communication and monitoring are disabled in this simulation profile.
The M5 scene explicitly excludes the unmatched `env_panel/VirtualShipPoint`
tool from the runtime RViz copy: this software combination has no consumer
for `/dynamic_ship/virtual_clicked_point`. Actual Gazebo targets still use
**Publish Point** (`/clicked_point`, panel heading) or **2D Pose Estimate**
(`/dynamic_ship/spawn_pose`, drag heading). Nav2 Goal is unchanged. This is
not an implementation of the separate virtual-target feature.
The v2 navigation/recovery acceptance remains separate from the v1 dynamics
validation below. Spin tuning/behaviour is deferred; no Spin acceptance is
claimed.

The current navigation profile uses the inherited position-based goal
checker (1.5 m XY tolerance), not a stationary-arrival criterion. An earlier
30 m test using the initial 1.5 m/s profile returned `SUCCEEDED` at 1.43 m
error while surge was still 1.62 m/s;
three wall seconds later surge was 1.28 m/s despite zero power. The gated
controller deliberately resets and commands zero power on a zero Twist or
fault; it does not promise active braking/position hold. Goal completion
and cancel-to-zero wiring are therefore not evidence of a stopped vessel.
Approach-speed planning and stationary-arrival/braking policy need separate
work; this integration does not silently change fail-closed stop semantics.

### Fast-response simulation profile

The current M5 profile requests **3.0 m/s cruise**. Full-sensor, calm-water
measurements at 20 kW per side gave 3.96145 m/s steady surge (14.261 km/h,
7.700 kn), with 2773.67 N per side from the chart endpoint. The often-used
"2 x 2800 N" is a rounded nominal figure; the power chart/model is unchanged.
Peak forward acceleration from rest was about 2.412 m/s².

`m5_control.yaml` uses speed PI 60/100 and yaw PI 180/350, with both D terms
zero. Direct velocity-loop step measurements (10-90% rise time) were:

| step | initial PI | current PI | current overshoot |
|---|---:|---:|---:|
| 0 -> 1.5 m/s | 2.236 s | 0.524 s | 1.54% |
| 1.5 -> 3.0 m/s | 13.600 s | 0.776 s | negligible |
| yaw 0 -> 0.08 rad/s at 1.5 m/s | 6.724 s | 0.368 s | 5.55% |

3.0 m/s with a sustained 0.06 rad/s turn was also tracked. These are
simulation response results, not hardware controller certification.
The 2 kW slew parameter is per control update, not kW/s; safety zero bypasses
it. Power remains limited to +/-20 kW per side, with reverse scaling only
in FossenPlanar.

The ALOS/smoother profile uses 2.0 m/s² linear reference limits, 0.08 rad/s
yaw reference cap, 0.3 rad/s² yaw reference limits, 10 m lookahead and 20 m
closest-point search. At 3 m/s the steady planar yaw capability is about
0.083 rad/s; pushing cruise near 3.96 m/s leaves almost no steering reserve.
The downstream smoother also caps the linear reference at 3.0 m/s, including
the inherited distance-based final-carrot speed branch. Reference limits
are not guarantees of instantaneous physical acceleration or stopped arrival.

A full-stack goal with an 80 m / 20 m offset returned `SUCCEEDED` at this
3.0 m/s cruise setting. Over the steady window (more than 15 m traveled and
more than 12 m remaining), surge was 3.0002 +/- 0.0042 m/s, with no side at
the power limit. Maximum surge during the run was 3.0894 m/s; cross-track
RMS/max were 0.871/1.489 m. This checks one clear-water turning/straight
route, not every path, encounter or sea condition. Stationary arrival and
Spin were explicitly excluded.

### Sensor mounts and mesh alignment

The legacy WAM-V template rendered the same DAE without a visual origin.
The M5 template translates the mesh by `(-0.0995, +0.029, +0.472) m` to
align the hull centre and keel. The M5 scene applies that **same translation
to all ten sensor mounts**, preserving the legacy sensor-to-hull geometry:

`inverse(T_mesh_new) * T_sensor_new = inverse(T_mesh_old) * T_sensor_old`.

In particular, camera height is `2.272 m`, not the old `1.8 m` relative to
the redefined hull origin. This places each optical origin about 7.5 cm
above the mesh's highest point. Frame names, orientations and topic names
are unchanged. Camera images and GPU-ray measurements depend on their
mounts; they are not merely frame definitions. These are legacy assumed
installation positions, not independently surveyed M5 sensor calibration.

## Scope and boundaries (read before reuse)

1. **Validation scope.** The v1 SDF is an isolated dynamics model. The v2
   M5 entry adds the company sensor/control/navigation chain, but does not
   establish real-vessel, collision or seakeeping accuracy. Ordinary CCS /
   `main.launch.py` defaults still select the legacy model.
2. **Conditional identification, not absolute truth.** M/C/D are the
   first-round strict fit under its recorded conditions (forward or trusted
   zero power per side, 0-20 kW, INS-referenced states). They are not
   independently calibrated absolute hydrodynamic coefficients.
3. **State reference point mismatch is unresolved.** The identification
   treated the INS output point as the state origin; this model uses a
   geometrically centred origin with the keel at z=0. The lever arm between
   the two has not been surveyed and is not compensated.
4. **Seakeeping proxies.** `hull_length=28`, the carrier inertia
   diag(1800, 3200, 3200) and the vertical damping (zW/kP/mQ) exist to make
   the 1800 kg carrier float level and settle; they are NOT identified M5
   6-DOF properties.
   The full sensor rig adds mass and includes a rotating radar joint, so
   v1's single-carrier precision is not automatically a full-rig guarantee.
   A separate full-rig straight/differential/coasting capture (26.02 s of
   powered/coasting comparison, 4 ms world step) gave maximum position
   difference 0.01955 m and yaw difference 0.000154 rad against the unchanged
   v1 RK4 equations using recorded input timing. This includes integration
   and attached-rig effects; it is not an independent vessel-accuracy test
   or proof of exact equivalence in every configuration.
5. **"Feedback-exact" is limited to calm-water planar 3-DOF.** With
   roll/pitch/heave motion (waves on) there are 3-D rotation couplings and
   further external forces; the model-2 horizontal contact-free backing
   does not transfer to that regime. Validation numbers below are
   calm-water only.
6. **Reverse thrust is an assumption.** The manufacturer chart has no
   reverse data; `reverse_scale=0.5` is an explicit ePropulsion-typical
   placeholder awaiting real reverse calibration.
7. **Power mapping is linear interpolation** over the 43 digitized knots
   (RMS vs chart centerline ~1.65 N, on par with the PCHIP refit at this
   knot density; max deviation vs PCHIP ~7 N below ~0.1 kW).

## Channel ownership

| Channel | Owner | Note |
|---|---|---|
| surge/sway/yaw dynamics | `vrx::FossenPlanar` | identified M, C(M), linear sway/yaw damping, thruster generalized force, still-air wind; exact via the identified M in calm water |
| quadratic surge damping | stock gz `Hydrodynamics` | `xUabsU = 353.20671159902133 × 1800/2299.1122 = 276.5293818811`; the carrier mass (1800 kg) divides this external force, so the pre-scaled coefficient makes the total surge dynamics equal the identified model |
| heave/roll/pitch | physics + `vrx::Surface` + Hydrodynamics zW/kP/mQ | physical proxies, see boundary 4 |

Telemetry `/usv_1/fossen/state` (21 fields): `t,x,y,yaw,u,v,r,
du,dv,dr (plugin-internal derivatives, excluding other systems' forces),
TL,TR, nu^T C nu, PL_kW,PR_kW, windFx,windFy,windN (body),
duA,dvA,drA (finite-difference actual derivatives)`. None of the
derivative fields feed back into the applied wrench.

`wind_direction` keeps the `vrx::USVWind` unit convention (degrees,
0 = +X, CCW positive); the wind model itself is the USVWind algebra applied
inside the plugin's tau. The runtime world-frame Vector3d command on
`/vrx/wind/velocity_cmd` overrides the configured mean flow, preserving the
environment-panel input. Non-finite commands and nonzero z are ignored.
The original random gust process is not implemented. Nonzero runtime wind
acceptance is separate from the calm-water checks below.

## Input

Per-side electrical power in kW on
`/usv_1/thrusters/{left,right}/power` (gz Double; ROS Float64 via the
validation launch bridge). `|P|` clamps to `max_power` (20 kW), thrust
clamps to `max_thrust` (2773.67 N); non-finite commands fail closed to 0;
per-side 0.5 s command timeout zeroes that side.

## Validation (2026-11-09, calm water, wave_gain 0)

RK4 reference (task-local, full-precision coefficients, dt = world step
4 ms), power-onset time alignment, per-case container restart hygiene:

| case | max x | max y | max yaw | max u |
|---|---|---|---|---|
| both 5 kW, 30 s | 8.0 mm | 0 | 0 | 1.0 mm/s |
| both 20 kW, 60 s | 12.8 mm | 0 | 0 | 2.6 mm/s |
| diff 2/8 kW, 40 s | 7.4 mm | 13.2 mm | 0.2 mrad | 0.9 mm/s |
| reverse −5 kW, 30 s | 5.7 mm | 0 | 0 | 0.5 mm/s |
| steps 5/12/3 kW, 55 s | 8.0 mm | 0 | 0 | 11.7 mm/s |
| off-knot 3.7 kW, 30 s | 7.3 mm | 0 | 0 | 0.8 mm/s |
| over-range 25 kW, 30 s | 12.8 mm | 0 | 0 | 2.6 mm/s |

The 25 kW case overlays the 20 kW trajectory exactly (max_power clamp).

Behaviour checks: the power schedule 6 kW -> Inf -> 6 kW -> NaN ensures
EACH non-finite command is applied from its own live commanded state (the
re-established 6 kW rules out "ignore and keep the old, already-zero
command") and must fail closed within a 0.3 s deadline, well inside the
0.5 s watchdog so timeout-zeroing cannot masquerade as fail-closed; a
held side must zero within 0.7 s of the schedule-known hold time (0.5 s
watchdog + publish margin) while the other side keeps its commanded thrust
and the boat turns to starboard (FLU r < 0, stronger left thruster); a
world reset issued with both sides still commanding 5 kW must clear both
immediately — this also covers the sim-time regression defeating the
watchdog arithmetic (stale thrust would otherwise persist ~10 s).
Offline fault injection additionally verifies the orchestrator contract:
body, cleanup or restart failures each fail the case while cleanup and
the post-experiment restart are still attempted.

Floating equilibrium: z = −0.2813 m, zero oscillation, zero drift;
buoyancy-point depth 0.181 m matches the analytic sizing. Wind units are
USVWind-compatible. The v1 tests use mean wind 0; a separate v2 runtime
check observed matching Fossen wind-force/torque samples for world-frame
commands (2,0), (0,2), (2,2) m/s and return to zero. This checks command
wiring/signs; it is not a gust or environmental-model calibration. With waves on
(`wave_gain 0.3`, company scene default via the world PublisherPlugin):
bounded response (heave p2p 0.22 m, pitch 2°) with ~3 m/min wave drift
inherent to the 4-point Surface model — recorded, not validated.
