# M5 identified planar model (v1, dynamics layer only)

This model realizes the M5 first-round strict identification
(`fossen_quadratic_bounded`, M5Experiment simple delivery 2026-11) on the
real ~5.26 m "_20型船" hull mesh, for use through
`usv_sim_full/launch/m5_dynamics_validation.launch.py`.

## Scope and boundaries (read before reuse)

1. **v1 dynamics entry only.** The normal CCS / `main.launch.py` / Nav2
   chains do NOT use this model yet; no session-manager integration, no
   sensor rig beyond the odometry publisher, no control chain.
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
inside the plugin's tau (static mean flow only; the random gust process of
the original is not implemented and the deployment config uses gain 0).

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
USVWind-compatible (code-level check); all runs use mean wind 0, so
non-zero wind directions are not simulation-validated. With waves on
(`wave_gain 0.3`, company scene default via the world PublisherPlugin):
bounded response (heave p2p 0.22 m, pitch 2°) with ~3 m/min wave drift
inherent to the 4-point Surface model — recorded, not validated.
