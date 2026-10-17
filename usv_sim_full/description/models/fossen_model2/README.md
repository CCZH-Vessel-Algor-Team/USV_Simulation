# Fossen model 2 (1.5 m visual, planar dynamics)

This is an opt-in alternative own ship. Existing WAM-V / CCS launches and
parameters are unchanged. Requires Gazebo Harmonic / ROS 2 Humble and the
installed `vrx_gz` plugin `libFossenPlanar.so`.

```bash
ros2 launch usv_sim_full fossen_model2.launch.py
# No velocity controller or navigation, only direct force inputs:
ros2 launch usv_sim_full fossen_model2.launch.py mode:=direct
# Velocity/yaw-rate PID without Nav2:
ros2 launch usv_sim_full fossen_model2.launch.py mode:=pid
```

Default world/map: Sydney regatta / sydney_map2.yaml. Gazebo server and GUI
start separately. `gui:=false rviz:=false` disables visible clients, not
sensor rendering. `sensors:=false` is useful for direct dynamics checks.
No device monitor, gateway, streaming, fusion or automatic encounter stack
is started by this entrypoint.

## Model and coordinates

Reproduces `MarineVesselModels.Fossen.sample_hydro_params_2`:
the reference attributes these coefficients to *Modeling and Experimental
Testing of an Unmanned Surface Vehicle with Rudderless Double Thrusters*.

```
M = diag(50.05, 84.36, 17.21)
D = diag(151.57, 132.5, 34.56)
C = [[0, 0, -m22*v], [0, 0, m11*u], [m22*v, -m11*u, 0]]
dnu = inv(M) * (tau - C*nu - D*nu)
```

M contains the **effective** surge/sway/yaw inertia, not independently known
rigid-body mass and added mass. All three coupling components, including
`(m11-m22)*u*v` in the yaw equation, are included. C is skew-symmetric and
does no work: `nu^T*C*nu = 0`.

Runtime uses ROS/Gazebo ENU/FLU; the reference uses FRD. Body u is unchanged,
v/r reverse signs. Positive Gazebo yaw is counterclockwise. Thus with physical
left/right thrust labels:

```
tau_FLU = [left + right, 0, (right - left) * 0.52 / 2]
```

Larger positive left thrust turns right. Commands are ROS `std_msgs/Float64`
on `/usv_1/thrusters/{left,right}/thrust`, in **newtons**, individually clamped
to [-125,125]. No RPS conversion, motor lag, thrust slew or propeller drag.
Each side has an independent 0.5 simulation-second command watchdog;
non-finite commands become zero. Never run a manual thrust publisher alongside
PID/navigation: select `mode:=direct` first.

The carrier is a single gravity-free rigid link. To avoid double-counting
Gazebo's rigid-body coupling, the plugin computes target dnu above and applies
the equivalent world wrench from body values:

```
force = m_carrier * [du - r*v, dv + r*u, 0]
torque = [0, 0, Izz_carrier*dr]
```

Gazebo/DART integrates the actual rigid body. There are no per-frame pose
or velocity overwrites. No Surface, Hydrodynamics, native fluid-added-mass,
USVWind or Thruster system should be attached in addition to this system.

**Validity:** horizontal, contact-free trajectories with direct thrust,
zero initial heave/roll/pitch and no external environment forces. Contact
impulses and other external wrenches use carrier inertia, not an identified
added-mass impact model. Six-DOF seakeeping and collision hydrodynamics are
not reproduced. Physics integration has timestep-dependent discretization
error; default step is 2 ms, not the reference Python RK4 integrator.

## Geometry and observations

- Existing M5 visual scaled isotropically to 1.5 m length (~0.746 m width),
  with its horizontal center corrected. This is an approximate visual,
  not the identified boat's CAD. Collision box: 1.5 x 0.75 x 0.25 m.
- The 0.52 m thruster spacing is analytical and independent of visual width.
- One 360-degree 720-ray lidar at z=0.65 m, 10 Hz, range 0.15–60 m:
  `/usv_1/sensors/lidar/front_lidar/scan` (`LaserScan`).
- One forward 640 x 480 camera at x=0.35,z=0.55 m, 10 Hz, 80-degree FOV:
  `/usv_1/sensors/camera/front_cam/image_raw` (`Image`).
- Gazebo ground-truth `/usv_1/odom` drives the odom/base TF and controller
  feedback; map/odom is identity. This is not a localization-noise simulation.
- Gazebo-only `/usv_1/fossen/state` (`gz.msgs.Double_V`) reports measured
  `t,x,y,yaw,u,v,r,du,dv,dr,left,right,nu^T*C*nu` each physics update.

## Control and navigation

Dedicated `config/fossen_model2_control.yaml` configures the existing PID and
night ALOS plugin; no Nav2 algorithm source changes are needed. Override with
`control_params_file:=<path>`.

Initial profile: 1 m/s cruise, 0.3 m/s turning fallback, 2 m lookahead,
0.7 rad/s guidance yaw limit and 0.5 rad/s² angular-acceleration envelope.
Maximum straight equilibrium speed at full
thrust is only 250/151.57 ~= 1.649 m/s; the large-vessel 3 m/s cruise is
infeasible. The PID produces direct newtons and enforces the same per-side
125 N limit as the dynamics plugin.

This basic navigation entrypoint uses the existing NavFn planner and ALOS,
small-vessel footprint, lidar obstacle layer and Sydney static map. It does
not claim COLREGS encounter certification. Nav2 Goal and the Navigation 2
panel are provided by `rviz/fossen_model2.rviz`.
An orange, broad-headed Own Ship arrow displays the latest odometry position
and heading (one marker only, no trail). It is a heading indicator, not a
collision footprint, and requires no additional publisher or sensor.
The panel's localization status can remain inactive because this profile uses
ground-truth odometry and identity map/odom, not AMCL or a localization manager.
