# ur5_real_ws — PLAN: Real UR5 Arm Control (endo_flex present but disabled)

> **Status:** PLAN (2026-10-23). Phases 2 & 3 have since been **implemented and validated
> in mock hardware** (no robot attached); Phase 0.6 calibration extracted — see §11.
> Remaining manual steps: pendant Host-IP check (§5 Phase 0.2) and the real-robot
> bring-up (§5 Phase 4 / §6 runbook). `ur5_sim_ws` remains untouched throughout.

---

## 1. Goal

Port the UR5 **arm** control stack from `ur5_sim_ws` (Gazebo / ROS 2 Jazzy) to **real-hardware
control** on this Jetson AGX Orin, driving a real **UR5 (CB3, PolyScope 3.15)** through the
official `ur_robot_driver` + **ExternalControl URCap**, with `external_control.urp` running on
the pendant.

1. Real control scripts at `./ur5_real_ws`, functionally equivalent to the sim stack
   (press-and-hold teleop, semi-auto poses, GUI panel, home pose).
2. **Real-time robot-state sync**: scripts and visualization driven live by the real robot
   (`/joint_states` @ 125 Hz from RTDE, TF `base → tool0`).
3. **Arm only**: endo_flex stays in the workspace with its code and URDF **active but
   actuation-less** — the `/flex/*` command topics keep publishing exactly as in sim, but on
   real hardware they have no consumer (no Gazebo bridge/plugins), so only the arm is
   physically controlled. (Owner decisions 2026-10-23, see §10: Q1 = RViz live viz, Q2 = code active, no target.)

## 2. Verified starting point (2026-10-23)

### 2.1 Environment (checked on this machine)

| Item | Value |
|---|---|
| PC | Jetson AGX Orin, Ubuntu 24.04, ROS 2 **Jazzy** (`/opt/ros/jazzy`) |
| UR driver stack | **Already installed (apt)**: `ros-jazzy-ur` **3.8.0** — incl. `ur_robot_driver`, `ur_description` 3.5.1, `ur_calibration`, `ur_controllers` (candidate 3.9.0; upstream jazzy is 3.10.x — upgrade optional, not required) |
| ExternalControl URCap | Available locally: `/opt/ros/jazzy/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap` (newer `1.0.6` exists on GitHub Releases) |
| Wired link | `end0` = **192.168.1.100/24** (static); robot `192.168.1.10` **ping OK**, 0.5 ms RTT |
| Other NICs | wlan (192.168.31.x), docker0, l4tbr0 — irrelevant: all ROS nodes run on this PC; Jazzy default discovery = localhost. Robot speaks raw TCP, not DDS. |
| Driver launch interface (verified via `--show-args` on installed 3.8.0) | `ur_control.launch.py` accepts `ur_type`, `robot_ip`, `description_launchfile`, `initial_joint_controller`, `launch_rviz`, `launch_dashboard_client`, `headless_mode`, `use_mock_hardware`, ports…; `ur_rsp.launch.py` (the default `description_launchfile`) declares `description_file`, `kinematics_params_file`, etc. |
| Default motion controller | `scaled_joint_trajectory_controller` — **same controller name/type the sim already commands** |
| Convenience launch | `ros2 launch ur_robot_driver ur5.launch.py` (preset of `ur_control.launch.py`) |

### 2.2 Robot / pendant

- UR5 CB3, PolyScope 3.15 (URCap minimum 3.14.3 — satisfied; 3.15.8 is the last CB3 release).
- `external_control.urp` with one External Control node exists in PolyScope (per your setup).
- CB3 specifics: keep `use_currents_as_efforts` at default `true`; RTDE update rate 125 Hz;
  no "remote control mode" needed (CB3); dashboard server available; headless mode works on CB3.

### 2.3 What the sim stack actually does (analysis of `ur5_sim_ws/src/ur5_endo_flex`)

```
Tk GUI ──(in-process)──► Ur5EndoFlexNode (robot_node.py)
   20–30 Hz tick: press-and-hold deltas → KDL IK (linear / RCM modes) → 2-point JointTrajectory
   ├──► /scaled_joint_trajectory_controller/joint_trajectory   (ARM: topic-streamed, 6 UR joints)   [portable]
   ├──► /flex/{base_yaw,ext_yaw,ext_pitch,ext_yaw_back} (Float64) ─► ros_gz_bridge ─► gz plugins    [sim-only]
   ├──► /joint_states (synthetic 29 flex joints → RViz)                                            [sim-only]
   ◄──┼── /joint_states (arm joints — real feedback path)                                          [portable]
   ◄──┼── /robot_description (TRANSIENT_LOCAL → KDL chain)                                         [portable]
```

Portability verdicts:
- **Arm path is robot-agnostic**: exact UR joint names, `JointTrajectory` topic, feedback via
  `/joint_states` + `/robot_description`, no MoveIt / actions / Gazebo calls. The real driver
  runs the *same* controller implementation (`ur_controllers/ScaledJointTrajectoryController`)
  with the *same* topic interface (`~/joint_trajectory`) — the sim's command primitive carries
  over directly.
- **Flex actuation is Gazebo-only**: 4 open-loop `Float64` setpoints → gz `JointPositionController`
  plugins; flex joint states are synthetic/open-loop. Dead on real hardware → gated off.
- Sim's controller config has `goal_time: 0.0`, `allow_partial_joints_goal: false`; the real
  driver's `ur_controllers.yaml` (3.8.0, verified) has the same semantics (goal_time 0.0,
  partial goals disallowed, per-joint tolerance trajectory 0.2 / goal 0.1) → streaming 2-point
  trajectories behaves equivalently.

## 3. Component disposition (sim → real)

| Sim component | Source | Real disposition |
|---|---|---|
| `scripts/kinematics.py` (pure KDL/numpy) | `ur5_endo_flex` | **Copy unchanged** (rig constants re-verified vs hardware, §8) |
| `scripts/robot_node.py` arm core (IK/RCM/teleop/semi-auto/tracking) | `ur5_endo_flex` | **Port** (edit list in §5 Phase 3) |
| `scripts/gui_panel.py` (Tk, embeds node in-process) | `ur5_endo_flex` | **Port** (flex row + STOP hardening) |
| `scripts/home_pose_publisher.py` | `ur5_endo_flex` | **Port** (QoS + duration fix) |
| `scripts/scene_setup.py` markers/targets (`base_link` frame) | `ur5_endo_flex` | Deferred — not launched initially; optional later without gz parts |
| `urdf/endo_flex_flexbase.xacro` (flex chain + gz tags) | `ur5_endo_flex` | **Keep in the URDF** — gz tags inert on real; flex visible in RViz via synthetic joint states (setpoints only, no bridge) |
| `ur5_endoscope` package (endoscope link/mesh, `tool0_to_endoscope_joint`) | `ur5_endoscope` | **Vendor into `ur5_real_ws` unchanged** (keeps `package://ur5_endoscope` mesh URIs + single-sourced mount transform) |
| `endo_flex_control` package (flex teleop, pure Python) | `endo_flex_control` | Vendor, **not launched** (flex disabled) |
| gz world/supporter scaffold, `<ros2_control>` gz block, gz plugins/sensors/camera | `ur5_endo_flex.urdf.xacro` | **Drop** — replaced by the driver's `ur_robot_driver/urdf/ur.ros2_control.xacro` |
| `ros_gz_bridge` nodes, `GZ_SIM_RESOURCE_PATH`, PYTHONPATH hack, `model.config` shims | `ur5_sim.launch.py` | **Drop** (PYTHONPATH pattern may be kept short-term for script imports) |
| `use_sim_time:=true` (5 places) | `ur5_sim.launch.py` | `false` everywhere |
| `/endoscope_camera/image_raw` (gz sensor) | gz | Absent on real (no camera driver yet) — RViz Image display disabled; real endoscope camera = future work |

## 4. Target architecture

### 4.1 Workspace layout

```
ur5_real_ws/
├── PLAN.md                        (this file)
└── src/
    ├── ur5_endo_flex/             # ported integration package (same name → minimal diffs)
    │   ├── urdf/
    │   │   ├── ur5_endo_flex_real.xacro   # ur5 (driver ros2_control) + endoscope + optional flex
    │   │   └── endo_flex_flexbase.xacro   # copied from sim (gz tags inert without Gazebo)
    │   ├── launch/
    │   │   ├── ur5_real.launch.py         # MAIN entry point
    │   │   └── ur5_real_rsp.launch.py     # copy of driver's ur_rsp.launch.py, our xacro default
    │   ├── scripts/                       # ported robot_node / kinematics / gui_panel / home_pose_publisher
    │   └── config/
    │       ├── view.rviz                  # adapted (world frame kept via static TF)
    │       └── ur5_calibration.yaml       # from Phase 0.6 extraction
    ├── ur5_endoscope/             # vendored unchanged (endoscope mesh/link/mount)
    └── endo_flex_control/         # vendored, NOT launched (flex disabled)
```

### 4.2 Real command / state flow

```
Tk GUI ──► Ur5EndoFlexNode ──► /scaled_joint_trajectory_controller/joint_trajectory (topic, 20–30 Hz stream)
                                    └─► ur_robot_driver: ScaledJointTrajectoryController
                                          └─► reverse interface (robot→PC :50001) ─► CB3 servo @ 125 Hz
/joint_states (125 Hz, joint_state_broadcaster ← RTDE) ─► node feedback + RViz RobotModel/TF   ← real-time sync
/robot_description (driver's RSP, our xacro + calibration) ─► KDL chain in node
/dashboard_client services (PC→robot :29999) ─► play / pause / stop / unlock_protective_stop / load_program
TF base → tool0 (RSP) — live arm pose;   world → base_link via static TF (supporter height 0.80 m)
```

### 4.3 Design decisions

- **D1 — Command interface:** keep the sim's primitive: 2-point `JointTrajectory` streamed on
  `/scaled_joint_trajectory_controller/joint_trajectory`. Identical controller name/type/topic
  as the sim; `goal_time: 0.0` in the real config means the streamed goals don't time out.
  New message replaces the active goal (ros2_controllers JTC topic semantics) — proven in sim.
  Optional Phase 5: migrate to `FollowJointTrajectory` **action** for goal status/feedback.
- **D2 — Custom description:** base our `ur5_endo_flex_real.xacro` on the driver's own
  `ur_robot_driver/urdf/ur.urdf.xacro` (which already includes `ur_description/ur_macro.xacro`,
  the real-hardware `ur.ros2_control.xacro` with `robot_ip`/ports/headless args, and
  transmissions), then attach the vendored endoscope at `tool0` and the flex chain behind
  `<xacro:if value="$(arg enable_flex)">` (default **true**). Kinematics = our extracted
  calibration YAML.
- **D3 — Launch wiring:** our `ur5_real_rsp.launch.py` = copy of the installed
  `ur_rsp.launch.py` with `description_file` → our xacro, `kinematics_params_file` →
  `config/ur5_calibration.yaml`. Main launch includes `ur_control.launch.py` with
  `description_launchfile:=` ours. (In driver 3.8.0, `ur_control.launch.py` does **not**
  take `kinematics_params_file` directly — it flows through the rsp launch.)
- **D4 — endo_flex active-but-inert (owner decision Q2):** the flex chain stays in the URDF and
  all flex code paths keep running exactly as in sim — the 4 `/flex/*` `Float64` publishers and
  the synthetic flex `/joint_states` stay active, so RViz shows the *commanded/assumed* flex pose
  (open-loop — same caveat as sim). On real hardware those command topics have **no consumer**
  (ros_gz_bridge and gz plugins are gone), so nothing physical actuates. An `enable_flex`
  kill-switch (default **true**) remains available to silence the inert topics; a real tendon
  actuator interface consuming `/flex/*` is future work.
- **D5 — use_sim_time:=false everywhere**; no Gazebo, no `ros_gz_bridge` in the real launch.
- **D6 — world frame:** static TF publisher `world → base_link` (z = 0.80 m, matching the sim's
  supporter) so `view.rviz` (fixed frame `world`) works unchanged and coordinate habits carry
  over. Scene markers (future) stay in `base_link`.
- **D7 — Safety hooks:** GUI STOP button additionally calls `/dashboard_client/stop`
  (`std_srvs/Trigger`); pendant E-stop remains primary. Documented recovery:
  `unlock_protective_stop`, `play`, or press Play on pendant after program interruption.
- **D8 — Home pose:** `home_pose_publisher` ports with QoS unified to RELIABLE (sim had
  BEST_EFFORT/RELIABLE mismatch) and a conservative default duration (≥ 5 s) for first runs.
  `HOME_DEG = [0, -120, -120, -30, 90, 45]` + wrist_1 −270° rule unchanged; must be
  collision-checked on the real rig with endoscope mounted before first execution.
- **D9 — Workspace hygiene:** never source `ur5_sim_ws` and `ur5_real_ws` in the same shell
  (both contain a package named `ur5_endo_flex`); never run the sim's gz driver and the real
  driver simultaneously (ports 50001–50004 + single-RTDE-client rule).
- **D10 — Mock-first validation:** the driver's `use_mock_hardware:=true` lets the entire
  ported stack be validated on this PC with no robot attached (commands mirror to states).

## 5. Implementation phases

### Phase 0 — One-time setup (pendant + Jetson) — *manual*

| # | Step | Detail / command |
|---|---|---|
| 0.1 | Install URCap on pendant | Copy `/opt/ros/jazzy/share/ur_robot_driver/resources/externalcontrol-1.0.5.urcap` (or v1.0.6 from GitHub Releases) via USB → pendant: *Setup Robot → URCaps → + → select → restart* |
| 0.2 | Configure URCap installation | *Program Robot → Installation → External Control*: **Host IP = 192.168.1.100**, **Custom port = 50002** (leave default) |
| 0.3 | Verify program | `external_control.urp` contains one External Control node; opens/loads without error (already in place per your setup) |
| 0.4 | Firewall on Jetson | `sudo ufw status` — if active: `sudo ufw allow from 192.168.1.10 to any port 50001:50004 proto tcp` (robot dials **into** the PC on 50001–50004) |
| 0.5 | Network sanity | `ip addr show end0` (192.168.1.100/24), `ping -c 3 192.168.1.10` — both already verified 2026-10-23 |
| 0.6 | Extract calibration | `ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.10 target_filename:=<ur5_real_ws>/src/ur5_endo_flex/config/ur5_calibration.yaml` (robot powered, idle) |

**Acceptance:** URCap listed active; program loads; ping OK; firewall rule (or inactive);
`ur5_calibration.yaml` exists with `hash: calib_…`.

### Phase 1 — Stock-driver bring-up validation (no custom code)

1. `ros2 launch ur_robot_driver ur5.launch.py robot_ip:=192.168.1.10` (default description,
   RViz on, dashboard on).
2. **Order matters: driver first, then pendant ▶ Play** on `external_control.urp`.
   Expect log: `Robot connected to reverse interface. Ready to receive control commands.`
3. Verify: `ros2 control list_controllers` → `scaled_joint_trajectory_controller [active]`;
   `ros2 topic hz /joint_states` ≈ 125 Hz; TF `base → tool0` in RViz moves with the real arm
   (move it by hand in freedrive/normal pendant mode? — no: pendant interaction drops the
   connection; just observe TF matches the physical pose).
4. Calibration check in driver log: `Calibration checked successfully` (rerun 0.6 if mismatch
   error appears).
5. **Sanity motion** (space cleared, pendant speed slider ≤ 25 %, thumb on E-stop):
   `ros2 run ur_robot_driver example_move.py` (sends `[0, -1.57, 0, -1.57, 0, 0]`; ensure the
   path from the current pose is collision-free) or
   `ros2 launch ur_robot_driver test_scaled_joint_trajectory_controller.launch.py`
   (only if the arm is already near its start pose).
6. **Stop drill:** `ros2 service call /dashboard_client/stop std_srvs/srv/Trigger {}` → arm
   stops, driver logs `Connection to reverse interface dropped` → resume with pendant Play.

**Acceptance:** all six checks pass. Any protective stop / E-stop recovery rehearsed once.

### Phase 2 — `ur5_real_ws` scaffold

1. Vendor packages: copy `ur5_endoscope` and `endo_flex_control` unchanged; copy `ur5_endo_flex`
   → strip to the layout in §4.1 (scripts copied, sim launch/URDF not reused as-is).
2. Write `urdf/ur5_endo_flex_real.xacro` (base: `ur_robot_driver/urdf/ur.urdf.xacro` structure;
   append vendored endoscope at `tool0`; flex chain included as in sim, `enable_flex` default **true**).
3. Write `launch/ur5_real_rsp.launch.py` (copy of installed `ur_rsp.launch.py`; defaults:
   our xacro + our calibration YAML; passes through `robot_ip`, `headless_mode`,
   `use_mock_hardware`).
4. Write `launch/ur5_real.launch.py`:
   - args: `robot_ip` (default `192.168.1.10`), `launch_gui` (true), `enable_flex` (**true**),
     `headless_mode` (false), `use_mock_hardware` (false), `home_duration` (6.0 s)
   - include `ur_robot_driver/ur_control.launch.py`: `ur_type:=ur5`, `robot_ip`, `launch_rviz:=false`,
     `description_launchfile:=` our rsp launch, `initial_joint_controller:=scaled_joint_trajectory_controller`,
     `launch_dashboard_client:=true`
   - static TF `world → base_link` (z 0.80)
   - `home_pose_publisher` (RELIABLE, waits for `/joint_states`)
   - `gui_panel` (launch_gui) **or** headless `robot_node` (enable_keyboard false)
   - RViz with `config/view.rviz` (Image display disabled)
   - **no** Gazebo / bridges / `GZ_SIM_RESOURCE_PATH` / scene_setup
5. Fix packaging: `package.xml` exec_depends (`ur_robot_driver`, `ur_description`, `controller_manager`,
   `ros2controlcli`…); CMakeLists installs scripts/launch/urdf/config; endo_flex_control gets its
   missing exec_depends declared.

**Acceptance:** `colcon build` clean; **mock smoke test on the Jetson alone**:
`ros2 launch ur5_endo_flex ur5_real.launch.py use_mock_hardware:=true` → RViz shows
ur5+endoscope+flex chain (driven by synthetic flex joint states), mock `/joint_states` present,
scaled JTC active, `robot_node` runs and streams without error; `/flex/*` topics present with
**zero subscribers** (inert by design); `enable_flex:=false` variant silences them.

### Phase 3 — Script port (edit list)

| File | Edits |
|---|---|
| `robot_node.py` | (a) declare `enable_flex` (default **true** — kill-switch for the inert `/flex/*` Float64 pubs, sim refs `:321-329, 832-835`, and the synthetic flex `/joint_states` publish `:313-319, 800-835`); (b) STOP path → also call `/dashboard_client/stop` (Trigger client); (c) `use_sim_time` false via launch. Flex code paths otherwise **unchanged** — they keep running, inert on real HW. Leave the stale `extension_*` joint names (`:824-825`) as-is (harmless; matches sim behavior) |
| `gui_panel.py` | Flex row + `WASD` flex bindings stay active (they only drive the inert topics); STOP button wired to dashboard stop (via node method); status readout adds controller/robot connection state if cheap |
| `home_pose_publisher.py` | QoS BEST_EFFORT → RELIABLE (sim ref `:84-89`); default duration ≥ 5 s (param); `use_sim_time` false |
| `kinematics.py` | **No edits.** (Constants vs real rig flagged in §8.) |
| `scene_setup.py` | Not launched in real bring-up; later optional port = markers/targets only, `spawn_in_gazebo:=false`, remove `ros_gz_sim` subprocess + `monotonic()` clock |

**Acceptance:** in mock launch (default): `/flex/*` topics present with **zero subscribers**
(inert by design); synthetic flex joints on `/joint_states` drive the flex chain in RViz; no
controller errors; with `enable_flex:=false` the flex topics/joint-states disappear. GUI teleop
drives the mock arm; semi-auto (home/reset/bird-eye/pull-out) all execute.

### Phase 4 — Real bring-up with ported stack

1. Robot free, speed slider ≤ 25 %, E-stop reachable. Start `ur5_real.launch.py` (headless first:
   `launch_gui:=false` — no motion commands issued).
2. Pendant ▶ Play; confirm reverse-interface connect; confirm home-pose trajectory executes
   slowly and stops at `HOME_DEG` (verify first with a dry inspection of the 2-point goal on
   `/scaled_joint_trajectory_controller/joint_trajectory` via `ros2 topic echo`, then let it run).
3. `launch_gui:=true`; press-and-hold arm teleop in **linear** mode at low tick speed; then RCM
   mode (verify cone guard + wrist_1 rule behave as in sim).
4. Semi-auto commands one at a time: `home`, `reset`, `bird-eye`, `pull-out` (two-stage retract —
   watch clearances with the real endoscope).
5. Interrupt drills: release keys (stream stops → arm holds); GUI STOP → dashboard stop;
   pendant Play resume; one deliberate protective-stop recovery.

**Acceptance:** all steps pass with no unintended stops; TF/`/joint_states` lag imperceptible;
STOP latency acceptable; operator comfortable with runbook §6.

### Phase 5 — Hardening & optional extensions (choose per need)

1. Migrate commanding to the `FollowJointTrajectory` **action** (`/scaled_joint_trajectory_controller/follow_joint_trajectory`)
   for goal results/feedback + clean cancel (`cancel_goal_async`), if topic streaming proves fragile.
2. `/joint_states` staleness watchdog in `robot_node` (auto dashboard-stop).
3. Dashboard-assisted startup node (`power_on`, `brake_release`, `load_program "external_control.urp"`, `play`).
4. Optional **Gazebo digital twin** mirroring the real arm (see Q1 in §10).
5. Speed-scaling policy: keep pendant slider as the master speed limit during trials.
6. Real endoscope camera driver publishing `/endoscope_camera/image_raw` (+ `camera_info`) to
   re-enable the RViz Image panel (out of scope for this port).
7. Real flex actuator interface that **consumes** the existing `/flex/*` topics (turns the
   currently inert setpoints into physical tendon commands + closes the loop on measured flex
   state).

## 6. Runbook (daily operation)

```
1. Jetson on → verify: ip addr show end0 (192.168.1.100); ping -c1 192.168.1.10
2. source /opt/ros/jazzy/setup.bash && source ~/…/ur5_real_ws/install/setup.bash
3. ros2 launch ur5_endo_flex ur5_real.launch.py
4. Pendant: load + ▶ Play external_control.urp → wait "Robot connected to reverse interface"
5. Control from the GUI (keep pendant speed slider ≤ 25 % during trials; hands off pendant while streaming)
6. Stop: GUI STOP / ros2 service call /dashboard_client/stop std_srvs/srv/Trigger {} / pendant stop
7. Resume: pendant ▶ Play (connection re-establishes)
8. Shutdown: Ctrl-C the launch; stop program on pendant
```

## 7. Network & ports quick reference

| Port | Direction | Purpose |
|---|---|---|
| 50002 | robot → PC | URCap fetches the control script from the driver (script sender) |
| 50001 | robot → PC | reverse interface — cyclic servo commands (the control channel) |
| 50003 | robot → PC | trajectory interface |
| 50004 | robot → PC | URScript command forwarding (`/urscript_interface/script_command`) |
| 29999 | PC → robot | dashboard (`/dashboard_client/*` services) |
| 30004 | PC → robot | RTDE state stream (125 Hz on CB3) |

PC = 192.168.1.100 (`end0`), robot = 192.168.1.10, mask /24, direct LAN cable.
URCap node: Host IP **192.168.1.100**, port 50002.

## 8. Risks & mitigations

| # | Risk | Mitigation |
|---|---|---|
| R1 | Topic-streamed trajectories misbehave on real scaled controller at low speed scaling | Constraints already match sim semantics (`goal_time: 0.0`); if issues → Phase 5.1 action migration; pendant slider low during trials |
| R2 | Calibration mismatch → cm-level TCP error | Phase 0.6 extraction; verify `Calibration checked successfully` log |
| R3 | Hardcoded rig geometry: endoscope tip `[0,0,0.20195]`, RCM point `[0,0,-0.30]`, flex mount `-0.398`, supporter 0.80 m | Re-measure on the real rig before trusting RCM mode / semi-auto; update `kinematics.py` constants (single file) |
| R4 | Home pose `HOME_DEG` + wrist_1 −270° rule unsafe with real endoscope mounted | Collision-check first execution (Phase 4.2 dry echo + slow run) |
| R5 | Pendant interaction / second program kills the connection (`Connection to reverse interface dropped`) | Runbook §6; hands-off rule; resume = Play |
| R6 | Port conflicts / dual RTDE client (sim driver, URSim, fieldbus) | D9 hygiene rules; only one driver instance |
| R7 | Both workspaces define package `ur5_endo_flex` | D9: never source both in one shell |
| R8 | First-motion safety | Speed slider ≤ 25 %, E-stop drills, home publisher ≥ 5 s |
| R9 | Jetson CPU (driver + RViz + Tk GUI) | 125 Hz is light for AGX Orin; `launch_rviz:=false` escape hatch |
| R10 | RViz flex pose is *commanded/assumed* (open-loop), not measured — identical caveat to sim; `/flex/*` topics are inert on real HW (no subscriber) | Documented in D4; `enable_flex:=false` silences them if confusing; Phase 5.7 = future actuator interface |

## 9. Out of scope (this port)

- Real endo_flex actuation (tendon motors) — flex remains geometry-only when re-enabled.
- Real endoscope camera driver (`/endoscope_camera/image_raw` stays absent).
- MoveIt; gastrectomy scene / phantom / breathing animation; target tracking with real targets.
- Multi-robot setups, `tf_prefix`, e-Series/PolyScope 5 specifics.
- Any modification to `ur5_sim_ws`.

## 10. Open questions (Q1–Q2 RESOLVED 2026-10-23; the rest are deferred decisions)

- **Q1 — RESOLVED (2026-10-23):** state sync = **RViz live visualization** driven by real
  `/joint_states` (plan default confirmed). No Gazebo digital twin.
- **Q2 — RESOLVED (2026-10-23):** endo_flex form = **code active, no target** — flex URDF chain
  and all flex publishers keep running as in sim; on real HW the `/flex/*` topics are inert
  (no bridge/plugins), so only the arm is physically controlled. `enable_flex` (default true)
  kept as a kill-switch (D4).
- (Deferred) Keep package name `ur5_endo_flex` in the real ws (current plan) vs rename to avoid
  dual-workspace confusion.
- (Deferred) `home_pose` duration and whether home should auto-run at launch or require a GUI press.
- (Deferred) Which apt driver version to pin (3.8.0 installed vs 3.9.0 candidate).
- (Deferred) Phase 5 items priority ordering.

---

## 11. Implementation status (2026-10-23, later same day)

**Done and validated (mock hardware, no robot attached):**

- Workspace `ur5_real_ws` built: `ur5_endo_flex` (ported), `ur5_endoscope` + `endo_flex_control` (vendored), `kdl_parser_py` (vendored, needed by `kinematics.py`), `ur_calibration` (source-built from the driver's `jazzy` branch — see deviation 2).
- `ros-jazzy-ros2controlcli` installed (`ros2 control` CLI was missing).
- Phase 0.6: **calibration extracted** from the real robot → `config/ur5_calibration.yaml`, `hash: calib_2635161614594986886`.
- Mock acceptance (`ros2 launch ur5_endo_flex ur5_real.launch.py use_mock_hardware:=true launch_rviz:=false launch_gui:=false`):
  - `scaled_joint_trajectory_controller` [active] + full driver controller set
  - `/robot_description` = `ur5_endo_flex_real` (86 links incl. endoscope + flex chain); robot_node logs `KDL chain built: 6 joints`
  - Home pose executes through the topic→controller→mock→`/joint_states` loop: `Arm reached home position (max error: 0.0198 rad)`
  - `/flex/base_yaw` etc.: 1 publisher, 0 subscribers (inert by design, D4)
  - TF verified: `world→base_link` z=0.800 (supporter), `tool0→endoscope_link` (mount), `endoscope_link→flex_base_link` z=−0.398 (flex mount)

**Deviations from the original plan (all recorded in-code):**

1. **Vendored `ur_control.launch.py` → `launch/ur5_real_control.launch.py`** with
   `description_launchfile` *defaulting* to our rsp launch. Reason: passing
   `description_launchfile` through `IncludeLaunchDescription` did not take effect in this
   machine's environment (the driver silently used its stock description → robot_node got a
   UR5-only URDF → "empty chain" error). Changing the default in a vendored copy is
   deterministic. Same pattern as the already-planned vendored rsp copy (D3).
2. **`ur_calibration` built from source (driver `jazzy` branch) inside the workspace.**
   Released apt 3.8.0/3.9.0 + `ur_client_library` 2.15.0 is a broken combination: the node
   never calls `pipeline.init()`, so the primary stream never connects
   (`Stream is invalid. Connect it before trying to read!` reconnect loop). Fixed upstream
   after 3.9.0 (jazzy HEAD calls `pipeline.init()`). Re-run extraction any time with:
   `ros2 launch ur_calibration calibration_correction.launch.py robot_ip:=192.168.1.10 target_filename:=src/ur5_endo_flex/config/ur5_calibration.yaml`
   (after `source install/setup.bash`, so the workspace build shadows apt).
3. **`home_pose_publisher.py` hardened**: accepts only `/joint_states` messages containing
   all 6 arm joints (flex-only messages used to fool it) and retries a *freshly built*
   trajectory every 2 s (up to `home_retry_attempts`, default 30) until the arm is within
   0.02 rad of home — covers controller-activation races on mock and the pre-Play window
   on the real robot.
4. Phase 0 findings: URCap + `external_control.urp` already installed on the pendant
   (program STOPPED, robotmode RUNNING, PolyScope 3.15.8); **no firewall exists on this
   Jetson** (0.4 = nothing to do); `end0` static IP already correct.
   Still to verify on the pendant: External Control **Host IP = 192.168.1.100** (0.2).

**Environment note:** this Jetson's shells source a full source-built overlay at
`~/Documents/ros2_jazzy` (in `AMENT_PREFIX_PATH`/`PYTHONPATH`/`PATH` before `/opt/ros/jazzy`
for some lookups — e.g. `robot_state_publisher` resolves from there). It is functional
(all mock validation above ran with it), but if odd behavior appears, re-run from a clean
shell (`env -i ... bash --noprofile --norc` + source `/opt/ros/jazzy` + the workspace).

**Remaining (manual, with robot):** Phase 0.2 pendant Host-IP check → Phase 1 stock-driver
bring-up (or jump to Phase 4 since the full stack is mock-validated; either way follow the
safety notes: clear space, speed slider ≤ 25 %, E-stop reachable, driver first → pendant ▶
Play, home pose will auto-execute once the controller activates).
