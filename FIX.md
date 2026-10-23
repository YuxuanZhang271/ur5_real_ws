# FIX.md — findings from the first real-robot session (2026-10-23)

> **Status (later same day): FIXES IMPLEMENTED & VERIFIED.**
> Issue 1: config fix applied and verified (0 errors on load). Issue 2: F2 + F4a
> implemented in `robot_node.py` and verified by a dedicated harness (10/10 checks,
> §"Implementation & verification" below). F1 (pendant slider) is an operating
> instruction for the next session; F5/F6 deferred until the real-robot test.
> Original analysis is preserved below for reference.

---

## Issue 1 — RViz fails to launch ("crashes into some issues")

### Symptom
With `launch_rviz` at its default `true`, the RViz window does not come up with the
robot; the launch log shows the rviz process starting but RViz is unusable.

### Reproduced & isolated (tests run on this Jetson today)

| Test | Config | rviz2 binary | Result |
|---|---|---|---|
| 1 | `ur5_sim_ws` original `view.rviz` (Image display `Enabled: true`) | `~/Documents/ros2_jazzy` source build | **loads fine** |
| 2 | same sim config | `/opt/ros/jazzy/bin/rviz2` | **loads fine** |
| 3 | our `view.rviz` (Image display `Enabled: false` — the only diff vs sim, verified with `diff`) | Documents source build | **`[ERROR] Could not load display config: Invalid argument`** |
| 4 | our `view.rviz` | `/opt/ros/jazzy/bin/rviz2` | **same error** |
| 5 | our `view.rviz` with the whole Image display block deleted | `/opt/ros/jazzy/bin/rviz2` | **loads fine, no errors** |

### Root cause
During the port, the endoscope-camera Image display was disabled by setting
`Enabled: false` (no camera on real hardware). This rviz generation **rejects the
entire display config when the Image display is present but disabled** ("Invalid
argument"), so RViz aborts config loading and runs with an *empty* configuration —
which reads as "RViz didn't launch properly". Both rviz2 builds on this machine show
the same behavior, so it is not a binary/version problem.

Notes:
- Both files are valid YAML (checked); the only textual difference from the sim
  config that loads is that one flag.
- Environment fact worth knowing (not the cause): `rviz2` resolves first to the
  source-built one at `~/Documents/ros2_jazzy/install/rviz2/bin/rviz2` (PATH
  precedence over `/opt/ros/jazzy/bin/rviz2`). It worked in all passing tests above.

### Proposed fix (F1 — trivial, config only) — **APPLIED & VERIFIED**
Deleted the entire Image display block from `src/ur5_endo_flex/config/view.rviz`
and rebuilt. Verified: `rviz2 -d <installed view.rviz>` loads with **zero** errors
(with the workspace sourced, so `package://ur5_endoscope` meshes resolve). Note:
launch RViz only from a shell where `ur5_real_ws/install/setup.bash` is sourced.

---

## Issue 2 — press-and-hold teleop shakes, and the shaking grows the longer you hold

### Symptom
ULDRFB moves the arm in both linear and RCM mode, but motion is not smooth like in
simulation — the arm vibrates, and the vibration intensifies the longer a key is
held. Your suspicion (control rate mismatch) is close: it is a **streamed-goal
resync mismatch**, with rate as one ingredient.

### What the code does today (verified in `scripts/robot_node.py`)

- Tick timer at `control_tick_hz` = 20 Hz → `self.dt = 0.05 s` constant
  (`robot_node.py:196,246-247`; launch passes 20).
- Each tick, a held key adds a Cartesian delta of `v · dt`
  (e.g. `arm_linear_vel` 25 mm/s → 1.25 mm per tick; `robot_node.py:509-537`),
  computed **from the latest measured pose**: FK is seeded on `self.joint_positions`
  — the `/joint_states` cache (`robot_node.py:569-605`).
- It then publishes a 2-point trajectory on the topic:
  **point0 = measured joint cache @ t=0, velocities = 0**; point1 = IK(measured+delta)
  @ t=0.05 s, velocities = 0 (`robot_node.py:1572-1587`, `_send_trajectory`).
- The real `ScaledJointTrajectoryController` **replaces its active goal on every
  topic message** — i.e. 20 goal replacements per second.

### Why the sim was smooth and the real robot is not (compounding mechanisms)

1. **Stale-point0 snap-back (primary).** On real hardware `self.joint_positions` is
   8 ms-quantized RTDE data (125 Hz) plus transport and executor latency, so point0
   is always a stale snapshot of where the arm *was*. Each of the 20 replacements
   per second re-seeds the controller's current command **backwards** to that stale
   position; the controller then chases point1. That is a 20 Hz sawtooth whose
   amplitude grows with the arm's actual velocity. In Gazebo the hardware mirror
   was instantaneous and scaling was fixed 1.0, so point0 was effectively "now".
2. **Speed-scaling mismatch (your "rate mismatch" instinct).** The scaled JTC
   stretches trajectory *time* by `1/speed_scaling` — the pendant speed slider.
   If the slider is below 100 % (as advised for first runs — good practice), only
   `scaling × 0.05 s` of each goal elapses before the next replacement: the arm is
   re-targeted before ever completing a goal, commanded-vs-achieved gap grows, and
   the snap-back amplitude grows with it. (In sim there was no speed scaling at all.)
3. **Zero-velocity endpoints on a 50 ms spline.** Both points declare `velocities
   = 0`, so each 50 ms goal contains accelerate-from-rest → decelerate-to-rest
   (peak velocity ≈ 2× the mean). At 20 Hz that is continuous high-frequency
   excitation, on top of the sawtooth.
4. **Growth over time.** Each snap-back decelerates the arm, the measured lag grows,
   and the next goal rides on an increasingly perturbed state — oscillation builds
   as long as the key is held, exactly what you observed. The arm's position-servo
   response rings at its own natural frequency on top of the 20 Hz forcing.

Ruled out / secondary:
- `trajectory_until_node` (new in driver 3.8.0) is an **action** forwarder
  (`control_msgs/FollowJointTrajectory` → `ur_msgs/FollowJointTrajectoryUntil`) —
  it does not touch the topic interface we use. Not involved.
- The log's FIFO-scheduling warning and the transient controller-activation
  overruns (32 ms) add jitter but are not the mechanism; `controller_manager` did
  get FIFO priority 50 successfully. Real-time hardening (per UR docs: lowlatency
  kernel, user memlock/rt limits) is worth doing eventually but won't fix this.

### Evidence to collect at the next session (read-only, 2 minutes)

```bash
ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling_factor --once   # pendant slider value during teleop (expect < 1.0)
ros2 topic hz /joint_states                                                   # ≈ 125 Hz
ros2 topic hz /scaled_joint_trajectory_controller/controller_state           # ≈ 100 Hz
# during a 5 s key hold, compare (one echo each):
ros2 topic echo /scaled_joint_trajectory_controller/controller_state --once  # desired.positions vs actual.positions divergence = the sawtooth, directly visible
```
Optionally record a bag of `/joint_states` + `controller_state` + the command topic
during a 5 s hold and plot desired-vs-actual — the 20 Hz snap-back will be visible.

### Proposed fixes, ranked

- **F1 (ops, zero code — do first):** set the pendant **speed slider to 100 %**
  during teleop trials. The node's own velocity limits (25 mm/s Cartesian,
  4 °/s tilt) are the real speed cap, so this is not as aggressive as it sounds —
  but keep clear-space + E-stop discipline. This removes mechanism 2 entirely and
  should visibly reduce the shaking. Verify the scaling topic reads 1.0.
- **F2 (code, the core fix):** stop re-seeding point0 from the stale *measured*
  state every tick. Maintain an internal **commanded-target integrator**: on the
  first tick of a hold (or after release / direction change / large divergence),
  seed from measured; thereafter publish `[q_prev_target @ 0, q_new_target @ dt]`
  with `q_new_target = q_prev_target + v·dt`. The replacement goal then starts
  exactly where the controller's command already is → seamless goal swaps, no
  snap-back. Re-sync the integrator to measured whenever
  `|measured − integrated|` exceeds a threshold (≈0.02–0.05 rad) — covers drift,
  protective stop, hand-back.
- **F3 (code, smaller alternative to part of F2):** seed point0 from
  `/scaled_joint_trajectory_controller/controller_state`'s **`desired.positions`**
  (100 Hz — the controller's own current command) instead of `/joint_states`.
  Fresher and semantically correct anchor; smaller change than F2.
- **F4 (code, cheap additions, combine with F2 or F3):**
  (a) declare the real per-tick joint velocities on the streamed points instead of
  `velocities = 0` — removes the double-peak spline; and/or
  (b) lengthen each streamed goal to 2–3 ticks (0.10–0.15 s) so replacements
  interrupt a smoother arc.
- **F5 (controller config, no code):** our launch can pass a custom
  `controllers_file` (copy of `ur_controllers.yaml` with
  `scaled_joint_trajectory_controller: open_loop_control: true`) — the controller
  then follows its internal clock without state correction, further desensitizing
  to stale seeds. Try only after F2/F4, as it changes error semantics.
- **F6 (structural — promotes PLAN.md Phase 5.1):** migrate teleop to the
  **FollowJointTrajectory action**: on key-down send ONE goal containing a
  multi-second velocity-limited ramp; on key-up cancel and re-anchor. No 20 Hz
  goal replacement at all. End-game option if F2+F4 still isn't sim-smooth.

Recommended sequence: **F1 (verify at slider 100 %) → F2 + F4 → re-test → F5/F6
only if needed.** Expected end state: motion indistinguishable from the sim
(velocity-limited, smooth, monotone) with the slider at 100 %.

---

## Implementation & verification (2026-10-23, later same day)

**Applied:**

1. **Issue 1 / config:** Image display block removed from `view.rviz` → verified
   loading with 0 errors.
2. **Issue 2 / F2 — teleop commanded-target integrator** (`robot_node.py`):
   - New `self._q_cmd` integrator (protected by `_state_lock`), new
     `teleop_resync_rad` parameter (default 0.05 rad).
   - `_arm_linear_step` / `_arm_rcm_tilt` seed their FK/IK on the integrator while
     a stream is active (first tick of a hold still anchors on measured).
   - `_send_trajectory(..., anchor='cmd')` for the teleop stream: point0 = previous
     commanded target; point1 = new target; after publishing, integrator = target.
     All other callers (HOME / RESET / BIRD-EYE / PULL OUT / tracking) keep the
     original measured-anchored rest-to-rest behavior and reset the integrator.
   - `_teleop_housekeeping()` in the tick: clears the integrator when no arm key is
     held; re-syncs to measured **only when the arm is farther than
     `teleop_resync_rad` AND not moving toward the command** (stopped: protective
     stop / program stop / hand-back). While the arm is following — including
     latency or speed-scaling lag — the integrator persists, so the stream stays
     seamless. (The naive "drift > threshold" version was caught by the harness:
     it re-anchored mid-hold under sustained lag and *became* the snap-back.)
   - `toggle_arm_mode()` resets the integrator.
3. **Issue 2 / F4a — endpoint velocities:** teleop-streamed goals now carry
   `velocities = (q_target − q_base)/duration` on BOTH points (constant-velocity
   spline; no accelerate-from-rest double peak). Semi-automatic moves keep 0/0
   (rest-to-rest at the goal).

**Verification harness** (`/tmp/opencode/test_teleop_stream.py`, rerunnable):
isolated `Ur5EndoFlexNode` + injected `/robot_description` + a first-order
"arm" model (80 ms lag) publishing `/joint_states`, driven through the public
`set_key()` API — results:

```
[PASS] 1 streamed while held (40 msgs)
[PASS] A first goal anchored on measured            (gap 0)
[PASS] B point0 == previous point1 (no snap-back)  (max gap 0.00e+00)
[PASS] C endpoint velocities = step velocity
[PASS] D steady per-tick target advance            (0.0043..0.0043 rad)
[PASS] E no re-sync while following                (final drift 0.0000 rad)
[PASS] F re-press re-anchors on measured           (gap 0)
[PASS] G frozen arm triggers re-sync
[PASS] H frozen-arm excursion stays bounded        (no run-away)
==== 10/10 checks passed ====
```

Full-stack mock smoke re-run after the changes: all green (scaled controller
active, KDL chain built, home pose converging, flex topics inert, no errors).

### Second iteration (after the next real-robot session, same day)

The session log showed the arm came up perfectly (calibration ✓, reverse
interface ✓, home pose ✓) but **teleop goals were being silently REJECTED**:

```
[ERROR] [scaled_joint_trajectory_controller]: Velocity of last trajectory point
        of joint shoulder_pan_joint is not zero: -0.00007...
```

Root cause: ros2_control's JTC `validate_trajectory_msg()` **returns false**
(rejects the goal) unless `allow_nonzero_velocity_at_trajectory_end` is set —
verified against the jazzy ros2_controllers source. My first F4a put the
per-step velocity on BOTH points, so all 20 teleop goals/second were dropped;
the arm never moved during teleop, which is exactly what the
"Teleop integrator re-synced ... (arm not following)" safety logs showed.

**F4a revised:** teleop goals now carry the step velocity on point-0 only
(spline starts at the motion velocity — no rest-to-rest double peak) and
**zero velocity on the last point** (validation requirement). Alternative not
taken (yet): enable `allow_nonzero_velocity_at_trajectory_end` in a custom
controllers_file if we later want perfectly constant-velocity goals.

Harness extended (all six linear keys + all four RCM keys + JTC-validity
assertion): **20/20 checks pass** — every key commands healthy 0.0036-0.006 rad
steps with dominant joints as expected, position continuity holds exactly,
goals are JTC-valid, re-anchor/frozen-arm behaviors intact.

**Next real-robot session (same checklist as before):** slider 100 %, launch,
pendant ▶ Play, taps, then 5 s holds in linear and RCM mode. With goals now
accepted, keys should finally move the arm smoothly.

**Not applied (deliberate):** F5 (`open_loop_control`) and F6 (action-based
streaming) — hold until the real-robot test; F1 (pendant slider 100 %) is an
operating instruction, see below.

## Next real-robot session checklist

1. **Pendant: speed slider to 100 %** (F1) — the node's velocity limits
   (25 mm/s, 4 °/s) are the actual speed cap; keep clear space + E-stop
   discipline.
2. Bring up as before:
   ```bash
   source /opt/ros/jazzy/setup.bash && source install/setup.bash
   ros2 launch ur5_endo_flex ur5_real.launch.py
   ```
   (RViz will now load properly; confirm the robot model + endoscope appear.)
3. Pendant ▶ Play `external_control.urp`; home pose executes as last time.
4. Short **taps** of U/D/L/R/F/B — each step should settle cleanly.
5. **5 s holds** per key (linear mode) — motion should be smooth and constant;
   no growing vibration. Repeat in RCM mode (Ctrl+M first, as before).
   Optional live check:
   `ros2 topic echo /scaled_joint_trajectory_controller/controller_state --once`
   — `desired` vs `actual` positions should track closely.
6. If any vibration remains: note the pendant slider value, capture
   `ros2 topic echo /speed_scaling_state_broadcaster/speed_scaling_factor --once`
   and a 5 s bag of `/joint_states` + `controller_state`, then we evaluate F5/F6.

### Post-fix verification checklist (next session)

1. Slider 100 %; short 'U' taps — each step settles cleanly, no ring-down.
2. 5 s holds per key (linear mode) — `controller_state` desired vs actual stay on
   top of each other; no growing divergence.
3. Same in RCM mode (IK + cone guard + wrist rule still behave as in sim).
4. Two-key combos (e.g. U+L) — smooth diagonal.
5. STOP paths still work (key release holds position; E-STOP button; pendant stop).

---

*Files referenced: `src/ur5_endo_flex/scripts/robot_node.py` (`_tick` :452-558,
`_arm_linear_step` :563-605, `_send_trajectory` :1572-1587, joint cache
`:411-418`), `src/ur5_endo_flex/config/view.rviz` (Image display block).*
