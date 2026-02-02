## Investigation Log: Lift Sagging, Command–State Discrepancy, and Incremental Control in Stretch MuJoCo Gym Environment

### Goal

While building a custom Gym environment for the Stretch robot in MuJoCo, I investigated several issues related to lift control and control semantics:

1. **Lift sagging**
   The lift joint appeared to move downward (or fail to move upward) under certain control formulations.

2. **Mismatch between commanded target and measured position**
   There was often a gap between the intended motion and the actual lift position.

3. **Choice of control interface**
   Whether to use a virtual target (`move_to` with an integrated command) or incremental sensor-based control (`move_by`).

The goals of this investigation were to determine:

* whether the observed behaviors are bugs or expected,
* which control formulation avoids sagging,
* how to interpret the command–measurement gap,
* and which approach is more appropriate for accuracy checking and future RL training.

---

## Setup

* Control loop implemented inside `env.step()` (Gym-style)
* **Action space**: normalized `[-1, 1]` for `(lift, arm, gripper)`
* **Action semantics**: velocity-style control

Action interpreted as:

```
delta = action * scales * dt
```

Typical values:

* `dt = 0.05 s`
* `scales = (0.06, 0.06, 0.20)`

Originally, lift delta per step was approximately:

```
0.06 * 0.05 ≈ 0.003 m
```

Commands were issued via either:

* `sim.move_to(Actuators.lift, target_lift)` (virtual target chasing), or
* `sim.move_by(Actuators.lift, delta_lift)` (incremental motion).

---

## 1. Baseline Check: Does the Lift Sag When Idle?

### Test

After `env.reset()` (which calls `sim.home()`), I advanced simulation time **without applying any actions**:

```
for i in range(200):
    t0 = sim.pull_status().time
    env._wait_for_sim_advance(t0)
    print(i, sim.pull_status().lift.pos)
```

### Result

```
0    0.5921568
...
199  0.5925236
```

### Conclusion

* No observable sag when idle
* The controller can hold a fixed lift setpoint

Any sag observed later is **not due to gravity alone**, but due to how commands are generated and updated.

---

## 2. Measured-Based Target Chasing Causes Sag (`meas + delta` with `move_to`)

### Control formulation

```
target_lift = meas_lift + delta_lift
sim.move_to(lift, target_lift)
```

### Observed behavior

Even with `action = [1, 0, 0]` (lift up), the lift moved downward:

```
i=0   meas_lift=0.588498
i=10  meas_lift=0.586372
i=20  meas_lift=0.584201
...
i=99  meas_lift=0.567051
```

### Explanation

* The lift experiences a small downward drift while moving
* Using `meas + delta`:

  * accepts the sagged position as the new reference
  * causes the controller to **track gravity instead of the intended upward motion**

If `delta_lift` is smaller than the gravity-induced sag per step, the net motion becomes downward.

### Conclusion

Using measured state as the integration base with a moving target is **unstable for gravity-loaded joints**.

---

## 3. Virtual Command Integration Fixes Sag but Introduces Lag

### Control formulation

```
cmd_lift += delta_lift
sim.move_to(lift, cmd_lift)
```

### Observed behavior

```
i=0   cmd_lift=0.594229  meas_lift=0.591527
...
i=99  cmd_lift=0.891229  meas_lift=0.862038
```

Typical steady-state:

```
cmd_lift ≈ 0.891
meas_lift ≈ 0.862
gap ≈ 0.029 m  (~3 cm)
```

### Interpretation

* Sag is eliminated
* A bounded gap remains between command and measurement
* This gap is **expected servo tracking lag** when chasing a moving target

### Limitation

The virtual command (`cmd_lift`) can move faster than the actuator can physically track, creating a persistent gap.

---

## 4. Why `wait_until_at_setpoint()` Does Not Fix the Gap

### Experiment

After issuing `move_to`, I attempted to block until convergence:

```
sim.wait_until_at_setpoint(
    Actuators.lift,
    timeout=2.0,
    position_tolerance=0.002
)
```

This was later extended to:

* `timeout = 8.0 s`
* `position_tolerance = 0.002 m`

### Result

Repeated timeouts:

```
Timeout: Joint lift did not reach 0.6065. Actual: 0.5950 Diff: 1.15 cm
```

Eventually, the physics loop terminated.

### Explanation

* The target moves **every step** (ramping target)
* Exact convergence is unrealistic unless the target stops updating
* Repeated blocking stresses the simulator loop

### Conclusion

`wait_until_at_setpoint()` is **incompatible with velocity-style ramp control** and should not be used inside Gym-style `step()`.

---

## 5. Decision: Switch to Incremental Sensor-Based Control (`move_by`)

Based on advisor feedback and observed behavior, the control strategy was changed to:

```
read measured joint state
→ issue small relative motion using move_by(delta)
→ wait for motion to settle (for accuracy checking)
```

### New control formulation

```
sim.move_by(Actuators.lift, delta_lift)
sim.wait_while_is_moving(Actuators.lift)
```

This removes the virtual target entirely and avoids command drift.

---

## 6. Empirical Results: Lift Up vs Lift Down with `move_by`

To evaluate accuracy, I tested equal-magnitude deltas:

* `delta = +0.05` (lift up)
* `delta = -0.05` (lift down)

### Moving down (`delta = -0.05`)

```
0.589 → 0.540 → 0.491 → 0.441 → ... → 0.097
```

Per-step change ≈ **0.048–0.050 m**

### Moving up (`delta = +0.05`)

```
0.354 → 0.389 → 0.423 → 0.457 → ... → 0.667
```

Per-step change ≈ **0.034–0.035 m**

### Velocity measurements

During motion:

```
lift vel ≈ ±0.014 – 0.016 m/s
```

Upward and downward velocities were symmetric.

---

## 7. Interpretation of Up vs Down Difference

The difference in achieved motion comes from:

1. **Velocity saturation**
   Effective maximum lift speed is about `0.015 m/s`.

2. **Insufficient simulated time per step**
   To move `0.05 m` at `0.015 m/s` requires:

```
time ≈ 0.05 / 0.015 ≈ 3.3 s
```

3. **Lower joint limit effects**
   Early upward steps started near the lower limit, reducing effective motion.

### Conclusion

The asymmetry is **not primarily gravity**, but a mismatch between:

* commanded delta size,
* available simulated time,
* actuator velocity limits.

---

## 8. Effect of Action Scale on `move_by`: Minimum Effective Delta (Deadzone)

After switching to non-blocking stepping and incremental control using `move_by`, I tested different lift action scales while holding `action = [1, 0, 0]` (lift up).

### Experiment

With `dt = 0.05`, the per-step lift delta is:

```
delta_lift = action * scale_lift * dt
```

I tested several values of `delta_lift` by varying `scale_lift`, while advancing a fixed amount of simulation time per `env.step()` (no blocking).

### Observed behavior

| delta_lift (m) | scale_lift | Observed lift behavior                  |
| -------------- | ---------- | --------------------------------------- |
| 0.00075        | 0.015      | Lift moves downward (gravity dominates) |
| 0.015          | 0.3        | Lift moves upward slowly, stable        |
| 0.025          | 0.5        | Lift moves upward with moderate speed   |
| 0.05           | 1.0        | Lift moves upward strongly              |

Representative logs:

```
delta = 0.00075 → lift vel ≈ -0.004 m/s
delta = 0.015   → lift vel ≈ +0.006 m/s
delta = 0.025   → lift vel ≈ +0.022 m/s
delta = 0.05    → lift vel ≈ +0.062 m/s
```

### Interpretation

These results indicate that `move_by(delta)` does **not** behave as a purely kinematic displacement command.

Instead, `move_by` appears to map `delta` into an internal motion primitive (velocity- or force-limited), with a **minimum effective delta** required to overcome gravity and internal controller deadzones.

For deltas below this threshold:

* the commanded motion is too weak or too short-lived,
* the lift is not actively held upward,
* gravity dominates and the joint drifts downward.

Once `delta_lift` exceeds this minimum effective range (≈ **1–2 cm** in this setup), the lift consistently moves upward.

### Conclusion

The earlier assumption that `delta = vmax * dt` would always produce upward motion is **not valid** when using `move_by`.

Instead, `move_by` requires an empirically chosen scale that exceeds the actuator/controller deadzone.

---

## 9. Final Design Choice (Current State)

For now, to verify accuracy and understand actuator limits:

* Use **`move_by`** (incremental, sensor-based control)
* Use **`wait_while_is_moving()`** for accuracy checking
* Choose deltas large enough to exceed the `move_by` deadzone, while remaining stable and interpretable
* Use `scale_lift ≈ 0.3` (delta ≈ `0.015 m` per step) as the **minimum effective upward command**

This behavior is easier to reason about and avoids virtual goal drift.

---

## Summary of Findings

| Issue                             | Cause                                       | Resolution / Decision                   |
| --------------------------------- | ------------------------------------------- | --------------------------------------- |
| Lift sagging                      | `meas + delta` target chasing under gravity | Avoid measured target integration       |
| Command–measurement gap           | Servo tracking lag                          | Expected and bounded                    |
| `wait_until_at_setpoint` timeouts | Moving target + strict tolerance            | Do not use for ramp control             |
| Up vs down motion asymmetry       | Velocity limit + insufficient sim time      | Reduce delta or increase settle time    |
| Virtual goal chasing              | Target can outrun actuator                  | Switch to `move_by` for accuracy checks |

---

## Current Recommendation

**For accuracy checking and analysis (current phase):**

* Use `move_by`
* Use `wait_while_is_moving`
* Log measured deltas and velocities
* Keep deltas small relative to actuator capability

**For future RL training:**

* Replace blocking waits with fixed substeps
* Match `delta = vmax * dt` to actuator limits
* Accept bounded tracking error as part of environment dynamics

---

This investigation confirms that the observed behavior is **physically consistent**, not a simulator bug, and that incremental sensor-based control is currently the most interpretable option for verifying lift behavior and constraints.
