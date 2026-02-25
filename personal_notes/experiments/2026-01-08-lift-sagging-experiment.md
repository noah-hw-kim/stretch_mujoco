## Investigation Log: Lift Sagging, Command–State Discrepancy, and Incremental Control in Stretch MuJoCo Gym Environment

### Goal

While building a custom Gym environment for the Stretch robot in MuJoCo, I investigated several issues related to lift control and control semantics:

1. **Lift sagging**
   If the commanded lift delta is below a minimum threshold (≈ 0.008–0.010 m),
   the measured lift velocity becomes negative and the lift drifts downward,
   even when the commanded direction is upward.

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




