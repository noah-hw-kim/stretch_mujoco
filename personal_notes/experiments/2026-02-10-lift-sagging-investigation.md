## 7. Effect of Action Scale on `move_by`: Minimum Effective Delta (Deadzone)

After switching to non-blocking stepping and incremental control using `move_by`, I tested different lift action scales while holding:

```text
action = [1, 0, 0]  # lift up
```

The goal was to identify the **minimum effective lift delta** required to overcome gravity and internal controller deadzones.

---

### Experiment

With `dt = 0.05`, the per-step lift delta is computed as:

```text
delta_lift = action * scale_lift * dt
```

I varied `scale_lift` to test different values of `delta_lift`, while advancing a **fixed amount of simulation time per `env.step()`** (no blocking waits).

---

### Observed Behavior (Coarse Sweep)

| delta_lift (m) | scale_lift | Observed lift behavior                  |
| -------------: | ---------: | --------------------------------------- |
|        0.00075 |      0.015 | Lift moves downward (gravity dominates) |
|          0.015 |        0.3 | Lift moves upward slowly, stable        |
|          0.025 |        0.5 | Lift moves upward with moderate speed   |
|           0.05 |        1.0 | Lift moves upward strongly              |

Representative velocity logs:

```text
delta = 0.00075 → lift vel ≈ -0.004 m/s
delta = 0.015   → lift vel ≈ +0.006 m/s   (minimum effective upward motion)
delta = 0.025   → lift vel ≈ +0.022 m/s
delta = 0.05    → lift vel ≈ +0.062 m/s
```

---

### Refined Threshold Sweep

A finer-grained sweep was performed between `0.006 m` and `0.015 m` to identify the transition point more precisely.

| delta_lift (m) | Average lift velocity |
| -------------: | --------------------- |
|          0.006 | Negative              |
|          0.008 | Barely positive       |
|          0.010 | Positive              |
|          0.012 | Clearly positive      |
|          0.014 | Strongly positive     |
|          0.015 | Strongly positive     |

This indicates that the minimum effective upward delta lies in the range:

```text
delta_min_up ≈ 0.008–0.010 m
```

The exact threshold varies slightly with simulation timing and controller state.

---

### Interpretation

These results indicate that `move_by(delta)` does **not** behave as a purely kinematic displacement command.

Instead:

* `move_by` maps `delta` into an internal motion primitive
* motion is velocity- or force-limited
* a **minimum effective delta** is required to overcome gravity and controller deadzones

For deltas below this threshold:

* the commanded motion is too weak or too short-lived
* the lift is not actively held upward
* gravity dominates and the joint drifts downward

Once `delta_lift > delta_min_up`, the lift consistently moves upward.

---

### Conclusion

The assumption:

```text
delta = vmax * dt  ⇒ upward motion
```

is **not valid** when using `move_by`.

Instead, `move_by` requires an empirically chosen scale that exceeds the actuator/controller deadzone.

---

## 9. Action Deadzone and Small-Command Handling (Current Fix)

Because the action space is continuous in `[-1, 1]`, small action magnitudes are frequently produced during exploration and near convergence.

Using the formulation:

```text
delta_lift = action * scale_lift * dt
```

this results in many very small positive lift deltas.

---

### Empirical Finding

Testing showed that:

* small **positive** lift deltas below `≈ 0.008–0.010 m`

  * do not generate sufficient upward authority
  * instead lead to **slow downward drift**
* these commands are not neutral
* the failure mode is asymmetric (upward only)

This creates an undesirable situation where a positive action can reliably produce motion in the opposite direction.

---

### Design Decision: Minimum Effective Delta Guard

To ensure sign correctness and stability, a minimum effective delta guard was added for the lift actuator:

* small upward deltas below the threshold are treated as **no-op**
* downward deltas are left unchanged
* larger deltas pass through unchanged

Conceptually:

```text
if 0 < delta_lift < delta_min_up:
    delta_lift = 0
```

This prevents weak upward commands from introducing slow sag over time, while preserving interpretable and consistent control behavior.

---

## 10. Alternative Action Mapping: Smooth Saturation (`tanh`)

The deadzone-based solution introduces a tradeoff between action sensitivity and stability.

With linear scaling:

```text
delta_lift = action * scale_lift * dt
```

and a minimum effective upward delta (`delta_min_up ≈ 0.01 m`), the smallest action that produces upward motion is:

```text
a_min = delta_min_up / (scale_lift * dt)
```

For example, with `dt = 0.05` and `scale_lift = 0.5`:

```text
a_min = 0.01 / (0.5 * 0.05) = 0.4
```

This means all positive actions in `(0, 0.4)` effectively become **no-op**.

Increasing `scale_lift` reduces this no-op band, but also increases the maximum per-step delta, often requiring an explicit `delta_max` cap. That cap, in turn, causes a large portion of the action range to saturate to the same motion.

---

### Smooth Saturation Idea

As an alternative, a smooth saturating mapping was considered for the lift action:

```text
delta_lift = delta_max * tanh(k * action) / tanh(k)
```

Where:

* `delta_max` is the maximum allowed per-step lift motion
* `k` controls how quickly the output saturates

Properties:

* near zero, `tanh(x) ≈ x`, so small actions still produce meaningful motion
* large actions smoothly approach `±delta_max`, avoiding hard clipping
* no hard no-op band
* no flat saturation plateau

---

### Comparison with Deadzone-Based Mapping

**Deadzone-based mapping**

**Pros**

* simple and highly interpretable
* directly enforces a minimum effective delta
* prevents positive commands from causing sag

**Cons**

* creates a no-op region in the action space
* requires careful tuning of `scale_lift`
* often requires a hard max-delta cap, saturating much of the action range

**Smooth saturation (`tanh`) mapping**

**Pros**

* continuous and monotonic over the entire action range
* small actions remain meaningful without sag
* large actions saturate smoothly, matching actuator limits
* more suitable for RL exploration

**Cons**

* less directly interpretable than linear delta semantics
* requires tuning of `delta_max` and `k`

---

### Takeaway

The difficulty in finding a single “sweet spot” with linear scaling arises from actuator deadzone and saturation behavior.

Smooth saturation reshapes the action-to-command mapping to better match these physical constraints, but is currently documented as an **alternative**, not the implemented solution.



## 9. Final Design Choice (Current State)

For now, to verify accuracy and understand actuator limits:

- Use **`move_by`** (incremental, sensor-based control)
- Use **`wait_while_is_moving()`** for accuracy checking
- Choose deltas large enough to exceed the `move_by` deadzone, while remaining stable and interpretable
- Use `scale_lift ≈ 0.3` (delta ≈ `0.015 m` per step) as the **minimum effective upward command**
- Apply a **minimum-delta guard** for upward lift commands to avoid actuator deadzone–induced sag

This behavior is easier to reason about and avoids virtual goal drift.


## Current Recommendation

**For accuracy checking and analysis (current phase):**

* Use `move_by`
* Use `wait_while_is_moving`
* Log measured deltas and velocities
* Keep deltas small relative to actuator capability

**For future RL training:**

* Replace blocking waits with fixed substeps

---

This investigation confirms that the observed behavior is **physically consistent**, not a simulator bug, and that incremental sensor-based control is currently the most interpretable option for verifying lift behavior and constraints.

---


### Action Scaling Validation (a = 1)
scales: Tuple[float, float, float] = (0.8, 0.3, 0.5)

| Actuator | Limit Range | Total Δ over 10 steps | Avg Δ / step | % of Range / step | Steps for Full Range | Verdict |
|--------|-------------|-----------------------|--------------|-------------------|----------------------|---------|
| Lift | 0.0 → 1.1 (1.10 m) | 0.22365 m | 0.02236 m | 2.03% | ~49 | Good (above deadzone, stable) |
| Arm | 0.0 → 0.52 (0.52 m) | 0.09323 m | 0.00932 m | 1.79% | ~56 | Good (reasonable exploration speed) |
| Gripper | -0.25 → 0.53 (0.78) | 0.14489 | 0.01449 | 1.86% | ~54 | Good (moderate speed, controllable) |


## Lift Deadzone Update (Upward-Only)
Lift mapping:
delta_lift = a_lift * scale_lift * dt

With:
- scale_lift = 0.8
- dt = 0.05
- delta_min_up = 0.008

0.8 * 0.05 = 0.04 > 0.008
action = [-1,1]


delta = action * scale * dt(0.05 for 20hz)

action = 0 to 0.2 * 0.4 < 0.008
0.4/2 = 20%

We get:
delta_lift = 0.04 * a_lift

Minimum action needed to exceed the effective upward delta:
|a_lift| >= 0.008 / 0.04 = 0.20

If a symmetric deadzone is applied using:
abs(delta_lift) >= delta_min_up

Then:
- actions in (-0.20, +0.20) produce no motion
- 20% of the positive action range is ignored
- 20% of the negative action range is ignored
- 40% of the total action space is lost

Observed behavior:
- Small upward lift commands below delta_min_up cause sag due to gravity
- Small downward commands still produce valid motion

Design decision:
- Apply deadzone only to small positive lift deltas
- Always allow downward lift deltas

Rationale:
- Lift is gravity-loaded and physically asymmetric
- The failure mode exists only for upward motion
- Avoids unnecessarily discarding 40% of the action space

### ETC
ARM doesn't need to have a strong guard. The velocity goes down equal or less than

```
sim.move_by(Actuators.arm, 0.000006)
PositionVelocity(pos=0.19706865969602427, vel=-2.0494114763446873e-06)
```

Gripper
Gripper doesn't need to have a strong guard. The velocity goes down equal or less than
```
sim.move_by(Actuators.gripper, 0.004)
PositionVelocity(pos=0.01611716571659011, vel=-2.5355908320734716e-07)
```

Measured Per-Tick Motion Summary (a = 1, dt = 0.05, scales: = (lift = 0.8, arm = 0.3, gripper = 0.5))

0.5 * 0.01



## Why `dt` Exists and Why It Is Used in Action Application

### What `dt` Means

`dt` represents how much **real time** passes during one environment step.

If:
- `dt = 0.05`

then:
- one `env.step()` corresponds to **0.05 seconds**
- the control frequency is **20 Hz** (20 decisions per second)

In reinforcement learning, an action does not mean “move by this distance”.
It means:

Apply this control **for `dt` seconds**.

---

### Why Actions Are Time-Based, Not Distance-Based

Physical systems evolve over time:
- motors apply velocity or force over time
- gravity acts continuously
- motion is the result of time passing

Therefore, an RL action represents:
- how strongly to act (direction and magnitude)
- for a fixed duration (`dt`)

To convert this into a distance that can be sent to the actuator, we compute:

distance = speed × time

Which in code is:

delta = action × scale × dt

Where:
- `action` ∈ [-1, 1] is the agent’s intent
- `scale` is the maximum speed (units per second)
- `dt` is how long the speed is applied

The distance passed to `move_by` is the **result**, not the command itself.

---

### Why We Do Not Directly Use Distance

It is possible to define:

delta = action × scale

But this makes the environment dependent on how often `step()` is called.

Without `dt`:
- changing control frequency changes the physics
- motion becomes “per step” instead of “per second”
- gravity and deadzone behavior become inconsistent
- learned policies depend on Python loop timing

Including `dt` ensures:
- consistent motion per second
- stable gravity behavior
- physically meaningful actions

---

### Why `dt` Is Used in `_apply_action`

`_apply_action()` is responsible for converting **intent** into **physical commands**.

This conversion must answer two questions:
- how strong is the command? (`action × scale`)
- how long is it applied? (`dt`)

So `_apply_action()` computes:

delta = (intended speed) × (time)

and sends that distance to the actuator.

---

### Why `dt = 0.05` Was Chosen in This Environment

`dt = 0.05` corresponds to:
- 20 control updates per second
- a good balance between control smoothness and simulation speed

For this Stretch MuJoCo environment:
- actuators are slow and gravity-loaded
- very small `dt` (e.g. 0.01) adds unnecessary computation
- very large `dt` (e.g. 0.2) makes control too coarse

With `dt = 0.05`:
- full-range motion occurs in ~50–60 steps
- a 100-step episode represents ~5 seconds
- this matches typical manipulation timescales

`dt = 0.05` is a deliberate design choice, not a magic number.

---

### Important Clarifications

- `dt` is not tuned by the RL algorithm
- `dt` is not changed during training
- `dt` defines what one environment step means in time

Once chosen, it usually stays fixed.

---

### One-Sentence Summary

`dt` exists so that one RL step corresponds to a fixed amount of real time, and `action × scale × dt` converts agent intent into physically meaningful motion.


### Possible Solutions
How professionals deal with this

They do three main things.

1️⃣ Deadbands (extremely common)

A deadband is exactly what you wrote:

if abs(command) < epsilon:
    command = 0


This is standard practice.

You will see this in:

motor drivers

PID controllers

servo firmware

flight controllers

industrial robots

Why?

Because:

hardware cannot respond meaningfully below some threshold

small oscillations cause jitter

small commands waste computation

tiny sign flips cause instability

Your 1e-4 check for arm and gripper?

That is professional practice.

2️⃣ Minimum effective command logic

For gravity-loaded axes (like your lift), professionals often do one of:

A) Gravity compensation

Instead of blocking small positive commands, they add an offset:

u = control + gravity_comp


So small positive commands don’t sag.

In your simplified case, you approximated this with:

minimum upward delta

up-only guard

That’s conceptually similar.

B) Command floor

If command is positive but too small:

if 0 < u < min_effort:
    u = min_effort


This is also common in actuators with static friction.

You already reasoned about this earlier.

3️⃣ Rate limiting / saturation

Professionals also clamp commands:

def apply_command(u, eps=1e-4, max_u=0.02):
    if abs(u) < eps:
        return 0.0
    return np.clip(u, -max_u, max_u)

To prevent:

unrealistic speeds

controller windup

actuator damage

You already reasoned about this when discussing scale and limits.


