# StretchReachEnv – Environment Design Notes

This document describes the stable architecture and design decisions for the custom StretchReach Gym environment.

---

## 1. Goal Format and Reward Geometry

The task goal is to move the end effector (EE) to the object.

Distance metric:

```
distance = ||achieved_goal - desired_goal||
```

Where:

* `achieved_goal` = current end-effector position
* `desired_goal` = object position

---

## 2. Action Application

### Action Space (Current)

```
Box(-1.0, 1.0, (3,), float32)
```

```
a = [a_lift, a_arm, a_gripper]
```

Each action value lies in `[-1, 1]`.

---

### Action-to-Command Mapping

Actions are converted into small per-step joint deltas:

```python
delta = clip(a, -1, 1) * scales
```

Example:

```
scales = [0.03, 0.03, 0.02]
a = [0.5, 1.0, 0.2]
delta = [0.015, 0.030, 0.004]
```

Targets are computed as:

```
tgt_lift = cur_lift + delta[0]
tgt_arm  = cur_arm  + delta[1]
tgt_grip = cur_grip + delta[2]
```

Targets are then clamped to joint limits and sent to the simulator:

```python
sim.move_to(Actuators.lift,    tgt_lift)
sim.move_to(Actuators.arm,     tgt_arm)
sim.move_to(Actuators.gripper, tgt_grip)
```

---

## 3. Joint Limits

```
limits = {
    Actuators.lift: (0.0, 1.1),
    Actuators.arm: (0.0, 0.52),
    Actuators.gripper: (-0.25, 0.53),
}
```

---

## 4. Object Configuration

Only the apple is used for this task.

```
obj_name = "apple0_main"
```

Fixed spawn configuration:

```
pos=[0.7, -0.6, 0.96]
quat=[0.974, 0., 0., -0.226]
```

* X/Y fixed for easier grasp testing
* Z and orientation remain as defined in object creation

---

## 5. Robot Spawn Configuration

Example spawn configuration:

```python
import math

x = 0.6713510702553015
y = -1.3006141423127842
theta = 3.1147300993742104

theta = math.atan2(math.sin(theta), math.cos(theta))

z0 = 0.0  # confirm correct base height

w = math.cos(theta / 2.0)
z = math.sin(theta / 2.0)

robot_spawn_pose = {
    "pos": f"{x} {y} {z0}",
    "quat": f"{w} 0 0 {z}",
}
```

---

## 6. Observation Structure

The environment follows Gym goal format and returns:

* `observation`
* `achieved_goal`
* `desired_goal`

Values are computed as:

* Joint states from `sim.pull_status()`
* EE position from `sim.get_ee_pose()`
* Object position from `sim.pull_objects_state()`

Packed observation example:

```
[lift, arm, grip,
 ee_x, ee_y, ee_z,
 obj_x, obj_y, obj_z]
```

---

## 7. Step Timing and Simulator Advance

Each `env.step()` must correspond to real physics progress.

Problem observed:

* `sim.move_to(...)` sets targets asynchronously.
* If observations are read immediately, physics may not have advanced.

Solution implemented:

After applying an action, wait until simulator time advances by at least one control tick (`dt`).

This ensures:

* Each RL step corresponds to real physics progress
* Observations change consistently
* Training behavior matches physical dynamics

---

## 8. dt and Time-Based Control

If:

```
dt = 0.05
```

Then:

* One `env.step()` represents 0.05 seconds
* Control frequency = 20 Hz

Actions represent velocity applied for `dt` seconds.

```
delta = action × scale × dt
```

This ensures:

* Physics consistency
* Stable gravity behavior
* Independence from Python loop timing

`dt = 0.05` was chosen as a balance between smooth control and simulation cost.

---

## 9. Future: Cartesian Action Space (Not Implemented)

Potential alternative:

```
Box(-1.0, 1.0, (4,), float32)
```

Representing EE displacement `(dx, dy, dz)` plus gripper control.

Currently documented for exploration, not active.
