## Create StretchReachEnv GYM API

### Define Joint Limits (meter)
```
limits = {
    Actuators.lift: (0.0, 1.1),
    Actuators.arm: (0.0, 0.52),
    Actuators.gripper: (-0.25, 0.53),
}
```


### Limit objects to apple and fix its position
```
obj_name = "apple0_main"
```

```
pos=[ 0.7, -0.6, 0.96], quat=[ 0.974, 0., 0., -0.226]
```
* Fixed value for x and y pos for easier grip
* Keep z position and quat as is when the object is created.


### Place Stretch3 to the fixed position
```
import math

# your measured base pose
x = 0.6713510702553015
y = -1.3006141423127842
theta = 3.1147300993742104

# wrap theta to [-pi, pi] (optional but nice)
theta = math.atan2(math.sin(theta), math.cos(theta))

# Use the z from the default fixture pose you saw printed once (replace this!)
z0 = 0.0  # <-- replace with the third value from the printed "Adding stretch..." pos

w = math.cos(theta / 2.0)
z = math.sin(theta / 2.0)

robot_spawn_pose = {
    "pos": f"{x} {y} {z0}",
    "quat": f"{w} 0 0 {z}",
}
```
* Need to confirm z0 value. The generator could expects a nonzero z (even a few cm), then z0=0.0 can cause issues. * Check the initial status when the simulator generated originally.


### Define Action Space
1. Current action space = Box (-1.0, 1.0, (3,), float32)
| Num | Action | Control Min | Control Max | Name (in corresponding XML file) | Joint | Unit |
|---:|---|---:|---:|---|---|---|
| 0 | lift | -1 | 1 |  | hinge | position (m) |
| 1 | arm | -1 | 1 |  | hinge | position (m) |
| 2 | gripper | -1 | 1 |  | hinge | position (m) |


2. Future action space = Box(-1.0, 1.0, (4,), float32) that represents the following

Action space: `Box(-1.0, 1.0, (4,), float32)`  
An action represents the Cartesian displacement *(dx, dy, dz)* of the end effector.  
The last action controls gripper open/close, but in **Reach** it has no effect (no object). :contentReference[oaicite:1]{index=1}

| Num | Action | Control Min | Control Max | Name (in corresponding XML file) | Joint | Unit |
|---:|---|---:|---:|---|---|---|
| 0 | Displacement of the end effector in the x direction `dx` | -1 | 1 | `robot0:mocap` | hinge | position (m) |
| 1 | Displacement of the end effector in the y direction `dy` | -1 | 1 | `robot0:mocap` | hinge | position (m) |
| 2 | Displacement of the end effector in the z direction `dz` | -1 | 1 | `robot0:mocap` | hinge | position (m) |
| 3 | `-` (gripper open/close, unused in Reach) | -1 | 1 | `-` | `-` | `-` |


### Define Observation Space


### Define _get_obs() to return observation, achieved_goal, and desired_goal
This env follows the common Gym “goal” format and returns a dict with:

- `observation`: features for the policy (joint + positions)
- `achieved_goal`: **where the robot is now** (end-effector position)
- `desired_goal`: **where the robot wants to be** (object position)

How they are computed:
- Read joints from `sim.pull_status()`:
  - `lift = s.lift.pos`, `arm = s.arm.pos`, `grip = s.gripper.pos`
- Read end-effector (EE) position from `sim.get_ee_pose()`:
  - `achieved_goal = ee = T[:3, 3]`
- Read object position from `sim.pull_objects_state()`:
  - `desired_goal = obj_pos`

Then `observation` is just everything packed into one vector:
```text
observation = [lift, arm, grip, ee_x, ee_y, ee_z, obj_x, obj_y, obj_z, obstacle_x, obstacle_y, obstacle_z, 1] [0.05, 0.02, -0.2]
```

Meaning in this task:
- The goal is to make the EE reach the object, so distance is:
  - `distance = ||achieved_goal - desired_goal||`



### Define _apply_action() to move lift, arm, and gripper to a target position
The action is 3 numbers in `[-1, 1]`:
```text
a = [a_lift, a_arm, a_gripper]
```

We convert it into a **small per-step joint change** using `scales`:
```python
delta = clip(a, -1, 1) * scales
```

Then we:
1) read current joints (`cur_*`) from `sim.pull_status()`
2) compute targets (`tgt_*`) and clamp to joint limits
3) send position commands:
```python
sim.move_to(Actuators.lift,    tgt_lift)
sim.move_to(Actuators.arm,     tgt_arm)
sim.move_to(Actuators.gripper, tgt_grip)
```

Example:
- `scales = [0.03, 0.03, 0.02]`
- `a = [0.5, 1.0, 0.2]`

Then:
```text
delta = [0.015, 0.030, 0.004]
```

So the env tries to move each joint a little from its current value:
```text
tgt_lift = cur_lift + 0.015
tgt_arm  = cur_arm  + 0.030
tgt_grip = cur_grip + 0.004
```
(then clipped to allowed joint ranges)

### Training is so slow - Sim is running 0.446x as fast as realtime
Every 20k time_steps take around 46~48 min
```
import time

s0 = sim.pull_status()
t_sim0 = float(s0.time)
t_wall0 = time.time()

# wait ~5s sim-time without doing anything else
target = t_sim0 + 5.0
while float(sim.pull_status().time) < target:
    time.sleep(0.01)

s1 = sim.pull_status()
t_sim1 = float(s1.time)
t_wall1 = time.time()

print("sim advanced:", t_sim1 - t_sim0, "sec")
print("wall elapsed:", t_wall1 - t_wall0, "sec")
print("sim/wall:", (t_sim1 - t_sim0) / (t_wall1 - t_wall0))
print("status:", s1.sim_to_real_time_ratio_msg)
```

Result:
```
sim advanced: 5.004000000023893 sec
wall elapsed: 11.035330533981323 sec
sim/wall: 0.45345266139650026
status: Sim is running 0.446x as fast as realtime
```

### Model Training - Can't get a better performance
1k evalution
mean_dist=0.2987 episodes=5 episode_successes=0 success_rate=0.00%

30k evaluation
mean_dist=0.2168 episodes=5 episode_successes=0 success_rate=0.00%

50k evaluation
mean_dist=0.2202 episodes=5 episode_successes=0 success_rate=0.00%

#### PROBLEM: Wait for dt - Robot doesn't have enough time to move
Issue observed: env.step() was running faster than the MuJoCo physics loop, so many steps returned nearly identical observations.
Evidence: sim_time didn’t advance on ~57% of steps and distance barely changed on ~70% of steps.
Root cause: sim.move_to(...) is effectively asynchronous (it sets targets). If we read observations immediately, we often read the same physics state because the simulator hasn’t stepped yet.
Why time.sleep(dt) is not ideal: it waits in wall-clock time, which can be unnecessarily slow (if sim runs faster than real-time) and still doesn’t strictly guarantee sim-time advanced by dt (if sim is overloaded).
Fix implemented: after applying the action, wait until simulator time (sim.pull_status().time) advances by at least a minimum amount (default: one control tick dt). This is Fetch-like: each RL step corresponds to real physics progress.
Result expected: near 0% “no sim-time advance” steps, and distance/state changes more consistently step-to-step.

#### PROBLEM: By looking the eval_run(), it seems the robot gets reset before it gets more closer to the target. I think we can increment the max_steps per episodes so that the robot has an enough time to approach.

SOLUTION:
Extended max_steps per episodes from 100 to 200

#### PROBLEM: Policy tends to extend the arm low, collide with/press into the counter overhang, and then keeps applying similar actions.
Distance-to-goal plateaus (little to no improvement), so training gets stuck in this local minimum even with longer episodes.

SOLUTION (env shaping):
Add a “stuck/overhang” penalty: track distance-to-goal over a short window (e.g., 15 steps).
If distance does not improve by at least a small threshold AND the robot posture suggests the overhang failure mode (arm extended, EE below target height),
subtract an extra penalty (and optionally truncate). This makes “keep pushing into the lip” less rewarding and encourages
retracting/lifting to try a new approach.
In case, the gripper stays right below the counter lip of the object, we penalize when the robot is below the object in certain below pos. If it's enough went up stop penelizing

After adding penalties
t=  2000  mean_dist=0.2794  succ=0.0%  episodes=5  stuck_rate=16.9%
t=  4000  mean_dist=0.2516  succ=0.0%  episodes=5  stuck_rate=0.0%
t=  6000  mean_dist=0.2039  succ=0.0%  episodes=5  stuck_rate=0.0%
t=  8000  mean_dist=0.2035  succ=0.0%  episodes=5  stuck_rate=0.0%
t= 10000  mean_dist=0.2154  succ=0.0%  episodes=5  stuck_rate=0.0%



