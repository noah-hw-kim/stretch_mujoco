# 2026-01 – Training and Step Timing Debug Log

This document records observed training behavior, simulator performance issues, and environment modifications made during debugging.

---

## 1. Simulator Speed Benchmark

### Observation

Training was extremely slow.

Every 20k timesteps required approximately 46–48 minutes.

### Measurement Code

```python
import time

s0 = sim.pull_status()
t_sim0 = float(s0.time)
t_wall0 = time.time()

# wait ~5s sim-time
while float(sim.pull_status().time) < t_sim0 + 5.0:
    time.sleep(0.01)

s1 = sim.pull_status()
t_sim1 = float(s1.time)
t_wall1 = time.time()

print("sim advanced:", t_sim1 - t_sim0)
print("wall elapsed:", t_wall1 - t_wall0)
print("sim/wall:", (t_sim1 - t_sim0) / (t_wall1 - t_wall0))
print("status:", s1.sim_to_real_time_ratio_msg)
```

### Result

```
sim advanced: 5.004 sec
wall elapsed: 11.035 sec
sim/wall: 0.453
status: Sim is running 0.446x as fast as realtime
```

Conclusion:
Simulation runs at ~0.45x realtime.

---

## 2. Training Performance Plateau

### Evaluation Results

1k steps:

```
mean_dist=0.2987  success_rate=0%
```

30k steps:

```
mean_dist=0.2168  success_rate=0%
```

50k steps:

```
mean_dist=0.2202  success_rate=0%
```

Training plateaued without achieving successful reaches.

---

## 3. Problem: Physics Not Advancing Per Step

### Symptom

* Observations repeated across steps
* Distance barely changed
* ~57% of steps had no sim-time advance

### Root Cause

* `sim.move_to(...)` is asynchronous
* Observations were read before physics advanced

### Fix

After applying an action, wait until simulator time advances by at least `dt`.

Expected result:

* Near 0% "no sim-time advance" steps
* Consistent state updates per RL step

---

## 4. Problem: Episodes Too Short

### Observation

Robot was reset before having enough time to approach the object.

### Solution

Increased episode length:

```
max_steps: 100 → 200
```

---

## 5. Problem: Policy Gets Stuck Under Counter Overhang

### Failure Mode

* Arm extends low
* Collides with counter lip
* Distance plateaus
* Policy repeatedly applies similar actions

### Environment Shaping Solution

Added "stuck/overhang" penalty:

* Track distance-to-goal over a short window (e.g., 15 steps)
* If distance does not improve AND posture matches overhang configuration
* Apply extra penalty (optionally truncate episode)

Penalty condition example:

* EE below object height threshold
* Arm extended forward

---

## 6. Training Results After Penalty

```
t=  2000  mean_dist=0.2794  stuck_rate=16.9%
t=  4000  mean_dist=0.2516  stuck_rate=0.0%
t=  6000  mean_dist=0.2039  stuck_rate=0.0%
t=  8000  mean_dist=0.2035  stuck_rate=0.0%
t= 10000  mean_dist=0.2154  stuck_rate=0.0%
```

Result:

* Stuck behavior eliminated
* Mean distance improved
* Success rate still 0%

---

## Current Status

* Step timing fix implemented
* Episode length increased
* Overhang penalty added
* Training still does not achieve full success

Further reward shaping or control refinement required.
