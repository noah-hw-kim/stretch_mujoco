### Limit objects to only apple

### Place object to the fixed position

### Place Stretch3 to the fixed position

## Create StretchReachEnv
### Define Joint Limits
limits = {
                Actuators.lift: (0.0, 1.1),
                Actuators.arm: (0.0, 0.52),
                Actuators.gripper: (-0.25, 0.53),
            }

### Define _get_obs() to return observation, achieved_goal, and desired_goal

### Define _apply_action() to move lift, arm, and gripper to a target position

### Define reset() and step() for GYM API

Copy/paste note for your .md file (issue + fix)
Issue observed: env.step() was running faster than the MuJoCo physics loop, so many steps returned nearly identical observations.
Evidence: sim_time didn’t advance on ~57% of steps and distance barely changed on ~70% of steps.
Root cause: sim.move_to(...) is effectively asynchronous (it sets targets). If we read observations immediately, we often read the same physics state because the simulator hasn’t stepped yet.
Why time.sleep(dt) is not ideal: it waits in wall-clock time, which can be unnecessarily slow (if sim runs faster than real-time) and still doesn’t strictly guarantee sim-time advanced by dt (if sim is overloaded).
Fix implemented: after applying the action, wait until simulator time (sim.pull_status().time) advances by at least a minimum amount (default: one control tick dt). This is Fetch-like: each RL step corresponds to real physics progress.
Result expected: near 0% “no sim-time advance” steps, and distance/state changes more consistently step-to-step.


### Model Training - Can't get a better performance
Every 20k time_steps take around 46~48 min

1k evalution
mean_dist=0.2987 episodes=5 episode_successes=0 success_rate=0.00%

30k evaluation
mean_dist=0.2168 episodes=5 episode_successes=0 success_rate=0.00%

50k evaluation
mean_dist=0.2202 episodes=5 episode_successes=0 success_rate=0.00%


### Training is so slow - Sim is running 0.446x as fast as realtime
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

