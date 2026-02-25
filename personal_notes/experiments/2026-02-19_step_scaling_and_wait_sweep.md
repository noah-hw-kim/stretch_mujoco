## SITE MAP PER PARTS
### LIFT_STEP Map (Going Up)

(10 steps of action = +1)
Action: [1, 0, 0]

| LIFT_STEP (commanded) | start meas_lift | end meas_lift | total Δ (10 steps) | avg Δ per step | approx steps to move 1.1 m |
| --------------------: | --------------: | ------------: | -----------------: | -------------: | -------------------------: |
|                  0.02 |        0.000286 |      0.017236 |          0.01695 m |      0.00170 m |                 ~649 steps |
|                  0.03 |        0.615637 |      0.778239 |          0.16260 m |      0.01626 m |                  ~68 steps |
|                  0.04 |        0.024533 |      0.251348 |          0.22682 m |      0.02268 m |                  ~49 steps |
|                  0.06 |        0.053044 |      0.459936 |          0.40689 m |      0.04069 m |                  ~27 steps |
|                  0.08 |        0.085657 |      0.671671 |          0.58601 m |      0.05860 m |                  ~19 steps |

---

### LIFT_STEP Map (Going Down)

(10 steps of action = -1)
Action: [-1, 0, 0]

> Note: You only measured “down” for two step sizes so far.

| LIFT_STEP (commanded) | total Δ (10 steps) | avg Δ per step | notes               |
| --------------------: | -----------------: | -------------: | ------------------- |
|                  0.04 |           -0.038 m |     -0.00380 m | measured going down |
|                  0.06 |           -0.058 m |     -0.00580 m | measured going down |

> Note: the 0.03 row comes from a separate run (start lift position was ~0.616, not reset to 0).


### ARM STEP (10 steps of action = +1, delta_arm = 0.015) with action [0,1,0]

| ARM_STEP (commanded) | start meas_arm | end meas_arm | total Δ (10 steps) | avg Δ per step | approx steps to move 0.52 m |
|---:|---:|---:|---:|---:|---:|
| 0.015 | 0.111947 | 0.220651 | 0.10870 m | 0.01087 m | ~48 steps |

### ARM Retraction

(10 steps of action = -1)
delta_arm = 0.015
Action: [0, -1, 0]

| ARM_STEP (commanded) | start meas_arm | end meas_arm | total Δ (10 steps) | avg Δ per step | approx steps to move 0.52 m |
| -------------------: | -------------: | -----------: | -----------------: | -------------: | --------------------------: |
|                0.015 |       0.176194 |     0.103928 |         -0.07227 m |     -0.00723 m |                   ~72 steps |

Raw log sample:

```
=== from info ===
i=0 lift pos/vel=(0.428469, -0.000002) arm pos/vel=(0.176194, -0.026975) grip pos/vel=(-0.063956, 0.000073)
=== from info ===
i=1 lift pos/vel=(0.428469, -0.000000) arm pos/vel=(0.168075, -0.021952) grip pos/vel=(-0.063976, -0.000012)
=== from info ===
i=2 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.162630, -0.030103) grip pos/vel=(-0.063970, -0.000014)
=== from info ===
i=3 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.155283, -0.024358) grip pos/vel=(-0.063991, -0.000011)
=== from info ===
i=4 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.146414, -0.019497) grip pos/vel=(-0.064007, -0.000007)
=== from info ===
i=5 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.138002, -0.020919) grip pos/vel=(-0.064007, -0.000008)
=== from info ===
i=6 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.130496, -0.023790) grip pos/vel=(-0.064004, -0.000009)
=== from info ===
i=7 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.121638, -0.019525) grip pos/vel=(-0.064014, -0.000006)
=== from info ===
i=8 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.112783, -0.019510) grip pos/vel=(-0.064014, -0.000006)
=== from info ===
i=9 lift pos/vel=(0.428468, -0.000000) arm pos/vel=(0.103928, -0.019510) grip pos/vel=(-0.064014, -0.000006)
```

---

### GRIPPER STEP (10 steps of action = +1, delta_grip = 0.025) with action [0,0,1]

| GRIP_STEP (commanded) | start meas_grip | end meas_grip | total Δ (10 steps) | avg Δ per step | approx steps to move 0.78 |
|---:|---:|---:|---:|---:|---:|
| 0.025 | -0.045328 | 0.104457 | 0.14979 | 0.01498 | ~52 steps |
-> slow than 0.025


## GRIPPER Closing
(10 steps of action = -1)
delta_grip = 0.025
Action: [0, 0, -1]

| GRIP_STEP (commanded) | start meas_grip | end meas_grip | total Δ (10 steps) | avg Δ per step | approx steps to move 0.78 |
| --------------------: | --------------: | ------------: | -----------------: | -------------: | ------------------------: |
|                 0.025 |        0.062487 |     -0.122828 |           -0.18531 |       -0.01853 |                 ~42 steps |

Raw log sample:

```
=== from info ===
i=0 lift pos/vel=(0.428465, -0.000001) arm pos/vel=(0.097746, -0.000641) grip pos/vel=(0.062487, -0.000499)
=== from info ===
i=1 lift pos/vel=(0.428465, -0.000001) arm pos/vel=(0.097731, -0.000550) grip pos/vel=(0.037024, -0.000509)
=== from info ===
i=2 lift pos/vel=(0.428465, -0.000004) arm pos/vel=(0.097736, -0.000904) grip pos/vel=(0.013074, -0.000618)
=== from info ===
i=3 lift pos/vel=(0.428465, -0.000003) arm pos/vel=(0.097727, -0.000817) grip pos/vel=(-0.009639, -0.000603)
=== from info ===
i=4 lift pos/vel=(0.428465, -0.000003) arm pos/vel=(0.097722, -0.000767) grip pos/vel=(-0.031149, -0.000613)
=== from info ===
i=5 lift pos/vel=(0.428465, -0.000003) arm pos/vel=(0.097720, -0.000724) grip pos/vel=(-0.051537, -0.000623)
=== from info ===
i=6 lift pos/vel=(0.428465, -0.000003) arm pos/vel=(0.097718, -0.000687) grip pos/vel=(-0.070828, -0.000632)
=== from info ===
i=7 lift pos/vel=(0.428465, -0.000002) arm pos/vel=(0.097717, -0.000651) grip pos/vel=(-0.089114, -0.000640)
=== from info ===
i=8 lift pos/vel=(0.428465, -0.000002) arm pos/vel=(0.097714, -0.000548) grip pos/vel=(-0.106452, -0.000627)
=== from info ===
i=9 lift pos/vel=(0.428465, -0.000002) arm pos/vel=(0.097717, -0.000585) grip pos/vel=(-0.122828, -0.000656)
```


## ----- WE WILL USE THESE PARAMETERS FOR MOVING FROM NOW ON -----
LIFT_MIN = 0.008
LIFT_STEP = 0.03

ARM_MIN = 0.0001 # actual min is 0.000006
ARM_STEP = 0.015

GRIPPER_MIN = 0.004
GRIPPER_STEP = 0.025




## Waiting the move_by()

### Without waiting, we have no time to advance each parts. Barely moves any
=== from info ===
i=0 lift pos/vel=(1.000623, 0.000000) arm pos/vel=(0.115073, 0.000000) grip pos/vel=(-0.038968, 0.000000)
sim time delta: 0.0020000000000095497
=== from info ===
i=1 lift pos/vel=(1.000623, 0.000000) arm pos/vel=(0.115073, 0.000000) grip pos/vel=(-0.038968, 0.000000)
sim time delta: 0.0
=== from info ===
i=2 lift pos/vel=(1.000623, 0.000000) arm pos/vel=(0.115073, 0.000000) grip pos/vel=(-0.038968, 0.000000)
sim time delta: 0.0020000000000095497
=== from info ===
i=3 lift pos/vel=(1.000623, 0.000000) arm pos/vel=(0.115073, 0.000000) grip pos/vel=(-0.038968, 0.000000)
sim time delta: 0.0
=== from info ===
i=4 lift pos/vel=(1.000623, 0.000000) arm pos/vel=(0.115073, 0.000000) grip pos/vel=(-0.038968, 0.000000)
sim time delta: 0.0
=== from info ===
i=5 lift pos/vel=(1.000629, 0.002900) arm pos/vel=(0.115077, 0.002252) grip pos/vel=(-0.038912, 0.001788)
sim time delta: 0.0020000000000095497
=== from info ===
i=6 lift pos/vel=(1.000629, 0.002900) arm pos/vel=(0.115077, 0.002252) grip pos/vel=(-0.038912, 0.001788)
sim time delta: 0.0
=== from info ===
i=7 lift pos/vel=(1.000640, 0.005644) arm pos/vel=(0.115086, 0.004300) grip pos/vel=(-0.038791, 0.003903)
sim time delta: 0.0020000000000095497
=== from info ===
...
sim time delta: 0.0
=== from info ===
i=9 lift pos/vel=(1.000640, 0.005644) arm pos/vel=(0.115086, 0.004300) grip pos/vel=(-0.038791, 0.003903)
sim time delta: 0.0


## ----- WE NEED TO WAIT SOMETIMES TO MAKE ROBOT MOVING BUT IT TAKES TOO LONG TO USE PREBUILT sim.wait_while_is_moving() -----

### Added custom waiting function: _wait_sim_dt()
_wait_sim_dt() iterates

t0 = float(self.sim.pull_status().time)
k = CONTROL_HOLD (adjust if I want to give long time for holding)
target = t0 + float(self.dt) * k

while True:
    t = float(self.sim.pull_status().time)
    if t >= target:
        break
    if time.time() - wall_start > timeout_s:
        break
    time.sleep(0.001)  # 1ms yield


## ----- WE WILL USE THESE PARAMETERS FOR TESTING -----
LIFT_MIN = 0.008
LIFT_STEP = 0.04

ARM_MIN = 0.0001 # actual min is 0.000006
ARM_STEP = 0.015

GRIPPER_MIN = 0.004
GRIPPER_STEP = 0.025

The reason we changed LIFT_STEP from 0.03 to 0.04 is that we no longer use sim.wait_while_is_moving() so the motion cuts off earlier



### after adding _wait_sim_dt() in _apply_action() right after each part move_by()
used k = 1 so the waiting time take only t0 + 0.05

=== from info ===
i=0 lift pos/vel=(0.684382, 0.935183) arm pos/vel=(0.099996, 0.000064) grip pos/vel=(-0.063958, 0.000005)
=== from info ===
i=1 lift pos/vel=(0.686250, 0.933746) arm pos/vel=(0.099997, 0.000085) grip pos/vel=(-0.063958, 0.000006)
=== from info ===
i=2 lift pos/vel=(0.686250, 0.933746) arm pos/vel=(0.099997, 0.000085) grip pos/vel=(-0.063958, 0.000006)
=== from info ===
i=3 lift pos/vel=(0.686250, 0.933746) arm pos/vel=(0.099997, 0.000085) grip pos/vel=(-0.063958, 0.000006)
=== from info ===
i=4 lift pos/vel=(0.688114, 0.932114) arm pos/vel=(0.099997, 0.000101) grip pos/vel=(-0.063957, 0.000008)
=== from info ===
i=5 lift pos/vel=(0.688114, 0.932114) arm pos/vel=(0.099997, 0.000101) grip pos/vel=(-0.063957, 0.000008)
=== from info ===
i=6 lift pos/vel=(0.688114, 0.932114) arm pos/vel=(0.099997, 0.000101) grip pos/vel=(-0.063957, 0.000008)
=== from info ===
i=7 lift pos/vel=(0.689893, 0.889615) arm pos/vel=(0.100001, 0.002321) grip pos/vel=(-0.063898, 0.001909)
=== from info ===
i=8 lift pos/vel=(0.689893, 0.889615) arm pos/vel=(0.100001, 0.002321) grip pos/vel=(-0.063898, 0.001909)
=== from info ===
i=9 lift pos/vel=(0.691589, 0.847877) arm pos/vel=(0.100010, 0.004368) grip pos/vel=(-0.063773, 0.004006)

result = lift barely moves around 0.002 per 3 ticks


### after adding _wait_sim_dt() in step() after _apply_action()
used k = 1 so the waiting time take only t0 + 0.05

=== from info ===
i=0 lift pos/vel=(0.793613, 0.060151) arm pos/vel=(0.100952, 0.030857) grip pos/vel=(-0.052250, 0.017020)
=== from info ===
i=1 lift pos/vel=(0.797260, 0.074590) arm pos/vel=(0.102811, 0.038415) grip pos/vel=(-0.037154, 0.013065)
=== from info ===
i=2 lift pos/vel=(0.801287, 0.077720) arm pos/vel=(0.104847, 0.039741) grip pos/vel=(-0.023395, 0.013390)
=== from info ===
i=3 lift pos/vel=(0.805395, 0.078394) arm pos/vel=(0.106928, 0.040246) grip pos/vel=(-0.010028, 0.012590)
=== from info ===
i=4 lift pos/vel=(0.809521, 0.078539) arm pos/vel=(0.109024, 0.040340) grip pos/vel=(0.002709, 0.012090)
=== from info ===
i=5 lift pos/vel=(0.813650, 0.078569) arm pos/vel=(0.111124, 0.040335) grip pos/vel=(0.014901, 0.011541)
=== from info ===
i=6 lift pos/vel=(0.817781, 0.078575) arm pos/vel=(0.113224, 0.040300) grip pos/vel=(0.026554, 0.011034)
=== from info ===
i=7 lift pos/vel=(0.821911, 0.078574) arm pos/vel=(0.115322, 0.040292) grip pos/vel=(0.037695, 0.010545)
=== from info ===
i=8 lift pos/vel=(0.826042, 0.078573) arm pos/vel=(0.117421, 0.040248) grip pos/vel=(0.048347, 0.010079)
=== from info ===
i=9 lift pos/vel=(0.830172, 0.078572) arm pos/vel=(0.119518, 0.040207) grip pos/vel=(0.058529, 0.009634)

result -> stable but too small per tick. ~0.0786 m/s steady (after first step)
Not ideal for your stated 100-step coverage constraint, unless you increase step sizes or episode length.


used k = 2 so the waiting time take only t0 + 0.1
=== from info ===
i=0 lift pos/vel=(0.794970, 0.070373) arm pos/vel=(0.102571, 0.036734) grip pos/vel=(-0.045958, 0.001718)
=== from info ===
i=1 lift pos/vel=(0.802427, 0.070833) arm pos/vel=(0.106329, 0.036335) grip pos/vel=(-0.028441, 0.001258)
=== from info ===
i=2 lift pos/vel=(0.809877, 0.070831) arm pos/vel=(0.110065, 0.036330) grip pos/vel=(-0.011887, 0.001235)
=== from info ===
i=3 lift pos/vel=(0.817326, 0.070830) arm pos/vel=(0.113804, 0.036270) grip pos/vel=(0.003779, 0.001193)
=== from info ===
i=4 lift pos/vel=(0.824775, 0.070827) arm pos/vel=(0.117543, 0.036229) grip pos/vel=(0.018607, 0.001154)
=== from info ===
i=5 lift pos/vel=(0.832224, 0.070827) arm pos/vel=(0.121283, 0.036151) grip pos/vel=(0.032641, 0.001121)
=== from info ===
i=6 lift pos/vel=(0.839673, 0.070824) arm pos/vel=(0.125022, 0.036111) grip pos/vel=(0.045925, 0.001091)
=== from info ===
i=7 lift pos/vel=(0.847121, 0.070822) arm pos/vel=(0.128760, 0.036069) grip pos/vel=(0.058499, 0.001063)
=== from info ===
i=8 lift pos/vel=(0.854570, 0.070823) arm pos/vel=(0.132499, 0.035971) grip pos/vel=(0.070402, 0.001036)
=== from info ===
i=9 lift pos/vel=(0.862018, 0.070820) arm pos/vel=(0.136237, 0.035937) grip pos/vel=(0.081670, 0.001011)

result -> close to my goal 70% of the full range in 100 steps ~0.0708 m/s steady
Most “principled default” for learning, if you adjust just a little to meet coverage reliably.
For now, let's use k=2 for the standard threshold + adjust LIFT_STEP = 0.045 to make it more moving

used k = 3 so the waiting time take only t0 + 0.15
=== from info ===
i=0 lift pos/vel=(0.797429, 0.066354) arm pos/vel=(0.104333, 0.033346) grip pos/vel=(-0.045380, 0.000606)
=== from info ===
i=1 lift pos/vel=(0.808045, 0.062020) arm pos/vel=(0.109642, 0.031020) grip pos/vel=(-0.027440, 0.000587)
=== from info ===
i=2 lift pos/vel=(0.818514, 0.062317) arm pos/vel=(0.114877, 0.031153) grip pos/vel=(-0.010478, 0.000600)
=== from info ===
i=3 lift pos/vel=(0.828993, 0.062294) arm pos/vel=(0.120118, 0.031132) grip pos/vel=(0.005583, 0.000611)
=== from info ===
i=4 lift pos/vel=(0.839471, 0.062294) arm pos/vel=(0.125359, 0.031109) grip pos/vel=(0.020790, 0.000621)
=== from info ===
i=5 lift pos/vel=(0.849948, 0.062291) arm pos/vel=(0.130599, 0.031117) grip pos/vel=(0.035189, 0.000631)
=== from info ===
i=6 lift pos/vel=(0.860426, 0.062290) arm pos/vel=(0.135838, 0.031099) grip pos/vel=(0.048823, 0.000640)
=== from info ===
i=7 lift pos/vel=(0.870903, 0.062289) arm pos/vel=(0.141076, 0.031068) grip pos/vel=(0.061734, 0.000648)
=== from info ===
i=8 lift pos/vel=(0.881379, 0.062287) arm pos/vel=(0.146314, 0.031051) grip pos/vel=(0.073961, 0.000656)
=== from info ===
i=9 lift pos/vel=(0.891856, 0.062286) arm pos/vel=(0.151551, 0.031022) grip pos/vel=(0.085541, 0.000663)

result -> k=3: ~0.0623 m/s steady
Best if your tasks are coarse reach/move tasks and you want guaranteed coverage, but may hurt fine control later.


## ----- WE WILL USE THESE PARAMETERS FOR TESTING -----
LIFT_MIN = 0.008
LIFT_STEP = 0.045

ARM_MIN = 0.0001 # actual min is 0.000006
ARM_STEP = 0.015

GRIPPER_MIN = 0.004
GRIPPER_STEP = 0.025


### after adding _wait_sim_dt() in every parts moving in _apply_action()
=== from info ===
i=0 lift pos/vel=(0.681730, 0.013436) arm pos/vel=(0.105951, 0.029485) grip pos/vel=(-0.045559, 0.001176)
=== from info ===
i=1 lift pos/vel=(0.698831, 0.040620) arm pos/vel=(0.114929, 0.027927) grip pos/vel=(-0.026468, 0.001107)
=== from info ===
i=2 lift pos/vel=(0.716500, 0.038885) arm pos/vel=(0.120216, 0.011151) grip pos/vel=(-0.008399, 0.001064)
=== from info ===
i=3 lift pos/vel=(0.734133, 0.038993) arm pos/vel=(0.127287, 0.028734) grip pos/vel=(0.008741, 0.001042)
=== from info ===
i=4 lift pos/vel=(0.751767, 0.038985) arm pos/vel=(0.136203, 0.027732) grip pos/vel=(0.025010, 0.001010)
=== from info ===
i=5 lift pos/vel=(0.769400, 0.038984) arm pos/vel=(0.141493, 0.010965) grip pos/vel=(0.040415, 0.000979)
=== from info ===
i=6 lift pos/vel=(0.787033, 0.038983) arm pos/vel=(0.148579, 0.027894) grip pos/vel=(0.042888, 0.000402)
=== from info ===
i=7 lift pos/vel=(0.804664, 0.038981) arm pos/vel=(0.157468, 0.027587) grip pos/vel=(0.056658, 0.000967)
=== from info ===
i=8 lift pos/vel=(0.822295, 0.038978) arm pos/vel=(0.166284, 0.027601) grip pos/vel=(0.070448, 0.000936)
=== from info ===
i=9 lift pos/vel=(0.839925, 0.038979) arm pos/vel=(0.175118, 0.026999) grip pos/vel=(0.072996, 0.000408)

Lift Average per-step = 0.158195 / 9 = 0.017577 m/step (≈ 17.6 mm/step)
Arm Average per-step = 0.069167 / 9 = 0.007684 m/step (≈ 7.68 mm/step)
Gripper Average per-step = 0.118555 / 9 = 0.013173 units/step (≈ 13.17 mm-equivalent/step in your units)

Cons
- Breaks synchronous MDP assumption that env.step(action) produces a single state transition after all commands are applied at the same sim time. This makes policies harder to learn because the same action vector can have different effects depending on internal ordering or which actuators hit the guard first.
- Total simulated time per env.step becomes variable (depends on how many actuators moved). That complicates reward shaping and temporal credit assignment. If you moved 3 actuators you may advance 3×dt_control vs 1×dt_control for a single actuator action.
- Introduces an ordering bias: actuator 0 is always applied earlier than actuator 1, so it consistently “sees” a different state and may always get more effective time to move. This biases learning and can produce surprising behaviors.