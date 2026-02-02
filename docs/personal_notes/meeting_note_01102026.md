### 1/10/2026 Meeting Note

#### Find what exactly ee position means (Is it the end of the gripper or the gripper joint that can be replaced)


#### Double check if achieved_goal is actually reflecting the object position and desired_goal is reflecting the robot EE position

#### When moving the arm forward/retract (or opening/closing the gripper), EE Z drops (lift “sag”)

#### Step execution sanity check (FetchReach)
**Goal:** Verify that each `env.step()` corresponds to a meaningful, consistent physical movement (so PPO isn’t trained on “steps” that don’t actually advance the system).

**Method:** In `FetchReach`, apply a constant action for several steps and record `achieved_goal[x]` each tick.
Compute per-step displacement:

- `Δx_t = x_{t+1} - x_t`  
  (next x minus current x)

---

==============================
Test: +x  action=[1.0, 0.0, 0.0, 0.0]
EE pos per tick [x,y,z]:
  step 00: x=+1.341835 y=+0.749101 z=+0.534725
  step 01: x=+1.372007 y=+0.749095 z=+0.534557
  step 02: x=+1.404587 y=+0.749087 z=+0.534284
  step 03: x=+1.436547 y=+0.749077 z=+0.534013
  step 04: x=+1.468125 y=+0.749065 z=+0.533713
  step 05: x=+1.499085 y=+0.749050 z=+0.533381
  step 06: x=+1.529082 y=+0.749031 z=+0.533015
  step 07: x=+1.557664 y=+0.749005 z=+0.532678
  step 08: x=+1.584765 y=+0.748973 z=+0.532531
  step 09: x=+1.610626 y=+0.748933 z=+0.532567
  step 10: x=+1.634900 y=+0.748883 z=+0.532643
mean dpos per tick: [ 2.9307e-02 -2.2000e-05 -2.0800e-04]
min  dpos per tick: [ 2.4275e-02 -5.0000e-05 -3.6500e-04]
max  dpos per tick: [ 3.258e-02 -6.000e-06  7.600e-05]

==============================
Test: +y  action=[0.0, 1.0, 0.0, 0.0]
EE pos per tick [x,y,z]:
  step 00: x=+1.341835 y=+0.749101 z=+0.534725
  step 01: x=+1.341613 y=+0.779906 z=+0.534643
  step 02: x=+1.341320 y=+0.813340 z=+0.534544
  step 03: x=+1.341014 y=+0.846474 z=+0.534443
  step 04: x=+1.340671 y=+0.879638 z=+0.534337
  step 05: x=+1.340291 y=+0.912794 z=+0.534226
  step 06: x=+1.339868 y=+0.945941 z=+0.534107
  step 07: x=+1.339396 y=+0.979073 z=+0.533980
  step 08: x=+1.338868 y=+1.012183 z=+0.533844
  step 09: x=+1.338276 y=+1.045258 z=+0.533698
  step 10: x=+1.337606 y=+1.078286 z=+0.533540
mean dpos per tick: [-0.000423  0.032918 -0.000119]
min  dpos per tick: [-0.000669  0.030805 -0.000158]
max  dpos per tick: [-2.2200e-04  3.3434e-02 -8.2000e-05]

==============================
Test: +z  action=[0.0, 0.0, 1.0, 0.0]
EE pos per tick [x,y,z]:
  step 00: x=+1.341835 y=+0.749101 z=+0.534725
  step 01: x=+1.341705 y=+0.749100 z=+0.565143
  step 02: x=+1.341270 y=+0.749095 z=+0.598429
  step 03: x=+1.340817 y=+0.749090 z=+0.631390
  step 04: x=+1.340289 y=+0.749084 z=+0.664372
  step 05: x=+1.339693 y=+0.749078 z=+0.697319
  step 06: x=+1.339023 y=+0.749072 z=+0.730217
  step 07: x=+1.338268 y=+0.749065 z=+0.763039
  step 08: x=+1.337408 y=+0.749058 z=+0.795750
  step 09: x=+1.336414 y=+0.749050 z=+0.828298
  step 10: x=+1.335231 y=+0.749041 z=+0.860597
mean dpos per tick: [-6.6000e-04 -6.0000e-06  3.2587e-02]
min  dpos per tick: [-1.1830e-03 -9.0000e-06  3.0417e-02]
max  dpos per tick: [-1.3000e-04 -1.0000e-06  3.3287e-02]

==============================
Test: -z (optional)  action=[0.0, 0.0, -1.0, 0.0]
EE pos per tick [x,y,z]:
  step 00: x=+1.341835 y=+0.749101 z=+0.534725
  step 01: x=+1.341424 y=+0.749097 z=+0.504152
  step 02: x=+1.341127 y=+0.749094 z=+0.470702
  step 03: x=+1.340824 y=+0.749091 z=+0.437568
  step 04: x=+1.340643 y=+0.749089 z=+0.414538
  step 05: x=+1.340645 y=+0.749089 z=+0.414977
  step 06: x=+1.340628 y=+0.749089 z=+0.415187
  step 07: x=+1.340615 y=+0.749089 z=+0.415254
  step 08: x=+1.340603 y=+0.749089 z=+0.415276
  step 09: x=+1.340592 y=+0.749089 z=+0.415283
  step 10: x=+1.340581 y=+0.749089 z=+0.415286
mean dpos per tick: [-1.2500e-04 -1.0000e-06 -1.1944e-02]
min  dpos per tick: [-4.110e-04 -4.000e-06 -3.345e-02]
max  dpos per tick: [2.00e-06 0.00e+00 4.39e-04]

---

##### Interpretation (what action means in robotics RL)

In robotics (and robotics RL envs), an action like `[+1, 0, 0, 0]` typically does **not** mean “move +1 meter”.  
It means “apply the maximum allowed command in +x for one control tick.”

Common mappings:

- **Velocity-based control:** `Δx ≈ a_x * v_max * dt`
  - `a_x`: normalized action in x ([-1, +1])
  - `v_max`: max speed in x (m/s)
  - `dt`: seconds per step

- **Delta-position (step-size) control:** `Δx ≈ a_x * Δx_max`
  - `Δx_max`: max position increment per step (m/step)

So `a_x = 1.0` means “maximum allowed movement this step”, not “+1 meter”.

---

##### Choosing `dt` and action scale (practical recipe)

There isn’t a universal standard, but a common approach is:

1. **Pick `dt`** to match a realistic high-level control rate
   - Many robots / RL envs: **20–50 Hz** → `dt = 0.05 .. 0.02`
   - Low-level motor control can be 100–1000 Hz, but RL “decision steps” are often slower.

2. **Choose an action meaning**
   - Velocity control: `action ∈ [-1,1]` maps to `v = a * v_max`, integrate `x += v * dt`
   - Delta-position control: `action ∈ [-1,1]` maps to `Δx = a * Δx_max`

3. **Tune so the task is solvable within the horizon**
   - Rule of thumb: with max action, the agent should traverse typical start→goal distance in **~20–50 steps**
   - Example: distance ≈ 0.2 m, target ≈ 40 steps  
     \[
     \Delta x\_{\max} \approx 0.2/40 = 0.005 \text{ m} \; (= 5\text{ mm})
     \]

4. **Always clamp**
   - Clamp action to `[-1, 1]`
   - Clamp resulting `v` or `Δx` to safe limits

---

##### Option A — Velocity control (speed-based)

- `a_x ∈ [-1,1]`: joystick value
- `v_max`: max speed (m/s)
- `dt`: seconds per step

Example:

- `v_max = 0.25 m/s`, `dt = 0.02 s`, `a_x = +1`
- \[
  \Delta x \approx 0.25 \cdot 0.02 = 0.005\text{ m} = 5\text{ mm per step}
  \]

---

##### Option B — Delta-position control (step-size-based)

- `a_x ∈ [-1,1]`
- `Δx_max`: max distance per step (m/step)

Example:

- `Δx_max = 0.01 m` (1 cm/step)
- `a_x = +1` → +1 cm/step
- `a_x = +0.3` → +3 mm/step

---

##### Placeholder (to add later)

- Same measurement for **custom Stretch env** (e.g., action `(-1, 0, 0, 0)` average per-step movement)
- Compare per-step movement vs episode horizon to ensure PPO training signal matches actual execution




#### Follow this for the simplicity

1. Home the lift above the table height
2. Freeze other parts except the arm. Train to reach the target above
3. Freeze other parts except the arm and gripper. Train to reach the target or above
4. Freeze other parts except the arm, gripper, and lift. Train to reach the target or above
5. Now try the under the table condition



#### How to solve the problem under the table (Advanced)

1. Find the table position from the XML
2. Use the camera to get the table positions?
3. Complex penalties?


#### Is there an inverse kinematic function to use? -> Let's do it later
- Inverse kinematic automatically calculate and move to x,y, or z with predefine combinations of using arm, lift, gripper, etc
- It seems like they don't have ik API set up
- We can possibly do something like:
  x (base rotation or wriste yaw (limited) in the future)
  y (arm)
  z (lift)