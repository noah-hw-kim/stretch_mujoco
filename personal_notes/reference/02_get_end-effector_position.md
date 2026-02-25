## Status
Verified 2026-01-08

# How to get the end-effector (EE) position from `sim.get_ee_pose()`

## What `sim.get_ee_pose()` returns
- It returns a **4×4 pose matrix**
- This matrix contains **both position and orientation** of the end-effector
- This is a standard format used in robotics and simulators like MuJoCo

Example:
[[ r11, r12, r13, x ],
[ r21, r22, r23, y ],
[ r31, r32, r33, z ],
[ 0 , 0 , 0 , 1 ]]

## The key idea (no math)
- The **last column** always stores the **position**
- The first 3 numbers of that column are:
  - x position
  - y position
  - z position
- This is not a convention of this project. It is a standard robotics format.

## How to extract EE position in code
```python
T = sim.get_ee_pose()
ee_pos = T[:3, 3]   # [x, y, z]
```

Why we use only the last column for reach tasks

* For a reach task, we only care about where the gripper is
* Orientation (rotation) does not matter yet
* Distance to the object is computed using position only

How you know this is correct (intuition check)
* If you move the arm or lift:
    * the numbers in the last column change

* If the robot does not move:
    * the last column stays the same

Summary
* sim.get_ee_pose() gives full pose (position + orientation)
* EE position is always in matrix[:3, 3]
* This [x, y, z] is what you use for:
    * reach distance
    * reward
    * success condition