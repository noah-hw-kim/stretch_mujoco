# docs/issues/waiting_time_control_design.md

# Issue — Waiting Time Control Design

## Problem

`move_by()` produces negligible motion unless the simulator is given time to advance.

Using built-in `sim.wait_while_is_moving()` causes excessive delay and slows training.

Without waiting:

* sim time delta frequently 0.0
* joints barely move

---

## Findings

1. Motion requires explicit simulation advancement.
2. Waiting inside `_apply_action()` is less effective than waiting in `step()`.
3. Velocity stabilizes only after a minimum hold duration.
4. Effective steady lift velocities observed:

| k | hold time | steady velocity |
| - | --------- | --------------- |
| 1 | 0.05 s    | ~0.0786 m/s     |
| 2 | 0.10 s    | ~0.0708 m/s     |
| 3 | 0.15 s    | ~0.0623 m/s     |

5. Larger k reduces velocity slightly but increases coverage reliability.

---

## Current Design

* Custom `_wait_sim_dt()` used.
* Waiting placed in `step()` after `_apply_action()`.
* Default: `CONTROL_HOLD (k) = 2`
* Built-in `sim.wait_while_is_moving()` no longer used.

---

## Open Questions

1. Should waiting be tied to:

   * fixed dt?
   * target velocity threshold?
   * joint error threshold?

2. Can this be replaced by:

   * fixed number of sim.step() calls?
   * deterministic physics stepping instead of time-based waiting?

3. How does waiting duration affect:

   * training stability?
   * policy smoothness?
   * credit assignment?

---

## Status

Temporary solution implemented.
Further refinement required for:

* principled control frequency
* deterministic stepping
* training efficiency