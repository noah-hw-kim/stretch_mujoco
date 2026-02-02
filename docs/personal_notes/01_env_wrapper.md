# Why I cannot use `env` from robocasa_gen.py for RL training

## Short answer
The RoboCasa `env` is only used to generate the kitchen scene and object placements.
My Stretch robot is controlled by a different simulator layer, so I must create my own Gym wrapper.

## Details
- `robocasa_gen.py` creates a robosuite/RoboCasa environment with a **dummy robot (PandaMobile)**.
- `env.step(action)` expects PandaMobile-style actions and uses robosuite’s internal controllers.
- I delete the dummy robot from the XML and insert **Stretch**, so that robosuite `env` is no longer valid.
- Stretch is controlled through `StretchMujocoSimulator` + `mujoco_server`, using APIs like:
  - `sim.move_by(Actuators.lift, delta)`
  - `sim.pull_status()`
  - `sim.pull_objects_state()`
- A MuJoCo `MjModel` is **not** an environment. It has no `reset()`, `step()`, reward, or done logic.

## Conclusion
RoboCasa is used as a **scene and object generator only**.
To train with Stable-Baselines3, I must write a **Gymnasium wrapper** that:
- maps RL actions → `sim.move_by(...)`
- builds observations from robot + object state
- defines reward and termination rules
