import time
from typing import Dict, Any, Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# Adjust this import path to your project structure
from stretch_mujoco.enums.actuators import Actuators

class StretchReachEnv(gym.Env):
    """
    Minimal Gymnasium env wrapper for StretchMujocoSimulator.

    Task: reach a single object with the end-effector.
    Actions: lift, arm, gripper (delta targets each step).
    Observation: dict with observation / achieved_goal / desired_goal (Fetch-style).
    Reward: dense = -||ee_pos - obj_pos||.
    """
    
    # ----------------------------
    # Sync / timing constants
    # ----------------------------
    ENABLE_SIM_TIME_SYNC: bool = True

    # Require at least this fraction of dt worth of simulated time per env.step().
    # 1.0 ~= "one control tick per step" (Fetch-like). If too slow, try 0.2, 0.1, etc.
    MIN_SIM_ADVANCE_MULTIPLIER: float = 0.2

    # Numerical epsilon to avoid target == t0 edge cases
    SIM_TIME_EPS_S: float = 1e-9

    # Safety timeout so a stalled sim doesn't hang training forever (wall-clock seconds)
    SIM_SYNC_TIMEOUT_S: float = 0.25

    # Small yield to avoid busy-waiting while polling sim time (wall-clock seconds)
    SIM_SYNC_POLL_SLEEP_S: float = 0.0005
    
    def __init__(
        self,
        sim,
        obj_name: str = "apple0_main",
        dt: float = 0.05,
        max_steps: int = 50,
        success_thresh: float = 0.06,
        scales: Tuple[float, float, float] = (0.02, 0.01, 0.01), # lift, arm, gripper
        limits: Optional[Dict[Actuators, Tuple[float, float]]] = None,
    ):
        super().__init__()
        self.sim = sim
        self.obj_name = obj_name
        self.dt = float(dt)
        self.max_steps = int(max_steps)
        self.success_thresh = float(success_thresh)
        self.scales = np.asarray(scales, dtype=np.float32)
        
        # Empirical limits measured (safe defaults)
        if limits is None:
            limits = {
                Actuators.lift: (0.0, 1.1),
                Actuators.arm: (0.0, 0.52),
                Actuators.gripper: (-0.25, 0.53),
            }
        self.limits = limits
        
        # Episode counter
        self._step_count = 0
        
        # Action space: normalized [-1, 1]
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)
        
        # Observation space:
        # observation = [lift, arm, gripper, ee_x, ee_y, ee_z, obj_x, obj_y, oPbj_z] -> 9 dims
        self.observation_space = spaces.Dict(
            {
                "observation": spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32),
                "achieved_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
                "desired_goal": spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            }
        )
        
    # ----------------------------
    # Helpers
    # ----------------------------
    def _get_status(self):
        return self.sim.pull_status()
    
    def _wait_for_sim_advance(self, t0: float) -> None:
        """
        Wait until simulator time has advanced by at least:
            min_sim_advance = MIN_SIM_ADVANCE_MULTIPLIER * dt

        This is a Fetch-like substitute for time.sleep(dt):
        - It waits for *physics progress* (sim time), not wall-clock time.
        - If sim runs fast, this returns quickly.
        - If sim runs slow/stalls, it times out and proceeds.
        """
        if not self.ENABLE_SIM_TIME_SYNC:
            return

        min_sim_advance = max(self.SIM_TIME_EPS_S, self.MIN_SIM_ADVANCE_MULTIPLIER * self.dt)
        target = t0 + min_sim_advance
        deadline = time.monotonic() + self.SIM_SYNC_TIMEOUT_S

        while time.monotonic() < deadline:
            t = float(self.sim.pull_status().time)
            if t >= target:
                return
            time.sleep(self.SIM_SYNC_POLL_SLEEP_S)

        # Timeout -> proceed anyway (prevents training hangs)
        return
    
    def _get_ee_pos(self) -> np.ndarray:
        T = self.sim.get_ee_pose()
        return np.asarray(T[:3, 3], dtype=np.float32)

    def _get_obj_pos(self) -> np.ndarray:
        mapping = self.sim.pull_objects_state()
        if self.obj_name not in mapping:
            raise KeyError(
                f"Object '{self.obj_name}' not found in pull_objects_state(). "
                f"Available: {list(mapping.keys())[:10]}"
            )
        return np.asarray(mapping[self.obj_name]["pos"], dtype=np.float32)
    
    def _get_obs(self) -> Dict[str, np.ndarray]:
        s = self._get_status()
        ee = self._get_ee_pos()
        obj = self._get_obj_pos()

        lift = float(s.lift.pos)
        arm = float(s.arm.pos)
        grip = float(s.gripper.pos)

        obs_vec = np.array([lift, arm, grip, ee[0], ee[1], ee[2], obj[0], obj[1], obj[2]], dtype=np.float32)

        achieved_goal = ee
        desired_goal = obj

        return {
            "observation": obs_vec,
            "achieved_goal": achieved_goal,
            "desired_goal": desired_goal,
        }

    def _compute_distance(self, obs: Dict[str, np.ndarray]) -> float:
        ag = obs["achieved_goal"]
        dg = obs["desired_goal"]
        return float(np.linalg.norm(ag - dg))
    
    def _apply_action(self, a: np.ndarray) -> None:
        """
        Convert normalized action a in [-1,1]^3 into actuator target updates.
        We do target = current + delta, then clip to actuator limits, then move_to.
        """
        a = np.asarray(a, dtype=np.float32)
        a = np.clip(a, -1.0, 1.0)
        delta = a * self.scales

        s = self._get_status()
        cur_lift = float(s.lift.pos)
        cur_arm = float(s.arm.pos)
        cur_grip = float(s.gripper.pos)

        lift_low, lift_high = self.limits[Actuators.lift]
        arm_low, arm_high = self.limits[Actuators.arm]
        grip_low, grip_high = self.limits[Actuators.gripper]

        tgt_lift = float(np.clip(cur_lift + float(delta[0]), lift_low, lift_high))
        tgt_arm = float(np.clip(cur_arm + float(delta[1]), arm_low, arm_high))
        tgt_grip = float(np.clip(cur_grip + float(delta[2]), grip_low, grip_high))

        self.sim.move_to(Actuators.lift, tgt_lift)
        self.sim.move_to(Actuators.arm, tgt_arm)
        self.sim.move_to(Actuators.gripper, tgt_grip)
    
    
    # ----------------------------
    # Gym API
    # ----------------------------
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        self._step_count = 0

        # Ensure objects are registered for tracking (do this once per reset)
        # If you want to track only the target object, replace with [self.obj_name]
        self.sim.register_tracked_objects([self.obj_name])

        # Put robot in a known starting pose (simple neutral)
        # These are safe-ish defaults, tweak as needed.
        self.sim.home()

        # Let it settle a bit (OK in reset)
        # time.sleep(0.3)

        obs = self._get_obs()
        info: Dict[str, Any] = {}
        return obs, info

    def step(self, action):
        self._step_count += 1
        
        t0 = float(self.sim.pull_status().time)
        # Apply action and wait one control tick
        self._apply_action(action)
        # time.sleep(self.dt)
        self._wait_for_sim_advance(t0)

        obs = self._get_obs()
        d = self._compute_distance(obs)

        reward = -d
        success = d < self.success_thresh

        terminated = bool(success)
        truncated = bool(self._step_count >= self.max_steps)

        info: Dict[str, Any] = {
            "is_success": float(success),
            "distance": float(d),
        }

        # Keep your "terminal_observation" convention for logging correctness
        if terminated or truncated:
            info["terminal_observation"] = obs

        return obs, float(reward), terminated, truncated, info

    def debug_state(self):
        s = self.sim.pull_status()
        ee = self._get_ee_pos()
        obj = self._get_obj_pos()
        return {
            "lift": float(s.lift.pos),
            "arm": float(s.arm.pos),
            "gripper": float(s.gripper.pos),
            "ee": ee,
            "obj": obj,
            "dist": float(np.linalg.norm(ee - obj)),
        }

    