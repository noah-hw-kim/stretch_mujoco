import time
from typing import Dict, Any, Optional, Tuple

import numpy as np
import gymnasium as gym
from gymnasium import spaces

# Adjust this import path to your project structure
from stretch_mujoco.enums.actuators import Actuators

from collections import deque

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
    BLOCKING_DEBUG: bool = False # True only when you want debugging
    N_SUBSTEPS: int = 5 # how many internal sim ticks per env step
    
    
    
    ENABLE_SIM_TIME_SYNC: bool = True

    # Require at least this fraction of dt worth of simulated time per env.step().
    # 1.0 ~= "one control tick per step" (Fetch-like). If too slow, try 0.2, 0.1, etc.
    MIN_SIM_ADVANCE_MULTIPLIER: float = 1.0

    # Numerical epsilon to avoid target == t0 edge cases
    SIM_TIME_EPS_S: float = 1e-9

    # Safety timeout so a stalled sim doesn't hang training forever (wall-clock seconds)
    SIM_SYNC_TIMEOUT_S: float = 2.0

    # Small yield to avoid busy-waiting while polling sim time (wall-clock seconds)
    SIM_SYNC_POLL_SLEEP_S: float = 0.0005
    
    # ----------------------------
    # "Stuck at overhang" heuristic
    # ----------------------------
    STUCK_WINDOW: int = 15                 # how many steps to look back
    STUCK_MIN_IMPROVEMENT_M: float = 0.003 # meters of progress over the window
    STUCK_PENALTY: float = 0.25            # reward penalty when stuck
    TRUNCATE_ON_STUCK: bool = False        # start False; turn on if it helps

    # Gate stuck detection to "counter-like posture"
    ARM_EXTENDED_THRESH: float = 0.20      # meters (Stretch arm prismatic)
    EE_BELOW_GOAL_MARGIN_M: float = 0.02   # only call it stuck if ee_z < goal_z - margin
    
    # Optional: only disable stuck penalty when close AND not underneath target height
    NEAR_GOAL_DISABLE_STUCK_MULT: float = 2.0  # e.g., disable if d < 2*success_thresh (and not below goal)
    
    def __init__(
        self,
        sim,
        obj_name: str = "apple0_main",
        dt: float = 0.05,
        max_steps: int = 50,
        success_thresh: float = 0.06,
        scales: Tuple[float, float, float] = (0.015, 0.06, 0.20),  # max speeds: lift m/s, arm m/s, gripper units/s
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
        
        # Persistent commanded setpoints (initialized on reset)
        self._cmd_lift: Optional[float] = None
        self._cmd_arm: Optional[float] = None
        self._cmd_grip: Optional[float] = None
        
        # Episode counter
        self._step_count = 0
        
        # Rolling window for stuck detection
        self._recent_d = deque(maxlen=int(self.STUCK_WINDOW))
        
        # Action space: normalized [-1, 1] with (lift, arm, gripper)
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
    
    
    # # LIFT NOT SAGGING WITH SAVING PREVIOUS POSITION GLOBALLY
    # def _apply_action(self, a: np.ndarray) -> None:
    #     """
    #     Convert normalized action a in [-1,1]^3 into actuator target updates.
    #     We do target = current + delta, then clip to actuator limits, then move_to.
    #     """
    #     a = np.asarray(a, dtype=np.float32)
    #     a = np.clip(a, -1.0, 1.0)
        
    #     # scales are max speeds (per second), so delta per step = speed * dt
    #     delta = a * self.scales * self.dt
        
    #     # # Safety: if step() called before reset()
    #     if self._cmd_lift is None or self._cmd_arm is None or self._cmd_grip is None:
    #         s = self._get_status()
    #         self._cmd_lift = float(s.lift.pos)
    #         self._cmd_arm  = float(s.arm.pos)
    #         self._cmd_grip = float(s.gripper.pos)
        
    #     lift_low, lift_high = self.limits[Actuators.lift]
    #     arm_low, arm_high = self.limits[Actuators.arm]
    #     grip_low, grip_high = self.limits[Actuators.gripper]
            
    #     # Integrate action into the COMMAND, not the measured state
    #     self._cmd_lift = float(np.clip(self._cmd_lift + float(delta[0]), lift_low, lift_high))
    #     self._cmd_arm  = float(np.clip(self._cmd_arm  + float(delta[1]), arm_low,  arm_high))
    #     self._cmd_grip = float(np.clip(self._cmd_grip + float(delta[2]), grip_low, grip_high))

    #     # self.sim.move_to(Actuators.lift, self._cmd_lift)
    #     # self.sim.move_to(Actuators.arm, self._cmd_arm)
    #     # self.sim.move_to(Actuators.gripper, self._cmd_grip)
        
    #     # send commands (fiter very small changes)
    #     if abs(a[0]) > 1e-6:
    #         self.sim.move_to(Actuators.lift, self._cmd_lift)
    #     if abs(a[1]) > 1e-6:
    #         self.sim.move_to(Actuators.arm, self._cmd_arm)
    #     if abs(a[2]) > 1e-6:
    #         self.sim.move_to(Actuators.gripper, self._cmd_grip)
        
    #     # iteration 10x
    #     # 1 ->  0.05 + 0.1 = 0.15
    #     # 2 -> 0.06 + 0.1 = 0.16
    #     # 10 -> 0.05 + 0.1 = 1.05
        
    #     # it's better to read the current sensor than having the virtual point
    #     # worry about later the virtual point gap can't prevent the
    #     # move_by could be more accurate
    #     # test with gripper as well (1,1,1)
        
    #     # Lift
    #     # Going up is not accurate with move_by and wait_move because of the gravity? (error of 40% of 0.05 scale)
    #     # Going down is accurate (6% of 0.05 scale)
        
    #     # test small vector
        
    #     # wait_moving -> leave for now how long it takes for the rl -> with headless. 
        
    # LIFT SAGGING WITH USING MEASURED POSITION (USE MOVE BY)
    def _apply_action(self, a: np.ndarray) -> None:
        """
        Convert normalized action a in [-1,1]^3 into actuator target updates.
        We do target = current + delta, then clip to actuator limits, then move_to.
        """
        a = np.asarray(a, dtype=np.float32)
        a = np.clip(a, -1.0, 1.0)
        
        # scales are max speeds (per second), so delta per step = speed * dt
        delta = a * self.scales * self.dt
        moved = [False, False, False]
        
        print("current delta =", delta)
        
        if abs(float(delta[0])) > 1e-4:
            print("=====LIFT======")
            print("moving lift by", delta[0])
            self.sim.move_by(Actuators.lift, float(delta[0]))
            # self.sim.wait_while_is_moving(Actuators.lift)
            moved[0] = True
        
        if abs(float(delta[1])) > 1e-4:
            print("=====ARM======")
            print("moving arm by", delta[1])
            self.sim.move_by(Actuators.arm,  float(delta[1]))
            # self.sim.wait_while_is_moving(Actuators.arm)
            moved[1] = True
        
        if abs(float(delta[2])) > 1e-4:
            print("=====GRIPPER======")
            print("moving gripper by", delta[2])
            self.sim.move_by(Actuators.gripper, float(delta[2]))
            # self.sim.wait_while_is_moving(Actuators.gripper)
            moved[2] = True
            
        return moved, delta
        

    # ----------------------------
    # Gym API
    # ----------------------------
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        self._step_count = 0
        self._recent_d.clear()

        # Ensure objects are registered for tracking (do this once per reset)
        # If you want to track only the target object, replace with [self.obj_name]
        self.sim.register_tracked_objects([self.obj_name])

        # Put robot in a known starting pose (simple neutral)
        # These are safe-ish defaults, tweak as needed.
        self.sim.home()
        
        # Initialize commanded setpoints from current measured state ONCE
        s = self.sim.pull_status()
        self._cmd_lift = float(s.lift.pos)
        self._cmd_arm  = float(s.arm.pos)
        self._cmd_grip = float(s.gripper.pos)

        # Let it settle a bit (OK in reset)
        # time.sleep(0.3)

        obs = self._get_obs()
        info: Dict[str, Any] = {}
        return obs, info

    def step(self, action):
        self._step_count += 1
        
        # t0 = float(self.sim.pull_status().time)
        # # Apply action and wait one control tick
        moved, delta = self._apply_action(action)
        # # self._wait_for_sim_advance(t0)
        
        # # advance multiple control ticks per env.step
        # n_substeps = 5
        # for _ in range(n_substeps):
        #     self._wait_for_sim_advance(t0)
        #     t0 = float(self.sim.pull_status().time)
        
        if self.BLOCKING_DEBUG:
            # # optional: wait only if you really want "almost reaches target each step"
            if moved[0]:
                self.sim.wait_while_is_moving(Actuators.lift)
            if moved[1]:
                self.sim.wait_while_is_moving(Actuators.arm)
            if moved[2]:
                self.sim.wait_while_is_moving(Actuators.gripper)
        else:
            # RL mode: DO NOT BLOCK. Advance a fixed amount of sim time.
            # This part depends on how your sim advances time.
            # If you have a MuJoCo step method, do something like:
            t0 = float(self.sim.pull_status().time)
            for _ in range(self.N_SUBSTEPS):
                self._wait_for_sim_advance(t0)
                t0 = float(self.sim.pull_status().time)
        

        obs = self._get_obs()
        d = self._compute_distance(obs)

        reward = -d
        success = d < self.success_thresh
        
        s = self._get_status()
        
        # Capture measured vs commanded so you can inspect it from the notebook
        meas_lift = float(s.lift.pos)
        meas_arm = float(s.arm.pos)
        meas_grip = float(s.gripper.pos)

        cmd_lift = float(self._cmd_lift) if self._cmd_lift is not None else np.nan
        cmd_arm = float(self._cmd_arm) if self._cmd_arm is not None else np.nan
        cmd_grip = float(self._cmd_grip) if self._cmd_grip is not None else np.nan
        
        arm = float(s.arm.pos)
        ee_z = float(obs["achieved_goal"][2])
        goal_z = float(obs["desired_goal"][2])
        
        # THIS PART IS FOR AVOIDING THE OBSTACLE (TABLE TOP PLATE)
        # self._recent_d.append(float(d))
        
        # below_goal = ee_z < (goal_z - float(self.EE_BELOW_GOAL_MARGIN_M))
        # near_goal_safe = (d < (self.NEAR_GOAL_DISABLE_STUCK_MULT * self.success_thresh)) and (not below_goal)

        # is_stuck = False
        # improvement = np.nan

        # if (not near_goal_safe) and len(self._recent_d) == self._recent_d.maxlen:
        #     improvement = float(self._recent_d[0] - self._recent_d[-1])  # >0 means progress
        #     posture_gate = (arm > float(self.ARM_EXTENDED_THRESH)) and below_goal
        #     if posture_gate and improvement < float(self.STUCK_MIN_IMPROVEMENT_M):
        #         is_stuck = True
        #         reward -= float(self.STUCK_PENALTY)

        terminated = bool(success)
        truncated = bool(self._step_count >= self.max_steps)

        info: Dict[str, Any] = {
            "is_success": float(success),
            "distance": float(d),
            "arm_pos": float(arm),
            "ee_z": float(ee_z),
            "goal_z": float(goal_z),
            
            # debug signals (print these in the notebook)
            "cmd_lift": cmd_lift,
            "cmd_arm": cmd_arm,
            "cmd_grip": cmd_grip,
            "meas_lift": meas_lift,
            "meas_arm": meas_arm,
            "meas_grip": meas_grip,
            
            # THIS PART IS FOR AVOIDING THE OBSTACLE (TABLE TOP PLATE)
            # "is_stuck": bool(is_stuck),
            # "stuck_improvement": float(improvement) if np.isfinite(improvement) else np.nan,
            # "below_goal_height": bool(below_goal),
            # "near_goal_no_stuck_penalty": bool(near_goal_safe),
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

    