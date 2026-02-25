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
    
    LIFT_MIN = 0.008
    LIFT_STEP = 0.045
    
    ARM_MIN = 0.0001 # actual min is 0.000006
    ARM_STEP = 0.015
    
    GRIPPER_MIN = 0.004
    GRIPPER_STEP = 0.025
    
    # ----------------------------
    # Sync / timing constants
    # ----------------------------
    BLOCKING_DEBUG: bool = True # True only when you want debugging
    N_SUBSTEPS: int = 5 # how many internal sim ticks per env step
    BLOCKING_DEBUG_MESSAGE = True
    
    # ----------------------------
    # Penalty to prevent the arm stays only in extended pos
    # ----------------------------
    ARM_NEAR_MAX_THRESH = 0.01   # meters from max (1 cm). Try 0.01–0.03
    ARM_NEAR_MAX_PENALTY = 0.2   # flat penalty. Try 0.1–0.5

    
    # ENABLE_SIM_TIME_SYNC: bool = True

    # # Require at least this fraction of dt worth of simulated time per env.step().
    # # 1.0 ~= "one control tick per step" (Fetch-like). If too slow, try 0.2, 0.1, etc.
    # MIN_SIM_ADVANCE_MULTIPLIER: float = 1.0

    # # Numerical epsilon to avoid target == t0 edge cases
    # SIM_TIME_EPS_S: float = 1e-9

    # # Safety timeout so a stalled sim doesn't hang training forever (wall-clock seconds)
    # SIM_SYNC_TIMEOUT_S: float = 2.0

    # # Small yield to avoid busy-waiting while polling sim time (wall-clock seconds)
    # SIM_SYNC_POLL_SLEEP_S: float = 0.0005
    
    # # ----------------------------
    # # "Stuck at overhang" heuristic
    # # ----------------------------
    # STUCK_WINDOW: int = 15                 # how many steps to look back
    # STUCK_MIN_IMPROVEMENT_M: float = 0.003 # meters of progress over the window
    # STUCK_PENALTY: float = 0.25            # reward penalty when stuck
    # TRUNCATE_ON_STUCK: bool = False        # start False; turn on if it helps

    # # Gate stuck detection to "counter-like posture"
    # ARM_EXTENDED_THRESH: float = 0.20      # meters (Stretch arm prismatic)
    # EE_BELOW_GOAL_MARGIN_M: float = 0.02   # only call it stuck if ee_z < goal_z - margin
    
    # # Optional: only disable stuck penalty when close AND not underneath target height
    # NEAR_GOAL_DISABLE_STUCK_MULT: float = 2.0  # e.g., disable if d < 2*success_thresh (and not below goal)
    
    def __init__(
        self,
        sim,
        obj_name: str = "apple0_main",
        dt: float = 0.05,
        max_steps: int = 100,
        success_thresh: float = 0.06,
        
        scales: Tuple[float, float, float] = (0.8, 0.3, 0.5),  # max speeds: lift m/s, arm m/s, gripper units/s
        limits: Optional[Dict[Actuators, Tuple[float, float]]] = None,
        
        lift_mapping: str = "deadzone",
        
        action_mode: str = "discrete",
        allowed_parts: str = "all",
        
        wait_motion = True,
        control_hold = 3,
    ):
        super().__init__()
        self.sim = sim
        self.obj_name = obj_name
        self.dt = float(dt)
        self.max_steps = int(max_steps)
        self.success_thresh = float(success_thresh)
        self.scales = np.asarray(scales, dtype=np.float32)
        self.LIFT_MAPPING = lift_mapping
        self.allowed_parts = allowed_parts # "lift", "lift_arm", or "all"
        self.action_mode = action_mode
        self.wait_motion = wait_motion
        self.control_hold = control_hold
        
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
        
        # # Rolling window for stuck detection
        # self._recent_d = deque(maxlen=int(self.STUCK_WINDOW))
        
        valid_action_modes = {"discrete", "continuous"}
        if action_mode not in valid_action_modes:
            raise ValueError(f"action_mode must be one of {valid_action_modes}")

        valid_parts = {"lift", "lift_arm", "all"}
        if allowed_parts not in valid_parts:
            raise ValueError(f"allowed_parts must be one of {valid_parts}")
        
        self.allowed_parts = allowed_parts

        # Action space: normalized [-1, 1] with (lift, arm, gripper)
        if action_mode == "discrete":
            # discrete version
            if self.allowed_parts == "lift":
                self.action_space = spaces.Discrete(3)      # down, noop, up
            elif self.allowed_parts == "lift_arm":
                self.action_space = spaces.Discrete(5)      # lift down/up/noop + arm in/out
            else:  # "all"
                self.action_space = spaces.Discrete(7)      # + gripper close/open
        else:  # continuous
            if allowed_parts == "lift":
                self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
            elif allowed_parts == "lift_arm":
                self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)
            else:
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
    
    # def _wait_for_sim_advance(self, t0: float) -> None:
    #     """
    #     Wait until simulator time has advanced by at least:
    #         min_sim_advance = MIN_SIM_ADVANCE_MULTIPLIER * dt

    #     This is a Fetch-like substitute for time.sleep(dt):
    #     - It waits for *physics progress* (sim time), not wall-clock time.
    #     - If sim runs fast, this returns quickly.
    #     - If sim runs slow/stalls, it times out and proceeds.
    #     """
    #     if not self.ENABLE_SIM_TIME_SYNC:
    #         return

    #     min_sim_advance = max(self.SIM_TIME_EPS_S, self.MIN_SIM_ADVANCE_MULTIPLIER * self.dt)
    #     target = t0 + min_sim_advance
    #     deadline = time.monotonic() + self.SIM_SYNC_TIMEOUT_S

    #     while time.monotonic() < deadline:
    #         t = float(self.sim.pull_status().time)
    #         if t >= target:
    #             return
    #         time.sleep(self.SIM_SYNC_POLL_SLEEP_S)

    #     # Timeout -> proceed anyway (prevents training hangs)
    #     return
    
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
    
    def _clamp_delta(self, pos: float, delta: float, lo: float, hi: float) -> float:
        target = pos + delta
        if target < lo:
            target = lo
        elif target > hi:
            target = hi
        return target - pos
    
    def _wait_sim_dt(self):
        t0 = float(self.sim.pull_status().time)
        target = t0 + float(self.dt) * self.control_hold

        # bounded wait so we never hang forever
        timeout_s = 0.5  # plenty for dt=0.05
        wall_start = time.time()

        while True:
            t = float(self.sim.pull_status().time)
            if t >= target:
                break
            if time.time() - wall_start > timeout_s:
                break
            time.sleep(0.001)  # 1ms yield
    
    def _apply_action_continuous(self, a: np.ndarray) -> None:
        """
        Convert normalized action a in [-1,1]^3 into incremental actuator commands (move_by).
        Returns:
        moved: [bool,bool,bool] indicating which actuators were commanded
        delta: np.ndarray of applied deltas [lift, arm, gripper] for logging
        """
        a = np.asarray(a, dtype=np.float32).reshape(-1)
        a = np.clip(a, -1.0, 1.0)
        
        # Expand to [lift, arm, gripper] based on allowed_parts
        if self.allowed_parts == "lift":
            a_full = np.array([a[0], 0.0, 0.0], dtype=np.float32)
        elif self.allowed_parts == "lift_arm":
            a_full = np.array([a[0], a[1], 0.0], dtype=np.float32)
        else:
            a_full = np.array([a[0], a[1], a[2]], dtype=np.float32)
        
        moved = [False, False, False]
        
        a_lift = float(a_full[0])
        a_arm  = float(a_full[1])
        a_grip = float(a_full[2])
        
        # -------------------------
        # Choose lift mapping mode
        # -------------------------
        # Set this somewhere (e.g., in __init__): self.LIFT_MAPPING = "deadzone" or "tanh"
        mapping = getattr(self, "LIFT_MAPPING", "deadzone")
        
        # Sign stepping (your current behavior)
        delta_lift = +self.LIFT_STEP if a_lift > 0 else (-self.LIFT_STEP if a_lift < 0 else 0.0)
        delta_arm  = +self.ARM_STEP  if a_arm  > 0 else (-self.ARM_STEP  if a_arm  < 0 else 0.0)
        delta_grip = +self.GRIPPER_STEP if a_grip > 0 else (-self.GRIPPER_STEP if a_grip < 0 else 0.0)
        
        
        # delta_arm = float(a[0]) * float(self.scales[0]) * float(self.dt)
        # delta_arm = float(a[1]) * float(self.scales[1]) * float(self.dt)
        # delta_grip = float(a[2]) * float(self.scales[2]) * float(self.dt)
        

        
        # === WILL HANDLE LIFT LOGIC LATER. NOW, WE WILL IGNORE DT ===
        # # -------------------------
        # # LIFT: either deadzone or tanh
        # # -------------------------
        # a_lift = float(a[0])
        # delta_min_up = 0.008  # empirically ~0.008–0.010; use 0.01 for safety
        
        # if mapping == "deadzone":
        #     # Deadzone-based lift control has a scale tradeoff:
        #     # - If scale is too small, many small actions become no-op.
        #     #   Example (your case): dt=0.05, scale=0.5, delta_min=0.01 -> actions in (0, 0.4) are ignored upward.
        #     # - If scale is increased to shrink this no-op band, max deltas get large unless you add a max cap,
        #     #   and that cap can saturate a large portion of the action range.
        #     delta_lift_raw = a_lift * float(self.scales[0]) * float(self.dt)

        #     # Up-only deadzone gate (recommended for your observed issue):
        #     # tiny upward deltas can sag; downward small deltas often still work.
        #     if 0.0 < delta_lift_raw < delta_min_up:
        #         delta_lift = 0.0
        #     else:
        #         delta_lift = float(delta_lift_raw)

        #     # Optional safety cap (uncomment if you later increase scale for RL)
        #     # delta_max = 0.05
        #     # delta_lift = float(np.clip(delta_lift, -delta_max, delta_max))
            
        # elif mapping == "tanh":
        #     # Smooth saturation mapping (tanh) avoids a hard no-op band and avoids hard clipping plateaus.
        #     # Small actions remain differentiable/meaningful; large actions smoothly saturate.
        #     delta_max_lift = 0.03  # max per env.step (meters). Try 0.03–0.05
        #     k = 2.5                # saturation sharpness. Try 2.0–3.0

        #     delta_lift = float(delta_max_lift * np.tanh(k * a_lift) / np.tanh(k))

        #     # Optional: add a small up-only guard if you STILL observe tiny-positive sag
        #     # delta_min_up = 0.008
        #     # if 0.0 < delta_lift < delta_min_up:
        #     #     delta_lift = 0.0
        
        # else:
        #     raise ValueError(f"Unknown LIFT_MAPPING='{mapping}'. Use 'deadzone' or 'tanh'.")
        

        # get current positions once
        s = self._get_status()

        # clamp each delta
        delta_lift = self._clamp_delta(
            float(s.lift.pos),
            delta_lift,
            self.limits[Actuators.lift][0],
            self.limits[Actuators.lift][1],
        )

        delta_arm = self._clamp_delta(
            float(s.arm.pos),
            delta_arm,
            self.limits[Actuators.arm][0],
            self.limits[Actuators.arm][1],
        )

        delta_grip = self._clamp_delta(
            float(s.gripper.pos),
            delta_grip,
            self.limits[Actuators.gripper][0],
            self.limits[Actuators.gripper][1],
        )
        
        # Assemble delta array for logging/return
        delta = np.array([delta_lift, delta_arm, delta_grip], dtype=np.float32)

        if not self.BLOCKING_DEBUG_MESSAGE:
            print("current delta (after clamp) =", delta)
        
        if abs(float(delta[0])) >= float(self.LIFT_MIN):
            if not self.BLOCKING_DEBUG_MESSAGE:
                print("=====LIFT======")
                print("moving lift by", delta[0])
            self.sim.move_by(Actuators.lift, float(delta[0]))
            if self.wait_motion:
                # self.sim.wait_while_is_moving(Actuators.lift)
                self._wait_sim_dt()
            moved[0] = True
        
        if abs(float(delta[1])) >= float(self.ARM_MIN):
            if not self.BLOCKING_DEBUG_MESSAGE:
                print("=====ARM======")
                print("moving arm by", delta[1])
            self.sim.move_by(Actuators.arm,  float(delta[1]))
            if self.wait_motion:
                # self.sim.wait_while_is_moving(Actuators.arm)
                self._wait_sim_dt()
            moved[1] = True
        
        if abs(float(delta[2])) >= float(self.GRIPPER_MIN):
            if not self.BLOCKING_DEBUG_MESSAGE:
                print("=====GRIPPER======")
                print("moving gripper by", delta[2])
            self.sim.move_by(Actuators.gripper, float(delta[2]))
            if self.wait_motion:
                # self.sim.wait_while_is_moving(Actuators.gripper)
                self._wait_sim_dt()
            moved[2] = True
            
        # if self.wait_motion:
        #     self._wait_sim_dt()
            
        return moved, delta
    
    def _apply_action_discrete(self, a):
        """
        Discrete action mapping (mode-dependent, no dead actions):

        mode="lift" (3):
        0: lift down
        1: no-op
        2: lift up

        mode="lift_arm" (5):
        0: lift down
        1: lift up
        2: no-op
        3: arm retract
        4: arm extend

        mode="all" (7):
        0: lift down
        1: lift up
        2: no-op
        3: arm retract
        4: arm extend
        5: gripper close
        6: gripper open
        """
        
        # SB3/DummyVecEnv sometimes passes array([action]) instead of int
        if isinstance(a, (np.ndarray, list, tuple)):
            a = int(np.asarray(a).reshape(-1)[0])
        else:
            a = int(a)

        # Determine number of actions for current mode
        if self.allowed_parts == "lift":
            n_actions = 3
        elif self.allowed_parts == "lift_arm":
            n_actions = 5
        else:  # "all"
            n_actions = 7

        if a < 0 or a >= n_actions:
            raise ValueError(f"Discrete action must be in [0..{n_actions-1}], got {a}")

        moved = [False, False, False]

        # Per-direction step sizes (fallback to your existing constants)
        lift_up = float(getattr(self, "LIFT_STEP_UP", self.LIFT_STEP))
        lift_dn = float(getattr(self, "LIFT_STEP_DN", self.LIFT_STEP))

        arm_out = float(getattr(self, "ARM_STEP_OUT", self.ARM_STEP))   # extend
        arm_in  = float(getattr(self, "ARM_STEP_IN",  self.ARM_STEP))   # retract

        grip_open  = float(getattr(self, "GRIPPER_STEP_OPEN",  self.GRIPPER_STEP))
        grip_close = float(getattr(self, "GRIPPER_STEP_CLOSE", self.GRIPPER_STEP))
        
        # Default deltas
        delta_lift = 0.0
        delta_arm  = 0.0
        delta_grip = 0.0

        # Map action -> delta (mode-specific)
        if self.allowed_parts == "lift":
            # 0 down, 1 noop, 2 up
            if a == 0:
                delta_lift = -lift_dn
            elif a == 2:
                delta_lift = +lift_up

        elif self.allowed_parts == "lift_arm":
            # 0 down, 1 up, 2 noop, 3 arm in, 4 arm out
            if a == 0:
                delta_lift = -lift_dn
            elif a == 1:
                delta_lift = +lift_up
            elif a == 3:
                delta_arm = -arm_in
            elif a == 4:
                delta_arm = +arm_out

        else:  # "all"
            # 0 down, 1 up, 2 noop, 3 arm in, 4 arm out, 5 grip close, 6 grip open
            if a == 0:
                delta_lift = -lift_dn
            elif a == 1:
                delta_lift = +lift_up
            elif a == 3:
                delta_arm = -arm_in
            elif a == 4:
                delta_arm = +arm_out
            elif a == 5:
                delta_grip = -grip_close
            elif a == 6:
                delta_grip = +grip_open

        # Clamp deltas to joint limits
        s = self._get_status()

        delta_lift = self._clamp_delta(
            float(s.lift.pos), float(delta_lift),
            self.limits[Actuators.lift][0], self.limits[Actuators.lift][1],
        )
        delta_arm = self._clamp_delta(
            float(s.arm.pos), float(delta_arm),
            self.limits[Actuators.arm][0], self.limits[Actuators.arm][1],
        )
        delta_grip = self._clamp_delta(
            float(s.gripper.pos), float(delta_grip),
            self.limits[Actuators.gripper][0], self.limits[Actuators.gripper][1],
        )

        delta = np.array([delta_lift, delta_arm, delta_grip], dtype=np.float32)
        
        # Execute moves (do NOT mask by mode; mode already handled by action space)
        if abs(float(delta_lift)) >= float(self.LIFT_MIN):
            self.sim.move_by(Actuators.lift, float(delta_lift))
            moved[0] = True

        if abs(float(delta_arm)) >= float(self.ARM_MIN):
            self.sim.move_by(Actuators.arm, float(delta_arm))
            moved[1] = True

        if abs(float(delta_grip)) >= float(self.GRIPPER_MIN):
            self.sim.move_by(Actuators.gripper, float(delta_grip))
            moved[2] = True
        
        if self.wait_motion:
            # self.sim.wait_while_is_moving(Actuators.arm)
            self._wait_sim_dt()
            
        # check attempted to move or not
        attempted = {"lift": False, "arm": False, "grip": False}
        if self.allowed_parts == "lift":
            if a in (0, 2): attempted["lift"] = True
        elif self.allowed_parts == "lift_arm":
            if a in (0, 2): attempted["lift"] = True
            if a in (3, 4): attempted["arm"] = True
        else:  # all
            if a in (0, 2): attempted["lift"] = True
            if a in (3, 4): attempted["arm"] = True
            if a in (5, 6): attempted["grip"] = True

        return moved, delta, attempted
        

    # ----------------------------
    # Gym API
    # ----------------------------
    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset(seed=seed)
        self._step_count = 0
        # self._recent_d.clear()

        # Ensure objects are registered for tracking (do this once per reset)
        # If you want to track only the target object, replace with [self.obj_name]
        self.sim.register_tracked_objects([self.obj_name])

        # Put robot in a known starting pose (simple neutral)
        # These are safe-ish defaults, tweak as needed.
        self.sim.home()
        
        self.sim.set_object_pose(self.obj_name, pos_xyz=(0.9, -0.4, 0.98), quat_wxyz=(1,0,0,0))
        
        # Move above the table for now
        self.sim.move_to(Actuators.lift, 1.0)
        time.sleep(1)

        # Let it settle a bit (OK in reset)
        # time.sleep(0.3)

        obs = self._get_obs()
        info: Dict[str, Any] = {}
        return obs, info

    def step(self, action):
        self._step_count += 1
        
        # # Apply action and wait one control tick
        
        if self.action_mode == "continuous":
            moved, delta = self._apply_action_continuous(action)
        
        else:
            moved, delta, attempted = self._apply_action_discrete(action)
        
        # if self.wait_motion:
        #     self._wait_sim_dt()
        
        # if self.BLOCKING_DEBUG:
        #     # # optional: wait only if you really want "almost reaches target each step"
        #     if moved[0]:
        #         self.sim.wait_while_is_moving(Actuators.lift)
        #     if moved[1]:
        #         self.sim.wait_while_is_moving(Actuators.arm)
        #     if moved[2]:
        #         self.sim.wait_while_is_moving(Actuators.gripper)
        # else:
        #     # RL mode: DO NOT BLOCK. Advance a fixed amount of sim time.
        #     # This part depends on how your sim advances time.
        #     # If you have a MuJoCo step method, do something like:
        #     t0 = float(self.sim.pull_status().time)
        #     for _ in range(self.N_SUBSTEPS):
        #         self._wait_for_sim_advance(t0)
        #         t0 = float(self.sim.pull_status().time)
        

        obs = self._get_obs()
        d = self._compute_distance(obs)

        reward = -d
        
        # Flat penalty: if arm is near max and action is "extend", punish
        if isinstance(action, (np.ndarray, list, tuple)):
            a = int(np.asarray(action).reshape(-1)[0])
        else:
            a = int(action)

        # only relevant if your action space includes arm extend
        if self.allowed_parts in ("lift_arm", "all") and a == 4:
            arm_pos = float(self._get_status().arm.pos)
            arm_max = float(self.limits[Actuators.arm][1])
            if arm_pos > (arm_max - self.ARM_NEAR_MAX_THRESH):
                reward -= self.ARM_NEAR_MAX_PENALTY
        
        success = d < self.success_thresh
        
        s = self._get_status()
        
        # # Capture measured vs commanded so you can inspect it from the notebook
        # meas_lift = float(s.lift.pos)
        # meas_arm = float(s.arm.pos)
        # meas_grip = float(s.gripper.pos)
        
        # arm = float(s.arm.pos)
        
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
            # ===== Task metrics =====
            "is_success": float(success),
            "distance": float(d),
            
            # ===== State summary =====
            "ee_z": float(ee_z),
            "goal_z": float(goal_z),
            
            # ===== Joint positions =====
            "lift_pos": float(s.lift.pos),
            "arm_pos": float(s.arm.pos),
            "grip_pos": float(s.gripper.pos),
            
            # ===== Joint velocities =====
            "lift_vel": float(s.lift.vel),
            "arm_vel": float(s.arm.vel),
            "grip_vel": float(s.gripper.vel),
            
            "delta_lift": float(delta[0]),
            "delta_arm": float(delta[1]),
            "delta_grip": float(delta[2]),
            
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

    