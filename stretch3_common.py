# stretch3_common.py
# Shared setup for all Stretch MuJoCo RL notebooks.
# Usage in any notebook:
#   %load_ext autoreload
#   %autoreload 2
#   %run stretch3_common.py
#
# Provides: mj_model, objects_info, LOG_DIR, make_env(),
#           run_eval(), plot_monitor_progress(), ProgressCallback

# Core libs
import mujoco
import numpy as np

# RoboCasa generator
try:
    from stretch_mujoco.robocasa_gen import model_generation_wizard
    print("Found model_generation_wizard()")
except Exception as e:
    print("Could not import model_generation_wizard:", e)

# robosuite / robocasa sanity
import robosuite
print("robosuite version:", getattr(robosuite, "__version__", "unknown"))

import robocasa
print("robocasa version:", getattr(robocasa, "__version__", "unknown"))

# Show robosuite macro backend if present
try:
    from robosuite import macros as RS_MACROS
    print("robosuite MUJOCO_GL:", getattr(RS_MACROS, "MUJOCO_GL", "not set"))
except Exception as e:
    print("robosuite macros import issue:", e)


import math

# your measured base pose
x = 0.8713510702553015
y = -1.3006141423127842
theta = 3.1147300993742104

# wrap theta to [-pi, pi] (optional but nice)
theta = math.atan2(math.sin(theta), math.cos(theta))

# Use the z from the default fixture pose you saw printed once (replace this!)
z0 = 0.0  # <-- replace with the third value from the printed "Adding stretch..." pos

w = math.cos(theta / 2.0)
z = math.sin(theta / 2.0)

robot_spawn_pose = {
    "pos": f"{x} {y} {z0}",
    "quat": f"{w} 0 0 {z}",
}

mj_model = xml = objects_info = None

try:
    # Non interactive example. Adjust task/layout/style later if you want.
    # These names are common defaults; if they ever change, the except block will let you pick via wizard.
    mj_model, xml, objects_info = model_generation_wizard(
        task="PnPCounterToCab",
        layout=0,
        style=0,
        robot_spawn_pose=robot_spawn_pose,
    )
    print("Generated RoboCasa model non-interactively.")
except TypeError:
    # Some versions use only the interactive wizard
    print("Non-interactive args not supported. Opening interactive wizard...")
    mj_model, xml, objects_info = model_generation_wizard()
except FileNotFoundError as e:
    print("Asset missing:", e)
    print("Re-run the RoboCasa asset downloader script and try again.")
    raise
except Exception as e:
    print("RoboCasa scene generation failed:", e)
    raise

print("Model OK:", isinstance(mj_model, mujoco.MjModel))
if xml:
    print("XML length:", len(xml))
if objects_info is not None:
    # objects_info is usually a dict from the generator
    print("Objects info keys:", list(objects_info)[:5])


# Pretty-print objects and their initial placements
for body_name, info in objects_info.items():
    print(f"{body_name:30s}  cat={info['cat']:12s}  pos={info['pos']}  quat={info['quat']}")

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import BaseCallback
from stretch_reach_env import StretchReachEnv


class ProgressCallback(BaseCallback):
    def __init__(self, check_freq=1000, verbose=0):
        super().__init__(verbose)
        self.check_freq = check_freq
        self.episode_count = 0

    def _on_step(self) -> bool:
        # Count finished episodes
        dones = self.locals.get("dones")
        if dones is not None:
            self.episode_count += sum(dones)
        
        if self.num_timesteps % self.check_freq == 0:
            try:
                lift = self.training_env.envs[0].env.env._last_lift_start
                self.logger.record("env/lift_start_pos", lift)
            except AttributeError:
                pass
            print(f"Steps: {self.num_timesteps} | Episodes: {self.episode_count}")
        return True

LOG_DIR = "./logs/stretch_reach"
os.makedirs(LOG_DIR, exist_ok=True)

def make_env(action_mode="discrete", allowed_parts="all", control_hold=3, default_mode="easy", headless=True, fixed_y = -0.5, success_thresh=0.15, lift_start_pos = 0.8, lift_start_random=False, run_name="run", ):
    def _init():
        sim = StretchMujocoSimulator(model=mj_model, cameras_to_use=[])
        sim.start(show_viewer_ui=False, headless=headless)
        
        env = StretchReachEnv(
            sim,
            obj_name="apple0_main",
            dt=0.05,
            max_steps=200,          # Fetch-like            success_thresh=0.1,
            success_thresh=success_thresh,
            
            action_mode = action_mode,
            allowed_parts = allowed_parts,
            
            wait_motion = True,
            control_hold = control_hold,
            
            default_mode=default_mode,   # pass through
            fixed_y=fixed_y,
            hard_y_range=(-0.6, -0.4),
            lift_start_pos=lift_start_pos,
            lift_start_random=lift_start_random,
        )
        
        run_dir = os.path.join(LOG_DIR, run_name)
        os.makedirs(run_dir, exist_ok=True)
        env = Monitor(
            env,
            filename=os.path.join(run_dir, f"monitor_{action_mode}"),
            info_keywords=("is_success", "distance", "lift_start_pos"),
        )
        return env
    return _init

# def make_env_subproc(rank: int, action_mode="discrete", allowed_parts="all", control_hold=3, run_name="run", default_mode="easy", seed=1000):
#     def _init():
#         sim = StretchMujocoSimulator(model=mj_model, cameras_to_use=[])
#         sim.start(show_viewer_ui=False, headless=True)

#         env = StretchReachEnv(
#             sim,
#             obj_name="apple0_main",
#             dt=0.05,
#             max_steps=200,
#             success_thresh=0.06,
#             action_mode=action_mode,
#             allowed_parts=allowed_parts,
#             wait_motion=True,
#             control_hold=control_hold,
#             default_mode=default_mode,
#             fixed_y=-0.5,
#             hard_y_range=(-0.6, -0.4),
#         )
#         env.reset(seed=seed + rank)

#         run_dir = os.path.join(LOG_DIR, run_name, f"env{rank}")
#         os.makedirs(run_dir, exist_ok=True)
#         env = Monitor(
#             env,
#             filename=os.path.join(run_dir, "monitor"),
#             info_keywords=("is_success", "distance")
#         )
#         return env
#     return _init

def run_eval(model, env, n_steps=500, deterministic=True):
    reset_out = env.reset()
    obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out

    dists = []
    episode_successes = 0
    episode_count = 0

    for _ in range(n_steps):
        action, _ = model.predict(obs, deterministic=deterministic)
        obs, reward, terminated, truncated, info = env.step(action)

        dists.append(info.get("distance", np.nan))
        if terminated or truncated:
            episode_count += 1
            episode_successes += int(info.get("is_success", 0.0) == 1.0)

            reset_out = env.reset()
            obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out
        
    mean_dist = float(np.nanmean(dists))
    success_rate = (episode_successes / max(1, episode_count))
    return mean_dist, episode_successes, episode_count, success_rate

def plot_monitor_progress(
    csv_path,
    smooth_window=20,
    show_reward=True,
    show_distance=True,
    show_success=True,
):
    """
    Plot training progress from Stable-Baselines3 Monitor CSV.

    Args:
        csv_path (str): Path to monitor CSV file.
        smooth_window (int): Moving average window size.
        show_reward (bool): Plot episode reward.
        show_distance (bool): Plot final episode distance (if exists).
        show_success (bool): Plot success rate (if exists).
    """

    # Load CSV (skip metadata row)
    df = pd.read_csv(csv_path, comment="#")

    episodes = range(len(df))

    # ---- Reward ----
    if show_reward and "r" in df.columns:
        plt.figure()
        plt.plot(episodes, df["r"])
        plt.xlabel("Episode")
        plt.ylabel("Episode Reward")
        plt.title("Reward per Episode")
        plt.show()

        # Smoothed reward
        df["reward_smooth"] = df["r"].rolling(window=smooth_window).mean()

        plt.figure()
        plt.plot(episodes, df["reward_smooth"])
        plt.xlabel("Episode")
        plt.ylabel(f"Smoothed Reward ({smooth_window})")
        plt.title("Smoothed Reward")
        plt.show()

    # ---- Distance ----
    if show_distance and "distance" in df.columns:
        plt.figure()
        plt.plot(episodes, df["distance"])
        plt.xlabel("Episode")
        plt.ylabel("Final Distance")
        plt.title("Distance per Episode")
        plt.show()

    # ---- Success Rate ----
    if show_success and "is_success" in df.columns:
        df["success_smooth"] = df["is_success"].rolling(window=smooth_window).mean()

        plt.figure()
        plt.plot(episodes, df["success_smooth"])
        plt.xlabel("Episode")
        plt.ylabel(f"Success Rate ({smooth_window})")
        plt.title("Success Rate (Moving Average)")
        plt.show()
        
# def run_eval_with_stuck(model, env, n_steps=500, deterministic=True):
#     reset_out = env.reset()
#     obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out

#     dists = []
#     episode_successes = 0
#     episode_count = 0
#     stuck_steps = 0
#     total_steps = 0

#     for _ in range(n_steps):
#         action, _ = model.predict(obs, deterministic=deterministic)
#         obs, reward, terminated, truncated, info = env.step(action)

#         total_steps += 1
#         dists.append(info.get("distance", np.nan))
#         stuck_steps += int(bool(info.get("is_stuck", False)))

#         if terminated or truncated:
#             episode_count += 1
#             episode_successes += int(info.get("is_success", 0.0) == 1.0)

#             reset_out = env.reset()
#             obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out

#     mean_dist = float(np.nanmean(dists))
#     success_rate = (episode_successes / max(1, episode_count))
#     stuck_rate = stuck_steps / max(1, total_steps)
#     return mean_dist, episode_successes, episode_count, success_rate, stuck_rate

# def run_eval_with_stuck_debug(model, env, n_steps=500, deterministic=True, near_dist=0.25, arm_attempt_thresh=0.20):
#     reset_out = env.reset()
#     obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out

#     dists = []
#     episode_successes = 0
#     episode_count = 0

#     total_steps = 0
#     stuck_steps = 0

#     attempt_steps = 0
#     stuck_steps_given_attempt = 0

#     for _ in range(n_steps):
#         action, _ = model.predict(obs, deterministic=deterministic)
#         obs, reward, terminated, truncated, info = env.step(action)

#         total_steps += 1
#         d = info.get("distance", np.nan)
#         dists.append(d)

#         is_stuck = bool(info.get("is_stuck", False))
#         stuck_steps += int(is_stuck)

#         # "Attempt" guard: either close-ish OR clearly extending the arm.
#         arm_pos = float(info.get("arm_pos", 0.0))  # only works if env puts this in info
#         is_attempt = (np.isfinite(d) and d < near_dist) or (arm_pos > arm_attempt_thresh)

#         if is_attempt:
#             attempt_steps += 1
#             stuck_steps_given_attempt += int(is_stuck)

#         if terminated or truncated:
#             episode_count += 1
#             episode_successes += int(info.get("is_success", 0.0) == 1.0)
#             reset_out = env.reset()
#             obs = reset_out[0] if isinstance(reset_out, tuple) else reset_out

#     mean_dist = float(np.nanmean(dists))
#     success_rate = (episode_successes / max(1, episode_count))
#     stuck_rate = stuck_steps / max(1, total_steps)
#     attempt_rate = attempt_steps / max(1, total_steps)
#     stuck_given_attempt = stuck_steps_given_attempt / max(1, attempt_steps)

#     return mean_dist, episode_successes, episode_count, success_rate, stuck_rate, attempt_rate, stuck_given_attempt

# ── Manual-mode helpers ─────────────────────────────────────────────────
def get_ee_pos(sim):
    T = sim.get_ee_pose()
    return T[:3, 3].astype(float)

def get_obj_pos(sim, obj_name):
    return sim.pull_objects_state()[obj_name]["pos"].astype(float)

def distance_to_object(sim, obj_name):
    ee_pos = get_ee_pos(sim)
    obj_pos = get_obj_pos(sim, obj_name)
    d = float(np.linalg.norm(ee_pos - obj_pos))
    return d, ee_pos, obj_pos

def manual_step(sim, a, obj_name):
    """
    a: array-like shape (3,), values in [-1, 1]
    returns: d, ee_pos, obj_pos
    """
    a = np.asarray(a, dtype=np.float32)
    a = np.clip(a, -1.0, 1.0) # a is just the policy saying “move up/down a bit” (a direction and strength).
    delta = a * scales # scales is shape (3,)

    s = sim.pull_status()
    current_lift = float(s.lift.pos)
    current_arm  = float(s.arm.pos)
    current_grip = float(s.gripper.pos)
    
    lift_low, lift_high = limits[Actuators.lift]
    arm_low,  arm_high  = limits[Actuators.arm]
    grip_low, grip_high = limits[Actuators.gripper]
    
    target_lift = np.clip(current_lift + float(delta[0]), lift_low, lift_high)
    target_arm  = np.clip(current_arm  + float(delta[1]), arm_low,  arm_high)
    target_grip = np.clip(current_grip + float(delta[2]), grip_low, grip_high)

    sim.move_to(Actuators.lift,    target_lift)
    sim.move_to(Actuators.arm,     target_arm)
    sim.move_to(Actuators.gripper, target_grip)

    # fixed control tick
    time.sleep(dt)

    return distance_to_object(sim, obj_name)
