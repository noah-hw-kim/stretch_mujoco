import gymnasium as gym
from gymnasium import spaces
import numpy as np

class RobotLiftEnv(gym.Env):
    def __init__(self):
        super(RobotLiftEnv, self).__init__()

        # Define Action Space: 0 = Down, 1 = Up
        self.action_space = spaces.Discrete(2)

        # Define Observation Space: [current_height]
        # Min height 0.0, Max height 10.0
        self.observation_space = spaces.Box(low=0.0, high=10.0, shape=(1,), dtype=np.float32)

        # Internal state
        self.state = np.array([5.0], dtype=np.float32) # Start in the middle
        self.goal_height = 8.0
        self.step_size = 0.5

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Reset robot to a random height or fixed starting point
        self.state = np.array([5.0], dtype=np.float32)
        
        return self.state, {}

    def step(self, action):
        # Apply action
        if action == 1:   # Up
            self.state[0] = min(10.0, self.state[0] + self.step_size)
        elif action == 0: # Down
            self.state[0] = max(0.0, self.state[0] - self.step_size)
        #elif action == 2 3 4 5
        
        # Calculate Reward (Simple: closer to goal is better)
        distance = abs(self.state[0] - self.goal_height)
        reward = -distance 
        
        # Check if "solved"
        terminated = bool(distance < 0.1)
        truncated = False # Can add time limits here

        return self.state, reward, terminated, truncated, {}

    def render(self):
        print(f"Robot Position: {self.state[0]:.2f} | Goal: {self.goal_height}")

# --- Test the Environment ---
env = RobotLiftEnv()
obs, info = env.reset()

for _ in range(10):
    action = env.action_space.sample() # Take a random action
    obs, reward, terminated, truncated, info = env.step(action)
    env.render()

    if terminated:
        print("Goal Reached!")
        break