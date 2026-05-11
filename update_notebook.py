import json

path = "stretch3_mujoco_rl_gym.ipynb"
with open(path, "r") as f:
    data = json.load(f)

for cell in data["cells"]:
    if cell["cell_type"] == "code":
        src = cell["source"]
        for i, line in enumerate(src):
            if "from stretch_reach_env import StretchReachEnv\n" == line:
                src[i] = "from stretch_reach_env import StretchReachEnv as LegacyStretchReachEnv\n"
                src.insert(i+1, "from stretch_reach_env_stuck_flag_o import StretchReachEnv as StuckFlagStretchReachEnv\n")
            elif line.startswith("def make_env("):
                if "env_version" not in line:
                    src[i] = line.replace('run_name="run", ):', 'run_name="run", env_version="stuck_flag_o"):').replace('run_name="run"):', 'run_name="run", env_version="stuck_flag_o"):')
            elif "env = StretchReachEnv(" in line:
                # Add EnvClass logic right before this line
                src.insert(i, "        EnvClass = LegacyStretchReachEnv if env_version == 'legacy' else StuckFlagStretchReachEnv\n")
                src[i+1] = line.replace("StretchReachEnv(", "EnvClass(")

with open(path, "w") as f:
    json.dump(data, f, indent=1)
    
print("Notebook updated!")
