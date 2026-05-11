import json

path = "stretch3_mujoco_rl_gym.ipynb"
with open(path, "r") as f:
    data = json.load(f)

for cell in data["cells"]:
    if cell["cell_type"] == "code":
        src = cell["source"]
        for i, line in enumerate(src):
            if line.startswith("def make_env("):
                # We need to add lift_start_below_range=(0.6, 0.88)
                src[i] = line.replace('lift_start_random=False,', 'lift_start_random=False, lift_start_below_range=(0.6, 0.88),')
            if "lift_start_random=lift_start_random," in line:
                # Add lift_start_below_range=lift_start_below_range right after it
                src.insert(i+1, "            lift_start_below_range=lift_start_below_range,\n")
            if "lift_start_random = False" in line and "lift_start_pos = 0.8" in "".join(src):
                src[i] = "lift_start_random = True\n"
                src.insert(i+1, "lift_start_below_range = (0.80, 0.88)\n")
            if "run_name=run_name," in line and "lift_start_pos=lift_start_pos," in "".join(src):
                # Update the make_env call in the training cell
                if "lift_start_pos=lift_start_pos," in "".join(src):
                     # Actually we can just add it before run_name
                     src.insert(i, "        lift_start_below_range=lift_start_below_range,\n")
            if "ent_coef=0.03," in line and "PPO(" in "".join(src):
                src[i] = "    ent_coef=0.05,\n"

with open(path, "w") as f:
    json.dump(data, f, indent=1)
    
print("Notebook updated!")
