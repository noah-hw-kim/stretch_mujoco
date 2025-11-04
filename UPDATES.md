## Explore Google Colab
- Tested all functions in demo
- How to run the streaming while moving?

## Completed Tasks
### available headless=False
- configure to use only nvidia gpu (my laptop has cpu and external gpu)
- streaming cameras on the background for now (need to move to main thread and tele op on the background) 

### move the robot (base, lift/arm, head) - completed
- used apis for move_by and move_to

## Current Tasks
### Pull Out Robot Status
- "arm" move range different (current: (0.0,0.13), document: (0.0, ~0.52) -> document is correct)
- "head_tilt" move range different (current: (-1.53, 0.79), document: (-2.02, 0.49) -> current set up is correct)
- ref: https://docs.hello-robot.com/0.3/getting_started/stretch_hardware_overview/#lift-and-arm

```
pprint(sim.pull_status())
pprint(sim.pull_camera_data())
pprint(sim.pull_sensor_data())
pprint(sim.pull_joint_limits())
```

### Object Generation and Locations
comments:
- default: objects and objects locations are randomly generated with model_generation_wizard() in robocasa_gen.py
- It's generated when we create the env file and I don't think there's a way to feed arguments to have objects I want specifically. -> I need to update the env file after it generates random objects.

1. How to get object locations in Robocasa?
- was able to get the initial locations (It only shows the initial location)

```
# Pretty-print objects and their initial placements
for body_name, info in objects_info.items():
    print(f"{body_name:30s}  cat={info['cat']:12s}  pos={info['pos']}  quat={info['quat']}")
```

- need to get the real time object locations

FINDINGS (on the default view of the room)
- X increases when the robot moves from left to right of the room
- Y increases when the robot moves from front to back of the room (counter/cabinet is closer to the backside of the room)
- Z increases when the robot moves from low to high of the room

- Z is the up and down (think the direction of the lift up and down)
- X is the left and right 
- Y is the forward and backward (think the direction of the arm forward and backward)




2. How to generate specific object in Robocasa (for same test env)?
- There are total 153 kitchen objects/categories and each object/category has "objaverse" and "aigen" multiple models
```
# List all categories
from robocasa.models.objects.kitchen_objects import OBJ_CATEGORIES
cats = sorted(OBJ_CATEGORIES.keys())
len(cats), cats[:25]  # show count and first 25
```

```
from robocasa.models.objects.kitchen_objects import OBJ_CATEGORIES

# Available registries are usually "objaverse" and/or "aigen"
list(OBJ_CATEGORIES["mug"].keys())  # e.g., ['objaverse', 'aigen']

# List concrete model xmls for a category + registry
paths = OBJ_CATEGORIES["mug"]["objaverse"].mjcf_paths
len(paths), paths[:5]
```

- Default method "model_generation_wizard()" in "robocasa_gen.py" generate random 3 objects from objaverse or aigen.



3. ETC
Category: the kind of thing (names like apple, mug, knife). You choose categories.
Registry: where the assets come from (usually “objaverse” or “aigen”).
Object 3D model (asset): one specific file for a category (e.g., objaverse/mug_12/model.xml). RoboCasa picks one of these when it spawns the scene.
Pose: where the object is placed in the world (position + orientation). This changes during simulation.
MuJoCo model (world): the entire compiled simulation (all bodies, joints, geoms) that the physics engine uses.



## Future Tasks
### real v.s. sim
- compare camera resolutions (real v.s. sim) (use array.shape)
- sim
1. d405 rgb (w,h,c)
(270, 480, 3)

2. d435 rgb (w,h,c)
(240, 424, 3)

3. nav (w,h,c)
(600, 800, 3)

- real
1. d405 rgb (w,h,c)

2. d435 rgb (w,h,c)

3. nav (w,h,c)

- joint value (double check)




### move camera streaming to main thread

### Neeed teleop like (WASD?)

### yolo detection