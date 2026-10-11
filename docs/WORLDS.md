# Changing the Gazebo world

A world is one SDF file in `src/ugv_description/worlds/`. The launch file takes it with `world:=`.

## A. Run a different world (no code)

```bash
cd ~/pathrage && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch ugv_navigation full_stack.launch.py world:=offroad_forest viewer:=true
# or only the simulator:
ros2 launch ugv_description gazebo.launch.py world:=offroad_forest
# a world file anywhere on disk also works:
ros2 launch ugv_description gazebo.launch.py world:=/home/you/my.world
```

`world:=` accepts a name from `worlds/` or a full path. Start position and heading:
`x:=2.0 y:=-1.0 yaw:=1.57` (works on both `full_stack.launch.py` and `gazebo.launch.py`).

Worlds shipped: `outdoor_offroad` (original: 2 rocks, flat grey ground), `offroad_sparse`, `offroad_forest`
(generated, textured ground).

## B. Make a new world with the generator

```bash
python3 tools/make_world.py --name offroad_dense --seed 5 --rocks 40 --trees 15 --ramps 1 \
    --size 50 --goal 22,-3 --out src/ugv_description/worlds/offroad_dense.world
colcon build --packages-select ugv_description --symlink-install   # only needed because it is a NEW file
```

| Option | Meaning |
|---|---|
| `--seed` | same seed = same world (reproducible benchmarks) |
| `--rocks / --trees / --ramps` | obstacle counts (rocks 0.18-0.55 m high, trunks 2-3 m, slopes 6-11 deg) |
| `--size` | side of the obstacle field in metres |
| `--start / --goal` | spawn point (keep in sync with `x:= y:=`) and green goal marker |
| `--clear-radius` | obstacle-free circle around the start |
| `--min-gap` | free distance between obstacles; robot is 0.40 m wide, so keep >= 0.8 |

If it prints "N did not fit", lower the counts or raise `--size`.

## C. Edit a world by hand

1. Copy an existing `.world` to a new name (do not edit generated files; regenerate them).
2. Add a model. Static box obstacle:
   ```xml
   <model name="wall_1">
     <static>true</static>
     <pose>8 2 0.25 0 0 0.3</pose>            <!-- x y z roll pitch yaw -->
     <link name="link">
       <collision name="c"><geometry><box><size>2 0.2 0.5</size></box></geometry></collision>
       <visual name="v"><geometry><box><size>2 0.2 0.5</size></box></geometry>
         <material><ambient>0.4 0.4 0.4 1</ambient><diffuse>0.4 0.4 0.4 1</diffuse></material></visual>
     </link>
   </model>
   ```
   `z` = half the height so the object sits on the ground. Always give `collision` AND `visual`.
3. Keep it detectable: obstacles taller than the 0.06 m ground clearance and at least 0.15 m high
   (the Tier-1 threshold) are real obstacles; anything lower is treated as ground.
4. Keep the start (0,0) free, and put a goal marker where Point B is.
5. Save under `src/ugv_description/worlds/`, rebuild once (new file), launch with `world:=<name>`.

## D. Rules that make a world usable for SLAM and stereo

* **Texture.** A flat single-colour ground has no features for ORB-SLAM3 or stereo matching. Use
  `model://terrain_textured` (already in the generated worlds). Regenerate or restyle the image with
  `python3 tools/make_ground_texture.py --seed N`.
* **Visual landmarks.** Put varied objects within 2-6 m of the route, never an empty field.
* **Range.** Stereo is trustworthy to about 6 m (docs/INTERFACES.md section 3). Objects beyond that
  are not mapped reliably; that is expected.
* **Lighting.** Change `<include><uri>model://sun</uri></include>` to a custom `<light>` (direction,
  diffuse) to test low sun or shadows (Phase 5.3).

## E. Real terrain (ditches, hills) - Phase 5

The flat plane cannot hold ditches. Use a heightmap:

1. Make a greyscale PNG, square, side `2^n + 1` (129, 257 or 513). White = high.
2. Put it in a model folder (copy `models/terrain_textured`), point a `<heightmap>` at it:
   ```xml
   <geometry><heightmap>
     <uri>model://terrain_hills/materials/textures/height.png</uri>
     <size>60 60 2.5</size>                 <!-- x y max-height in metres -->
     <pos>0 0 0</pos>
   </heightmap></geometry>
   ```
   in both `<collision>` and `<visual>`; add a `<texture>` block for colour.
3. Replace the `terrain_textured` include with your model. Spawn the robot at a `z` above the local height.

The viewer's ground reference is the robot's own height, so slopes read correctly; the Tier-1 ground-plane
fit (Phase 3.2) is what handles slopes properly.

## F. Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `World file not found` | name typo, or file not in `worlds/`; the error lists what exists |
| Ground is white/grey, no texture | `GAZEBO_MODEL_PATH` missing the models folder: launch through `gazebo.launch.py` (it sets it), or `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/pathrage/src/ugv_description/models` |
| Texture looks stretched or huge | tiling is set by the UVs in `models/terrain_textured/meshes/ground.obj` (30 repeats over 120 m = one tile per 4 m); change the `vt` values |
| Gazebo window stays black on the very first start | Gazebo Classic can spend a while building/fetching its model database the first time; wait, or start `gzserver --verbose` to see what it waits for |
| Robot spawns inside an obstacle | raise `--clear-radius`, or move the spawn with `x:= y:=` |
| New world not found after adding it | new files need `colcon build ... --symlink-install` once |
