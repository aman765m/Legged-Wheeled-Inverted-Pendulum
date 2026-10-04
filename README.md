# Legged-Wheeled Inverted Pendulum with an active arm (ROS 2 Humble)

Simulation of a **wheel-legged biped robot with a 2-link arm** that uses the arm to keep its
balance. The robot is an inverted pendulum on two wheels, balanced by an LQR controller. When it
gets pushed, the arm accelerates the other way, and in curves it swings towards the inside of the
turn. This raises the push it can take before falling and moves its balance point towards the
inside of curves.

It is a re-implementation of

> F. Raza, W. Zhu, M. Hayashibe, *"Balance Stability Augmentation for Wheel-Legged Biped Robot
> Through Arm Acceleration Control"*, IEEE Access, vol. 9, 2021. doi:10.1109/ACCESS.2021.3071055

The authors released no code or CAD. The robot model, meshes and all code here were written
from scratch using the paper.

<p align="center"><img src="wheel_legged_arm_balance/docs/robot.png" width="420"></p>

**Contents**
[Repository layout](#repository-layout) ·
[Set up the workspace](#1-set-up-the-workspace) ·
[Quick start](#2-quick-start) ·
[Launch files and arguments](#3-launch-files-and-arguments) ·
[Other commands](#4-other-commands) ·
[Worlds](#5-worlds) ·
[What the code does](#6-what-the-code-does) ·
[Results](#7-results) ·
[Troubleshooting](#8-troubleshooting)

---

## Repository layout

This repository is the `src/` folder of a colcon workspace. It holds one ROS 2 package:

```
wheel_legged_arm_balance/
├── urdf/        robot model (xacro): body, fixed legs, wheels, 2-link arm, can, ros2_control + Gazebo tags
├── meshes/      STL meshes (procedurally generated, see scripts/generate_meshes.py)
├── worlds/      flat_ground, rough_terrain (+ heightmap), figure8_track
├── config/      controllers.yaml (balance + arm controllers), arm_fsm.yaml (arm state machine + CMP)
├── launch/      sim, push_experiment, infinity_loop, display
├── src/         C++: 2 ros2_control controllers + 2 nodes
├── include/     C++ headers: robot model, LQR solver, arm dynamics
├── scripts/     Python tools: start-up, pushes, path tracking, logging, analysis, reset, generators
├── test/        unit tests (gtest)
├── rviz/        RViz configs
└── docs/        figures and reproduced results
```

`wheel_legged_arm_balance/README.md` is the technical companion to this file. It covers how each
part maps to the paper, every assumption made, and the detailed results.

## 1. Set up the workspace

**Requirements:** Ubuntu 22.04 with [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
(desktop) and Gazebo Classic 11. Gazebo comes with `ros-humble-gazebo-ros-pkgs`.

```bash
# 1. create a workspace and clone this repository as its src/ folder
mkdir -p ~/ros2_ws
cd ~/ros2_ws
git clone git@github.com:aman765m/Legged-Wheeled-Inverted-Pendulum.git src

# 2. install the dependencies listed in package.xml
source /opt/ros/humble/setup.bash
sudo rosdep init        # only once per machine; ignore the error if it was already done
rosdep update
rosdep install --from-paths src --ignore-src -y

# 3. build
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# 4. source the workspace (do this in every new terminal, or add it to ~/.bashrc)
source ~/ros2_ws/install/setup.bash
```

`rosdep` installs, among others: `gazebo_ros_pkgs`, `gazebo_ros2_control`, `ros2_control`,
`ros2_controllers` (incl. `imu_sensor_broadcaster`), `xacro`, `robot_state_publisher`,
`joint_state_publisher_gui`, `rviz2`, `teleop_twist_keyboard`, Eigen, urdfdom,
python3-numpy / scipy / matplotlib / pil.

> **Using Anaconda / Miniconda?** Its `python3` shadows the system Python that ROS 2 uses, and the
> build or the launch files then fail with import errors. Before building or running, use
> `conda deactivate`. If `~/miniconda/bin` (or `~/anaconda3/bin`) is on your `PATH` permanently,
> also remove it for that terminal, then source ROS again.

Check the installation (unit tests of the model, LQR solver and arm dynamics):

```bash
colcon test --packages-select wheel_legged_arm_balance && colcon test-result --verbose
```

## 2. Quick start

```bash
# Gazebo + RViz, robot balancing on flat ground, arm in active mode
ros2 launch wheel_legged_arm_balance sim.launch.py
```

Gazebo starts **paused**, the controllers come up, and then physics starts. This takes about
10–20 s. In RViz the **blue sphere** is the robot's centre of mass and the **pink sphere** is its
**CMP** (centroidal moment pivot, the balance indicator). The further they are apart, the
harder the robot is working to stay up.

In a second terminal (workspace sourced):

```bash
# drive with the keyboard (press z a few times first: it starts at 0.5 m/s, max is 1 m/s)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/lqr_balance_controller/cmd_vel

# give it a 29 N shove (it falls at this force with arm_mode:=fixed, survives with active)
ros2 run wheel_legged_arm_balance push_disturbance.py --ros-args -p use_sim_time:=true -p force:=29.0 -p start_time:=0.0

# put it back at the start (e.g. after it fell) without restarting Gazebo
ros2 run wheel_legged_arm_balance reset_sim.py
```

## 3. Launch files and arguments

Arguments are passed as `name:=value`. Run `ros2 launch wheel_legged_arm_balance <file> --show-args`
to list them.

### `sim.launch.py`: the simulation

Starts Gazebo, spawns the robot, starts the controllers, the arm state machine, the CMP estimator
and RViz. The other launch files include this one.

| argument | default | meaning |
|---|---|---|
| `world` | `flat` | `flat`, `rough`, `track`, or a path to any `.world` file |
| `arm_mode` | `active` | `active`: arm reacts to pushes / turns. `fixed`: arm stays at home (baseline) |
| `payload_mass` | `0.36` | mass of the can held by the arm [kg] (paper: 0.36 and 0.72) |
| `gui` | `true` | start the Gazebo window (`false` = headless, faster) |
| `rviz` | `true` | start RViz |
| `gain_source` | `paper` | LQR gain: `paper` (printed in the paper) or `computed` (solved from the model) |
| `wheel_torque_limit` | `4.5` | wheel motor torque limit [Nm]; sets how hard a push the robot survives |
| `K_x` | *(from config: 6.0)* | arm reaction gain: arm acceleration = −K_x × robot acceleration |
| `lump_arm` | `false` | include the arm mass in the model used for `gain_source:=computed` |
| `x`, `y`, `yaw` | `0` | spawn pose |
| `verbose` | `false` | verbose Gazebo output |

```bash
ros2 launch wheel_legged_arm_balance sim.launch.py world:=rough arm_mode:=fixed
ros2 launch wheel_legged_arm_balance sim.launch.py payload_mass:=0.72 gui:=false
```

### `push_experiment.launch.py`: one push test

Headless by default. Applies a push to the front of the robot at t = 10 s, logs everything, and
**shuts itself down at t = 20 s**. It writes `<output_dir>/<run_name>.csv` (time series) and
`.json` (summary: did it fall, integrated CMP error).

| argument | default | meaning |
|---|---|---|
| `force` | `27.0` | push force [N] (backwards, on the body) |
| `push_duration` | `1.0` | how long the force is applied [s] |
| `push_time` | `10.0` | when the push starts [s, sim time] |
| `start_jitter` | `0.0` | random ± offset on `push_time` (for repeated trials) |
| `duration` | `20.0` | when the experiment ends [s] |
| `output_dir` | `~/wlbr_results` | where results are written |
| `run_name` | `push` | file name of the results |
| `arm_mode`, `payload_mass`, `world`, `gui`, `rviz`, `gain_source`, `wheel_torque_limit`, `K_x` | | as for `sim.launch.py` (`gui`/`rviz` default `false`) |

```bash
ros2 launch wheel_legged_arm_balance push_experiment.launch.py force:=29 arm_mode:=fixed  run_name:=f29_fixed
ros2 launch wheel_legged_arm_balance push_experiment.launch.py force:=29 arm_mode:=active run_name:=f29_active gui:=true
```

### `infinity_loop.launch.py`: drive a figure-8

The robot drives one figure-8 (pure-pursuit path tracking), logs it, and shuts down when the lap
is done.

| argument | default | meaning |
|---|---|---|
| `v_max` | `0.3` | forward speed [m/s] (paper: 0.3 normal, 0.7 fast) |
| `w_max` | `0.3` | max turn rate [rad/s] (paper: 0.3 normal, 0.6 fast) |
| `size` | `4.0` | half width of the figure-8 [m] (the `track` world is drawn for 4.0) |
| `laps` | `1.0` | number of laps |
| `path_anchor` | `robot` (`world` when `world:=track`) | `robot`: path starts where the robot is; `world`: path fixed at the origin (the drawn track) |
| `duration` | `120.0` | safety stop [s] |
| `output_dir`, `run_name` | `~/wlbr_results`, `loop` | results files |
| `arm_mode`, `payload_mass`, `world`, `gui`, `rviz`, `gain_source`, `wheel_torque_limit` | | as for `sim.launch.py` |

```bash
ros2 launch wheel_legged_arm_balance infinity_loop.launch.py world:=track
ros2 launch wheel_legged_arm_balance infinity_loop.launch.py world:=track v_max:=0.7 w_max:=0.6 arm_mode:=fixed
ros2 launch wheel_legged_arm_balance infinity_loop.launch.py world:=rough
```

### `display.launch.py`: look at the model only

RViz with sliders for the arm and wheel joints; no simulation, no arguments.

```bash
ros2 launch wheel_legged_arm_balance display.launch.py
```

## 4. Other commands

All of these run in a second terminal while a simulation is running, except where noted.

| command | what it does |
|---|---|
| `ros2 run wheel_legged_arm_balance reset_sim.py` | put the robot back at its spawn pose and reset the controllers |
| `ros2 run wheel_legged_arm_balance push_disturbance.py --ros-args -p use_sim_time:=true -p force:=29.0 -p start_time:=0.0` | push the robot now (`duration` param, default 1 s) |
| `ros2 topic pub -r 10 /lqr_balance_controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.3}}"` | drive with a fixed command (use `-r 10`: commands expire after 0.5 s) |
| `ros2 run wheel_legged_arm_balance run_push_campaign.py --forces 27 29 31 33 --masses 0.36 0.72 --repeats 5` | **no sim needed**: runs many push experiments headless and builds the results table (`table2.md`) and plots |
| `ros2 run wheel_legged_arm_balance analyze_results.py push <folder>` | results table and CMP plots from push runs |
| `ros2 run wheel_legged_arm_balance analyze_results.py loop <run>.csv [...]` | path / CMP plots of figure-8 runs |
| `ros2 run wheel_legged_arm_balance compute_lqr.py --Q 1 1 1 1 1 1 --R 1 1` | **no sim needed**: offline LQR design (like MATLAB `lqr`) |
| `python3 src/wheel_legged_arm_balance/scripts/generate_meshes.py` | regenerate the STL meshes |
| `python3 src/wheel_legged_arm_balance/scripts/generate_worlds.py` | regenerate `rough_terrain` (try `--seed`, `--bump-height 0.12`) and `figure8_track` |

Useful topics to watch (`ros2 topic echo <topic>` or `rqt_plot`):

| topic | content |
|---|---|
| `/lqr_balance_controller/state` | position, yaw, lean angle β, speed, yaw rate, lean rate, wheel torques, references |
| `/arm_fsm/state` | arm state (0 home, 1 push, 2 right, 3 left, 4 hold), measured acceleration, target |
| `/rac_arm_controller/state` | end-effector position / target, arm torques |
| `/cmp_estimator/cmp_error` | CMP minus CoM ground projection (x forward, y left) [m] |
| `/cmp_estimator/cmp_error_integral` | running integral of the above [m·s] |

## 5. Worlds

| `world:=` | description |
|---|---|
| `flat` | flat ground (used for all the paper's experiments) |
| `rough` | 32 m × 32 m bumpy terrain, ~8 cm bumps, max slope ~6°, flat 1.5 m pad at the spawn point. Built on Gazebo's stock `heightmap.world` (same heightmap model and textures) with a generated bumpy heightmap, because Gazebo's own heightmaps are smooth slopes |
| `track` | grass with an asphalt figure-8 road (0.8 m wide, white centre line, yellow start line at the spawn point). Built on Gazebo's stock `road.world` elements. It is exactly the path that `infinity_loop.launch.py` follows |

Tested: one figure-8 at 0.3 m/s completes without falling in both worlds, staying on average 4–5 cm
from the path (max 13 cm).

## 6. What the code does

### The robot

The model follows the paper's parameter table: a 7.5 kg body whose centre of mass is 0.59 m above
the wheel axle, two 0.35 kg wheels of radius 0.1016 m, and a 2-link horizontal arm (0.3 m + 0.3 m)
holding a can. The legs are rigid; as in the paper, only wheels and arm are actuated. Total
mass: 10.3 kg.

### How the pieces fit together

```
                 ┌──────────────────────── Gazebo (1 kHz physics) ──────────────────────────┐
                 │ robot + IMU + ground-truth pose        gazebo_ros2_control                │
                 └───────┬───────────────────────────────────────────────┬───────────────────┘
          IMU, wheel and arm joint states                       wheel torques, arm torques
                         │                                               │
      ┌──────────────────▼─────────────┐                ┌────────────────┴───────────────┐
      │ lqr_balance_controller (1 kHz) │                │ rac_arm_controller (1 kHz)      │
      │  keeps the robot upright and   │                │  moves the hand along the       │
      │  follows the velocity command  │                │  commanded path (Cartesian)     │
      └──────────────────▲─────────────┘                └────────────────▲───────────────┘
                         │ /lqr_balance_controller/cmd_vel               │ /rac_arm_controller/reference
         teleop / infinity_loop_tracker ─────────────┐    ┌──────────────┴───────────────┐
                                                     └───►│ arm_fsm_node (500 Hz)          │◄── IMU
                                                          │  decides where the arm goes    │
                                                          └────────────────────────────────┘
      cmp_estimator_node: whole-body CoM + CMP from joint states and the ground-truth pose → metric, RViz
```

### Components

- **`lqr_balance_controller`** (C++ ros2_control plugin, `src/lqr_balance_controller.cpp`)
  is the wheeled-inverted-pendulum balance controller. Each 1 ms tick it measures the robot
  state: travelled distance, heading, lean angle of the centre of mass, and their rates. It then
  sets both wheel torques with a linear-quadratic regulator, `torque = −K·(state − reference)`.
  The gain `K` is the one printed in the paper. Alternatively, the controller derives the
  linearised robot model and solves the Riccati equation itself (`gain_source:=computed`). The
  lean angle includes the arm's position, so moving the arm shifts the balance point. The
  reference comes from `cmd_vel` (forward speed + turn rate) and includes a wheel-friction
  feed-forward.

- **`rac_arm_controller`** (C++ ros2_control plugin) is resolved acceleration control of the
  arm. Given a desired hand position, velocity and acceleration, it computes joint torques using
  the arm's dynamics: inertia matrix, Coriolis terms and Jacobian. The hand then follows the
  commanded motion accurately, even when it moves fast.

- **`arm_fsm_node`** (C++) is the paper's state machine, deciding where the arm should go:
  - **Push detected** (robot acceleration above about 1.9 m/s², which corresponds to a 20 N
    push): accelerate the hand **opposite** to the robot's acceleration, hold it there for
    1 s while the robot recovers, then return home.
  - **Turning** (averaged turn-rate command ≠ 0): move the hand towards the **inside** of the
    curve, to the right for clockwise turns and to the left for counter-clockwise. This
    counteracts the centrifugal effect.
  - **Otherwise:** keep the hand at home, 0.3 m in front of the shoulder.

  `arm_mode:=fixed` keeps the arm at home for comparison.

- **`cmp_estimator_node`** (C++) computes the whole-body centre of mass from the URDF masses,
  the joint angles and the robot pose. From its acceleration it derives the **centroidal moment
  pivot**: the point on the ground where the reaction force would have to act for the robot not
  to rotate. The CMP–CoM distance is the balance metric, and its time integral is the paper's
  error measure. The node also publishes the RViz markers and the TF from world to robot.

- **Python tools** (`scripts/`):
  - `sim_starter.py`: activates the controllers and unpauses physics at the same instant, so
    the robot is never uncontrolled.
  - `push_disturbance.py`: applies pushes through Gazebo.
  - `infinity_loop_tracker.py`: pure-pursuit path following.
  - `experiment_logger.py`: writes the CSV and JSON results.
  - `analyze_results.py` and `run_push_campaign.py`: results table and plots.
  - `reset_sim.py`: resets the robot without restarting Gazebo.
  - `compute_lqr.py`: offline LQR design.
  - `generate_meshes.py` and `generate_worlds.py`: regenerate the meshes and the extra worlds.

### Main tuning knobs

- `config/controllers.yaml`: LQR gain source, Q/R weights, torque limit, speed and acceleration
  limits, arm controller gains.
- `config/arm_fsm.yaml`: push threshold `a_th`, arm gain `K_x`, hold time, home / left / right
  hand positions.

Some values are not in the paper and were chosen or calibrated: the 4.5 Nm wheel torque limit, the
1 s push, `K_x`, and the robot's geometry. They are listed with reasons in
`wheel_legged_arm_balance/README.md` → *Assumptions*.

## 7. Results

The package reproduces both of the paper's experiments in simulation. Details and per-run numbers
are in the package README.

| | this simulation | paper |
|---|---|---|
| largest push survived, fixed arm (0.36 kg can) | 28 N | 27 N |
| largest push survived, active arm | 31 N (falls at 33 N) | 31.5 N |
| CMP error reduction with the active arm (27 N) | ~8 % | 12–19 % |
| CMP side relative to curves (figure-8, arm active) | moves to the inside (+4 to +6.5 cm) | moves to the inside |

The heavier 0.72 kg can does **not** raise the push limit further here (both modes fall at 31 N),
unlike in the paper.

<p align="center"><img src="wheel_legged_arm_balance/docs/fig8_infinity_loops.png" width="900"></p>

## 8. Troubleshooting

| symptom | fix |
|---|---|
| build or launch fails with Python import errors | conda is active: see the note in [section 1](#1-set-up-the-workspace) |
| Gazebo hangs or the robot never appears; log shows `Address already in use` | an old Gazebo is still running: `pkill -x gzserver; pkill -x gzclient`, then relaunch |
| `ros2 topic pub ...` fails with a YAML `mapping values` error | put the message on one line in quotes: `"{linear: {x: 0.1}}"` |
| robot falls right after a reset | use `reset_sim.py`, not Gazebo's *Reset* menu or `/reset_simulation` (rewinding time breaks the controllers) |
| robot drives off or stutters with `ros2 topic pub` | publish continuously with `-r 10`: commands time out after 0.5 s |
| two people / two sims on the same network interfere | give each one its own `export ROS_DOMAIN_ID=<0-101>` before launching |

## Citation

If you use this work, please cite the original paper:

```bibtex
@article{raza2021balance,
  title   = {Balance Stability Augmentation for Wheel-Legged Biped Robot Through Arm Acceleration Control},
  author  = {Raza, Fahad and Zhu, Wei and Hayashibe, Mitsuhiro},
  journal = {IEEE Access},
  volume  = {9},
  pages   = {54022--54031},
  year    = {2021},
  doi     = {10.1109/ACCESS.2021.3071055}
}
```
