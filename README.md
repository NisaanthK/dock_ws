# Hex Docking Demo — How to Run

hex_docking/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── hex_docking
├── hex_docking/
│   ├── __init__.py
│   └── docking_node.py          # docking state machine (FORMATION, FACE_DIST, etc.)
├── launch/
│   └── dock_launch.py           # spawns robots, starts Gazebo, bridges topics
├── models/
│   └── hex_bot.sdf              # hexagonal robot model (chassis + wheels + plugin)
└── worlds/
    └── dock_world.sdf           # empty ground-plane world

  
Simulation of hex robots docking onto a leader robot to form one connected
structure.

## Setup (one time)

```bash
cd ~/dock_ws        # wherever you clone this repo into
colcon build
source install/setup.bash
```

If you're on WSL2, also run this once per terminal you open:
```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

---

## Terminal 1 — start the simulation

```bash
cd ~/dock_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch hex_docking dock.launch.py
```

Wait until the Gazebo window opens and all robots are visible (~10 seconds).
**Leave this terminal running** — don't close it or press Ctrl+C.

---

## Terminal 2 — start the docking behavior

Open a **new** terminal, then:

```bash
cd ~/dock_ws
source install/setup.bash
ros2 run hex_docking docking_node
```

Watch the Gazebo window — the follower robots will drive over and dock onto
the leader one at a time. The terminal will print `DOCKED!` as each one
finishes.

---

## To stop and run again

- Ctrl+C in **terminal 2** first (stops the docking behavior)
- Ctrl+C in **terminal 1** (closes the simulation)
- Re-run both steps above to start fresh

---

## If something looks wrong

- **Robots not moving / nothing prints in terminal 2** -> check terminal 1
  is still running and the robots are visible in Gazebo.
- **Docked robots touching at a corner instead of a flat side** -> open
  `hex_docking/docking_node.py`, find this line near the top:
  ```python
  FACE_ANGLE_OFFSET = math.radians(30)
  ```
  Change `30` to `-30` (or vice versa), save, then rebuild:
  ```bash
  colcon build
  source install/setup.bash
  ```
  and run both terminals again.
- **Want to change spacing (gap between docked robots)** -> same file, find:
  ```python
  FACE_DIST = 0.45
  ```
  Smaller number = tighter fit. Rebuild after changing.
