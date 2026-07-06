# Hex Docking Demo | How to Run

Simulation of hex robots docking onto a leader robot to form a honeycomb structure.

## File Structure
\`\`\`
hex_docking/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── hex_docking
├── hex_docking/
│   ├── __init__.py
│   └── docking_node.py      # docking state machine (FORMATION, FACE_DIST, etc.)
├── launch/
│   └── dock.launch.py       # spawns robots, starts Gazebo, bridges topics
├── models/
│   └── hex_bot.sdf          # hexagonal robot model (chassis + wheels + plugin)
└── worlds/
    └── dock_world.sdf       # empty ground-plane world
\`\`\`

## Setup (one time)
\`\`\`bash
cd ~/dock_ws        # wherever you clone this repo into
colcon build
source install/setup.bash
\`\`\`

If you're on WSL2, also run this once per terminal you open:
\`\`\`bash
export LIBGL_ALWAYS_SOFTWARE=1
\`\`\`

## Terminal 1 — start the simulation
\`\`\`bash
cd ~/dock_ws
source install/setup.bash
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch hex_docking dock.launch.py
\`\`\`
Wait until the Gazebo window opens and all 9 robots are visible (~10 seconds). Leave this terminal running — don't close it or press Ctrl+C.

## Terminal 2 — start the docking behavior
Open a new terminal, then:
\`\`\`bash
cd ~/dock_ws
source install/setup.bash
ros2 run hex_docking docking_node
\`\`\`

## Formation
9 robots total: robot1 is the fixed leader. Each satellite docks to a specific hex face of its assigned anchor:

| Robot | Anchor | Face | Direction |
|---|---|---|---|
| robot2 | robot1 | 1 | 60°  |
| robot3 | robot1 | 2 | 120° |
| robot4 | robot1 | 4 | 240° |
| robot5 | robot1 | 5 | 300° |
| robot6 | robot1 | 0 | 0° (right) |
| robot7 | robot3 | 3 | 180° (second tier, off robot3) |
| robot8 | robot2 | 1 | 60° (second tier, off robot2) |
| robot9 | robot5 | 5 | 300° (second tier, off robot5) |

Docking is sequential — each robot waits for its anchor to be docked before starting its own approach.

## Key parameters (`docking_node.py`)
- `FACE_DIST = 0.312` — target center-to-center docking distance (2× hex apothem, flush contact)
- `OBSTACLE_RADIUS = 0.22` — collision-avoidance radius used when routing around other docked robots
- `DOCK_TOL = 0.02` — final position tolerance (2cm) before locking in
- `ORBIT_LIN_SPEED = 0.2` — speed while circling into position before final approach
