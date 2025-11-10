# Autonomous Obstacle-Aware Exploration and Path Planning

## Overview

This project implements a **four-wheel Ackermann-steered mobile robot** capable of autonomously exploring and navigating a 2D environment with 3D visualization. The robot generates collision-free trajectories, discovers unexplored regions, and maintains smooth motion control under non-holonomic constraints.

## Features

### ✅ Implemented Components

1. **Robot Modeling**
   - Ackermann steering kinematics (bicycle model)
   - Trapezoidal velocity profiles for smooth acceleration/deceleration
   - 4-wheel visualization with realistic dimensions

2. **Environment & Visualization**
   - 2D occupancy grid mapping with probabilistic updates
   - 3D visualization using Matplotlib
   - Multiple obstacle types: Box, Cylinder, Cone, Hemisphere
   - Real-time LiDAR scanning and mapping

3. **Path Planning**
   - **A* global planner** with obstacle inflation for safety
   - Path smoothing for efficient trajectories
   - Collision checking and validation

4. **Exploration Strategy**
   - **Frontier-based exploration** to identify unexplored regions
   - Information gain calculation for frontier selection
   - Automatic goal selection and replanning

5. **Motion Control**
   - **Pure pursuit path following** controller
   - Trapezoidal velocity profiles for smooth motion
   - Adaptive speed control based on path curvature

## Project Structure

```
take2/
├── src/
│   ├── robot/
│   │   └── ackermann.py          # Ackermann robot model
│   ├── sensors/
│   │   └── lidar.py               # LiDAR sensor simulation
│   ├── map/
│   │   ├── occupancy_grid.py      # Probabilistic occupancy mapping
│   │   └── occupancy_map.py       # Legacy map interface
│   ├── planning/
│   │   ├── astar.py               # A* path planner
│   │   └── frontier_explorer.py   # Frontier detection & selection
│   ├── control/
│   │   └── velocity_controller.py # Trapezoidal velocity & path following
│   ├── objects/
│   │   ├── primitives.py          # 3D obstacle primitives
│   │   └── generator.py           # Environment generation
│   ├── env/
│   │   ├── env_3d.py              # 3D visualization
│   │   ├── env_2d_map.py          # 2D map visualization
│   │   └── demo_env.py            # Simple demo
│   └── autonomous_explorer.py     # Main autonomous system
├── requirements.txt
└── README.md
```

## Installation

1. **Create virtual environment:**
```bash
cd /Users/aryanmathur/Desktop/RDCP/take2
python3 -m venv venv
source venv/bin/activate
```

2. **Install dependencies:**
```bash
pip install -r requirements.txt
```

Required packages:
- numpy
- matplotlib
- scipy

## Usage

### Run Autonomous Exploration

```bash
source venv/bin/activate
python3 -m src.autonomous_explorer
```

This will:
1. Generate a random environment with obstacles
2. Initialize the robot at position (2, 2)
3. Start autonomous exploration using frontier-based strategy
4. Display real-time 3D visualization
5. Continue until 85% of the map is explored or max iterations reached

### Run Simple Demo

```bash
python3 src/env/demo_env.py
```

This runs a simpler demo with manual control patterns.

## How It Works

### 1. Sensing
- LiDAR sensor scans the environment (270° FOV, 108 beams)
- Returns range measurements to obstacles

### 2. Mapping
- Occupancy grid updated using probabilistic ray tracing
- Bresenham's algorithm for efficient line tracing
- Log-odds representation for robust updates

### 3. Planning
- **Frontier Detection**: Identifies boundaries between free and unknown space
- **Frontier Selection**: Chooses best frontier based on distance and information gain
- **A* Planning**: Computes optimal collision-free path to selected frontier
- **Path Smoothing**: Removes unnecessary waypoints

### 4. Control
- **Pure Pursuit**: Follows planned path with lookahead distance
- **Trapezoidal Velocity**: Ensures smooth acceleration/deceleration
- **Adaptive Speed**: Slows down for sharp turns, speeds up on straight paths

### 5. Visualization
- Real-time 3D rendering of environment
- Robot shown with 4-wheel model
- Obstacles rendered with colors
- Path visualization (optional)

## Key Parameters

### Robot Parameters
- `wheelbase`: 0.3m (distance between front and rear axles)
- `max_velocity`: 0.5 m/s
- `max_acceleration`: 0.3 m/s²
- `body_length`: 2.4m
- `body_width`: 1.5m

### Sensor Parameters
- `fov`: 270° (field of view)
- `num_beams`: 108
- `max_range`: 6.0m

### Planning Parameters
- `resolution`: 0.1m (grid cell size)
- `inflation_radius`: 3 cells (safety margin)
- `lookahead_distance`: 0.5m (pure pursuit)
- `min_frontier_size`: 5 cells

### Exploration Parameters
- `exploration_threshold`: 85% (when to stop)
- `max_iterations`: 2000

## Performance Metrics

The system tracks:
- **Exploration coverage**: Percentage of map explored
- **Distance traveled**: Total path length
- **Time elapsed**: Simulation time
- **Average speed**: Distance/time ratio

## Future Extensions

### Planned Improvements
1. **Hybrid-A*** for better handling of non-holonomic constraints
2. **Dynamic obstacle handling** for moving objects
3. **Multi-robot coordination** for faster exploration
4. **Better path smoothing** using splines or Bezier curves

### Optional: Reinforcement Learning
- **PPO/DQN** for local obstacle avoidance
- **Stable-Baselines3** integration
- Learning-based control from sensor inputs
- Hybrid classical + RL approach

## Troubleshooting

### Import Errors
Make sure you're running from the project root and using the virtual environment:
```bash
cd /Users/aryanmathur/Desktop/RDCP/take2
source venv/bin/activate
```

### Visualization Issues
If matplotlib windows don't appear:
```bash
# On macOS, you might need:
pip install --upgrade matplotlib
```

### No Path Found
- Increase `inflation_radius` if robot is too cautious
- Decrease `inflation_radius` if no paths are found
- Adjust `min_frontier_size` for frontier detection

## References

- Ackermann Steering: Bicycle model kinematics
- A* Algorithm: Hart, Nilsson, Raphael (1968)
- Frontier Exploration: Yamauchi (1997)
- Pure Pursuit: Coulter (1992)
- Occupancy Grid Mapping: Moravec & Elfes (1985)

## License

Educational project for autonomous robotics research.

## Author

Aryan Mathur
November 2025
