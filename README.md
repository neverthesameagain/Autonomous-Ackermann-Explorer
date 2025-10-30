# Autonomous Ackermann Explorer

A Python-based autonomous exploration system for an Ackermann-steered robot, featuring frontier-based exploration, dynamic path planning, and 3D visualization.

## Features

- **Ackermann Vehicle Simulation**
  - Realistic vehicle dynamics and constraints
  - Smooth motion control with velocity and steering smoothing
  - Configurable vehicle parameters (velocity, acceleration, steering limits)

- **Intelligent Exploration**
  - Frontier-based exploration algorithm
  - Smart goal selection with multi-criteria scoring
  - Obstacle avoidance and enclosure detection
  - Dynamic path planning with A* algorithm

- **Advanced Visualization**
  - Real-time 2D top-down view
  - Dynamic 3D visualization with following camera
  - Obstacle and path rendering
  - Exploration progress tracking

- **Robust Control System**
  - Pure Pursuit controller for path following
  - Smooth velocity and steering transitions
  - Dynamic obstacle avoidance
  - Automatic path replanning

## Project Structure

```
├── main.py                 # Main simulation entry point
├── ackermann_vehicle.py    # Vehicle dynamics and motion model
├── exploration.py          # Frontier-based exploration logic
├── controller.py           # Pure Pursuit path following controller
├── planner.py             # A* path planning implementation
├── visualize.py           # 2D visualization components
├── visualize3d.py         # 3D visualization with following camera
├── utils.py               # Utility functions
└── metrics_logger.py      # Exploration metrics logging
```

## Current Progress

- [x] Implemented Ackermann vehicle dynamics with smoothing
- [x] Developed frontier-based exploration algorithm
- [x] Added intelligent goal selection with scoring system
- [x] Created 3D visualization with following camera
- [x] Implemented obstacle enclosure detection
- [x] Added path planning and replanning capabilities
- [x] Integrated metrics logging and visualization

## Running the Simulation

1. **Setup:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   pip install numpy matplotlib scipy
   ```

2. **Run:**
   ```bash
   python3 main.py
   ```

## Key Parameters

- **Vehicle Parameters**
  - Maximum velocity: 4.0 m/s
  - Maximum acceleration: 2.0 m/s²
  - Steering smoothing: 0.7-0.8
  - Velocity smoothing: 0.7-0.8

- **Exploration Parameters**
  - Sensor range: 5 units
  - Minimum frontier size: 3 units
  - Goal timeout: 200 iterations
  - Coverage threshold: 60%

- **Visualization Settings**
  - Camera follow distance: 5 units
  - Camera height: 3 units
  - Look-down angle: 25 degrees

## Recent Improvements

1. **Exploration Logic**
   - Enhanced frontier selection algorithm
   - Added obstacle enclosure detection
   - Improved goal selection criteria

2. **Motion Control**
   - Added motion smoothing parameters
   - Improved obstacle avoidance
   - Enhanced path following

3. **Visualization**
   - Added 3D view with following camera
   - Improved obstacle rendering
   - Real-time exploration progress display

## Future Work

1. **Exploration**
   - Implement dynamic sensor range adjustment
   - Add multi-robot coordination support
   - Enhance frontier clustering

2. **Control**
   - Add adaptive velocity control
   - Implement dynamic motion constraints
   - Enhance obstacle avoidance behavior

3. **Visualization**
   - Add interactive camera controls
   - Implement real-time metrics display
   - Enhance 3D environment rendering