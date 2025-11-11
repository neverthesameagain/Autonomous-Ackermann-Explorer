# Autonomous Ackermann Explorer

An advanced autonomous exploration system for Ackermann-steered robots, featuring:

- Real-time 2D/3D visualization
- Hybrid A* path planning
- Frontier-based exploration
- Dynamic obstacle avoidance
- Per-wheel dynamics simulation
- Pure Pursuit and Trapezoidal velocity control

## Quick Start

1. **Install Dependencies**:

   ```bash
   pip install -r requirements.txt
   ```
2. **Run the Explorer**:

   ```bash
   python run.py
   ```

## 🧩 Project Structure

```
src/
├── control/               # Motion control algorithms
│   └── velocity_controller.py  # Pure Pursuit and Trapezoidal velocity controllers
├── map/                   # Mapping components
│   └── occupancy_grid.py  # Grid-based environment representation
├── objects/               # Environment objects
│   └── generator.py       # Procedural environment generation
├── planning/              # Path planning algorithms
│   ├── astar.py           # A* path planning
│   └── frontier_explorer.py # Frontier detection and selection
├── robot/                 # Robot models
│   └── ackermann.py       # Ackermann steering kinematics
├── sensors/               # Sensor models
│   └── lidar.py           # LiDAR sensor simulation
└── goal_directed_explorer.py  # Main application
```

## Key Features

### 1. Hybrid A* Path Planning

- Combines discrete graph search with continuous state-space sampling
- Dynamic obstacle inflation for safety margins
- Path smoothing and optimization

### 2. Frontier Exploration

Frontier exploration is the process of identifying and navigating to the boundaries between explored and unexplored areas. Our implementation includes:

#### Key Components:

- **Frontier Detection**

  - Uses edge detection on the occupancy grid to find boundaries
  - Groups adjacent frontier cells into regions
  - Filters out small or unreachable frontiers
- **Information Gain Calculation**

  - Estimates potential new information from each frontier
  - Considers visible area and potential new paths
  - Prioritizes frontiers that lead towards unexplored regions
- **Dynamic Re-planning**

  - Continuously updates frontier information as the map changes
  - Re-evaluates frontier selection when new obstacles are detected
  - Handles dynamic environments with moving obstacles

#### Frontier Selection Strategy:

1. **Scoring System**

   - Distance to robot (closer frontiers preferred)
   - Distance to goal (directs exploration towards the goal)
   - Information gain (prioritizes high-yield frontiers)
   - Path quality (considers path length and safety)
2. **Adaptive Behavior**

   - Balances exploration and exploitation
   - Adjusts strategy based on remaining battery/time
   - Handles dead-ends and local minima

### 3. Motion Control System

The motion control system combines multiple controllers to achieve smooth and precise robot movement:

#### 3.1 Pure Pursuit Controller

```python
controller = PurePursuitController(
    lookahead=1.0,      # Lookahead distance (m)
    Kp=1.5,            # Steering gain
    v_ref=0.5,         # Reference velocity (m/s)
    dist_threshold=0.5  # Waypoint switching threshold
)
```

- **Path Following**: Uses a lookahead point to generate smooth trajectories
- **Adaptive Lookahead**: Adjusts based on path curvature and speed
- **Target Selection**: Dynamically selects the most appropriate target point
- **Obstacle Avoidance**: Reduces speed near obstacles and sharp turns

#### 3.2 Trapezoidal Velocity Profile

```python
velocity_controller = TrapezoidalVelocityController(
    max_velocity=0.5,        # m/s
    max_acceleration=0.3,    # m/s²
    max_angular_velocity=0.8 # rad/s
)
```

- **Smooth Transitions**: Ensures jerk-limited motion
- **Velocity Ramping**: Prevents wheel slip and ensures stability
- **Dynamic Adjustment**: Modifies profile based on path curvature

#### 3.3 Ackermann Kinematics

- **Steering Geometry**: Implements correct Ackermann steering angles
- **Wheel Speed Control**: Computes individual wheel speeds
- **Dynamic Constraints**: Enforces mechanical limits

#### 3.4 Integrated Control Flow

1. **Path Following**

   - Pure Pursuit generates steering commands
   - Velocity profile determines safe speeds
   - Commands are sent to the Ackermann controller
2. **Obstacle Response**

   - Reduces speed when approaching obstacles
   - Adjusts path to avoid collisions
   - Recovers from potential dead-ends
3. **Performance Optimization**

   - Minimizes path tracking error
   - Reduces energy consumption
   - Ensures passenger comfort

### 4. Visualization

- Real-time 2D map with exploration progress
- 3D environment rendering
- Dynamic path visualization
- Performance metrics display

## 🛠 Configuration

Modify these parameters in `goal_directed_explorer.py`:

```python
explorer = GoalDirectedExplorer(
    start_pos=None,      # Auto-generated if None
    goal_pos=None,       # Auto-generated if None
    map_size=(10, 10),   # Environment size in meters
    resolution=0.1       # Grid resolution in meters
)
```

## Performance Metrics

- **Exploration Rate**: Percentage of environment mapped
- **Path Length**: Total distance traveled
- **Computation Time**: Per-iteration processing time
- **Success Rate**: Goal-reaching reliability
