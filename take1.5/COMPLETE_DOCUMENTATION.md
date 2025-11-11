# Complete Documentation: Autonomous Obstacle-Aware Exploration System

## 📋 Table of Contents
1. [Project Overview](#project-overview)
2. [System Architecture](#system-architecture)
3. [Installation & Setup](#installation--setup)
4. [Usage Guide](#usage-guide)
5. [Component Details](#component-details)
6. [Visualization Guide](#visualization-guide)
7. [Algorithm Explanations](#algorithm-explanations)
8. [Performance Tuning](#performance-tuning)
9. [Troubleshooting](#troubleshooting)
10. [Future Extensions](#future-extensions)

---

## 🎯 Project Overview

### Goal
Design and simulate a **four-wheel Ackermann-steered mobile robot** capable of autonomously navigating from a start position to a goal position while:
- **Exploring** unknown regions of the environment
- **Mapping** obstacles using LiDAR sensor
- **Planning** collision-free paths
- **Avoiding** obstacles in real-time
- **Visualizing** the entire process in 2D and 3D

### Key Features
✅ **Ackermann Steering Model** - Realistic car-like kinematics  
✅ **Trapezoidal Velocity Profiles** - Smooth acceleration/deceleration  
✅ **A* Path Planning** - Optimal collision-free paths  
✅ **Frontier-Based Exploration** - Intelligent discovery of unknown areas  
✅ **Probabilistic Occupancy Mapping** - Robust environment representation  
✅ **Dual Visualization** - Simultaneous 2D map and 3D environment views  
✅ **Real-Time Updates** - Live frontier detection and path replanning  

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   AUTONOMOUS EXPLORER                        │
└─────────────────────────────────────────────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
   ┌────▼────┐         ┌────▼────┐        ┌────▼────┐
   │ SENSING │         │ MAPPING │        │PLANNING │
   └────┬────┘         └────┬────┘        └────┬────┘
        │                   │                   │
   ┌────▼────────┐    ┌─────▼──────┐     ┌─────▼──────┐
   │ LiDAR       │───▶│ Occupancy  │────▶│ A* Planner │
   │ - 270° FOV  │    │ Grid       │     │ - Inflation│
   │ - 108 beams │    │ - Log-odds │     │ - Smoothing│
   │ - 6m range  │    │ - Ray trace│     └─────┬──────┘
   └─────────────┘    └────────────┘           │
                                          ┌─────▼──────┐
                                          │ Frontier   │
                                          │ Explorer   │
                                          │ - Detect   │
                                          │ - Select   │
                                          └─────┬──────┘
                                                │
        ┌───────────────────────────────────────┘
        │
   ┌────▼────┐
   │ CONTROL │
   └────┬────┘
        │
   ┌────▼────────────┐
   │ Path Following  │
   │ - Pure Pursuit  │
   │ - Trapezoidal V │
   └────┬────────────┘
        │
   ┌────▼────────┐
   │ Ackermann   │
   │ Robot       │
   │ - Bicycle   │
   │   Model     │
   └─────────────┘
```

---

## 💻 Installation & Setup

### Prerequisites
- Python 3.8 or higher
- macOS, Linux, or Windows

### Step 1: Clone/Navigate to Project
```bash
cd /Users/aryanmathur/Desktop/RDCP/take2
```

### Step 2: Create Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

### Step 3: Install Dependencies
```bash
pip install -r requirements.txt
```

**Required packages:**
- `numpy>=1.20.0` - Numerical computations
- `matplotlib>=3.3.0` - Visualization
- `scipy>=1.7.0` - Scientific computing (frontier detection)

### Step 4: Verify Installation
```bash
python3 -c "import numpy, matplotlib, scipy; print('✓ All dependencies installed')"
```

---

## 🚀 Usage Guide

### Quick Start: Goal-Directed Exploration

```bash
# Activate virtual environment
source venv/bin/activate

# Run goal-directed explorer
python3 -m src.goal_directed_explorer
```

This will:
1. Start robot at position **(2, 2)**
2. Set goal at position **(18, 18)**
3. Begin autonomous exploration
4. Display dual 2D/3D visualization

### Custom Start and Goal Positions

Edit `src/goal_directed_explorer.py`:

```python
def main():
    # Define your custom positions
    START = (3.0, 5.0)   # (x, y) in meters
    GOAL = (15.0, 12.0)  # (x, y) in meters
    
    explorer = GoalDirectedExplorer(
        start_pos=START,
        goal_pos=GOAL,
        map_size=(20, 20),
        resolution=0.1
    )
    
    explorer.run_exploration(max_iterations=3000)
```

### Alternative: Pure Exploration (No Goal)

```bash
python3 -m src.autonomous_explorer
```

This explores the entire environment without a specific goal.

---

## 🔧 Component Details

### 1. Robot Model (`src/robot/ackermann.py`)

**Ackermann Steering Kinematics:**
```
x' = v * cos(θ) * dt
y' = v * sin(θ) * dt
θ' = (v / L) * tan(δ) * dt
```

Where:
- `(x, y)` = position
- `θ` = heading angle
- `v` = linear velocity
- `δ` = steering angle
- `L` = wheelbase (0.3m)
- `dt` = time step (0.1s)

**Parameters:**
- Body: 2.4m × 1.5m
- Wheelbase: 0.3m
- Max velocity: 0.5 m/s
- Max acceleration: 0.3 m/s²

### 2. LiDAR Sensor (`src/sensors/lidar.py`)

**Specifications:**
- Field of View: 270° (front hemisphere)
- Number of Beams: 108
- Max Range: 6.0m
- Angular Resolution: 2.5°

**Operation:**
- Ray-casting from robot position
- Collision detection with obstacles
- Returns range array for each beam

### 3. Occupancy Grid (`src/map/occupancy_grid.py`)

**Probabilistic Mapping:**
- Resolution: 0.1m (10cm cells)
- Representation: Log-odds
- States:
  - **Free**: P < 0.35 (light blue)
  - **Occupied**: P > 0.65 (dark red)
  - **Unknown**: 0.35 ≤ P ≤ 0.65 (gray)

**Update Algorithm:**
1. Ray trace from robot to LiDAR endpoint
2. Mark cells along ray as FREE
3. Mark endpoint as OCCUPIED
4. Convert log-odds to probabilities

### 4. A* Path Planner (`src/planning/astar.py`)

**Features:**
- 8-connected grid search
- Euclidean distance heuristic
- Obstacle inflation (safety margin)
- Path smoothing

**Algorithm:**
```python
1. Initialize: open_set = {start}, closed_set = {}
2. While open_set not empty:
   a. current = node with lowest f-score
   b. If current == goal: return path
   c. For each neighbor of current:
      - Calculate g = cost from start
      - Calculate h = heuristic to goal
      - Calculate f = g + h
      - Add to open_set if better path
3. Return None (no path found)
```

### 5. Frontier Explorer (`src/planning/frontier_explorer.py`)

**Frontier Detection:**
- Frontier = boundary between FREE and UNKNOWN space
- Minimum size: 5 cells
- Grouped into regions using connected components

**Selection Strategy:**
```python
score = (progress_to_goal * 2.0 + info_gain * 0.1) / (1.0 + distance * 0.5)
```

Balances:
- Progress towards goal
- Information gain
- Distance to frontier

### 6. Motion Control (`src/control/velocity_controller.py`)

**Trapezoidal Velocity Profile:**
```
v(t) = min(v_max, v_current + a_max * dt)
```

**Pure Pursuit Controller:**
- Lookahead distance: 0.5m
- Adaptive speed based on curvature:
  - Sharp turns (δ > 0.5): 0.2 m/s
  - Medium turns (δ > 0.3): 0.3 m/s
  - Straight paths: 0.5 m/s

---

## 📊 Visualization Guide

### 2D Map View (Left Panel)

**Color Coding:**
- 🔵 **Light Blue** = Explored free space
- ⚫ **Dark Red** = Detected obstacles
- ⚪ **Gray** = Unknown/unexplored areas
- 🔴 **Red dashed lines** = Ground truth obstacles

**Markers:**
- 🟢 **Green Circle** = START position
- 🔴 **Red X** = GOAL position
- 🔵 **Blue Circle** = Current robot position
- ⭐ **Orange Stars** = Detected frontiers (labeled F1, F2, ...)
- ⭐ **Yellow Star (red border)** = Current target frontier
- 🔵 **Blue Line** = Robot's traveled path
- 🟢 **Green Dashed Line** = Planned path to target

**Legend:**
- All elements are labeled in the legend
- Frontier count shown: "Frontiers (N)"

### 3D Environment View (Right Panel)

**Elements:**
- **Obstacles**: Rendered as 3D shapes
  - Boxes: Rectangular prisms
  - Cylinders: Vertical cylinders
  - Cones: Conical shapes
  - Hemispheres: Half-spheres
- **Robot**: Blue sphere at current position
- **Trajectory**: Blue line showing path traveled
- **Start/Goal**: Green and red markers

---

## 🧮 Algorithm Explanations

### Bresenham's Line Algorithm (Ray Tracing)

Used for efficient ray tracing in occupancy grid:

```python
def bresenham(x0, y0, x1, y1):
    cells = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    
    while True:
        cells.append((x, y))
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    
    return cells
```

**Complexity:** O(max(dx, dy))

### Log-Odds Occupancy Update

Probabilistic update for robust mapping:

```python
# For free cells
log_odds += l_free  # l_free = -0.4

# For occupied cells
log_odds += l_occ   # l_occ = 0.85

# Convert to probability
P = 1 - 1 / (1 + exp(log_odds))
```

**Advantages:**
- Handles sensor noise
- Accumulates evidence over time
- Prevents saturation

### Frontier Detection Algorithm

```python
1. For each free cell (x, y):
   a. Check 8-connected neighbors
   b. If any neighbor is unknown:
      - Mark (x, y) as frontier cell
2. Group frontier cells using connected components
3. Filter groups smaller than min_size
4. Calculate centroid of each group
5. Return frontier centroids
```

---

## ⚙️ Performance Tuning

### Adjust Exploration Speed

```python
# In src/goal_directed_explorer.py
velocity_controller = TrapezoidalVelocityController(
    max_velocity=0.8,        # Increase for faster exploration
    max_acceleration=0.5,    # Increase for quicker response
    max_angular_velocity=1.0
)
```

### Change Grid Resolution

```python
# Finer resolution (slower but more accurate)
explorer = GoalDirectedExplorer(
    start_pos=START,
    goal_pos=GOAL,
    resolution=0.05  # 5cm cells instead of 10cm
)

# Coarser resolution (faster but less accurate)
resolution=0.2  # 20cm cells
```

### Adjust Safety Margins

```python
# In path planning calls
self.current_path = self.path_planner.plan(
    self.robot.x, self.robot.y,
    goal_x, goal_y,
    inflation_radius=5  # Increase for more caution (default: 3)
)
```

### Modify Frontier Selection

```python
# In src/planning/frontier_explorer.py
self.min_frontier_size = 10  # Larger frontiers only (default: 5)
```

---

## 🐛 Troubleshooting

### Issue: No Path Found

**Symptoms:** Console shows "No path found"

**Solutions:**
1. Reduce `inflation_radius` (robot being too cautious)
2. Increase grid resolution
3. Check if goal is in occupied space

```python
# Check goal validity
gx, gy = occupancy_grid.world_to_grid(goal_x, goal_y)
print(f"Goal occupied: {occupancy_grid.is_occupied(gx, gy)}")
```

### Issue: Robot Gets Stuck

**Symptoms:** Robot stops moving, no frontiers found

**Solutions:**
1. Reduce `min_frontier_size`
2. Increase LiDAR range
3. Check for local minima

```python
# Increase LiDAR range
self.lidar = LidarSensor(
    fov=270,
    num_beams=108,
    max_range=8.0  # Increased from 6.0
)
```

### Issue: Visualization Not Updating

**Symptoms:** Windows freeze or don't show

**Solutions:**
1. Check matplotlib backend:
```python
import matplotlib
print(matplotlib.get_backend())
# Should be: 'MacOSX', 'TkAgg', or 'Qt5Agg'
```

2. Force backend:
```python
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
```

### Issue: Import Errors

**Symptoms:** `ModuleNotFoundError`

**Solutions:**
```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Reinstall dependencies
pip install -r requirements.txt

# Run from project root
cd /Users/aryanmathur/Desktop/RDCP/take2
python3 -m src.goal_directed_explorer
```

---

## 🚀 Future Extensions

### 1. Hybrid-A* Planner

Replace A* with Hybrid-A* for better non-holonomic planning:

```python
# Considers robot kinematics in planning
# Generates smoother, more feasible paths
# Accounts for minimum turning radius
```

**Implementation:** `src/planning/hybrid_astar.py`

### 2. Dynamic Obstacle Handling

Add moving obstacle detection and avoidance:

```python
# Track obstacle velocities
# Predict future positions
# Replan paths dynamically
```

### 3. Multi-Robot Coordination

Coordinate multiple robots for faster exploration:

```python
# Distributed frontier allocation
# Collision avoidance between robots
# Shared occupancy map
```

### 4. Reinforcement Learning Integration

**Optional Extension:** Add RL-based local control

```python
# Use PPO or DQN from Stable-Baselines3
# Learn obstacle avoidance from experience
# Hybrid classical + learning approach
```

**Setup:**
```bash
pip install stable-baselines3
```

**Implementation:**
```python
from stable_baselines3 import PPO

# Train agent
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)

# Use for local control
action = model.predict(observation)
```

---

## 📈 Performance Metrics

The system tracks:

| Metric | Description |
|--------|-------------|
| **Exploration Coverage** | Percentage of map explored |
| **Distance Traveled** | Total path length (meters) |
| **Time Elapsed** | Simulation time (seconds) |
| **Average Speed** | Distance / Time |
| **Frontiers Visited** | Number of frontiers explored |
| **Path Efficiency** | Ratio of straight-line to actual distance |

**Example Output:**
```
======================================================================
EXPLORATION SUMMARY
======================================================================
Goal Reached: YES ✓
Total iterations: 1247
Total distance: 24.53m
Total time: 124.7s
Map explored: 87.3%
Final position: (17.98, 17.92)
======================================================================
```

---

## 📚 References

### Academic Papers
1. **Ackermann Steering:** Bicycle model kinematics for car-like robots
2. **A* Algorithm:** Hart, Nilsson, Raphael (1968) - "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"
3. **Frontier Exploration:** Yamauchi (1997) - "A Frontier-Based Approach for Autonomous Exploration"
4. **Pure Pursuit:** Coulter (1992) - "Implementation of the Pure Pursuit Path Tracking Algorithm"
5. **Occupancy Grid Mapping:** Moravec & Elfes (1985) - "High Resolution Maps from Wide Angle Sonar"

### Code Structure
```
take2/
├── src/
│   ├── robot/
│   │   └── ackermann.py           # Robot kinematics
│   ├── sensors/
│   │   └── lidar.py                # LiDAR simulation
│   ├── map/
│   │   ├── occupancy_grid.py       # Probabilistic mapping
│   │   └── occupancy_map.py        # Legacy interface
│   ├── planning/
│   │   ├── astar.py                # A* path planner
│   │   └── frontier_explorer.py    # Frontier detection
│   ├── control/
│   │   └── velocity_controller.py  # Motion control
│   ├── objects/
│   │   ├── primitives.py           # 3D shapes
│   │   └── generator.py            # Environment generation
│   ├── env/
│   │   ├── env_3d.py               # 3D visualization
│   │   ├── env_2d_map.py           # 2D map viz
│   │   └── demo_env.py             # Simple demo
│   ├── autonomous_explorer.py      # Pure exploration
│   └── goal_directed_explorer.py   # Goal-directed exploration
├── requirements.txt
├── README.md
├── BUGFIXES.md
└── COMPLETE_DOCUMENTATION.md
```

---

## 🎓 Educational Value

This project demonstrates:
- **Robotics:** Mobile robot kinematics and control
- **AI/Planning:** Search algorithms, heuristics
- **Computer Vision:** Sensor processing, mapping
- **Software Engineering:** Modular design, documentation
- **Visualization:** Real-time 2D/3D graphics

---

## 📝 License & Author

**Author:** Aryan Mathur  
**Date:** November 2025  
**Purpose:** Educational project for autonomous robotics research  
**License:** MIT (for educational use)

---

## 🙏 Acknowledgments

- Matplotlib for visualization
- NumPy for numerical computing
- SciPy for scientific algorithms
- Python community for excellent documentation

---

**For questions or issues, please refer to the troubleshooting section or create an issue in the project repository.**

**Happy Exploring! 🤖🗺️**
