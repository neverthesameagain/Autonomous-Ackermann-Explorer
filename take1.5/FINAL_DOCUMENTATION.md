# Complete Project Documentation
## Autonomous Obstacle-Aware Exploration and Path Planning System

**Author:** Aryan Mathur  
**Date:** November 10, 2025  
**Project:** Four-Wheel Ackermann Robot Autonomous Navigation

---

## 📋 Table of Contents
1. [Executive Summary](#executive-summary)
2. [System Overview](#system-overview)
3. [Implementation Details](#implementation-details)
4. [Key Features](#key-features)
5. [Algorithm Explanations](#algorithm-explanations)
6. [Usage Instructions](#usage-instructions)
7. [Visualization Guide](#visualization-guide)
8. [Performance Metrics](#performance-metrics)
9. [Future Work](#future-work)

---

## 🎯 Executive Summary

This project implements a **fully autonomous exploration and navigation system** for a four-wheel Ackermann-steered mobile robot. The robot navigates from a start position to a goal position while:

- **Exploring** unknown regions using frontier-based strategy
- **Mapping** the environment with LiDAR sensor
- **Planning** optimal collision-free paths using A*
- **Avoiding** obstacles in real-time
- **Visualizing** progress in both 2D and 3D

### Key Achievement
✅ **Complete autonomous navigation** with intelligent frontier selection, smooth motion control, and comprehensive visualization.

---

## 🏗️ System Overview

### Architecture

```
┌──────────────────────────────────────────────────────────┐
│          GOAL-DIRECTED AUTONOMOUS EXPLORER                │
└──────────────────────────────────────────────────────────┘
                          │
        ┌─────────────────┼─────────────────┐
        │                 │                 │
   ┌────▼────┐       ┌────▼────┐      ┌────▼────┐
   │ SENSING │       │ MAPPING │      │PLANNING │
   └────┬────┘       └────┬────┘      └────┬────┘
        │                 │                 │
   LiDAR Scan      Occupancy Grid    Frontier Detection
   270° FOV        Probabilistic     + A* Path Planning
   108 beams       Log-odds          + Hybrid A* Scoring
   6m range        Ray tracing       
                                          │
                                     ┌────▼────┐
                                     │ CONTROL │
                                     └────┬────┘
                                          │
                                   Pure Pursuit
                                   + Trapezoidal
                                     Velocity
                                          │
                                     ┌────▼────┐
                                     │ ROBOT   │
                                     └─────────┘
                                   Ackermann
                                   Steering
```

### Core Components

| Component | File | Purpose |
|-----------|------|---------|
| **Robot Model** | `robot/ackermann.py` | Bicycle kinematics, state management |
| **LiDAR Sensor** | `sensors/lidar.py` | Environment sensing, ray-casting |
| **Occupancy Grid** | `map/occupancy_grid.py` | Probabilistic mapping |
| **A* Planner** | `planning/astar.py` | Global path planning |
| **Frontier Explorer** | `planning/frontier_explorer.py` | Frontier detection & selection |
| **Motion Controller** | `control/velocity_controller.py` | Path following, velocity profiles |
| **Main System** | `goal_directed_explorer.py` | Integration & orchestration |

---

## 🔧 Implementation Details

### 1. Robot Kinematics (Ackermann Model)

**Bicycle Model Equations:**
```
x_{t+1} = x_t + v * cos(θ) * dt
y_{t+1} = y_t + v * sin(θ) * dt
θ_{t+1} = θ_t + (v / L) * tan(δ) * dt
```

Where:
- `(x, y)` = position in meters
- `θ` = heading angle in radians
- `v` = linear velocity (m/s)
- `δ` = steering angle (radians)
- `L` = wheelbase = 0.3m
- `dt` = time step = 0.1s

**Parameters:**
- Body dimensions: 2.4m × 1.5m
- Max velocity: 0.5 m/s
- Max acceleration: 0.3 m/s²
- Max steering: ±45°

### 2. LiDAR Sensor

**Specifications:**
- Field of View: 270° (front hemisphere)
- Number of Beams: 108
- Angular Resolution: 2.5°
- Maximum Range: 6.0m
- Update Rate: 10 Hz

**Ray-Casting Algorithm:**
```python
for each beam angle:
    for distance in [0, max_range]:
        point = robot_pos + distance * [cos(angle), sin(angle)]
        if collision_with_obstacle(point):
            return distance
    return max_range
```

### 3. Occupancy Grid Mapping

**Probabilistic Update (Log-Odds):**

```python
# For cells along ray (free space)
log_odds[cell] += l_free  # l_free = -0.4

# For endpoint cell (occupied)
log_odds[cell] += l_occ   # l_occ = 0.85

# Convert to probability
P(occupied) = 1 - 1 / (1 + exp(log_odds))
```

**Grid Properties:**
- Resolution: 0.1m (10cm cells)
- Size: 200×200 cells (20m×20m)
- States:
  - **Free**: P < 0.35 (light blue)
  - **Occupied**: P > 0.65 (dark red)
  - **Unknown**: 0.35 ≤ P ≤ 0.65 (gray)

### 4. A* Path Planning

**Algorithm:**
```
1. Initialize: open_set = {start}, closed_set = {}
2. While open_set not empty:
   a. current = node with lowest f = g + h
   b. If current == goal: reconstruct path
   c. For each neighbor:
      - g = cost from start
      - h = Euclidean distance to goal
      - f = g + h
      - Add to open_set if better
3. Return path or None
```

**Enhancements:**
- **Obstacle Inflation**: 3-cell safety margin
- **Path Smoothing**: Remove unnecessary waypoints
- **8-Connected Grid**: Diagonal movements allowed

### 5. Frontier-Based Exploration

**Frontier Definition:**
> A frontier is a boundary between **free** and **unknown** space

**Detection Algorithm:**
```python
for each free cell (x, y):
    if any 8-connected neighbor is unknown:
        mark (x, y) as frontier cell

Group frontier cells using connected components
Filter groups smaller than min_size (5 cells)
Calculate centroid of each group
```

**Selection Strategy (Hybrid A* Inspired):**

```python
score = (progress_to_goal * 5.0 + info_gain * 0.05) / (1.0 + dist_to_frontier * 0.3)
```

**Weights:**
- **Progress to goal**: 5.0 (highest priority)
- **Information gain**: 0.05 (moderate bonus)
- **Distance penalty**: 0.3 (efficiency)

This ensures the robot always moves towards the goal while exploring.

### 6. Motion Control

**Pure Pursuit Controller:**
```python
# Find lookahead point on path
lookahead_point = find_point_at_distance(path, lookahead_dist=0.5m)

# Calculate steering angle
alpha = atan2(lookahead_y - robot_y, lookahead_x - robot_x) - robot_theta
steering = alpha  # Simplified for bicycle model
```

**Trapezoidal Velocity Profile:**
```python
if |v_target - v_current| > a_max * dt:
    v_current += sign(v_target - v_current) * a_max * dt
else:
    v_current = v_target
```

**Adaptive Speed:**
- Sharp turns (|δ| > 0.5): 0.2 m/s
- Medium turns (|δ| > 0.3): 0.3 m/s
- Straight paths: 0.5 m/s

---

## ✨ Key Features

### 1. Intelligent Frontier Selection
- ✅ Evaluates ALL frontiers at each step
- ✅ Scores based on progress to goal
- ✅ Displays top 5 candidates with metrics
- ✅ Selects optimal frontier using Hybrid A* strategy

### 2. Clean Visualization
- ✅ Shows ONLY current target frontier (yellow star)
- ✅ Displays visited frontiers with numbers (green circles)
- ✅ Clears old frontiers when reached
- ✅ Dual 2D/3D views

### 3. Comprehensive Logging
- ✅ Initial frontier scan with full details
- ✅ Frontier evaluation scores
- ✅ Progress updates every 50 iterations
- ✅ Final summary with statistics

### 4. Robust Navigation
- ✅ Obstacle avoidance with safety margins
- ✅ Smooth acceleration/deceleration
- ✅ Path replanning when needed
- ✅ Goal-directed exploration

---

## 📊 Algorithm Explanations

### Bresenham's Line Algorithm

Used for efficient ray tracing in occupancy grid:

```python
def bresenham(x0, y0, x1, y1):
    """Trace line from (x0,y0) to (x1,y1) in grid."""
    cells = []
    dx, dy = abs(x1-x0), abs(y1-y0)
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

### Hybrid A* Scoring

Combines multiple factors for intelligent frontier selection:

```python
# Components
progress = current_dist_to_goal - frontier_dist_to_goal  # How much closer?
info_gain = count_unknown_cells_near_frontier(radius=2m)  # How much to learn?
dist_cost = distance_from_robot_to_frontier  # How far to travel?

# Weighted combination
score = (progress * 5.0 + info_gain * 0.05) / (1.0 + dist_cost * 0.3)
```

**Rationale:**
- Heavily favor progress towards goal (5.0×)
- Moderate reward for information (0.05×)
- Penalize distant frontiers (0.3×)

---

## 🚀 Usage Instructions

### Basic Usage

```bash
# 1. Activate virtual environment
cd /Users/aryanmathur/Desktop/RDCP/take2
source venv/bin/activate

# 2. Run goal-directed exploration
python3 -m src.goal_directed_explorer
```

### Custom Start/Goal

Edit `src/goal_directed_explorer.py`:

```python
def main():
    START = (3.0, 5.0)   # Your start position
    GOAL = (17.0, 15.0)  # Your goal position
    
    explorer = GoalDirectedExplorer(
        start_pos=START,
        goal_pos=GOAL,
        map_size=(20, 20),
        resolution=0.1
    )
    
    explorer.run_exploration(max_iterations=3000)
```

### Adjust Parameters

```python
# Faster exploration
velocity_controller = TrapezoidalVelocityController(
    max_velocity=0.8,        # Increase speed
    max_acceleration=0.5
)

# More cautious navigation
self.current_path = self.path_planner.plan(
    ...,
    inflation_radius=5  # Larger safety margin
)

# Finer mapping
explorer = GoalDirectedExplorer(
    ...,
    resolution=0.05  # 5cm cells instead of 10cm
)
```

---

## 📺 Visualization Guide

### 2D Map (Left Panel)

**Color Coding:**
- 🔵 **Light Blue**: Explored free space
- 🔴 **Dark Red**: Detected obstacles
- ⚪ **Gray**: Unknown/unexplored areas
- 🔴 **Red Dashed**: Ground truth obstacles

**Markers:**
- 🟢 **Green Circle**: START position
- ❌ **Red X**: GOAL position
- 🔵 **Blue Circle**: Current robot position
- 🟢 **Green Numbered Circles**: Visited frontiers (1, 2, 3, ...)
- ⭐ **Yellow Star**: Current target frontier
- 🔵 **Blue Line**: Robot's traveled path
- 🟢 **Green Dashed**: Planned path to target

### 3D View (Right Panel)

- **Obstacles**: Rendered as 3D shapes (boxes, cylinders, cones, hemispheres)
- **Robot**: Blue sphere
- **Trajectory**: Blue line
- **Start/Goal**: Green/Red markers

### Console Output

```
🔍 INITIAL ENVIRONMENT SCAN
----------------------------------------------------------------------
📍 INITIAL FRONTIERS DETECTED: 63

🔍 EVALUATING 63 FRONTIERS:
----------------------------------------------------------------------
Top 5 Frontier Candidates:
👉 1. (4.85, 4.05) - Score: 12.45
      Progress to goal: 3.21m | Dist to frontier: 2.91m
      Dist to goal: 19.17m | Info gain: 831 cells
   2. (5.15, 4.25) - Score: 11.98
      ...

🎯 SELECTED BEST FRONTIER: (4.85, 4.05)
   Strategy: Moving towards goal while exploring
----------------------------------------------------------------------

[Iteration 1] 🎯 Targeting F36 at (4.85, 4.05)
  ✓ Path planned with 15 waypoints
  📏 Path length: 3.12m

======================================================================
✅ FRONTIER #1 REACHED!
   Position: (4.85, 4.05)
   Total distance traveled: 3.15m
======================================================================

🔍 SCANNING FROM NEW POSITION...
📍 NEW FRONTIERS DETECTED: 47
...
```

---

## 📈 Performance Metrics

### Tracked Metrics

| Metric | Description |
|--------|-------------|
| **Goal Reached** | YES/NO status |
| **Iterations** | Total simulation steps |
| **Frontiers Visited** | Number of frontiers explored |
| **Distance Traveled** | Total path length (meters) |
| **Time Elapsed** | Simulation time (seconds) |
| **Average Speed** | Distance / Time (m/s) |
| **Map Explored** | Percentage of environment mapped |

### Example Output

```
======================================================================
EXPLORATION SUMMARY
======================================================================
Goal Reached: YES ✓
Total iterations: 847
Frontiers visited: 5
Total distance: 22.47m
Total time: 84.7s
Average speed: 0.27m/s
Map explored: 78.3%
Final position: (17.98, 17.92)

📍 FRONTIER EXPLORATION PATH:
----------------------------------------------------------------------
  1. Frontier at (4.85, 4.05)
  2. Frontier at (8.25, 7.15)
  3. Frontier at (12.45, 11.35)
  4. Frontier at (15.65, 14.25)
  5. Frontier at (17.25, 16.85)
======================================================================
```

---

## 🔮 Future Work

### Immediate Improvements

1. **Hybrid-A* Implementation**
   - Full non-holonomic planning
   - Consider turning radius constraints
   - Generate kinematically feasible paths

2. **Dynamic Obstacles**
   - Moving obstacle detection
   - Velocity estimation
   - Predictive avoidance

3. **Multi-Resolution Planning**
   - Coarse global planning
   - Fine local planning
   - Hierarchical approach

### Advanced Extensions

4. **Reinforcement Learning**
   ```python
   from stable_baselines3 import PPO
   
   # Train RL agent for local control
   model = PPO("MlpPolicy", env, verbose=1)
   model.learn(total_timesteps=100000)
   
   # Hybrid: Classical planning + RL control
   global_path = astar_planner.plan(start, goal)
   local_action = rl_model.predict(lidar_scan)
   ```

5. **Multi-Robot Coordination**
   - Distributed frontier allocation
   - Inter-robot communication
   - Shared occupancy map

6. **Semantic Mapping**
   - Object recognition
   - Scene understanding
   - Task-oriented exploration

---

## 📚 References

### Academic Papers

1. **Yamauchi, B. (1997).** "A Frontier-Based Approach for Autonomous Exploration"  
   *IEEE International Conference on Robotics and Automation*

2. **Hart, P., Nilsson, N., Raphael, B. (1968).** "A Formal Basis for the Heuristic Determination of Minimum Cost Paths"  
   *IEEE Transactions on Systems Science and Cybernetics*

3. **Moravec, H., Elfes, A. (1985).** "High Resolution Maps from Wide Angle Sonar"  
   *IEEE International Conference on Robotics and Automation*

4. **Coulter, R. C. (1992).** "Implementation of the Pure Pursuit Path Tracking Algorithm"  
   *Carnegie Mellon University Robotics Institute Technical Report*

5. **Dolgov, D., et al. (2010).** "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments"  
   *International Journal of Robotics Research*

### Implementation

- **NumPy**: Numerical computing
- **Matplotlib**: Visualization
- **SciPy**: Scientific algorithms
- **Python 3.13**: Programming language

---

## 🎓 Learning Outcomes

This project demonstrates:

✅ **Robotics**: Mobile robot kinematics, sensor integration  
✅ **AI/Planning**: Search algorithms, heuristic design  
✅ **Computer Vision**: Occupancy mapping, ray tracing  
✅ **Control Systems**: Feedback control, trajectory following  
✅ **Software Engineering**: Modular design, documentation  
✅ **Visualization**: Real-time 2D/3D graphics  

---

## 📝 Conclusion

This project successfully implements a **complete autonomous exploration and navigation system** for a four-wheel Ackermann robot. The system:

- ✅ Navigates from start to goal autonomously
- ✅ Explores unknown regions intelligently
- ✅ Maps the environment probabilistically
- ✅ Plans optimal collision-free paths
- ✅ Visualizes progress comprehensively

The implementation combines **classical robotics algorithms** (A*, Pure Pursuit, Occupancy Grids) with **intelligent exploration strategies** (Frontier-based, Hybrid A* scoring) to achieve robust autonomous navigation.

**Status: FULLY FUNCTIONAL AND DOCUMENTED** ✅

---

**Author:** Aryan Mathur  
**Institution:** [Your Institution]  
**Course:** Autonomous Robotics  
**Date:** November 10, 2025

---

*For questions or improvements, please refer to the troubleshooting section in COMPLETE_DOCUMENTATION.md*
