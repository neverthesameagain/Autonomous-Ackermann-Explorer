Autonomous Exploration & Path Planning System

### *Ackermann Robot · Frontier Exploration · A* · Pure Pursuit · Real-Time Mapping*


Submission by Aryan Mathur -122201017

This project implements a **fully autonomous exploration & navigation stack** for an Ackermann-steered robot.

The robot  **builds an occupancy map** , detects  **frontiers** , **plans paths** using an enhanced A* planner, and **tracks** them using Pure Pursuit with realistic wheel dynamics.

---

## Features

### **1. Occupancy Grid Mapping**

* 2D probability-based grid (free / occupied / unknown)
* Real-time updates from simulated LiDAR
* Consistent world ↔ grid conversions

### **2. Frontier-Based Exploration**

* Detects “frontiers” → boundary between known & unknown space
* Clusters frontiers and computes centroids
* Filters invalid and small clusters
* Selects best frontier using:
  * Information Gain
  * Distance to robot
  * Goal proximity

### **3. A* Global Path Planning**

* 8-connected A* search
* Obstacle inflation for safety margins
* Automatic fallback if start/goal is blocked
* Path post-processing:
  * Line-of-sight smoothing
  * Densification for Ackermann motion

### **4. Path Tracking (Pure Pursuit + Velocity Controller)**

* Pure Pursuit steering law
* Adaptive lookahead distance
* Trapezoidal velocity profile for smooth acceleration
* Realistic Ackermann kinematics
* Per-wheel velocity computation (FL, FR, RL, RR)
* Collision handling + reverse escape

### **5. Visualization**

* 2D live occupancy grid with:
  * Trajectory
  * Frontiers
  * Goal markers
  * Inflated obstacle zones
* 3D rendering environment (follow-camera view)
* Velocity + wheel dynamics plot panels

---

## How the System Works

### **1. Sense → Map**

* LiDAR scans obstacles
* Occupancy grid updates in real time

### **2. Explore**

* If goal is unknown → explore frontiers
* Compute information gain
* Select best frontier

### **3. Plan**

* Run A* from robot → frontier (or final goal)
* Smooth + densify path

### **4. Control**

* Compute steering via Pure Pursuit
* Smooth speed via trapezoidal velocity controller
* Simulate Ackermann movement
* Log wheel velocities

### **5. React**

* Detect path blockage
* Replan if obstacles appear
* Escape if stuck or colliding

---

## ▶ Running the Explorer

```bash
python run .py
```

Modify initial settings:

```python
explorer = GoalDirectedExplorer(
    start_pos=None,
    goal_pos=None,
    map_size=(15, 15),
    resolution=0.1
)
```

---

## Visualization Output

The system displays:

* **2D Exploration Map**
* **Frontiers & Selected Frontier**
* **Robot pose (with steering wheels)**
* **Explored trajectory**
* **Inflated obstacles**
* **Velocity plots (v, ω)**
* **Wheel speed plots (FL, FR, RL, RR)**
* **3D follow-camera environment**

---

## Key Algorithms

### A* Planning

* f = g + h (Euclidean distance heuristic)
* 8-connected neighborhood
* Dynamic obstacle inflation
* Path shortcut smoothing
* Dense waypoint generation for smooth tracking

### Frontier Detection

Frontier = free cell adjacent to unknown cell.

Clusters frontiers with `scipy.ndimage.label()` and uses centroid for navigation.

### Pure Pursuit Tracking

* Steering angle:

  \delta = \tan^{-1}\left(\frac{2L \sin\alpha}{d_{\text{lookahead}}}\right)
* Naturally curvature-bound
* Ensures stable convergence to path

## Final Goal

The robot continues exploring until:

* The final goal is visible in known space
* A collision-free A* path is found
* Robot navigates precisely to target
* Exploration statistics are printed (distance, map coverage, time)

---

## Requirements

* Python 3.9+
* numpy
* scipy
* matplotlib

Install:

```bash
pip install numpy scipy matplotlib
```

---
