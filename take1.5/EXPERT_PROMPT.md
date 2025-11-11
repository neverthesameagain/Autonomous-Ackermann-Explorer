# Expert LLM Prompt: Autonomous Robot Navigation System

## Context

I have implemented an **autonomous exploration and navigation system** for a four-wheel Ackermann-steered mobile robot. The robot needs to navigate from a **start position** to a **goal position** while exploring and mapping an unknown environment filled with obstacles.

## System Overview

### Components
1. **Robot Model** (`robot/ackermann.py`): Bicycle kinematics with Ackermann steering
2. **LiDAR Sensor** (`sensors/lidar.py`): 270° FOV, 108 beams, 6m range
3. **Occupancy Grid** (`map/occupancy_grid.py`): Probabilistic mapping with log-odds
4. **A* Path Planner** (`planning/astar.py`): Global path planning with obstacle inflation
5. **Frontier Explorer** (`planning/frontier_explorer.py`): Detects boundaries between free and unknown space
6. **Motion Controller** (`control/velocity_controller.py`): Pure pursuit + trapezoidal velocity profiles
7. **Main System** (`goal_directed_explorer.py`): Orchestrates everything

### Current Algorithm Flow

```
1. START at position (2, 2)
2. SCAN environment with LiDAR
3. UPDATE occupancy grid (free/occupied/unknown cells)
4. DETECT frontiers (boundaries between free and unknown space)
5. IF direct path to goal exists:
      Go directly to goal
   ELSE:
      SELECT frontier closest to goal
      PLAN path to frontier using A*
      NAVIGATE to frontier
6. When frontier reached:
      CLEAR old frontiers
      RE-SCAN from new position
      DETECT new frontiers
      REPEAT from step 5
7. Continue until GOAL reached
```

## Current Problem

**The robot gets stuck in a small loop near the start position instead of progressing towards the goal.**

### Symptoms:
```
[Iteration 50]  Position: (2.39, 2.79)  Distance to goal: 21.79m
[Iteration 100] Position: (2.25, 2.13)  Distance to goal: 22.36m  ← INCREASING!
[Iteration 150] Position: (2.45, 2.76)  Distance to goal: 21.77m
[Iteration 200] Position: (2.29, 2.09)  Distance to goal: 22.36m  ← STUCK IN LOOP
```

The robot:
- Moves in a tiny area (2.25-2.45 in X, 2.09-2.79 in Y)
- Distance to goal oscillates between 21.77m and 22.36m
- Only explores 14.6% of the map
- Never makes progress towards goal at (18, 18)

### What I've Tried

1. **Initial approach**: Complex scoring function
   ```python
   score = (progress_to_goal * 5.0 + info_gain * 0.05) / (1.0 + dist_to_frontier * 0.3)
   ```
   **Problem**: Robot would go backwards (away from goal)

2. **Current approach**: Simple closest-to-goal selection
   ```python
   # Select frontier with minimum distance to goal
   best_frontier = min(frontiers, key=lambda f: distance(f, goal))
   ```
   **Problem**: Robot still gets stuck in loops

## Key Questions for Expert LLM

### 1. Frontier Selection Strategy
**Question**: What's the best way to select frontiers to ensure monotonic progress towards the goal?

**Options I'm considering**:
- A) Pure greedy: Always closest to goal (current)
- B) Weighted: Balance distance to goal + distance from robot
- C) Angle-based: Prefer frontiers in direction of goal
- D) Hybrid A*: Consider kinematic constraints
- E) Something else?

### 2. Loop Detection & Avoidance
**Question**: How do I detect when the robot is stuck in a loop and break out?

**Ideas**:
- Track last N positions, detect if revisiting same area
- Blacklist frontiers that have been visited recently
- Add randomness/exploration bonus after X iterations without progress
- Increase search radius when stuck

### 3. Path Planning Issues
**Question**: Could the A* planner be causing issues?

**Observations**:
- Inflation radius = 3 cells (safety margin)
- 8-connected grid
- Euclidean heuristic
- Path smoothing enabled

**Potential issues**:
- Too conservative (inflation too large)?
- Not considering robot kinematics?
- Smoothing removing important waypoints?

### 4. Frontier Detection
**Question**: Are we detecting frontiers correctly?

**Current method**:
```python
for each free cell:
    if any 8-connected neighbor is unknown:
        mark as frontier
group into regions
filter small regions (< 5 cells)
return centroids
```

**Potential issues**:
- Too many small frontiers near start?
- Not filtering frontiers behind robot?
- Should we only consider frontiers in "forward" direction?

### 5. Goal-Directed Exploration
**Question**: What's the best balance between exploration and exploitation?

**Current**: Pure exploitation (always go towards goal)
**Alternative**: Mix exploration (information gain) with goal-seeking

## Desired Behavior

```
START (2, 2) → Frontier 1 (5, 5) → Frontier 2 (8, 9) → Frontier 3 (12, 13) → GOAL (18, 18)
```

**Requirements**:
1. ✅ Monotonic progress towards goal (distance should decrease)
2. ✅ No loops or backtracking
3. ✅ Reasonable exploration coverage (60-80%)
4. ✅ Efficient path (not too circuitous)
5. ✅ Robust to different obstacle configurations

## Code Structure

### Main Loop (simplified)
```python
while not goal_reached:
    # Sense
    lidar_scan = lidar.scan(robot, obstacles)
    
    # Map
    occupancy_grid.update(robot.pose, lidar_scan)
    
    # Plan
    if can_reach_goal_directly():
        path = plan_to_goal()
    else:
        frontiers = detect_frontiers()
        best_frontier = select_frontier(frontiers)  # ← THIS IS THE PROBLEM
        path = plan_to_frontier(best_frontier)
    
    # Control
    velocity, steering = follow_path(path)
    
    # Act
    robot.step(velocity, steering)
```

### Frontier Selection (current)
```python
def select_frontier(frontiers):
    best = None
    min_dist = inf
    
    for frontier in frontiers:
        dist_to_goal = distance(frontier, goal)
        if dist_to_goal < min_dist:
            min_dist = dist_to_goal
            best = frontier
    
    return best
```

## What I Need

**Please provide**:

1. **Root cause analysis**: Why is the robot getting stuck?

2. **Improved frontier selection algorithm**: 
   - Pseudocode or Python implementation
   - Explanation of why it works
   - Edge cases it handles

3. **Loop detection mechanism**:
   - How to detect loops
   - How to break out of them

4. **Parameter tuning guidance**:
   - Inflation radius
   - Frontier filtering
   - Lookahead distance
   - Any other critical parameters

5. **Alternative approaches**:
   - Should I use RRT/RRT* instead of frontiers?
   - Should I use potential fields?
   - Should I use sampling-based methods?

## Additional Context

### Environment
- 20m × 20m square
- 12 random obstacles (boxes, cylinders, cones, hemispheres)
- Grid resolution: 0.1m (200×200 cells)
- Robot size: 2.4m × 1.5m

### Performance Constraints
- Real-time visualization required
- Python implementation
- Should complete in < 5 minutes

### Success Criteria
- Goal reached: YES
- Distance traveled: < 40m (straight line is ~22.6m)
- Map explored: > 60%
- No collisions
- Smooth motion

## Files to Review

If you need to see the actual code:
1. `src/goal_directed_explorer.py` - Main system (lines 200-250 for planning logic)
2. `src/planning/frontier_explorer.py` - Frontier detection
3. `src/planning/astar.py` - Path planning
4. `src/control/velocity_controller.py` - Motion control

## Expected Output Format

Please structure your response as:

### 1. Problem Diagnosis
- Root cause of the loop
- Why current approach fails

### 2. Recommended Solution
- Algorithm description
- Pseudocode
- Implementation notes

### 3. Implementation Steps
1. Step 1: ...
2. Step 2: ...
3. Step 3: ...

### 4. Alternative Approaches
- Option A: ...
- Option B: ...
- When to use each

### 5. Testing Strategy
- How to verify the fix works
- Edge cases to test

---

**Thank you for your expertise! This is for an autonomous robotics project and I want to ensure the robot navigates intelligently and efficiently.**
