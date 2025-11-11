# Quick Reference Guide
## Autonomous Exploration System

---

## 🚀 Quick Start

```bash
cd /Users/aryanmathur/Desktop/RDCP/take2
source venv/bin/activate
python3 -m src.goal_directed_explorer
```

---

## 🎯 What You'll See

### Console Output

```
🔍 INITIAL ENVIRONMENT SCAN
📍 INITIAL FRONTIERS DETECTED: 69

🔍 EVALUATING 69 FRONTIERS:
Top 5 Frontier Candidates:
👉 1. (3.55, 7.25) - Score: 26.03  ← BEST (highest score)
      Progress to goal: 4.62m       ← Moves 4.62m closer to goal
      Dist to frontier: 5.47m       ← 5.47m away from robot
      Dist to goal: 18.01m          ← 18.01m from goal
      Info gain: 914 cells          ← Will explore 914 unknown cells

🎯 SELECTED BEST FRONTIER: (3.55, 7.25)

✅ FRONTIER #1 REACHED!
🔍 SCANNING FROM NEW POSITION...
📍 NEW FRONTIERS DETECTED: 47
... (repeats until goal reached)

🎯 FINAL GOAL REACHED!
```

### 2D Visualization (Left)

| Symbol | Meaning |
|--------|---------|
| 🟢 Green Circle | START position |
| ❌ Red X | GOAL position |
| 🔵 Blue Circle | Current robot |
| ⭐ Yellow Star | Current target frontier |
| 🟢 Numbered Circles | Visited frontiers (1, 2, 3...) |
| 🔵 Blue Line | Path traveled |
| 🟢 Dashed Line | Planned path |
| 🔵 Light Blue | Explored free space |
| 🔴 Dark Red | Detected obstacles |
| ⚪ Gray | Unknown areas |

### 3D Visualization (Right)

- Obstacles as 3D shapes
- Robot as blue sphere
- Trajectory as blue line

---

## ⚙️ Customization

### Change Start/Goal

Edit `src/goal_directed_explorer.py` line 670:

```python
def main():
    START = (5.0, 3.0)   # Your start (x, y)
    GOAL = (15.0, 17.0)  # Your goal (x, y)
```

### Adjust Speed

Line 60-64:

```python
velocity_controller = TrapezoidalVelocityController(
    max_velocity=0.8,      # Increase for faster (default: 0.5)
    max_acceleration=0.5   # Increase for quicker (default: 0.3)
)
```

### Change Safety Margin

Line 203, 367:

```python
inflation_radius=5  # Larger = more cautious (default: 3)
```

---

## 🔍 Understanding the Algorithm

### Frontier Selection Score

```
score = (progress_to_goal × 5.0 + info_gain × 0.05) / (1.0 + distance × 0.3)
```

**Higher score = Better frontier**

- **Progress to goal** (×5.0): Most important - how much closer to goal
- **Info gain** (×0.05): Bonus for exploring more unknown cells
- **Distance** (×0.3): Penalty for far frontiers

### Example

```
Frontier A: Progress=4.62m, Distance=5.47m, Info=914
Score = (4.62×5.0 + 914×0.05) / (1.0 + 5.47×0.3)
      = (23.1 + 45.7) / 2.64
      = 26.03  ← BEST!

Frontier B: Progress=2.15m, Distance=3.21m, Info=650
Score = (2.15×5.0 + 650×0.05) / (1.0 + 3.21×0.3)
      = (10.75 + 32.5) / 1.96
      = 22.09  ← Lower score
```

---

## 📊 Key Metrics

| Metric | Typical Value |
|--------|---------------|
| Frontiers visited | 3-8 |
| Distance traveled | 20-30m |
| Time to goal | 60-120s |
| Map explored | 70-90% |
| Average speed | 0.25-0.35 m/s |

---

## 🐛 Troubleshooting

### Robot goes to wrong frontier
✅ **FIXED!** Now uses Hybrid A* scoring (progress×5.0)

### Too many frontiers shown
✅ **FIXED!** Only shows current target + visited history

### Robot gets stuck
- Reduce `inflation_radius` from 3 to 2
- Increase `max_velocity` from 0.5 to 0.7

### No path found
- Check if goal is in obstacle
- Reduce `inflation_radius`
- Increase grid `resolution`

---

## 📁 File Structure

```
src/
├── goal_directed_explorer.py  ← MAIN FILE (run this)
├── robot/ackermann.py         ← Robot kinematics
├── sensors/lidar.py            ← LiDAR sensor
├── map/occupancy_grid.py       ← Mapping
├── planning/
│   ├── astar.py                ← Path planning
│   └── frontier_explorer.py    ← Frontier detection
└── control/
    └── velocity_controller.py  ← Motion control
```

---

## 🎓 What It Does

1. **Start** at (2, 2)
2. **Scan** environment with LiDAR
3. **Detect** all frontiers (boundaries of unknown space)
4. **Evaluate** each frontier with scoring function
5. **Select** best frontier (highest score = most progress to goal)
6. **Plan** path using A*
7. **Navigate** to frontier with smooth motion
8. **Reach** frontier → Clear old frontiers
9. **Repeat** steps 2-8 until goal reached
10. **Success!** 🎯

---

## 📈 Performance Tips

### Faster Exploration
```python
max_velocity=0.8
max_acceleration=0.5
resolution=0.15  # Coarser grid
```

### More Accurate
```python
max_velocity=0.3
resolution=0.05  # Finer grid
inflation_radius=4  # More cautious
```

### Balanced (Default)
```python
max_velocity=0.5
resolution=0.1
inflation_radius=3
```

---

## 🎯 Success Criteria

✅ Goal reached  
✅ No collisions  
✅ Smooth motion  
✅ Efficient path  
✅ Good exploration coverage  

---

## 📞 Need Help?

1. Check `COMPLETE_DOCUMENTATION.md` for details
2. Check `FINAL_DOCUMENTATION.md` for algorithms
3. Check console output for errors
4. Verify virtual environment is activated

---

**Status: FULLY FUNCTIONAL** ✅

*Last Updated: November 10, 2025*
