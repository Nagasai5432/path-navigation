#A* Path Planning with Trajectory Smoothing & Curve Generation

An advanced path planning simulation that goes beyond basic A* by incorporating **path smoothing**, **line-of-sight optimization**, and **cubic spline trajectory generation**—mimicking real-world robotic motion planning.

---

## Project Overview

This project demonstrates how an autonomous system:

1. Finds a path using **A*** algorithm
2. Optimizes the path using **line-of-sight smoothing**
3. Converts it into a **continuous trajectory** using cubic splines

This bridges the gap between:

* **Discrete path planning (grid-based)**
* **Continuous motion planning (real robot movement)**

---

## Key Features

--- A* Path Planning (8-direction movement)
--- Obstacle-aware navigation with corner constraints
--- Real-time visualization of:

* Open set (frontier)
* Closed set (explored nodes)

--- Path Optimization:

* Line-of-sight smoothing (removes unnecessary turns)

--- Trajectory Generation:

* Cubic spline interpolation for smooth curves

--- Step-by-step animation:

* Exploration → Path → Smoothed path → Final trajectory

---

## Core Concepts Implemented

### A* Search Algorithm

* Uses:

  * **g(n)** → cost from start
  * **h(n)** → heuristic (Euclidean distance)
* Guarantees optimal path in grid space

---

### Line-of-Sight Smoothing

* Removes zig-zag paths
* Connects nodes directly if no obstacle lies in between
* Produces shorter and more realistic paths

---

###  Cubic Spline Interpolation

* Converts discrete waypoints into smooth curves
* Ensures:

  * Continuous motion
  * Reduced jerk (important for robots)

---

##  Demo

> *(Add your demo GIF or video here)*

```id="demo02"
![Demo](assets/demo.gif)
```

---

##  Installation

```bash id="inst02"
git clone https://github.com/your-username/astar-trajectory-planner.git
cd astar-trajectory-planner
pip install numpy matplotlib scipy
```

---

## Run the Project

```bash id="run02"
python3 path.py
```

---

## Workflow

```id="flow02"
Grid → A* Search → Raw Path → Line-of-Sight Smoothing → Cubic Spline → Smooth Trajectory
```

---

## Project Structure

```id="struct02"
astar-trajectory-planner/
│── path.py
│── README.md
│── assets/
│    └── demo.gif
```

---

##  Real-World Applications

This project directly relates to:

* Autonomous robots (ROS2 Nav2)
*  Self-driving vehicles
*  Drone navigation systems
* Industrial robotic arms

---

## Key Learnings

* Difference between:

  * **Path planning vs Trajectory planning**
* Importance of:

  * Smooth motion for real robots
* Trade-offs between:

  * Accuracy vs computational cost

---

## Why This Project Stands Out

Unlike basic A* implementations, this project:

✔ Converts discrete paths into **continuous motion**
✔ Implements **post-processing optimization**
✔ Simulates **real robotic movement constraints**

---

## Future Improvements

* Dynamic obstacle handling
* Velocity profiling
* ROS2 integration (Nav2 stack)
* RRT / RRT* implementation
* Real robot deployment

---

##  Author

**Nagasai P**
Robotics | Computer Vision | ROS2


If you found this project useful, give it a ⭐ and share!

---
