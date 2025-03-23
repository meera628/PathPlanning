# **Optimized Path Planning for Quadrotors**  

This project explores **improved RRT*** for **waypoint identification** in drone path planning, followed by **Minimum Snap Trajectory Optimization** using **Safe Flight Corridors** for smooth and efficient navigation.  

## **Software Used**  
- **ROS Noetic**  
- **Gazebo 11**  
- **Rviz**  

## **Project Milestones & Branch Links**  

### 1. Understanding 2D Path Planning  
- Analyzed **A***, **RRT***, and **PRM** algorithms for pathfinding in a **grid environment with obstacles**.  
- Compared **efficiency, smoothness, clearance, and obstacle-handling** capabilities.  
- 📂 **Code & report:** [**astar_planner_turtlebot3 branch**](https://github.com/meera628/PathPlanning/tree/astar_planner_turtlebot3)  

### 2. Extending A* to 3D
- Implemented and tested **A*** in **3D obstacle-ridden environments**.  
- 🎥 [**Simulation Video**](https://github.com/meera628/PathPlanning/blob/astar_3d_hector_quadrotor/drone_astar_ver1_compressed.mp4)  
- 📂 **Code & launch files:** [**astar_3d_hector_quadrotor branch**](https://github.com/meera628/PathPlanning/tree/astar_3d_hector_quadrotor)  

### 3. Extending & Improving RRT* in 3D _(In Progress)_  
- Implemented **Bidirectional RRT** with enhancements for **faster convergence** and **better path optimality**.
- 📂 **Package:** [**rrt_star_3d_quadrotor**](https://github.com/meera628/PathPlanning/tree/rrt_star_3d_quadrotor)
- 📂 [**RRT Star Bidirectional**](https://github.com/meera628/PathPlanning/blob/rrt_star_3d_quadrotor/hector_rrt_star/src/rrt_star_bidirectional.cpp)

### 4. Safe Flight Corridors & Minimum Snap Trajectory _(In Progress)_  
-  📂 Used DecompROS library to generate [Safe Flight Corridors](https://github.com/meera628/PathPlanning/tree/safe_flight_corridors) around RRT* path
- Researching **trajectory smoothing** techniques using **Minimum Snap optimization**.
- Developed [**Minimum Snap Trajectory Optimization**](https://github.com/meera628/PathPlanning/blob/trajectory_optimization/trajectory_optimization/scripts/minisnap_trajectory.py) to generate smooth and dynamically feasible paths (without corridor constraints). 
- Currently extending optimization to incorporate Safe Flight Corridor constraints for collision-free motion.

### 5. Algorithm Comparisons _(Upcoming)_  
- Comparing **RRT*** with **A***, **PRM**, and other algorithms for real-world feasibility.  

### 6. Actual Drone Implementation _(Upcoming)_  
- Deploying final algorithms on a **real quadrotor** for validation.  

---
