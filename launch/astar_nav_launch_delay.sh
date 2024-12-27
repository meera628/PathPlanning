#!/bin/bash
# Function to handle cleanup on Ctrl+C or termination
cleanup() {
    echo "Stopping all running processes..."
    pkill -f gazebo
    pkill -f roslaunch
    pkill -f rosrun
    echo "All processes stopped. Exiting."
    exit 0
}
trap cleanup SIGINT

# Set the TurtleBot3 model environment variable
export TURTLEBOT3_MODEL=burger
echo "TURTLEBOT3_MODEL set to $TURTLEBOT3_MODEL"

# Launch Gazebo with TurtleBot3 world
echo "Launching Gazebo with TurtleBot3 world..."
roslaunch turtlebot3_gazebo turtlebot3_world.launch &
gazebo_pid=$!
# Allow some time for Gazebo to start up
sleep 2  # Delay of 10 seconds

# Launch the TurtleBot3 Navigation with the provided map
echo "Launching TurtleBot3 navigation stack with map..."
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=/home/meera/catkin_ws/src/astar_ver2/maps/map1_maze.yaml &
nav_pid=$!  
# Allow some time for the navigation stack to initialize
sleep 2  # Delay of 10 seconds

# Launch the A* Planner node
echo "Launching A* planner node..."
rosrun astar_ver2 astar_planner.py &
planner_pid=$!
# Allow some time for the planner to initialize
sleep 2  # Delay of 5 seconds

# Run the A* Path Follower script
echo "Launching A* path follower script..."
rosrun astar_ver2 astar_path_follower.py &
follower_pid=$!

wait $gazebo_pid $nav_pid $planner_pid $follower_pid

