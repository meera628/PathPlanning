#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import math
import numpy as np
import matplotlib.pyplot as plt

class AStarPlanner():
    def __init__(self, resolution, robot_radius):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.map_data = None
        self.obstacle_map = None
        self.obstacle_map_done = False
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.x_width, self.y_width = 0, 0
        self.motion = self.get_motion_model()
        self.path = None

        # ROS Subscribers and Publishers
    
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.path_pub = rospy.Publisher("/astar_path", Path, queue_size=10)
        self.goal_x = 1.5  # Set goal x-coordinate (example)
        self.goal_y = 1.5  # Set goal y-coordinate (example)
        self.start_x = -2.0  # Set start x-coordinate (example)
        self.start_y = -0.5  # Set start y-coordinate (example)

    def map_callback(self, msg):
        self.map_data = msg
        ox, oy = [], []
        width, height = msg.info.width, msg.info.height
        resolution = msg.info.resolution
        self.min_x, self.min_y = msg.info.origin.position.x, msg.info.origin.position.y
        self.max_x = self.min_x + width * resolution
        self.max_y = self.min_y + height * resolution
        self.x_width = width
        self.y_width = height
        
        # Build obstacle map
        for y in range(height):
            for x in range(width):
                index = y * width + x
                if msg.data[index] > 30:  # Obstacle threshold
                    ox.append(x); oy.append(y)

        self.calc_obstacle_map(ox, oy)


    def publish_path(self, rx, ry):
        path = Path()
        path.header = Header()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        for x, y in zip(rx, ry):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    def visualize_path(self, rx, ry):
        width, height = self.map_data.info.width, self.map_data.info.height
        map_array = np.array(self.map_data.data).reshape((height, width))

        # Convert the map data for visualization (e.g., free = 0, obstacle = 1, unknown = 0.5)
        visualization_map = np.zeros_like(map_array, dtype=float)
        visualization_map[map_array == -1] = 0.5  # Unknown
        visualization_map[map_array > 50] = 0   # Obstacles
        visualization_map[map_array <= 50] = 1.0  # Free space

        path_indices_x = [self.calc_xy_index(x, self.min_x) for x in rx]
        path_indices_y = [self.calc_xy_index(y, self.min_y) for y in ry]

        # Overlay the path on the map visualization
        for px, py in zip(path_indices_x, path_indices_y):
            if 0 <= py < height and 0 <= px < width:
                visualization_map[py, px] = 0.3  # Highlight the path in gray

        plt.figure(figsize=(8, 8))
        plt.imshow(visualization_map, cmap="gray", origin="lower")
        plt.title("Occupancy Grid Map with A* Path")
        plt.xlabel("Grid X")
        plt.ylabel("Grid Y")
        plt.colorbar(label="Map Values")
        plt.grid(True)
        plt.savefig("/home/meera/Pictures/astar_output1.png")
        rospy.loginfo("Path visualization saved as 'astar_path.png'")

        plt.imshow(self.obstacle_map, cmap='gray', origin='lower')
        plt.title("Obstacle Map")
        plt.savefig("/home/meera/Pictures/obstacle_map.png")
        rospy.loginfo("Obs map saved as 'obstacle_map.png'")

    def goal_callback(self, msg):
        # This function is no longer needed as we set goal manually
        pass

    def start_planning(self):
        if self.map_data is None:
            rospy.logerr("Map data not yet received!")
            return

        start_x = self.start_x
        start_y = self.start_y
        goal_x = self.goal_x
        goal_y = self.goal_y

        sx = self.calc_xy_index(start_x, self.min_x)
        sy = self.calc_xy_index(start_y, self.min_y)
        gx = self.calc_xy_index(goal_x, self.min_x)
        gy = self.calc_xy_index(goal_y, self.min_y)

        rx, ry = self.planning(sx, sy, gx, gy)

        if rx and ry:
            self.publish_path(rx, ry)
            self.visualize_path(rx, ry)
            rospy.loginfo("Path generated successfully!")
        
        # Continuously publish the path
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            self.publish_path(rx, ry)
            rate.sleep() 
    
    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, sx, sy, gx, gy):

        print("Planning started")
        start_node = self.Node(sx, sy, 0.0, -1)
        goal_node = self.Node(gx, gy, 0.0, -1)
        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while open_set:
            c_id = min(open_set, key=lambda o: (open_set[o].cost + self.calc_heuristic(goal_node, open_set[o])))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                rospy.loginfo("Goal found!")
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                return self.calc_final_path(goal_node, closed_set)

            del open_set[c_id]
            closed_set[c_id] = current

            for i, _ in enumerate(self.motion):
                node = self.Node(current.x + self.motion[i][0],
                                current.y + self.motion[i][1],
                                current.cost + self.motion[i][2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                else:
                    if open_set[n_id].cost > node.cost:
                        open_set[n_id] = node

        rospy.logerr("No path found!")
        print("Planning ended")
        return [], []

    def calc_final_path(self, goal_node, closed_set):
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx[::-1], ry[::-1]

    def calc_heuristic(self, n1, n2):
        return math.hypot(n1.x - n2.x, n1.y - n2.y)

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True


    def calc_obstacle_map(self, oxs, oys):
        print("Entered calc_obstacle_map")

        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]        
        robot_radius = self.robot_radius

        for i in range(len(oxs)):
            ox = oxs[i]; oy = oys[i]
            for dx in range(-robot_radius-1, robot_radius+1):
                for dy in range(-robot_radius-1, robot_radius+1):
                    # To do - Need hypotenuse to check circle, rather than square
                    self.obstacle_map[ox+dx][oy+dy] = True
    
        self.obstacle_map_done = True
        print("Calculated obstacle map")

    @staticmethod
    def get_motion_model():
        return [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]

def main():
    rospy.init_node("astar_planner_node")
    resolution = 0.05  # Adjust based on map resolution
    robot_radius = 1  # Adjust based on TurtleBot3 size

    planner = AStarPlanner(resolution, robot_radius)
    while (not planner.obstacle_map_done) and not rospy.is_shutdown():
        rospy.sleep(0.1)
    planner.start_planning()  # Start planning with fixed start and goal
    rospy.spin()

if __name__ == "__main__":
    main()
