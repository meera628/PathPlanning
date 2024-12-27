#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math
from nav_msgs.msg import Odometry



class PathFollower:
    def __init__(self):
        rospy.init_node("astar_path_follower_node")
        self.path_sub = rospy.Subscriber("/astar_path", Path, self.path_callback)
        print("Subscriber registered:", self.path_sub) 
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.current_pose = None
        self.path = None
        self.path_received = False 
        self.current_waypoint_index = 0
        self.tolerance = 0.1  # Distance tolerance to consider a waypoint as reached
        self.angular_tolerance = 0.1  
        self.rate = rospy.Rate(10)  # 10 Hz loop rate
    
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def path_callback(self, msg):
        if not (self.path_received):
            self.path = msg
            self.current_waypoint_index = 0  # Reset waypoint index when a new path is received
            self.path_received = True 
            #rospy.loginfo("Received new path with {} waypoints.".format(len(self.path.poses)))

    def distance(self, pose1, pose2):
        return math.sqrt(
            (pose1.position.x - pose2.position.x) ** 2 +
            (pose1.position.y - pose2.position.y) ** 2
        )

    def angle_to_goal(self, pose1, pose2):
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.atan2(dy, dx)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def stop_robot(self):
        """Stop the robot."""
        stop_msg = Twist()
        self.cmd_pub.publish(stop_msg)

    def follow_path(self):
        """Control loop for following the path."""
        if not self.path:
            rospy.logwarn("No path to follow.")
            return

        while not rospy.is_shutdown():
            if self.current_waypoint_index >= len(self.path.poses):
                rospy.loginfo("Path completed.")
                self.stop_robot()
                break

            if not self.current_pose:
                rospy.logwarn("Waiting for current pose...")
                self.rate.sleep()
                continue

            # Get the current goal waypoint
            goal_pose = self.path.poses[self.current_waypoint_index].pose

            # Calculate distance and angle to the waypoint
            distance_to_goal = self.distance(self.current_pose, goal_pose)
            angle_to_goal = self.angle_to_goal(self.current_pose, goal_pose)

            # Get the robot's current orientation
            quaternion = self.current_pose.orientation
            _, _, current_yaw = self.euler_from_quaternion(quaternion)

            # Calculate the angular difference
            angular_diff = self.normalize_angle(angle_to_goal - current_yaw)

            # Control logic
            cmd_vel = Twist()
            if distance_to_goal > self.tolerance:
                if abs(angular_diff) > self.angular_tolerance:
                    # Rotate to align with the waypoint
                    cmd_vel.angular.z = 0.7 * angular_diff
                else:
                    # Move forward
                    cmd_vel.linear.x = 0.4
            else:
                rospy.loginfo("Reached waypoint {}.".format(self.current_waypoint_index))
                self.current_waypoint_index = self.current_waypoint_index + 1

            self.cmd_pub.publish(cmd_vel)
            self.rate.sleep()

    @staticmethod
    def euler_from_quaternion(quaternion):
        """Convert a quaternion to Euler angles."""
        import tf.transformations
        return tf.transformations.euler_from_quaternion(
            [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        )
    
if __name__ == "__main__":
    follower = PathFollower()
    while (not follower.path_received) and not rospy.is_shutdown():
        rospy.sleep(0.1)
    follower.follow_path()
