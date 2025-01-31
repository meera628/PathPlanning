#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <gazebo_msgs/ModelStates.h>
#include <tf/tf.h>

struct Point3D {
    double x, y, z;
};

// Load the path from file
std::vector<Point3D> loadPath(const std::string& filename) {
    std::ifstream path_file(filename);
    std::vector<Point3D> path;
    double x, y, z;
    while (path_file >> x >> y >> z) {
        path.push_back({x, y, z});
    }
    return path;
}
Point3D current_position = {0.5, 0.5, 0.0};
double current_yaw = 0.0; 
// Check if the drone is close enough to the target
bool isAtTarget(const Point3D& current, const Point3D& target, double threshold) {
    double dist = std::sqrt(std::pow(current.x - target.x, 2) +
                            std::pow(current.y - target.y, 2) +
                            std::pow(current.z - target.z, 2));
    return dist < threshold;
}
//Update current position from Gazebo's model states
void modelStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
        current_position.x = msg->pose[i].position.x;
        current_position.y = msg->pose[i].position.y;
        current_position.z = msg->pose[i].position.z;
        tf::Quaternion q(
                msg->pose[i].orientation.x,
                msg->pose[i].orientation.y,
                msg->pose[i].orientation.z,
                msg->pose[i].orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw);
        
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower2");
    ros::NodeHandle nh;

    
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Subscriber model_state_sub = nh.subscribe("/gazebo/model_states", 10, modelStateCallback);

    std::string path_file = "/home/meera/catkin_ws/src/hector_astar1/src/path_output.txt";
    auto path = loadPath(path_file);

    if (path.empty()) {
        ROS_ERROR("Path is empty. Check the file: %s", path_file.c_str());
        return 1;
    }

    ROS_INFO("Path loaded with %lu points.", path.size());

    ros::Rate rate(10);  // 10 Hz
    
    size_t index = 0;
    double kp_linear = 2.0;   // Increased proportional control gain for faster movement
    double kp_angular = 1.5;  // Control gain for yaw correction
    double max_speed = 1.0;   // Max linear speed
    double max_yaw_rate = 1.0; // Max yaw speed
    double threshold = 0.05;   // Proximity threshold

    // Velocity control parameters
    double kp = 3.0;  
    //double threshold = 0.02;  
    
    while (ros::ok() && index < path.size()) {
                Point3D target = path[index];

        // Compute direction vector
        double dx = target.x - current_position.x;
        double dy = target.y - current_position.y;
        double dz = target.z - current_position.z;

        if (isAtTarget(current_position, target, threshold)) {
            ROS_INFO("Reached waypoint %lu: (%.2f, %.2f, %.2f)", index, target.x, target.y, target.z);
            ++index;  // Move to the next waypoint
            continue;
        }

        // Compute desired yaw angle (atan2 gives the angle between current and target point)
        double desired_yaw = atan2(dy, dx);
        double yaw_error = desired_yaw - current_yaw;

        // Normalize yaw error to the range [-pi, pi]
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        // Compute yaw rate command
        double yaw_rate = kp_angular * yaw_error;
        yaw_rate = std::max(std::min(yaw_rate, max_yaw_rate), -max_yaw_rate); // Limit yaw rate

        // Compute speed using proportional control
        double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
        double speed = kp_linear * distance;
        speed = std::min(speed, max_speed);  // Limit speed

        // Normalize velocity direction
        double norm_factor = std::sqrt(dx * dx + dy * dy + dz * dz);
        double vx = (norm_factor > 0) ? (dx / norm_factor) * speed : 0.0;
        double vy = (norm_factor > 0) ? (dy / norm_factor) * speed : 0.0;
        double vz = (norm_factor > 0) ? (dz / norm_factor) * speed : 0.0;

        // Create velocity command
        geometry_msgs::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.linear.z = vz;
        cmd.angular.z = yaw_rate;  // Control yaw to align with path

        ROS_INFO("Moving to waypoint %lu: (%.2f, %.2f, %.2f) | Velocities: (%.2f, %.2f, %.2f) | Yaw: %.2f -> %.2f (Rate: %.2f)",
                 index, target.x, target.y, target.z, cmd.linear.x, cmd.linear.y, cmd.linear.z,
                 current_yaw, desired_yaw, yaw_rate);

        // Publish command
        cmd_pub.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Path following complete.");
    return 0;
}
