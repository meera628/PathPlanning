#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <gazebo_msgs/ModelStates.h>


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
Point3D current_position = {0, 0, 0.0};
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
        
    }
}
int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower2");
    ros::NodeHandle nh;

    
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    ros::Subscriber model_state_sub = nh.subscribe("/gazebo/model_states", 10, modelStateCallback);

    std::string path_file = "/home/meera/catkin_ws/src/hector_astar1/src/path_output_2.txt";
    auto path = loadPath(path_file);

    if (path.empty()) {
        ROS_ERROR("Path is empty. Check the file: %s", path_file.c_str());
        return 1;
    }

    ROS_INFO("Path loaded with %lu points.", path.size());

    ros::Rate rate(10);  // 10 Hz
    size_t index = 0;

    // Velocity control parameters
    double kp = 1.2;  
    double threshold = 0.02;  
    
    while (ros::ok() && index < path.size()) {
        Point3D target = path[index];

       
        geometry_msgs::Twist cmd;
        double dx = target.x - current_position.x;
        double dy = target.y - current_position.y;
        double dz = target.z - current_position.z;

        if (isAtTarget(current_position, target, threshold)) {
            cmd.linear.x = 0.0;
            cmd.linear.y = 0.0;
            cmd.linear.z = 0.0;  // Stop the drone
            ROS_INFO("Reached waypoint %lu: (%.2f, %.2f, %.2f)", index, target.x, target.y, target.z);
            ++index;  // Move to the next waypoint
        } else {
            // Proportional velocity control
            cmd.linear.x = kp * dx;
            cmd.linear.y = kp * dy;
            cmd.linear.z = kp * dz;
            ROS_INFO("Moving to waypoint %lu: (%.2f, %.2f, %.2f) with velocities (%.2f, %.2f, %.2f)", 
                     index, target.x, target.y, target.z, cmd.linear.x, cmd.linear.y, cmd.linear.z);
        }

        cmd_pub.publish(cmd);

        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Path following complete.");
    return 0;
}
