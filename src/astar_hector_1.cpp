#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <queue>
#include <algorithm>
#include <unordered_set> 
#include <unordered_map> 
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

struct Node {
    int x, y, z;  
    double cost;  // G cost (distance from start)
    double heuristic;  // H cost (distance to goal)
    Node* parent;  // Pointer to the parent node

    
    Node(int x, int y, int z, double cost = 0, double heuristic = 0, Node* parent = nullptr)
        : x(x), y(y), z(z), cost(cost), heuristic(heuristic), parent(parent) {}

    // F cost
    double getF() const { return cost + heuristic; }

    // Priority queue (min-heap by F cost)
    bool operator>(const Node& other) const {
        return getF() > other.getF();
    }
};
bool isGoalWithinTolerance(const Node& current, const Node& goal, int tolerance) {
    return abs(current.x - goal.x) <= tolerance &&
           abs(current.y - goal.y) <= tolerance &&
           abs(current.z - goal.z) <= tolerance;
}

//Read the grid from a file
std::vector<std::vector<std::vector<bool>>> readGridFromFile(const std::string& filename, int& x_size, int& y_size, int& z_size,
                                                             double& resolution, double& min_bound_x, double& min_bound_y, double& min_bound_z,
                                                             double& max_bound_x, double& max_bound_y, double& max_bound_z) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Error opening file for reading: " << filename << std::endl;
        exit(1);
    }

    
    file >> resolution;
    file >> min_bound_x >> min_bound_y >> min_bound_z;
    file >> max_bound_x >> max_bound_y >> max_bound_z;

    
    file >> x_size >> y_size >> z_size;

    std::vector<std::vector<std::vector<bool>>> grid(x_size, std::vector<std::vector<bool>>(y_size, std::vector<bool>(z_size, false)));

    // (0 for unoccupied, 1 for occupied)
    int value;  
    for (int i = 0; i < x_size; ++i) {
        for (int j = 0; j < y_size; ++j) {
            for (int k = 0; k < z_size; ++k) {
                file >> value;
                grid[i][j][k] = (value == 1);
            }
        }
    }

    file.close();
    return grid;
}
// Convert world coordinates to grid indices
bool convertToGridIndex(double wx, double wy, double wz, int& gx, int& gy, int& gz, double resolution,
                        double min_x, double min_y, double min_z, int x_size, int y_size, int z_size) {
    gx = static_cast<int>((wx - min_x) / resolution);
    gy = static_cast<int>((wy - min_y) / resolution);
    gz = static_cast<int>((wz - min_z) / resolution);

    return gx >= 0 && gx < x_size &&
           gy >= 0 && gy < y_size &&
           gz >= 0 && gz < z_size;  
}

// Heuristic calculation (Euclidean distance)
double heuristic(const Node& current, const Node& goal) {
    return sqrt(
        pow( current.x - goal.x, 2) +
        pow(current.y - goal.y, 2) +
        pow(current.z - goal.z, 2)
    );
}

// Check if a position is valid
bool isValid(int x, int y, int z, const std::vector<std::vector<std::vector<bool>>>& grid) {
    bool condition1= true; 
    //Allowing robot radius so that the blades of the drone don't hit obstacles
    for (int dx = -4; dx <= 4; ++dx) {
        for (int dy = -4; dy <= 4; ++dy) {
            for (int dz = -3; dz <= 3; ++dz) {
                int nx = x + dx;
                int ny = y + dy;
                int nz = z + dz;

                if (nx >= 0 && nx < grid.size() &&
                    ny >= 0 && ny < grid[0].size() &&
                    nz >= 0 && nz < grid[0][0].size() &&
                    grid[nx][ny][nz]) {  
                    condition1= false; 
                }
            }
        }
    }
    bool condition2= x >= 0 && x < grid.size() &&
           y >= 0 && y < grid[0].size() &&
           z >= 0 && z < grid[0][0].size() &&
           !grid[x][y][z];  
    return condition1 && condition2;
}
//Reconstruct path
std::vector<Node> reconstructPath(Node* node) {
    std::vector<Node> path;
    while (node) {
        path.push_back(*node);
        node = node->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Node> aStar(const Node& start, const Node& goal, const std::vector<std::vector<std::vector<bool>>>& grid) {
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
    std::unordered_set<std::string> closedSet;
    std::unordered_map<std::string, double> gCostMap;

    // Generate a unique key for each node
    auto getKey = [](int x, int y, int z) {
        return std::to_string(x) + "," + std::to_string(y) + "," + std::to_string(z);
    };

    // Push the start node into the open set
    openSet.push(start);

    while (!openSet.empty()) {
        Node current = openSet.top();
        openSet.pop();

        // If the goal is reached, reconstruct and return the path
        if (current.x == goal.x && current.y == goal.y && current.z == goal.z) {
            return reconstructPath(&current);
        }
        // if (isGoalWithinTolerance(current, goal, 6)) {  
        //     return reconstructPath(&current);
        // }   
        // Mark the current node as visited
        closedSet.insert(getKey(current.x, current.y, current.z));

    
        std::vector<std::tuple<int, int, int>> directions;
        directions.push_back({1, 0, 0});directions.push_back({-1, 0, 0}); 
        directions.push_back({0, 1, 0}); directions.push_back({0, -1, 0}); 
        directions.push_back({0, 0, 1}); directions.push_back({0, 0, -1});
        directions.push_back({1, 1, 0}); directions.push_back({-1, -1, 0});
        directions.push_back({1, 0, 1}); directions.push_back({-1, 0, -1});
        directions.push_back({0, 1, 1}); directions.push_back({0, -1, -1});



        for (auto [dx, dy, dz] : directions) {
            int nx = current.x + (dx);
            int ny = current.y + (dy);
            int nz = current.z + (dz);
            std::string key = getKey(nx, ny, nz);
            if (isValid(nx, ny, nz, grid) && closedSet.find(getKey(nx, ny, nz)) == closedSet.end()) {
                double newGCost = current.cost + sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz,2));
                double hCost = heuristic(Node(nx, ny, nz), goal);

                // Node neighbor(nx, ny, nz, gCost, hCost, new Node(current));
                // openSet.push(neighbor);
                if (gCostMap.find(key) == gCostMap.end() || newGCost < gCostMap[key]) {
                    gCostMap[key] = newGCost;
                    Node* newParent = new Node(current);
                    Node neighbor(nx, ny, nz, newGCost, hCost, newParent);
                    openSet.push(neighbor);
                }
            }
        }
    }

    return {};
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_path_publisher");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("astar_path", 10);
    
    std::string filename = "/home/meera/catkin_ws/src/hector_astar1/src/grid_output.txt";

    
    int x_size, y_size, z_size;
    double resolution, min_bound_x, min_bound_y, min_bound_z, max_bound_x, max_bound_y, max_bound_z;

    
    auto grid = readGridFromFile(filename, x_size, y_size, z_size, resolution, min_bound_x, min_bound_y, min_bound_z, max_bound_x, max_bound_y, max_bound_z);

    std::cout << "Loaded grid with dimensions: " << x_size << "x" << y_size << "x" << z_size << std::endl;

    // Start and goal positions in world coordinates
    double start_x = 1.0, start_y = 1.5, start_z = 0.5;
    double goal_x = -1.2, goal_y = -1.5, goal_z = 1.0;

    // Convert world coordinates to grid indices
    int start_gx, start_gy, start_gz;
    int goal_gx, goal_gy, goal_gz;

    if (!convertToGridIndex(start_x, start_y, start_z, start_gx, start_gy, start_gz, resolution, min_bound_x, min_bound_y, min_bound_z, x_size, y_size, z_size) ||
        !convertToGridIndex(goal_x, goal_y, goal_z, goal_gx, goal_gy, goal_gz, resolution, min_bound_x, min_bound_y, min_bound_z, x_size, y_size, z_size)) {
        std::cerr << "Start or goal position is out of bounds." << std::endl;
        return 1;
    }

     // Check if the start cell is occupied
    if (grid[start_gx][start_gy][start_gz]) {
        std::cout << "Path not available: Start cell is occupied." << std::endl;
        return 0;
    }

    // Check if the goal cell is occupied
    if (grid[goal_gx][goal_gy][goal_gz]) {
        std::cout << "Path not available: Goal cell is occupied." << std::endl;
        return 0;
    }

    Node start(start_gx, start_gy, start_gz);
    Node goal(goal_gx, goal_gy, goal_gz);
    auto path = aStar(start, goal, grid);

    if (!path.empty()) {
        std::cout << "Path found:" << std::endl;
        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";  // Set the frame ID (change as per your TF)

        std::ofstream path_file("/home/meera/catkin_ws/src/hector_astar1/src/path_output.txt");
        for (const auto& node : path) {
            if (path_file.is_open()) {
                for (const auto& node : path) {
                    double world_x = min_bound_x + node.x * resolution;
                    double world_y = min_bound_y + node.y * resolution;
                    double world_z = min_bound_z + node.z * resolution;
                    
                    //path_file << node.x << " " << node.y << " " << node.z << "\n";
                    path_file << world_x << " " << world_y << " " << world_z << "\n";

                    std::cout << "Grid: (" << node.x << ", " << node.y << ", " << node.z << ") "
                          << "World: (" << world_x << ", " << world_y << ", " << world_z << ")" << std::endl;
                    geometry_msgs::PoseStamped pose;
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.pose.position.x = min_bound_x + node.x * resolution;
                    pose.pose.position.y = min_bound_y + node.y * resolution;
                    pose.pose.position.z = min_bound_z + node.z * resolution;

                    ros_path.poses.push_back(pose);
                }
                path_pub.publish(ros_path);

                path_file.close();
            }
            //std::cout << "(" << node.x << ", " << node.y << ", " << node.z << ")" << std::endl;
             ros::Rate loop_rate(10);
            while (ros::ok()) {
                path_pub.publish(ros_path);
                ros::spinOnce();
                loop_rate.sleep();
            }
        }
    } else {
        std::cout << "No path found." << std::endl;
    }
}