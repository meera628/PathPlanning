#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <random> // Required for random number generation
#include <chrono> // Required for seeding the random number generator
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

using namespace std;

struct Node {
    int x, y, z;
    double cost;  // G cost (distance from start)
    double heuristic;  // H cost (distance to goal - used for connection check)
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

// Helper function to calculate Euclidean distance between two nodes
double calculateDistance(const Node& node1, const Node& node2) {
    return sqrt(pow(node1.x - node2.x, 2) + pow(node1.y - node2.y, 2) + pow(node1.z - node2.z, 2));
}

// Function to generate a random point in the grid
Node getRandomNode(int x_size, int y_size, int z_size, const std::vector<std::vector<std::vector<bool>>>& grid) {
    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> dis_x(0, x_size - 1);
    uniform_int_distribution<> dis_y(0, y_size - 1);
    uniform_int_distribution<> dis_z(0, z_size - 1);

    while (true) {
        int x = dis_x(gen);
        int y = dis_y(gen);
        int z = dis_z(gen);
        if (!grid[x][y][z]) {
            return Node(x, y, z);
        }
    }
}

// Function to find the nearest node in the tree to a given node
Node* findNearestNode(const Node& rand_node, const vector<Node*>& tree) {
    Node* nearest_node = nullptr;
    double min_dist = numeric_limits<double>::max();

    for (const Node* node : tree) {
        double dist = calculateDistance(rand_node, *node);
        if (dist < min_dist) {
            min_dist = dist;
            nearest_node = const_cast<Node*>(node);
        }
    }
    return nearest_node;
}

// Function to steer from the nearest node to the random node
Node steer(Node* nearest_node, const Node& rand_node, double step_size, const std::vector<std::vector<std::vector<bool>>>& grid) {
    double dist = calculateDistance(*nearest_node, rand_node);

    if (dist > step_size) {
        double dx = rand_node.x - nearest_node->x;
        double dy = rand_node.y - nearest_node->y;
        double dz = rand_node.z - nearest_node->z;

        int new_x = nearest_node->x + (int)(dx * step_size / dist);
        int new_y = nearest_node->y + (int)(dy * step_size / dist);
        int new_z = nearest_node->z + (int)(dz * step_size / dist);

        if (new_x >= 0 && new_x < grid.size() && new_y >= 0 && new_y < grid[0].size() && new_z >= 0 && new_z < grid[0][0].size() && !grid[new_x][new_y][new_z])
        {
             return Node(new_x, new_y, new_z);
        } else {
            return Node(nearest_node->x, nearest_node->y, nearest_node->z); // Return the nearest node if the new node is invalid
        }

    } else {
        return rand_node;
    }
}

// Function to check if a path between two nodes is collision-free
bool isPathCollisionFree(Node* node1, Node* node2, const std::vector<std::vector<std::vector<bool>>>& grid) {
    // Simple line check: just check the intermediate cells between node1 and node2
    int dx = node2->x - node1->x;
    int dy = node2->y - node1->y;
    int dz = node2->z - node1->z;
    int steps = max({abs(dx), abs(dy), abs(dz)});

    if (steps == 0) return true; // Same node

    double x_step = (double)dx / steps;
    double y_step = (double)dy / steps;
    double z_step = (double)dz / steps;

    double x = node1->x;
    double y = node1->y;
    double z = node1->z;

    for (int i = 0; i < steps; ++i) {
        x += x_step;
        y += y_step;
        z += z_step;

        int grid_x = round(x);
        int grid_y = round(y);
        int grid_z = round(z);

        if (grid_x < 0 || grid_x >= grid.size() || grid_y < 0 || grid_y >= grid[0].size() || grid_z < 0 || grid_z >= grid[0][0].size()) {
            return false; // Out of bounds
        }

        if (grid[grid_x][grid_y][grid_z]) {
            return false; // Collision
        }
    }

    return true;
}

// Function to find nearby nodes
vector<Node*> findNearbyNodes(const Node& new_node, const vector<Node*>& tree, double radius) {
    vector<Node*> nearby_nodes;
    for (Node* node : tree) {
        double dist = calculateDistance(new_node, *node);
        if (dist <= radius) {
            nearby_nodes.push_back(node);
        }
    }
    return nearby_nodes;
}

// Function to choose parent
Node* chooseParent(Node* nearest_node, const Node& new_node, const vector<Node*>& nearby_nodes, const std::vector<std::vector<std::vector<bool>>>& grid) {
    Node* best_parent = nearest_node;
    double min_cost = nearest_node->cost + calculateDistance(*nearest_node, new_node);

    for (Node* nearby_node : nearby_nodes) {
        if (isPathCollisionFree(nearby_node, const_cast<Node*>(&new_node), grid)) {
            double cost = nearby_node->cost + calculateDistance(*nearby_node, new_node);
            if (cost < min_cost) {
                min_cost = cost;
                best_parent = nearby_node;
            }
        }
    }
    return best_parent;
}

// Function to rewire the tree
void rewire(Node* new_node, const vector<Node*>& nearby_nodes, const std::vector<std::vector<std::vector<bool>>>& grid) {
    for (Node* nearby_node : nearby_nodes) {
        if (nearby_node != new_node->parent && isPathCollisionFree(new_node, nearby_node, grid)) {
            double cost_to_nearby = new_node->cost + calculateDistance(*new_node, *nearby_node);
            if (cost_to_nearby < nearby_node->cost) {
                nearby_node->parent = new_node;
                nearby_node->cost = cost_to_nearby;
            }
        }
    }
}
// Function to extract the path from the tree
vector<Node> reconstructPath(Node* node) {
    vector<Node> path;
    while (node != nullptr) {
        path.push_back(*node);
        node = node->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

// Function to free memory allocated for the tree nodes
void freeTreeMemory(vector<Node*>& tree) {
    for (Node* node : tree) {
        delete node;
    }
    tree.clear();
}

// Bidirectional RRT* function
pair<vector<Node>, bool> bidirectionalRRTStar(Node start_node, Node goal_node, const std::vector<std::vector<std::vector<bool>>>& grid,
    int x_size, int y_size, int z_size, double step_size, double radius, int max_iterations) {

    vector<Node*> start_tree;
    vector<Node*> goal_tree;

     // Create start and goal nodes on the heap
    Node* start = new Node(start_node.x, start_node.y, start_node.z);
    Node* goal = new Node(goal_node.x, goal_node.y, goal_node.z);

    start_tree.push_back(start);
    goal_tree.push_back(goal);

    Node* start_nearest_node = nullptr;
    Node* goal_nearest_node = nullptr;

    for (int i = 0; i < max_iterations; ++i) {
        // Grow start tree
        Node rand_node_start = getRandomNode(x_size, y_size, z_size, grid);
        start_nearest_node = findNearestNode(rand_node_start, start_tree);
        Node new_node_start = steer(start_nearest_node, rand_node_start, step_size, grid);

        if (start_nearest_node->x != new_node_start.x || start_nearest_node->y != new_node_start.y || start_nearest_node->z != new_node_start.z) {
            vector<Node*> nearby_nodes_start = findNearbyNodes(new_node_start, start_tree, radius);
            Node* best_parent_start = chooseParent(start_nearest_node, new_node_start, nearby_nodes_start, grid);

            // Allocate memory for the new node using 'new'
            Node* newNodePtr = new Node(new_node_start.x, new_node_start.y, new_node_start.z);
            newNodePtr->parent = best_parent_start;
            newNodePtr->cost = best_parent_start->cost + calculateDistance(*best_parent_start, new_node_start);

            start_tree.push_back(newNodePtr);

            rewire(newNodePtr, nearby_nodes_start, grid);
        }

         // Check for connection from start tree
        goal_nearest_node = findNearestNode(new_node_start, goal_tree);
        if (isPathCollisionFree(const_cast<Node*>(&new_node_start), goal_nearest_node, grid)) {
            cout << "Path found after iterations: " << i << endl;
            // **Print Connecting Nodes (Start to Goal)**
            cout << "Connection: Start Tree to Goal Tree" << endl;
            cout << "  Start Tree Connecting Node: (" << new_node_start.x << ", " << new_node_start.y << ", " << new_node_start.z << ")" << endl;
            cout << "  Goal Tree Connecting Node: (" << goal_nearest_node->x << ", " << goal_nearest_node->y << ", " << goal_nearest_node->z << ")" << endl;
            // Connect the trees
            //goal_nearest_node->parent = &new_node_start;

            // Extract and return the combined path
            vector<Node> start_path = reconstructPath(start_tree.back());
            vector<Node> goal_path = reconstructPath(goal_nearest_node);
            reverse(goal_path.begin(), goal_path.end());
            start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());

             // Free memory before returning
            freeTreeMemory(start_tree);
            freeTreeMemory(goal_tree);

            return { start_path, true };
        }

         // Grow goal tree
        Node rand_node_goal = getRandomNode(x_size, y_size, z_size, grid);
        goal_nearest_node = findNearestNode(rand_node_goal, goal_tree);
        Node new_node_goal = steer(goal_nearest_node, rand_node_goal, step_size, grid);

        if (goal_nearest_node->x != new_node_goal.x || goal_nearest_node->y != new_node_goal.y || goal_nearest_node->z != new_node_goal.z) {
            vector<Node*> nearby_nodes_goal = findNearbyNodes(new_node_goal, goal_tree, radius);
            Node* best_parent_goal = chooseParent(goal_nearest_node, new_node_goal, nearby_nodes_goal, grid);

            // Allocate memory for the new node using 'new'
            Node* newNodePtr = new Node(new_node_goal.x, new_node_goal.y, new_node_goal.z);
            newNodePtr->parent = best_parent_goal;
            newNodePtr->cost = best_parent_goal->cost + calculateDistance(*best_parent_goal, new_node_goal);

            goal_tree.push_back(newNodePtr);

            rewire(newNodePtr, nearby_nodes_goal, grid);
        }
          // Check for connection from goal tree
        start_nearest_node = findNearestNode(new_node_goal, start_tree);
        if (isPathCollisionFree(const_cast<Node*>(&new_node_goal), start_nearest_node, grid)) {
            cout << "Path found after iterations: " << i << endl;
             // **Print Connecting Nodes (Goal to Start)**
            cout << "Connection: Goal Tree to Start Tree" << endl;
            cout << "  Goal Tree Connecting Node: (" << new_node_goal.x << ", " << new_node_goal.y << ", " << new_node_goal.z << ")" << endl;
            cout << "  Start Tree Connecting Node: (" << start_nearest_node->x << ", " << start_nearest_node->y << ", " << start_nearest_node->z << ")" << endl;
            // Connect the trees
            //start_nearest_node->parent = &new_node_goal;

            // Extract and return the combined path
            vector<Node> goal_path = reconstructPath(goal_tree.back());
            vector<Node> start_path = reconstructPath(start_nearest_node);
            reverse(goal_path.begin(), goal_path.end());
            start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());

            //std::reverse(start_path.begin(), start_path.end());
            //goal_path.insert(goal_path.end(), start_path.begin(), start_path.end());


            // Free memory before returning
            freeTreeMemory(start_tree);
            freeTreeMemory(goal_tree);

            return { start_path, true };
        }
    }

      // Free memory before returning
    freeTreeMemory(start_tree);
    freeTreeMemory(goal_tree);

    cout << "Path not found" << endl;
    return { {}, false };
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


int main(int argc, char** argv) {
    ros::init(argc, argv, "rrt_path_publisher");
    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("rrt_bid_path", 10);

    std::string filename = "/home/meera/catkin_ws/src/hector_astar1/src/grid_output_2.txt";


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

    Node start_node(start_gx, start_gy, start_gz);
    Node goal_node(goal_gx, goal_gy, goal_gz);

    // RRT* parameters
    double step_size = 1.2;
    double radius = 10.0;
    int max_iterations = 10000;

    auto result = bidirectionalRRTStar(start_node, goal_node, grid, x_size, y_size, z_size, step_size, radius, max_iterations);
    auto path = result.first;
    bool pathFound = result.second;

    if (pathFound) {
        std::cout << "Path found:" << std::endl;
        nav_msgs::Path ros_path;
        ros_path.header.stamp = ros::Time::now();
        ros_path.header.frame_id = "map";  // Set the frame ID (change as per your TF)

        std::ofstream path_file("/home/meera/catkin_ws/src/hector_rrt_star/src/rrt_path_output.txt");
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
            std::cout << "Entered into path_output_2.cpp" << std::endl;
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
    else {
        std::cout << "No path found." << std::endl;
    }
    return 0;
}
