#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <fstream>  // For file output
#include <cstddef>
// Function to load OctoMap and create a 3D grid
std::vector<std::vector<std::vector<bool>>> parseOctomapToGrid(const std::string& file_path, double& resolution, octomap::point3d& min_bound, octomap::point3d& max_bound) {
    // Load the .bt file into an OctoMap
    octomap::OcTree tree(file_path);
    
    // Get resolution
    resolution = tree.getResolution();
    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    tree.getMetricMin(min_x, min_y, min_z);
    tree.getMetricMax(max_x, max_y, max_z);

    // Create bounds as octomap::point3d
    min_bound = octomap::point3d(min_x, min_y, min_z);
    max_bound = octomap::point3d(max_x, max_y, max_z);
    
    // Calculate grid dimensions
    int x_size = static_cast<int>((max_bound.x() - min_bound.x()) / resolution) + 1;
    int y_size = static_cast<int>((max_bound.y() - min_bound.y()) / resolution) + 1;
    int z_size = static_cast<int>((max_bound.z() - min_bound.z()) / resolution) + 1;

    // Create a 3D grid initialized to false
    std::vector<std::vector<std::vector<bool>>> grid(x_size, std::vector<std::vector<bool>>(y_size, std::vector<bool>(z_size, false)));

    // Traverse the octree
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {
            // Get the coordinates of the occupied voxel
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            // Convert to grid indices
            int i = static_cast<int>((x - min_bound.x()) / resolution);
            int j = static_cast<int>((y - min_bound.y()) / resolution);
            int k = static_cast<int>((z - min_bound.z()) / resolution);

            // Mark the voxel as occupied
            grid[i][j][k] = true;
        }
    }

    return grid;
}

// Function to write the grid to a file
void writeGridToFile(const std::vector<std::vector<std::vector<bool>>>& grid, 
                    const std::string& filename, const octomap::point3d& min_bound, 
                     const octomap::point3d& max_bound, 
                     double resolution) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    //Resolution
    file << resolution << "\n";
    //Min bounds x y z
    file << min_bound.x() << " " << min_bound.y() << " " << min_bound.z() << "\n";
    //Max bounds x y z
    file << max_bound.x() << " " << max_bound.y() << " " << max_bound.z() << "\n";

    // Write grid dimensions on the first line
    int x_size = grid.size();
    int y_size = grid[0].size();
    int z_size = grid[0][0].size();
    //Grid dimensions
    file << x_size << " " << y_size << " " << z_size << "\n";

    for (int i = 0; i < grid.size(); ++i) {
        for (int j = 0; j < grid[i].size(); ++j) {
            for (int k = 0; k < grid[i][j].size(); ++k) {
                file << grid[i][j][k] << " ";  // Write 0 or 1 for unoccupied or occupied
            }
            file << "\n";
        }
    }

    file.close();
    std::cout << "Grid written to file: " << filename << std::endl;
}

int main() {
    // Path to your .bt file
    std::string file_path = "/home/meera/catkin_ws/src/hector_astar1/maps/hexagon1.bt";

    // Variables to store metadata
    double resolution;
    octomap::point3d min_bound, max_bound;

    // Parse the .bt file and create the grid
    auto grid = parseOctomapToGrid(file_path, resolution, min_bound, max_bound);

    // Write the grid to a file
    writeGridToFile(grid, "grid_output.txt", min_bound, max_bound, resolution);

    return 0;
}
