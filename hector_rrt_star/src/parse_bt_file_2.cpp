#include <octomap/octomap.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <queue>
#include <cstddef>

// Load OctoMap and create a 3D grid with obstacle inflation
std::vector<std::vector<std::vector<bool>>> parseOctomapToGrid(
    const std::string& file_path, double& resolution, octomap::point3d& min_bound,
    octomap::point3d& max_bound, int inflation_radius) {
    
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
    int x_size = static_cast<int>((max_x - min_x) / resolution) + 1;
    int y_size = static_cast<int>((max_y - min_y) / resolution) + 1;
    int z_size = static_cast<int>((max_z - min_z) / resolution) + 1;
    auto inBounds = [&](int i, int j, int k) {
        return i >= 0 && i < x_size && j >= 0 && j < y_size && k >= 0 && k < z_size;
    };
    // Create a 3D grid initialized to false
    std::vector<std::vector<std::vector<bool>>> grid(
        x_size, std::vector<std::vector<bool>>(y_size, std::vector<bool>(z_size, false)));

    // Store occupied cells
    std::queue<std::tuple<int, int, int>> occupied_cells;

    // Traverse the octree
    for (auto it = tree.begin_leafs(), end = tree.end_leafs(); it != end; ++it) {
        if (tree.isNodeOccupied(*it)) {
            double x = it.getX(), y = it.getY(), z = it.getZ();
            int i = static_cast<int>((x - min_x) / resolution);
            int j = static_cast<int>((y - min_y) / resolution);
            int k = static_cast<int>((z - min_z) / resolution);
            //int inflation_radius_voxels = static_cast<int>(inflation_radius / resolution);
            int inflation_radius_voxels = static_cast<int>(inflation_radius);

            // Mark all surrounding voxels in a cubic range around the occupied voxel
            for (int di = -inflation_radius_voxels; di <= inflation_radius_voxels+1; ++di) {
                for (int dj = -inflation_radius_voxels; dj <= inflation_radius_voxels+1; ++dj) {
                    for (int dk = -inflation_radius_voxels; dk <= inflation_radius_voxels+1; ++dk) {
                        int ni = i + di;
                        int nj = j + dj;
                        int nk = k + dk;

                        // Only mark the voxel if it's within bounds
                        if (inBounds(ni, nj, nk)) {
                                grid[ni][nj][nk] = true;
                                //std::cout << "inflating " <<  std::endl;

                            }
                        }
                    }
                }
            }
        }        


    return grid;
}

// Function to write the grid to a file
void writeGridToFile(const std::vector<std::vector<std::vector<bool>>>& grid, 
                     const std::string& filename, const octomap::point3d& min_bound, 
                     const octomap::point3d& max_bound, double resolution) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    
    // Write metadata
    file << resolution << "\n";
    file << min_bound.x() << " " << min_bound.y() << " " << min_bound.z() << "\n";
    file << max_bound.x() << " " << max_bound.y() << " " << max_bound.z() << "\n";
    file << grid.size() << " " << grid[0].size() << " " << grid[0][0].size() << "\n";

    // Write grid data
    for (const auto& xy_plane : grid) {
        for (const auto& row : xy_plane) {
            for (bool cell : row) {
                file << cell << " ";
            }
            file << "\n";
        }
    }
    file.close();
    std::cout << "Grid written to file: " << filename << std::endl;
}

int main() {
    std::string file_path = "/home/meera/catkin_ws/src/hector_rrt_star/maps/hexagon1.bt";
    double resolution;
    octomap::point3d min_bound, max_bound;
    int inflation_radius = 4; // Adjust this value based on drone size
    
    auto grid = parseOctomapToGrid(file_path, resolution, min_bound, max_bound, inflation_radius);
    writeGridToFile(grid, "/home/meera/catkin_ws/src/hector_rrt_star/src/grid_output_2.txt", min_bound, max_bound, resolution);
    return 0;
}
