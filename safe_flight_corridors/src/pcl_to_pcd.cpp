#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/surface/poisson.h>
#include <iostream>
#include "safe_flight_corridors/cloud_utils.h"
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>

using namespace pcl;
using namespace std;


// Save mesh as an OBJ file
//pcl::io::saveVTKFile("output_hexagon_mesh.obj", mesh);
int main(){
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloudp(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/meera/catkin_ws/src/safe_flight_corridors/maps/output_cloud.pcd", *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    std::cout << "Loaded point cloud with " << cloud->size() << " points." << std::endl;

    //Complete normals
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);
    ne.compute(*normals);
    
    // Combine points and normals into a PointNormal cloud
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>());
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    
    // Perform Poisson surface reconstruction
    pcl::Poisson<pcl::PointNormal> poisson;
    poisson.setDepth(9);  // Higher depth = finer mesh
    poisson.setInputCloud(cloud_with_normals);

    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);
    std::cout << "Computed " << normals->size() << " normals." << std::endl;

    
    pcl::io::saveVTKFile("/home/meera/catkin_ws/src/safe_flight_corridors/maps/output_hexagon_mesh.obj", mesh);
    return 0;
}