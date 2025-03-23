#include "safe_flight_corridors/cloud_utils.h"
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
std::string filename_bt = "/home/meera/catkin_ws/src/safe_flight_corridors/maps/hexagon1.bt";

void downSamplePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f); // Set the leaf size for downsampling
    sor.filter(*cloud);
}