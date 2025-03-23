#ifndef CLOUD_UTILS_H
#define CLOUD_UTILS_H
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
//External pcl cloud pointer that can be shared across files
extern std::string filename_bt;
extern pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
void downSamplePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

#endif 