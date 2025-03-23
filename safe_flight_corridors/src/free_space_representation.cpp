#include <octomap/octomap.h>
#include <iostream>
#include <cstddef>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>  // For saving PCD file
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h> 
#include <vector>  
#include "safe_flight_corridors/cloud_utils.h"
using namespace std;
using namespace pcl;

int main(){
       
    octomap::OcTree tree(filename_bt); 
}