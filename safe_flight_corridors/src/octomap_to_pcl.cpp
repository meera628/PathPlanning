#include <octomap/octomap.h>
#include <iostream>
#include <cstddef>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_field_conversion.h>
#include <pcl/io/pcd_io.h>  // For saving PCD file
#include <pcl/filters/voxel_grid.h>
#include "safe_flight_corridors/cloud_utils.h"

using namespace std;
ros::Publisher pcl_pub; //Publishing pcl2 data
//ros::Publisher pcl_one_pub; //Publishing pcl data
void octomap_to_pcl_converter_bt(const string& filename){
    octomap::OcTree tree(filename); //loading file into an octree type
    for (octomap::OcTree::leaf_iterator it=tree.begin_leafs(); it!= tree.end_leafs();it++){
        if (tree.isNodeOccupied(*it)){
            cloud->push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
        }
    }
    downSamplePCL(cloud);
    pcl::io::savePCDFileASCII("/home/meera/catkin_ws/src/safe_flight_corridors/maps/output_cloud.pcd", *cloud);
    ROS_INFO("Saved %lu points to output_cloud.pcd", cloud->points.size());
    
    sensor_msgs::PointCloud2 output_cloud;

    pcl::toROSMsg(*cloud, output_cloud);
   
    output_cloud.header.frame_id = "map"; 
    output_cloud.header.stamp = ros::Time::now();
    pcl_pub.publish(output_cloud);
    ROS_INFO("Publishing PointCloud2 with %lu points", cloud->points.size());


    //sensor_msgs::PointCloud output_cloud_one;
    //sensor_msgs::convertPointCloud2ToPointCloud(output_cloud, output_cloud_one);
    //pcl_one_pub.publish(output_cloud_one);
    
    //ROS_INFO("Publishing PointCloud with %lu points", output_cloud_one.points.size());

}

int main(int argc, char** argv){
    ros::init(argc,argv, "octomap_to_pcl");
    ros::NodeHandle nh;
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("safe_flight_corridors/pointcloud2", 10);
    //pcl_one_pub = nh.advertise<sensor_msgs::PointCloud>("safe_flight_corridor/pointcloud", 1); // New topic for rrt_decomp
    
    //string filename_bt = "/home/meera/catkin_ws/src/safe_flight_corridors/maps/hexagon1.bt";
    octomap_to_pcl_converter_bt(filename_bt);
    //ros::spin();
    ros::Rate loop_rate(10);  // Publish at 1Hz

    while (ros::ok()) {
        sensor_msgs::PointCloud2 output_cloud;
        pcl::toROSMsg(*cloud, output_cloud);

        output_cloud.header.frame_id = "map"; 
        output_cloud.header.stamp = ros::Time::now();
        pcl_pub.publish(output_cloud);

        ROS_INFO("Publishing PointCloud2 with %lu points", cloud->points.size());
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}
