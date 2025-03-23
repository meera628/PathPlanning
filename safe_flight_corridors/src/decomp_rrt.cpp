#include "/home/meera/decomp_ws/src/DecompROS/decomp_test_node/src/txt_reader.hpp"
#include <decomp_ros_utils/data_ros_utils.h>
#include <ros/ros.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

ros::Publisher es_pub;
ros::Publisher poly_pub;
ros::Publisher path_pub;


void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {

    ROS_INFO("[cloudCallback] Received PointCloud2 message");

    // pcl::PCLPointCloud2 pcl_pc2;
    // pcl_conversions::toPCL(*msg,pcl_pc2);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::fromROSMsg(*msg, *temp_cloud);
    sensor_msgs::PointCloud cloud_msg;
    sensor_msgs::convertPointCloud2ToPointCloud(*msg, cloud_msg);

    vec_Vec3f obs = DecompROS::cloud_to_vec(cloud_msg);


    ROS_INFO("[cloudCallback] Converted point cloud to vec_Vec3f with %lu points", obs.size());
    // Read path from file
    std::string path_file;
    ros::NodeHandle nh;
    nh.param("path_file", path_file, std::string("/home/meera/catkin_ws/src/hector_rrt_star/src/rrt_path_output.txt"));
    
    vec_Vec3f path;
    if (!read_path<3>(path_file, path)) {
        ROS_ERROR("Fail to read a path!");
        return;
    }

    nav_msgs::Path path_msg = DecompROS::vec_to_path(path);
    path_msg.header.frame_id = "map";
    path_pub.publish(path_msg);
    ROS_INFO("[cloudCallback] Published path message with %lu points", path_msg.poses.size());

    // Perform Ellipsoid Decomposition
    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(obs);
    decomp_util.set_local_bbox(Vec3f(1, 2, 1));
    decomp_util.dilate(path);
    ROS_INFO("[cloudCallback] Ellipsoid Decomposition completed");

    // Publish visualization messages
    decomp_ros_msgs::EllipsoidArray es_msg = DecompROS::ellipsoid_array_to_ros(decomp_util.get_ellipsoids());
    es_msg.header.frame_id = "map";
    es_pub.publish(es_msg);
    ROS_INFO("[cloudCallback] Published EllipsoidArray with %lu ellipsoids", es_msg.ellipsoids.size());

    decomp_ros_msgs::PolyhedronArray poly_msg = DecompROS::polyhedron_array_to_ros(decomp_util.get_polyhedrons());
    poly_msg.header.frame_id = "map";
    poly_pub.publish(poly_msg);
    ROS_INFO("[cloudCallback] Published PolyhedronArray with %lu polyhedrons", poly_msg.polyhedrons.size());
}

int main(int argc, char **argv){
    ros::init(argc, argv, "rrt_decomp");
    ros::NodeHandle nh;
    es_pub = nh.advertise<decomp_ros_msgs::EllipsoidArray>("ellipsoid_array", 1, true);
    poly_pub = nh.advertise<decomp_ros_msgs::PolyhedronArray>("polyhedron_array", 1, true);
    path_pub = nh.advertise<nav_msgs::Path>("rrt_connect_path", 1, true);

    // Subscribe to the PointCloud topic from octomap_to_pcl
    ros::Subscriber cloud_sub = nh.subscribe("safe_flight_corridors/pointcloud2", 10, cloudCallback);

    ros::spin();
    return 0;
}