/**
    pc_transform.cpp

    a ROS node to transform a point cloud in PCD. A point cloud (.pcd) taken from a specified path
    (e.g. ~/.ros) is transformed using a homogeneous transformation and saved to another PCD file.

    Input File:		specified in launch
    Output File:	specified in launch

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

// ROS INCLUDES
#include <ros/ros.h>

// PCL INCLUDES
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

// OTHER INCLUDES
#include <cmath>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>
#include "point_cloud_utilities/pcd_transform.h"

#define DEBUG			0										// if DEBUG 1 prints additional couts

// GLOBAL VARIABLES
Eigen::Affine3d transform;										// Transformation affine matrix for transforming the pc


// SERVICE CALLBACK
bool transform_and_save(point_cloud_utilities::pcd_transform::Request &req, point_cloud_utilities::pcd_transform::Response &res){

	pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// Loading the .pcd file specified in arguments
	if (pcl::io::loadPCDFile (req.pcd_path, *source_cloud) < 0)  {
      std::cout << "Error loading point cloud " << req.pcd_path << std::endl << std::endl;
      return -1;
    }

	// Success message
	std::cout << "Loaded PCD from specified path!" << std::endl;

	// Getting the desired transform
	geometry_msgs::Pose tmp_transform;
	tf::poseMsgToEigen(tmp_transform, transform);

	// Executing the transformation
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());

	// You can either apply transform_1 or transform_2; they are the same
	pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

	// Success message
	std::cout << "Transformed point cloud, now saving into destination folder!" << std::endl;

	// Save the cloud to pcd ASCII
    int result_writing = pcl::io::savePCDFileASCII("transfromed_pcd.pcd", *transformed_cloud);

    // Print message and exit
	std::cout << "Saved " << transformed_cloud->points.size () << " data points to test_pcd.pcd and the result was " << result_writing << "." << std::endl;

	std::cout << "Saved transformed point cloud to PCD, shutting down the node!" << std::endl;

}


// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "transform_pcd_node");
	ros::NodeHandle pc_nh;

	ros::ServiceServer service = pc_nh.advertiseService("pcd_transform_service", &transform_and_save);

	std::cout << "Started PDC converting service!" << std::endl;

	ros::spin();

}
