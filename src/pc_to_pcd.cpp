/**
    pc_to_pcd.cpp

    a ROS node to save the point cloud as PCD. A point cloud (sensor_msgs::PointCloud2) taken
    from a specified topic (e.g. /kinect2/sd/points) is saved to a PCD file.

    Input Topic: 	/kinect2/sd/points

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

// OTHER INCLUDES
#include <cmath>

#define DEBUG			0										// if DEBUG 1 prints additional couts

// GLOBAL VARIABLES
bool quit_requested = false;									// bool to shutdown once .pcd is saved

// CALLBACK FUNCTION
void convert_and_save_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud){

  	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

	// Convert to PCL data type
	pcl_conversions::toPCL(*input_cloud, *cloud);

	// Ony x, y and z are saved to .pcd
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(*cloud, *temp_cloud);
	
	// Save the cloude to pcd ASCII
    int result_writing = pcl::io::savePCDFileASCII("test_pcd.pcd", *temp_cloud);

    // Print message and exit
	std::cerr << "Saved " << temp_cloud->points.size () << " data points to test_pcd.pcd and the result was " << result_writing << "." << std::endl;

	std::cout << "Saved point cloud to PCD, shutting down the node!" << std::endl;

	quit_requested = true;

	delete cloud;
}

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "pc_to_pcd_node");
	ros::NodeHandle pc_nh;

	// Creating a ROS subscriber for the input point cloud
	ros::Subscriber sub = pc_nh.subscribe("input_topic", 1, convert_and_save_cb);

	// Success message
	std::cout << "Saving PCD from input topic!" << std::endl;

	// Spin if pcd file not saved yet
	while(!quit_requested){
		ros::spinOnce();
	}
}
