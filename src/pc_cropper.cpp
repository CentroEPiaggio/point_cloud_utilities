/**
    pc_cropper.cpp

    Purpose: a ROS node to crop a pointcloud. This code crops a generic resolution
    point cloud to a fixed res at center (x_res * y_res) keeping the PointCloud2 organized
    and pubishes it to a topic.

    Input Topic: 	/kinect2/sd/points
    Output Topic: 	/cropped_points

    @authors Pollayil George Jose, Pollayil Mathew Jose
*/

// ROS INCLUDES
#include <ros/ros.h>

// PCL INCLUDES
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OTHER INCLUDES
#include <cmath>

#define DEBUG			0										// if DEBUG 1 prints additional couts

// GLOBAL VARIABLES
ros::Publisher pub_cropped_points; 								// publisher for cropped point cloud
int x_res = 60;
int y_res = 60;

// CALLBACK FUNCTION
void crop_and_publish_cb(const sensor_msgs::PointCloud2ConstPtr& input_cloud){

  	// Container for original & filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

	// Convert to PCL data type
	pcl_conversions::toPCL(*input_cloud, *cloud);

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB> cropped_cloud;
    pcl::fromPCLPointCloud2(*cloud,*temp_cloud);

	// Checking if downsamplable
    if(temp_cloud->width < x_res || temp_cloud->height < y_res){
    	ROS_ERROR_STREAM("Desired dimension is bigger than original pointcloud dimensions!");
    }

    // Cropping using iterator
	int x_margin = std::floor((temp_cloud->width - x_res) / 2);		// number of points to remove from left and right
	int y_margin = std::floor((temp_cloud->height - y_res) / 2);	// number of points to remove from up and down

	// First push all columns in a row then change row
	for(int j = y_margin; j < (y_margin + y_res); j++){
	    for(int i = x_margin; i < (x_margin + x_res); i++){
	        cropped_cloud.push_back(temp_cloud->at(i,j));
	    }
	}

	if(DEBUG){
		std::cout << "Cloud just cropped has width " << cropped_cloud.width << " and height " << cropped_cloud.height << "!" << std::endl;
	}

	// Setting correctly width and height
	cropped_cloud.width = x_res;
	cropped_cloud.height = y_res;
	
  	// Convert to ROS data type
  	pcl::PCLPointCloud2 out_cloud;
  	pcl::toPCLPointCloud2(cropped_cloud, out_cloud);
	sensor_msgs::PointCloud2 output_pc;
	pcl_conversions::fromPCL(out_cloud, output_pc);

	// Write frame info to msg
	output_pc.width = cropped_cloud.width;
	output_pc.height = cropped_cloud.height;

	if(DEBUG){
		std::cout << "Cloud to be published has width " << cropped_cloud.width << " and height " << cropped_cloud.height << "!" << std::endl;
	}

	// Setting other message fields
	output_pc.header.stamp = input_cloud->header.stamp;
	output_pc.header.frame_id = input_cloud->header.frame_id;
	output_pc.is_dense = input_cloud->is_dense;

	// Publish the data
	pub_cropped_points.publish(output_pc);

	delete cloud;

}

// MAIN
int main(int argc, char** argv){

	// Initializing ROS node
	ros::init(argc, argv, "pc_cropper_node");
	ros::NodeHandle pc_nh;

	// Setting resolution params from param server
	pc_nh.param("x_res", x_res, 60);
	pc_nh.param("y_res", y_res, 60);

	// Creating a ROS subscriber for the input point cloud
	ros::Subscriber sub = pc_nh.subscribe("input_topic", 1, crop_and_publish_cb);

	// Creating a ROS publisher for the output point cloud
	pub_cropped_points = pc_nh.advertise<sensor_msgs::PointCloud2>("output_topic", 1);

	// Success message
	std::cout << "Downscaled points (to " << x_res << "x" << y_res << ") are being published from input_topic to output_topic!" << std::endl;

	// Spin
	ros::spin ();

}
